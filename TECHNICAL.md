# V1TAL — technical overview

V1TAL makes a first-generation Roborock/Rockrobo **V1** keep its map and expose
**room-segmented maps** to stock **Valetudo**. Everything runs on the robot: an `LD_PRELOAD`
shim patches the running `AppProxy` in memory, a producer writes a sidecar next to `last_map`,
and two small daemons keep the map and segments alive.

This is an overview, not an implementation reference. Exact layouts, thresholds, patch bytes,
and ABI details live in the source files linked at the end.

> **Firmware pin.** The shim only supports V1 firmware **004028** and refuses any `AppProxy`
> whose SHA-256 is not `df9b124c324842394d9f74ace99cb8eee387798922c991e755736e604376f4cc`.

## 1. Architecture

```
last_map + slam_info.cfg
        │
        ├── v1guard ─────────────── snapshot / restore across nav clears
        │
        ├── v1seg ────────────────> last_segmap (RRSEG sidecar)
        │      libroomseg + model
        │
        └── AppProxy + libv1seg_addon.so (LD_PRELOAD)
               • serves the 28-byte IMAGE header Valetudo expects
               • answers the standard room commands
                         │
                         ▼
                 stock Valetudo (one model definition)
```

`last_map` is nav-owned and never rewritten by V1TAL. The add-on produces `last_segmap`, keeps
both files alive, and presents a standard blob and command surface to Valetudo.

### Module map

| file | role |
|---|---|
| `src/addon.c` | `LD_PRELOAD` shim: in-memory patch, blob encoding, command handlers |
| `src/gates.S` | Thumb gates that branch from patched `AppProxy` sites into the shim |
| `src/v1seg.cpp` | producer: SLAM grid or `last_map` → `last_segmap` |
| `src/persist.{c,h}` | RRSEG format, snapshot/recover/reset, frame-link, guard logic |
| `src/guard.{c,h}` | `v1guard` keep-alive daemon |
| `src/v1appd.c` | producer driver and one-shot AppProxy decision |
| `src/segedit.{c,h}` | `merge_segment` / `split_segment` grid edits |
| `src/reanchor.{c,h}` | optional occupancy-grid registration for frame drift |
| `init/` | upstart jobs for `v1appd` and `v1guard` |
| `valetudo/` | the one model definition and upstream patch |

The producer is short-lived; measured peak RSS on a 348x402 map is ~7.5 MB.
It runs with `oom_score_adj=1000` (first OOM victim) and `v1appd` retries it
with backoff.  `v1seg` checks every allocation and `_exit(1)`s with
`[v1seg] allocation failed: ...`, so a genuine allocation failure is a clean
producer failure, not a null dereference.

### Language choices

Everything is C except `src/v1seg.cpp`. The producer calls the S5's C++ segmentation library,
which can throw `cv::Exception`; the single C++ translation unit exists to catch that and fall
back to geometry-only output.  A `std::bad_alloc` is fatal (`_exit(1)`) so `v1appd` retries
instead of publishing a degraded sidecar. Keeping the shim in C avoids pulling `libstdc++` and C++
initialisation into a process we only minimally patch.

## 2. On-robot data

### 2.1 `last_map` — nav-owned

Little-endian SLAM map: a fixed header followed by `width × height` `float32` log-odds cells
(20 cells/m, i.e. 5 cm per cell). The nav writes it as a stream, so readers must tolerate a
partially written file. `slam_info.cfg` carries the nav's `timestamp`; `timestamp == 0` means
the map was cleared.

Exact header fields and readers: `src/persist.h`, `src/v1seg.cpp`.

### 2.2 `last_segmap` — V1TAL sidecar

A self-describing `RRSEG` file next to `last_map`:

- a fixed header with a frame link (`map_timestamp`, `map_id`, scale, origin, size),
- one segment id per cell (`byte & 0x3f`),
- a room table (`id`, area, centroid),
- a CRC32 over the header and payload.

Writes are atomic (`.tmp` + `fsync` + `rename`). The frame link prevents a segment set from a
different map frame being applied. Exact layout and checks: `src/persist.h`,
`src/persist.c`.

### 2.3 The RRMap IMAGE block

Stock V1 `AppProxy` emits an IMAGE block with a **24-byte** header. Valetudo's parser expects
**28** bytes: the extra `u32` is `segment_count`. Segment ids are packed into the upper bits of
the existing pixel bytes, while the low 3 bits keep the stock floor/wall type.

The shim patches the header initialiser and the emitter so every served block is a valid
28-header block, and makes the read-back path walk blocks by their own
`header_length + data_length`. The exact edit table and pixel masking rule are in
`src/addon.c`; the gate stubs are in `src/gates.S`.

### 2.4 Snapshot and frame-link

`persist/` under the data directory holds a protected copy of the map, sidecar, and
`slam_info.cfg`, plus a frame record that fingerprints all three. `v1guard` restores them
together when the nav clears the map. A sidecar is only reused when its frame matches; a
mismatch is ignored rather than risk cleaning the wrong room.

Exact record and file names: `src/persist.h`, `src/persist.c`.

### 2.5 Persistence is unconditional; `reset_map` is the escape hatch

There is no user-facing toggle and no `lab_status` flag. The guard always protects the
snapshot. A deliberate `reset_map` clears both the live map and the snapshot, so the guard
cannot resurrect a map the user just reset.

### 2.6 Segment edits

`merge_segment` and `split_segment` operate directly on the byte-per-cell sidecar. The add-on
reads the current file, applies the grid edit, rebuilds the room table, and rewrites the
sidecar atomically with a fresh CRC32. Ids follow the S5 convention (`auto < 16`,
`user 16..31`). The blit offset is derived from the map extents and sidecar padding at runtime.

The edit semantics and bit layout are in `src/segedit.c` and `src/segedit.h`.

## 3. Command surface

The shim handles these methods and passes everything else through to stock `AppProxy`:

| method | behaviour |
|---|---|
| `app_segment_clean` | ids → per-room zones, rewritten to `app_zoned_clean`; stock cleaning path does the work. The stock zone packet holds 5 zones. With more rooms, room rectangles are merged, cheapest first, but only where the merged rectangle covers no cell of an unselected room; if that cannot reach 5, or no requested room has cells, the request is refused with an error |
| `get_room_mapping` | room ids and names from the sidecar / `room_names.cfg` |
| `name_segment` | atomic rewrite of `room_names.cfg` |
| `merge_segment` | relabel listed rooms into one user room |
| `split_segment` | cut one room along a line into two user rooms |
| `save_map` | explicit snapshot |
| `reset_map` | clear live map and snapshot |
| `get_recover_maps` / `recover_map` | list and restore snapshots |
| `set_lab_status` | not handled; passes through |
| anything else | passes through to stock `AppProxy` |

Zone geometry uses the 5 cm cell frame; the stock zoned-clean handler performs the final
coordinate conversion.

A failed command is answered with a miio error reply, `{"id":N,"error":{"code":-10000,
"message":"..."}}`. That is the only reply shape Valetudo reports as a failure; a
`result` of `["error"]` would be shown as success. A refused merge or split, a failed
names write, and a failed save/reset/recover all answer this way. The command parsing, reply shaping, and zone mapping are in
`src/addon.c`.

### Runtime knobs

The shim accepts optional environment overrides for the data root, sidecar paths, logging,
timeouts, dry-run cleaning, and zone endpoint behaviour. They are defined and documented in
`src/addon.c` (`cfg_init`) and the upstart job.

## 4. AppProxy patch strategy

The shim is an `LD_PRELOAD` object whose constructor runs before `main`. It only proceeds when:

- `/proc/self/exe` is `AppProxy` (any other process that inherits `LD_PRELOAD` is left
  untouched: no log, no allocation, no file access), and
- every target instruction still contains its expected original bytes (the full 12 bytes
  at each gate site).

The shim does not hash the binary itself. The SHA-256 pin is checked by the installer
(`tools/install-remote.sh`) before it enables the preload line.

It then patches `.text` in memory and installs four gates plus the in-place instruction edits.
Nothing on disk is modified. If any check fails, it logs and does nothing.

The patch changes three things conceptually:

1. the emitted IMAGE header from 24 to 28 bytes,
2. the pixel base and data length to match the larger header,
3. the deserializer to walk blocks by their declared length rather than assuming 24.

The exact offsets, expected bytes, gate targets, and ABI details are intentionally not repeated
here. `src/addon.c` contains the authoritative edit table and hook implementations;
`src/gates.S` contains the gate stubs. Revert by dropping `LD_PRELOAD` / `ld.so.preload`.

## 5. Drift handling

Gen1 frame instability is handled in three layers:

1. **Keep-alive:** `v1guard` watches `last_map` + `slam_info.cfg`, snapshots a good frame, and
   restores map + sidecar + metadata together after a clear. A deliberate `reset_map` clears
   the snapshot too.
2. **Frame-link safety:** a sidecar is only applied when its scale, origin, size, timestamp,
   and map id match the current map (within the documented tolerances). Mismatches are ignored,
   not guessed.
3. **Registration:** `reanchor` can register a stored grid against the current one and
   transform the stored segments. It abstains unless the match is confident and unambiguous.
   It is built and shipped, but not wired into the runtime path yet.

Thresholds, matching rules, and the transform implementation are in `src/reanchor.c`,
`src/reanchor.h`, and `src/persist.c`. Drift correction has only been exercised with
synthesized frames, not captured live motion.

## 6. Limits

- V1 firmware 004028 and the pinned `AppProxy` only.
- No flashing; installation changes a runtime kit, two upstart jobs, and one
  `ProcessList.conf` line.
- `reanchor` is not wired into the producer/persist path yet.
- Segment ids are not stable across recomputes.
- `split_segment` is a straight half-plane cut, not a byte-exact S5 contour split.
- Segmentation parameters are empirical; room counts vary with the floor plan.
- Valetudo must be rebuilt with the model definition; see `BUILD.md`.

## 7. Where to look in the code

| question | read |
|---|---|
| How does the shim patch and answer commands? | `src/addon.c`, `src/gates.S` |
| What exactly is in `last_map` / `last_segmap`? | `src/persist.h`, `src/v1seg.cpp` |
| How does snapshot/restore and frame-link work? | `src/persist.c`, `src/persist.h` |
| How does the guard decide what to restore? | `src/guard.c` |
| How are rooms produced? | `src/v1seg.cpp` |
| How do merge/split edits work? | `src/segedit.c`, `src/segedit.h` |
| How is drift corrected? | `src/reanchor.c`, `src/reanchor.h` |
| How is the runtime driven? | `src/v1appd.c`, `init/v1appd.conf`, `init/v1guard.conf` |
| How is it built and installed? | `BUILD.md`, `Makefile`, `tools/install*.sh` |
| What is the one Valetudo change? | `valetudo/` |
