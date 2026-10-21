> **EARLY ALPHA STAGE.** First tests on-robot worked surprisingly well though :)
>
> This software is delivered as is. The author takes no responsibility for damage to your
> robot, PC, or ego.
>
> This was made possible by using AI. While I did my best to make this not "FULL SLOP MODE", it
> might very well still contain slop in the code. Stay away from it if that bothers you beyond
> forgiveness.

# V1TAL

**Persistent Maps for the Xiaomi/Roborock V1 running Valetudo**

V1TAL gives a first-generation Roborock/Rockrobo V1 two things the stock firmware lacks:
**persistent maps** and **room-segmented maps**. Everything runs on the robot. There is no
Valetudo fork and no flashing: the stock `AppProxy` is patched in memory by an `LD_PRELOAD`
shim, and the map + segment sidecar live on the data partition.

> **Requirement: Valetudo.** V1TAL requires a V1 running Valetudo. The robot-side add-on
> provides the map/room protocol and persistence; Valetudo is the UI and the client that
> triggers room cleaning. The small model-definition change must be built into Valetudo as
> described in [`BUILD.md`](BUILD.md).

> **Firmware pin.** V1 firmware **004028** only. The shim is anchored to the stock `AppProxy`
> with SHA-256 `df9b124c324842394d9f74ace99cb8eee387798922c991e755736e604376f4cc`. If your
> binary differs, V1TAL refuses to patch it.

## What it does

- **Persistent map.** A keep-alive guard snapshots `last_map`, `last_segmap`, and
  `slam_info.cfg`, and restores them together if the navigation stack clears the map.
  `reset_map` is the deliberate escape hatch; it clears the snapshot too.
- **Rooms.** A producer runs the S5's own segmentation model over the V1's SLAM grid and writes
  `last_segmap` (an `RRSEG` sidecar). The shim makes the V1 serve the standard 28-byte-header
  RRMap IMAGE block, with segment ids in the pixel bytes.
- **Clean by room.** The shim answers the standard Roborock methods (`app_segment_clean`,
  `get_room_mapping`, `name_segment`, `merge_segment`, `split_segment`, `save_map`,
  `reset_map`, `get_recover_maps`, `recover_map`). `app_segment_clean` is rewritten to the
  stock zoned-clean path, so cleaning uses the robot's own machinery.
- **Drift handling.** A standalone `reanchor` helper can register a stored grid against the
  current one and transform the stored segments; it abstains when the match is ambiguous.
  It is built and shipped, but not wired into the runtime path yet.

The only Valetudo change is one **model definition** (`valetudo/RoborockV1SegmentedValetudoRobot.js`)
plus its registry line. It registers the standard capabilities and contains no parser, UI, or
`model === ...` special cases.

## How it works

```
last_map + slam_info.cfg
        │
        ├── v1guard ─────────────── keep-alive snapshot / restore
        │
        ├── v1seg ────────────────> last_segmap (RRSEG sidecar)
        │      libroomseg + model
        │
        └── AppProxy + libv1seg_addon.so (LD_PRELOAD)
               • 24-byte IMAGE header -> 28 bytes + segment bits
               • standard command surface
                         │
                         ▼
                 stock Valetudo (one model definition)
```

`last_map` itself is never rewritten. Persistence and segmentation are additive; dropping
`LD_PRELOAD` and removing the kit and jobs returns the robot to stock behaviour.

## Repository layout

```
Makefile          ARM build, host checks, runtime kit
README.md         this file
TECHNICAL.md      architecture, data formats, command surface, patch strategy
BUILD.md          prerequisites, build, kit, install/revert, Valetudo rebuild
artifacts/        where downloaded toolchain/firmware goes (see artifacts/README.md)
src/              on-robot C sources + the one C++ producer
init/             upstart jobs for v1appd and v1guard
valetudo/         the one model definition + upstream patch
tools/            S5 unpacker, installer, optional operator CLI
```

`tools/unpack_s5.sh` extracts the seven S5 files needed by `make kit`. The optional operator CLI
is [`tools/v1ctl.py`](tools/v1ctl.py); neither is needed to build the ARM binaries.

## Build

Put the extracted Bootlin toolchain under `artifacts/toolchain/` (see
[`artifacts/README.md`](artifacts/README.md)), then:

```bash
make            # ARM: libv1seg_addon.so, v1seg, v1appd, v1guard, reanchor

# one-time: pull the seven S5 files out of the firmware image
tools/unpack_s5.sh /path/to/roborock.vacuum.s5_2034_fw.tar.gz

make kit        # runtime kit + init jobs (uses the S5 payload)
```

`make` needs only the toolchain. `make kit` additionally needs the seven S5 files for
`libroomseg`, OpenCV/Boost, and `roomseg.mdl`; the unpacker extracts just those. See
[`BUILD.md`](BUILD.md) for the artifact layout and the Valetudo rebuild.

## Install

```bash
make install ROBOT=root@192.168.1.50
make status ROBOT=root@192.168.1.50
make uninstall ROBOT=root@192.168.1.50
```

`make install` builds the kit, runs the robot-side installer, and asks before rebooting.
`make status` is read-only; `make uninstall` restores the stock `ProcessList.conf` line and
removes the kit and jobs. Use `REBOOT=1` to skip the reboot prompt. SSH keys are recommended.

The installer changes only the runtime kit, the two upstart jobs, and one
`/opt/rockrobo/watchdog/ProcessList.conf` line (backed up once as `.v1tal-orig`).

## Docs

- [BUILD.md](BUILD.md) — what to download, where to put it, how to build and install.
- [TECHNICAL.md](TECHNICAL.md) — architecture, formats, commands, patch strategy, drift handling.
- [artifacts/README.md](artifacts/README.md) — checksums and extraction instructions.

## Limits

- V1 004028 / the pinned `AppProxy` only.
- No flashing. The install procedure modifies only the runtime kit, the two upstart jobs, and
  one `ProcessList.conf` line (with a one-time backup).
- `reanchor` is not wired into the producer/persist path yet.
- Segmentation ids are not stable across recomputes; room edits are a straight half-plane cut,
  not a byte-exact reproduction of the S5 contour split.
- The Valetudo model definition must be built into the V1's single SEA binary; see BUILD.md.
