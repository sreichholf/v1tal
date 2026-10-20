# BUILD — prerequisites, compilation, kit, and installation

V1TAL builds cross-compiled ARM binaries for the V1. Nothing here flashes the robot.

## 1. What you need

| need | when | where it goes |
|---|---|---|
| Linux host with `make`, `gcc`, `python3` | always | system |
| Bootlin `armv7-eabihf--glibc--stable-2018.02-2` toolchain | `make` | `artifacts/toolchain/` |
| extracted S5 payload (V002034) | `make kit` and room segmentation | `artifacts/s5/rootfs/` |

The toolchain is the only hard requirement for `make`. The S5 payload supplies the segmentation
model and its shared libraries; without it the kit has no `roomseg.mdl`, `libroomseg`, or
OpenCV/Boost. The stock `AppProxy` is verified on the robot before installing (see §4).

Download URLs, checksums, and extraction commands are in
[`artifacts/README.md`](artifacts/README.md).

## 2. Build

```bash
make                 # ARM deliverables
make host            # host-side compile checks (not for the robot)
make clean

# one-time: pull the seven S5 files out of the firmware image
tools/unpack_s5.sh /path/to/roborock.vacuum.s5_2034_fw.tar.gz

make kit             # runtime kit + init jobs + Valetudo inputs
```

If the toolchain is somewhere else:

```bash
make TOOLCHAIN=/path/to/armv7-eabihf--glibc--stable-2018.02-2
```

`make` writes to `build/`; `make kit` writes to `dist/`.

### Outputs

| output | what it is |
|---|---|
| `build/libv1seg_addon.so` | the `LD_PRELOAD` shim: 28-header blob + command surface |
| `build/v1seg` | segment producer (`last_map` → `last_segmap`) |
| `build/v1appd` | producer driver; one-shot AppProxy decision by default |
| `build/v1guard` | keep-alive map/segment snapshot and restore |
| `build/reanchor` | optional drift registration helper (shipped, not wired) |
| `build/host/…` | host compile checks; **do not run on the robot** |

The recipes are the same flags the project was developed and tested with: GCC 6.4, glibc 2.19,
`-O2`, `-Wall -Wextra`. `v1seg` is the only C++ translation unit; it exists to catch
`cv::Exception` from the S5's segmentation library.

## 3. Runtime kit

```bash
make kit
```

```
dist/
  kit/
    v1appd
    v1seg
    v1guard
    reanchor
    libv1seg_addon.so
    roomseg.mdl
    lib/            libroomseg, OpenCV core/imgproc/dnn, Boost system/filesystem
    SHA256SUMS
  init/
    v1appd.conf
    v1guard.conf
  valetudo/
    RoborockV1SegmentedValetudoRobot.js
    UPSTREAM_PATCH.diff
```

`tools/unpack_s5.sh` verifies the firmware image, extracts only the seven files V1TAL needs,
and checks each SHA-256. Use `--keep-image` if you also want the 510 MB `disk.img` retained.

The `lib/` directory includes the SONAME symlinks (`libroomseg.so.0`,
`libopencv_*.so.3.4`, `libboost_*.so`) that the dynamic linker needs. The kit is the same
payload the upstart jobs expect; it is roughly 5.9 MiB.

The `dist/valetudo/` files are **inputs to a Valetudo rebuild**, not files to copy onto the
robot. The V1 runs one SEA binary; see §6.

## 4. Install on the robot

From the build host:

```bash
make install ROBOT=root@192.168.1.50
```

`make install` builds and stages the kit, preflights the target (`AppProxy` hash,
`ProcessList.conf` shape, `initctl`), copies the kit, verifies `SHA256SUMS` on the robot,
installs the upstart jobs, backs up `ProcessList.conf` once, enables the preload line, and asks
before rebooting. Use `REBOOT=1` to skip the reboot prompt.

Check the installation read-only:

```bash
make status ROBOT=root@192.168.1.50
```

It checks the `AppProxy` hash, kit hashes, init jobs, `ProcessList.conf` line, service state,
whether the shim is mapped into the running `AppProxy`, and whether `last_segmap` exists.

Remove V1TAL:

```bash
make uninstall ROBOT=root@192.168.1.50
```

It restores `ProcessList.conf` from the one-time backup, removes the init jobs and the kit, and
asks before rebooting. The map data (`last_map`, `last_segmap`) is left alone.

The installer only touches:

- `/mnt/data/rockrobo/rrtest`
- `/etc/init/v1appd.conf`
- `/etc/init/v1guard.conf`
- one line in `/opt/rockrobo/watchdog/ProcessList.conf` (backup: `.v1tal-orig`)

SSH keys are recommended. With password authentication you will be prompted for the copy and for
the remote run. `make install` and `make uninstall` accept `REBOOT=1`.

### Manual fallback

The jobs are hard-wired to `/mnt/data/rockrobo/rrtest`. Keep that path.

1. **Verify the pinned stock binary.**

   ```bash
   sha256sum /opt/rockrobo/cleaner/bin/AppProxy
   # must be df9b124c324842394d9f74ace99cb8eee387798922c991e755736e604376f4cc
   ```

   If it differs, stop: the patch sites are for V1 firmware 004028 only.

2. **Copy the kit and jobs.**

   ```bash
   ssh root@<robot> 'mkdir -p /mnt/data/rockrobo/rrtest'
   scp -r dist/kit/. root@<robot>:/mnt/data/rockrobo/rrtest/
   scp dist/init/v1appd.conf dist/init/v1guard.conf root@<robot>:/etc/init/
   ```

3. **Make WatchDoge start AppProxy preloaded.** Back up the file once, then change the
   `AppProxy` line:

   ```
   before: AppProxy,setsid AppProxy&,0,3,0
   after:  AppProxy,setsid env LD_PRELOAD=/mnt/data/rockrobo/rrtest/libv1seg_addon.so AppProxy&,0,3,0
   ```

   ```bash
   cp -a /opt/rockrobo/watchdog/ProcessList.conf \
         /opt/rockrobo/watchdog/ProcessList.conf.v1tal-orig   # once
   vi /opt/rockrobo/watchdog/ProcessList.conf
   ```

   WatchDoge caches `ProcessList.conf` at startup, so the reliable activation is a **reboot**.
   For the current boot only:

   ```bash
   initctl reload-configuration
   start v1appd v1guard
   initctl restart rrwatchdoge
   ```

   The two jobs start on `started rrwatchdoge`.

4. **Confirm.**

   ```bash
   initctl status v1appd v1guard
   tail -f /var/log/upstart/v1appd.log      # upstart may use this path
   ```

   With the shim active, Valetudo should see rooms after the next map fetch.

### Revert

```bash
initctl stop v1appd v1guard
rm -f /etc/init/v1appd.conf /etc/init/v1guard.conf
cp -a /opt/rockrobo/watchdog/ProcessList.conf.v1tal-orig \
      /opt/rockrobo/watchdog/ProcessList.conf
rm -rf /mnt/data/rockrobo/rrtest
# reboot, or restart the stock AppProxy
```

`last_map` / `last_segmap` under `/mnt/data/rockrobo` are left alone; delete them only if you
also want to drop the persisted map.

## 5. Run the producer by hand (no shim)

This is the low-risk first check, and useful when debugging segmentation:

```bash
LD_LIBRARY_PATH=/mnt/data/rockrobo/rrtest/lib \
ROOMSEG_MODEL=/mnt/data/rockrobo/rrtest/roomseg.mdl \
/mnt/data/rockrobo/rrtest/v1seg \
  --lastmap /mnt/data/rockrobo/last_map \
  /mnt/data/rockrobo/last_segmap \
  500 --door-radius auto
```

The flags matter: `500` (minimum area) and `--door-radius auto` are the tuned defaults for the
reference map. The producer's own defaults are more conservative and will usually report fewer
rooms. `LD_LIBRARY_PATH` and `ROOMSEG_MODEL` are only needed for the model path; the upstart job
sets both.

## 6. Rebuild Valetudo (rooms in the UI)

The V1 ships Valetudo as one static-pie ARM SEA binary at `/usr/local/bin/valetudo`. There is no
runtime plugin mechanism, so the model definition must be compiled in.

Pinned upstream: release `2026.08.0` at commit
`c249e72abda4b8aa93c776562716085d58deb6dd` (the build that matches the shipped V1 binary), or
the tested master commit `359a212e6897a31cea8df623b107fcb4a1373386`.

```bash
git clone https://github.com/Hypfer/Valetudo.git
cd Valetudo
git checkout c249e72abda4b8aa93c776562716085d58deb6dd

git apply /path/to/v1tal/valetudo/UPSTREAM_PATCH.diff
npm ci                                   # do NOT use --ignore-scripts
npm run build_openapi_schema
npm run build --workspace=frontend
npm run build_armv7 --workspace=backend
# -> build/armv7/valetudo
```

The normal `npm ci` is required: it applies the `path-to-regexp` ICU patch. Without it the
armv7 static Node binary dies while loading `express`.

Deploy with a backup:

```bash
scp build/armv7/valetudo root@<robot>:/usr/local/bin/valetudo.new
ssh root@<robot> 'cp -a /usr/local/bin/valetudo /usr/local/bin/valetudo.v1tal-orig && \
                  mv /usr/local/bin/valetudo.new /usr/local/bin/valetudo && \
                  restart valetudo'
```

Select the model in Valetudo's configuration:

```yaml
robot:
  implementation: "RoborockV1SegmentedValetudoRobot"
```

The model registers only standard capabilities:
`RoborockMapSegmentationCapability`, `RoborockMapSegmentEditCapability`,
`RoborockMapSegmentRenameCapability`, and `RoborockMapResetCapability`.

## 7. Common problems

| symptom | cause |
|---|---|
| `ARM C compiler not found` | toolchain not extracted under `artifacts/toolchain/`; set `TOOLCHAIN=` |
| `S5 payload not found` on `make kit` | run `tools/unpack_s5.sh` (see `artifacts/README.md`) |
| `dlopen(libroomseg.so.0) FAILED` when running `v1seg` | set `LD_LIBRARY_PATH` to the kit's `lib/` |
| wrong room count | check the `500 --door-radius auto` flags; segmentation is empirical |
| shim refuses to patch | `AppProxy` hash does not match 004028; do not force it |
| `make install` says `AppProxy hash is …` | wrong firmware; V1TAL only supports 004028 |
| `make install` says `unexpected AppProxy line` | custom `ProcessList.conf`; use the manual fallback |
| `run this as root on the robot` | the SSH target is not root |
| `make status` says the shim is not mapped | reboot has not happened, or the preload line is missing |
| rooms do not appear in Valetudo | the rebuilt Valetudo binary is not installed, or the model is not selected |
| install has no effect until reboot | WatchDoge caches `ProcessList.conf` at startup |
