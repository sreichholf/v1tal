# artifacts — external downloads

Nothing in this directory is committed except this README and the `.gitkeep` placeholders.
Put the downloaded/extracted inputs here; `.gitignore` ignores the payload directories.

```
artifacts/
  toolchain/armv7-eabihf--glibc--stable-2018.02-2/   Bootlin GCC 6.4 cross toolchain
  s5/rootfs/                                          the seven S5 files V1TAL needs
  firmware/                                           downloaded firmware tarballs (optional)
```

| artifact | needed for | size (approx.) |
|---|---|---|
| toolchain | `make` (ARM build) | 290 MB |
| S5 payload | `make kit`, room segmentation | ~6 MB |

## 1. Bootlin toolchain (required for `make`)

Download `armv7-eabihf--glibc--stable-2018.02-2`:

- <https://toolchains.bootlin.com/downloads/releases/toolchains/armv7-eabihf/tarballs/armv7-eabihf--glibc--stable-2018.02-2.tar.bz2>

Verify and extract:

```bash
sha256sum armv7-eabihf--glibc--stable-2018.02-2.tar.bz2
# 32723810be37418bb9e5a7da5bec96719bf0c2eb2b919f85dfd5a02c449a451f

mkdir -p artifacts/toolchain
tar -xjf armv7-eabihf--glibc--stable-2018.02-2.tar.bz2 -C artifacts/toolchain/
# -> artifacts/toolchain/armv7-eabihf--glibc--stable-2018.02-2/bin/arm-linux-gcc
```

The archive is location-relocatable as shipped. If you use a different toolchain, point
`TOOLCHAIN=` at its directory; the Makefile expects `bin/arm-linux-{gcc,g++}` and
`arm-buildroot-linux-gnueabihf/sysroot`.

## 2. S5 payload (required for `make kit`)

V1TAL borrows the S5's segmentation model and its libraries. You need the **S5 (roborock.vacuum.s5)
V002034 (2020-11-27)** firmware. Obtain the update tarball from the Roborock/Xiaomi OTA channel
or a firmware mirror for exactly that version.

Do not unpack the full rootfs. Run the unpacker against the firmware tarball:

```bash
tools/unpack_s5.sh /path/to/roborock.vacuum.s5_2034_fw.tar.gz
```

It finds `disk.img`, verifies its MD5, extracts only the seven files V1TAL needs into
`artifacts/s5/rootfs/`, and verifies each file's SHA-256. If you already have `disk.img`, pass
that instead. `--keep-image` retains the extracted `disk.img` under `artifacts/firmware/`;
`--dest DIR` changes the output root.

The expected payload and hashes (the script checks these):

```
2f5e61a9abfd3af049605ca1ad6b7b72d7ef1c68d2a1f9192c030848792bebe8  opt/rockrobo/cleaner/lib/libroomseg.so.0.8.60
9ba7b603faf324a301d0012959c6a3c110aeb3f69fbff62054e5d17db72c9cfd  opt/rockrobo/cleaner/lib/libopencv_core.so.3.4.2
7c7abcf1084477bc37f8bee2817848391b9fe8a3bd6dc62dffd013b16a9f7245  opt/rockrobo/cleaner/lib/libopencv_imgproc.so.3.4.2
185fec4a99fc00dee3bbfc5ed296e87540c788fbd9a5c6784ab223b4a31de4e0  opt/rockrobo/cleaner/lib/libopencv_dnn.so.3.4.2
d2901e3979a2c587f0b9505cbd226b50fb45e87bb3a8de9975225eeda06baa57  usr/lib/arm-linux-gnueabihf/libboost_system.so.1.55.0
e54a24d14cb287da7db25a82adc4a257e050d858a1664a333158bc5abc1e5ac3  usr/lib/arm-linux-gnueabihf/libboost_filesystem.so.1.55.0
e90934c4660b88bad2e88f2d97165deb77e124e4572884a642bed0b48d5bf725  opt/rockrobo/cleaner/conf/roomseg.mdl.4
```

`make kit` copies these into `dist/kit/` (the model is renamed `roomseg.mdl`) and creates the
SONAME symlinks the dynamic linker expects.

## 3. V1 firmware (not needed locally)

The compiled shim is pinned to V1 firmware **004028 (2021-01-09)**, but you do not need a local
copy of the rootfs. Verify the stock binary on the robot before installing:

```bash
sha256sum /opt/rockrobo/cleaner/bin/AppProxy
# df9b124c324842394d9f74ace99cb8eee387798922c991e755736e604376f4cc
```

If the hash differs, do not run the shim: the byte-level patch sites belong to this exact build.

## 4. Firmware tarballs

Keep the downloaded `.tar.gz` files in `artifacts/firmware/` if you want a self-contained
workspace. They are ignored by git. Only the extracted S5 payload is used by the build.
