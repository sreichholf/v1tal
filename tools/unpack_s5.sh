#!/usr/bin/env bash
# unpack_s5.sh -- extract only the S5 files V1TAL needs from an S5 firmware image.
#
# The S5 rootfs is ~136 MB, but V1TAL uses seven files from it. This script takes
# either the S5 firmware tarball or an already-extracted disk.img, verifies the
# image, pulls only those files with debugfs, and verifies each SHA-256.
#
# Usage:
#   tools/unpack_s5.sh roborock.vacuum.s5_2034_fw.tar.gz
#   tools/unpack_s5.sh /path/to/disk.img
#   tools/unpack_s5.sh disk.img --dest /tmp/s5-rootfs --force
#
# Options:
#   --dest DIR       output root (default: <repo>/artifacts/s5/rootfs)
#   --keep-image     keep the extracted disk.img under artifacts/firmware/
#   --force          overwrite files that already exist with a different hash
#   -h, --help       show this help
#
# Requires: bash, tar, debugfs (e2fsprogs), md5sum, sha256sum.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/.." && pwd)"
DEST="$ROOT/artifacts/s5/rootfs"
KEEP_IMAGE=0
FORCE=0

die()  { echo "unpack_s5: error: $*" >&2; exit 1; }
note() { echo "unpack_s5: $*"; }

usage() {
    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

INPUT=""
while [ "$#" -gt 0 ]; do
    case "$1" in
        --dest)       DEST="${2:?--dest needs a directory}"; shift 2 ;;
        --keep-image) KEEP_IMAGE=1; shift ;;
        --force)      FORCE=1; shift ;;
        -h|--help)    usage; exit 0 ;;
        -*)           die "unknown option: $1" ;;
        *)            [ -z "$INPUT" ] || die "unexpected argument: $1"; INPUT="$1"; shift ;;
    esac
done

[ -n "$INPUT" ] || { usage; exit 2; }
[ -f "$INPUT" ] || die "no such file: $INPUT"

for tool in debugfs md5sum sha256sum; do
    command -v "$tool" >/dev/null 2>&1 || die "missing '$tool' (install e2fsprogs/coreutils)"
done

# The exact seven files V1TAL consumes, and their expected hashes. Keep in sync
# with the make kit recipe and artifacts/README.md.
FILES=(
    "opt/rockrobo/cleaner/lib/libroomseg.so.0.8.60|2f5e61a9abfd3af049605ca1ad6b7b72d7ef1c68d2a1f9192c030848792bebe8"
    "opt/rockrobo/cleaner/lib/libopencv_core.so.3.4.2|9ba7b603faf324a301d0012959c6a3c110aeb3f69fbff62054e5d17db72c9cfd"
    "opt/rockrobo/cleaner/lib/libopencv_imgproc.so.3.4.2|7c7abcf1084477bc37f8bee2817848391b9fe8a3bd6dc62dffd013b16a9f7245"
    "opt/rockrobo/cleaner/lib/libopencv_dnn.so.3.4.2|185fec4a99fc00dee3bbfc5ed296e87540c788fbd9a5c6784ab223b4a31de4e0"
    "usr/lib/arm-linux-gnueabihf/libboost_system.so.1.55.0|d2901e3979a2c587f0b9505cbd226b50fb45e87bb3a8de9975225eeda06baa57"
    "usr/lib/arm-linux-gnueabihf/libboost_filesystem.so.1.55.0|e54a24d14cb287da7db25a82adc4a257e050d858a1664a333158bc5abc1e5ac3"
    "opt/rockrobo/cleaner/conf/roomseg.mdl.4|e90934c4660b88bad2e88f2d97165deb77e124e4572884a642bed0b48d5bf725"
)

TMP_DIR="$(mktemp -d "${TMPDIR:-/tmp}/v1tal-s5.XXXXXX")"
cleanup() { rm -rf "$TMP_DIR"; }
trap cleanup EXIT

case "$INPUT" in
    *.img) IMG="$INPUT" ;;
    *.tar.gz|*.tgz|*.tar.bz2|*.tbz2|*.tar.xz|*.txz|*.tar)
        command -v tar >/dev/null 2>&1 || die "missing 'tar'"
        member="$(tar -tf "$INPUT" | grep -E '(^|/)disk\.img$' | head -n 1 || true)"
        [ -n "$member" ] || die "no disk.img found in $INPUT"
        note "extracting $member from $(basename "$INPUT")"
        case "$INPUT" in
            *.tar.gz|*.tgz)     tar -xzf "$INPUT" -C "$TMP_DIR" "$member" ;;
            *.tar.bz2|*.tbz2)   tar -xjf "$INPUT" -C "$TMP_DIR" "$member" ;;
            *.tar.xz|*.txz)     tar -xJf "$INPUT" -C "$TMP_DIR" "$member" ;;
            *.tar)              tar -xf  "$INPUT" -C "$TMP_DIR" "$member" ;;
        esac
        IMG="$TMP_DIR/$member"
        ;;
    *) die "expected a firmware .tar.* or a disk.img" ;;
esac

[ -f "$IMG" ] || die "image not found after extraction: $IMG"

actual_md5="$(md5sum "$IMG" | awk '{print $1}')"
expected_md5="b5a6ab5760e8692813dec103632d2a05"
if [ "$actual_md5" != "$expected_md5" ]; then
    die "disk.img md5 is $actual_md5, expected $expected_md5 (wrong S5 firmware?)"
fi
note "disk.img md5 OK ($expected_md5)"

if [ "$KEEP_IMAGE" = 1 ]; then
    keep="$ROOT/artifacts/firmware/s5-disk.img"
    mkdir -p "$(dirname "$keep")"
    cp -- "$IMG" "$keep"
    note "kept image at $keep"
fi

extracted=0
skipped=0
for entry in "${FILES[@]}"; do
    rel="${entry%%|*}"
    expected_sha="${entry##*|}"
    dst="$DEST/$rel"

    if [ -f "$dst" ]; then
        actual_sha="$(sha256sum "$dst" | awk '{print $1}')"
        if [ "$actual_sha" = "$expected_sha" ]; then
            note "skip  $rel (already present)"
            skipped=$((skipped + 1))
            continue
        fi
        [ "$FORCE" = 1 ] || die "$rel exists with a different hash (use --force)"
        rm -f -- "$dst"
    fi

    mkdir -p "$(dirname "$dst")"
    debugfs -R "dump /$rel $dst" "$IMG" >/dev/null 2>&1 || true

    [ -f "$dst" ] || die "debugfs could not extract /$rel"
    actual_sha="$(sha256sum "$dst" | awk '{print $1}')"
    if [ "$actual_sha" != "$expected_sha" ]; then
        rm -f -- "$dst"
        die "$rel hash is $actual_sha, expected $expected_sha"
    fi
    note "ok     $rel"
    extracted=$((extracted + 1))
done

note "done: $extracted extracted, $skipped already present"
note "output: $DEST"
note "next:   make kit"
