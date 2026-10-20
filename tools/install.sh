#!/usr/bin/env bash
# install.sh -- V1TAL installer from the build host.
#
# Copies the built kit to a rooted V1, installs the upstart jobs, and changes
# exactly one WatchDoge ProcessList.conf line (with a one-time backup).
#
# Usage:
#   tools/install.sh root@192.168.1.50
#   tools/install.sh --status root@192.168.1.50
#   tools/install.sh --uninstall root@192.168.1.50
#
# Options:
#   --install          install/update (default)
#   --status           read-only health check
#   --uninstall        remove V1TAL and restore the stock AppProxy line
#   --reboot           reboot without asking
#   --dry-run          print the remote plan; do not write
#   --kit DIR          built kit directory (default: dist/kit)
#   --init DIR         built init-job directory (default: dist/init)
#   -h, --help         show this help
#
# SSH keys are recommended. With password authentication you will be prompted
# for the copy and for the remote run.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
REMOTE_SCRIPT="$HERE/install-remote.sh"
MODE="install"
TARGET=""
REBOOT=0
DRY_RUN=0
KIT_DIR="$REPO/dist/kit"
INIT_DIR="$REPO/dist/init"

die() { echo "install: error: $*" >&2; exit 1; }
say() { echo "install: $*"; }

usage() {
    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --install)   MODE="install"; shift ;;
        --status)    MODE="status"; shift ;;
        --uninstall) MODE="uninstall"; shift ;;
        --reboot)    REBOOT=1; shift ;;
        --dry-run)   DRY_RUN=1; shift ;;
        --kit)       KIT_DIR="${2:?--kit needs a directory}"; shift 2 ;;
        --init)      INIT_DIR="${2:?--init needs a directory}"; shift 2 ;;
        -h|--help)   usage; exit 0 ;;
        -*)          die "unknown option: $1" ;;
        *)           [ -z "$TARGET" ] || die "unexpected argument: $1"; TARGET="$1"; shift ;;
    esac
done

[ -n "$TARGET" ] || { usage; exit 2; }
case "$TARGET" in
    *@*) ;;
    *)   TARGET="root@$TARGET" ;;
esac

command -v ssh >/dev/null 2>&1 || die "ssh not found"
command -v scp >/dev/null 2>&1 || die "scp not found"
[ -f "$REMOTE_SCRIPT" ] || die "missing remote installer: $REMOTE_SCRIPT"

TMP="$(mktemp -d "${TMPDIR:-/tmp}/v1tal-install.XXXXXX")"
trap 'rm -rf "$TMP"' EXIT

if [ "$MODE" = install ]; then
    command -v tar >/dev/null 2>&1 || die "tar not found"
    [ -d "$KIT_DIR" ]  || die "kit not found: $KIT_DIR (run 'make kit')"
    [ -d "$INIT_DIR" ] || die "init jobs not found: $INIT_DIR (run 'make kit')"

    mkdir -p "$TMP/bundle"
    cp -a "$KIT_DIR" "$TMP/bundle/kit"
    cp -a "$INIT_DIR" "$TMP/bundle/init"
    cp -a "$REMOTE_SCRIPT" "$TMP/bundle/install-remote.sh"
    tar -C "$TMP/bundle" -czf "$TMP/bundle.tar.gz" kit init install-remote.sh
    [ -s "$TMP/bundle.tar.gz" ] || die "could not create the install bundle"

    remote_tar="/tmp/v1tal-install.$$.tar.gz"
    remote_dir="/tmp/v1tal-install.$$"
    flags=""
    [ "$DRY_RUN" = 1 ] && flags="--dry-run"

    say "copying kit to $TARGET"
    scp "$TMP/bundle.tar.gz" "$TARGET:$remote_tar" || die "copy failed"

    cmd="d='$remote_dir'; t='$remote_tar'; rm -rf \"\$d\"; mkdir -p \"\$d\" || exit 1; \
tar -xzf \"\$t\" -C \"\$d\" || { rc=\$?; rm -rf \"\$d\" \"\$t\"; exit \$rc; }; \
\"\$d/install-remote.sh\" --install $flags; rc=\$?; rm -rf \"\$d\" \"\$t\"; exit \$rc"
    ssh "$TARGET" "$cmd" || die "remote install failed"
else
    remote_script="/tmp/v1tal-install.$$.sh"
    scp "$REMOTE_SCRIPT" "$TARGET:$remote_script" || die "copy failed"

    flags=""
    [ "$DRY_RUN" = 1 ] && flags="--dry-run"
    cmd="s='$remote_script'; \"\$s\" --$MODE $flags; rc=\$?; rm -f \"\$s\"; exit \$rc"
    ssh "$TARGET" "$cmd" || die "remote $MODE failed"
fi

if [ "$MODE" != status ] && [ "$DRY_RUN" != 1 ]; then
    if [ "$REBOOT" = 1 ]; then
        say "rebooting $TARGET"
        ssh "$TARGET" 'reboot' >/dev/null 2>&1 || true
    else
        read -r -p "Reboot now to activate? [y/N] " answer || answer=""
        case "$answer" in
            y|Y|yes|YES)
                say "rebooting $TARGET"
                ssh "$TARGET" 'reboot' >/dev/null 2>&1 || true
                ;;
            *)
                echo "Reboot later with: ssh $TARGET reboot"
                ;;
        esac
    fi
fi
