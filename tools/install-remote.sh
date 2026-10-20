#!/usr/bin/env bash
# install-remote.sh -- V1TAL on-robot installer.
#
# Normally invoked by tools/install.sh with a bundle containing:
#   kit/                 the runtime payload from `make kit`
#   init/                v1appd.conf, v1guard.conf
#   install-remote.sh    this file
#
# It can also be run directly on a robot from such a bundle:
#   ./install-remote.sh --install
#   ./install-remote.sh --status
#   ./install-remote.sh --uninstall
#
# Options:
#   --install            install/update (default)
#   --status             read-only health check
#   --uninstall          remove V1TAL and restore the stock ProcessList line
#   --bundle DIR         bundle root (default: this script's directory)
#   --dry-run            print the plan, then exit before writing
#   -h, --help           show this help
#
# V1TAL_ROOT is a testing hook (empty on the robot): it prefixes /opt, /etc,
# and /mnt paths and suppresses initctl/process checks.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="${V1TAL_ROOT:-}"
MODE="install"
BUNDLE="$HERE"
DRY_RUN=0

APPPROXY_SHA="df9b124c324842394d9f74ace99cb8eee387798922c991e755736e604376f4cc"
STOCK_LINE='AppProxy,setsid AppProxy&,0,3,0'
PRELOAD_LINE='AppProxy,setsid env LD_PRELOAD=/mnt/data/rockrobo/rrtest/libv1seg_addon.so AppProxy&,0,3,0'

APP="$ROOT/opt/rockrobo/cleaner/bin/AppProxy"
PROC_CONF="$ROOT/opt/rockrobo/watchdog/ProcessList.conf"
PROC_ORIG="$PROC_CONF.v1tal-orig"
DATA="$ROOT/mnt/data/rockrobo"
KIT="$DATA/rrtest"
STAGE="$DATA/rrtest.new"
OLD="$DATA/rrtest.old"
INIT_DIR_OUT="$ROOT/etc/init"

die()  { echo "install: error: $*" >&2; exit 1; }
say()  { echo "install: $*"; }
ok()   { echo "  ok   $*"; }
warn() { echo "  warn $*"; }

usage() {
    sed -n '2,/^$/p' "$0" | sed 's/^# \{0,1\}//'
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --install)   MODE="install"; shift ;;
        --status)    MODE="status"; shift ;;
        --uninstall) MODE="uninstall"; shift ;;
        --bundle)    BUNDLE="${2:?--bundle needs a directory}"; shift 2 ;;
        --dry-run)   DRY_RUN=1; shift ;;
        -h|--help)   usage; exit 0 ;;
        *)           die "unknown argument: $1" ;;
    esac
done

KIT_SRC="$BUNDLE/kit"
INIT_SRC="$BUNDLE/init"

run_initctl() {
    [ -n "$ROOT" ] && return 0
    initctl "$@"
}

proc_state() {
    local ours stock
    [ -f "$PROC_CONF" ] || { echo missing; return; }
    ours=$(grep -c -F -- "$PRELOAD_LINE" "$PROC_CONF" 2>/dev/null || true)
    stock=$(grep -c -F -- "$STOCK_LINE" "$PROC_CONF" 2>/dev/null || true)
    if [ "${ours:-0}" -eq 1 ] && [ "${stock:-0}" -eq 0 ]; then
        echo ours
    elif [ "${stock:-0}" -eq 1 ] && [ "${ours:-0}" -eq 0 ]; then
        echo stock
    else
        echo unknown
    fi
}

check_appproxy() {
    [ -f "$APP" ] || die "AppProxy not found at $APP"
    local actual
    actual="$(sha256sum "$APP" | awk '{print $1}')"
    [ "$actual" = "$APPPROXY_SHA" ] || die "AppProxy hash is $actual, expected $APPPROXY_SHA (wrong firmware?)"
}

check_kit() {
    [ -d "$KIT_SRC" ] || die "kit not found: $KIT_SRC"
    for f in v1appd v1seg v1guard reanchor libv1seg_addon.so roomseg.mdl SHA256SUMS; do
        [ -f "$KIT_SRC/$f" ] || die "kit is incomplete: missing $f"
    done
    [ -d "$KIT_SRC/lib" ] || die "kit is incomplete: missing lib/"
    [ -d "$INIT_SRC" ] || die "init jobs not found: $INIT_SRC"
    for f in v1appd.conf v1guard.conf; do
        [ -f "$INIT_SRC/$f" ] || die "init jobs are incomplete: missing $f"
    done
}

replace_proc_line() {
    local from="$1" to="$2" tmp="$PROC_CONF.v1tal.tmp"
    cp -a "$PROC_CONF" "$tmp"
    awk -v from="$from" -v to="$to" '$0 == from { print to; next } { print }' \
        "$PROC_CONF" > "$tmp"
    mv "$tmp" "$PROC_CONF"
}

preflight_common() {
    if [ -z "$ROOT" ] && [ "$(id -u)" -ne 0 ]; then
        die "run this as root on the robot"
    fi
    if [ -z "$ROOT" ]; then
        command -v initctl >/dev/null 2>&1 || die "initctl not found (is this the V1?)"
    fi
    command -v sha256sum >/dev/null 2>&1 || die "sha256sum not found"
    [ -f "$PROC_CONF" ] || die "ProcessList.conf not found at $PROC_CONF"
}

preflight_install() {
    preflight_common
    check_appproxy
    check_kit
    local state
    state="$(proc_state)"
    [ "$state" != unknown ] || die "unexpected AppProxy line in $PROC_CONF; refusing to edit"
    say "AppProxy hash OK"
    say "ProcessList.conf: $state"
    say "kit: $KIT_SRC"
}

do_install() {
    preflight_install
    local state
    state="$(proc_state)"

    if [ "$DRY_RUN" = 1 ]; then
        say "dry-run: stop v1appd/v1guard"
        say "dry-run: stage $KIT with $KIT_SRC"
        say "dry-run: install $INIT_SRC/{v1appd,v1guard}.conf to $INIT_DIR_OUT"
        if [ "$state" = stock ]; then
            say "dry-run: back up $PROC_CONF and enable the preload line"
        else
            say "dry-run: ProcessList.conf already preloads V1TAL"
        fi
        say "dry-run: no changes written"
        return 0
    fi

    run_initctl stop v1appd v1guard >/dev/null 2>&1 || true
    mkdir -p "$DATA"
    rm -rf "$STAGE"
    mkdir -p "$STAGE"
    cp -a "$KIT_SRC/." "$STAGE/"
    if ! (cd "$STAGE" && sha256sum -c SHA256SUMS >/dev/null 2>&1); then
        rm -rf "$STAGE"
        die "kit hash verification failed"
    fi
    ok "kit hashes verified"

    rm -rf "$OLD"
    if [ -d "$KIT" ]; then
        mv "$KIT" "$OLD"
    fi
    if ! mv "$STAGE" "$KIT"; then
        [ -d "$OLD" ] && mv "$OLD" "$KIT"
        die "could not move the new kit into place"
    fi
    ok "installed $KIT"

    mkdir -p "$INIT_DIR_OUT"
    local conf src dst
    for conf in v1appd.conf v1guard.conf; do
        src="$INIT_SRC/$conf"
        dst="$INIT_DIR_OUT/$conf"
        if [ -f "$dst" ] && cmp -s "$src" "$dst"; then
            continue
        fi
        if [ -f "$dst" ] && [ ! -f "$dst.v1tal-orig" ]; then
            cp -a "$dst" "$dst.v1tal-orig"
        fi
        cp -a "$src" "$dst"
    done
    ok "installed init jobs"

    if [ "$state" = stock ]; then
        if [ ! -f "$PROC_ORIG" ]; then
            cp -a "$PROC_CONF" "$PROC_ORIG"
        fi
        replace_proc_line "$STOCK_LINE" "$PRELOAD_LINE"
        [ "$(proc_state)" = ours ] || die "ProcessList.conf edit did not verify"
        ok "enabled AppProxy preload"
    else
        ok "ProcessList.conf already enabled"
    fi

    run_initctl reload-configuration >/dev/null 2>&1 || true
    sync
    say "installation complete; a reboot is required"
}

preflight_uninstall() {
    preflight_common
    local state
    state="$(proc_state)"
    if [ ! -f "$PROC_ORIG" ] && [ "$state" = unknown ]; then
        die "no ProcessList.conf backup and the AppProxy line is unrecognised; refusing to edit"
    fi
    say "ProcessList.conf: $state"
}

do_uninstall() {
    preflight_uninstall

    if [ "$DRY_RUN" = 1 ]; then
        say "dry-run: stop v1appd/v1guard"
        say "dry-run: remove/restore init jobs"
        say "dry-run: restore the stock ProcessList line"
        say "dry-run: remove $KIT and $OLD"
        say "dry-run: no changes written"
        return 0
    fi

    run_initctl stop v1appd v1guard >/dev/null 2>&1 || true

    local conf dst
    for conf in v1appd.conf v1guard.conf; do
        dst="$INIT_DIR_OUT/$conf"
        if [ -f "$dst.v1tal-orig" ]; then
            mv -f "$dst.v1tal-orig" "$dst"
        else
            rm -f "$dst"
        fi
    done
    ok "removed init jobs"

    if [ -f "$PROC_ORIG" ]; then
        mv -f "$PROC_ORIG" "$PROC_CONF"
        ok "restored ProcessList.conf backup"
    else
        replace_proc_line "$PRELOAD_LINE" "$STOCK_LINE"
        ok "restored the stock AppProxy line"
    fi

    rm -rf "$KIT" "$STAGE" "$OLD"
    ok "removed runtime kit"

    run_initctl reload-configuration >/dev/null 2>&1 || true
    sync
    say "uninstall complete; a reboot is required"
}

do_status() {
    preflight_common
    local fail=0 state actual
    state="$(proc_state)"

    if [ -f "$APP" ]; then
        actual="$(sha256sum "$APP" | awk '{print $1}')"
        if [ "$actual" = "$APPPROXY_SHA" ]; then
            ok "AppProxy hash matches 004028"
        else
            warn "AppProxy hash is $actual (expected $APPPROXY_SHA)"; fail=1
        fi
    else
        warn "AppProxy not found at $APP"; fail=1
    fi

    if [ "$state" = ours ]; then
        ok "ProcessList.conf preloads V1TAL"
    else
        warn "ProcessList.conf state: $state"; fail=1
    fi

    if [ -d "$KIT" ]; then
        if (cd "$KIT" && sha256sum -c SHA256SUMS >/dev/null 2>&1); then
            ok "kit hashes match"
        else
            warn "kit hash mismatch"; fail=1
        fi
    else
        warn "kit not installed at $KIT"; fail=1
    fi

    local conf
    for conf in v1appd.conf v1guard.conf; do
        if [ -f "$INIT_DIR_OUT/$conf" ]; then
            ok "init job installed: $conf"
        else
            warn "init job missing: $conf"; fail=1
        fi
    done

    if [ -z "$ROOT" ]; then
        local unit out
        for unit in v1appd v1guard; do
            out="$(initctl status "$unit" 2>&1 || true)"
            if echo "$out" | grep -q 'start/running'; then
                ok "$unit is running"
            else
                warn "$unit is not running ($out)"; fail=1
            fi
        done

        local pid
        pid="$(pidof AppProxy 2>/dev/null | awk '{print $1}' || true)"
        if [ -z "$pid" ] && command -v pgrep >/dev/null 2>&1; then
            pid="$(pgrep -x AppProxy 2>/dev/null | head -n 1 || true)"
        fi
        if [ -n "$pid" ] && grep -q 'libv1seg_addon' "/proc/$pid/maps" 2>/dev/null; then
            ok "shim is mapped into AppProxy (pid $pid)"
        else
            warn "shim is not mapped into a running AppProxy"; fail=1
        fi
    fi

    if [ -f "$DATA/last_segmap" ]; then
        ok "last_segmap present ($(wc -c < "$DATA/last_segmap") bytes)"
    else
        warn "last_segmap not present yet (producer has not run)"
    fi

    [ "$fail" = 0 ] || return 1
    say "status: healthy"
}

case "$MODE" in
    install)   do_install ;;
    uninstall) do_uninstall ;;
    status)    do_status ;;
esac
