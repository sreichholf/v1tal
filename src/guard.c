/* guard.c -- keep-alive guard for the V1 map (standalone process).
 *
 * See guard.h.  This program is meant to run alongside AppProxy/player
 * *without patching them*: it watches the V1 data dir (/mnt/data/rockrobo by
 * default) and keeps the protected snapshot taken by src/persist.c's
 * `persist_map_save()` alive across the nav's own clear triggers
 * (`slam_info.cfg` timestamp -> 0, `last_map` removed).  Persistence is
 * unconditional: there is no user toggle.  It also detects a
 * *changed* `last_map` frame (a rotation / full revisit) and applies a
 * configurable rotation/re-anchor policy (default: refuse / hold).
 *
 * The snapshot/recover/reset internals are the shared `persist_*` functions
 * of src/persist.c, compiled in unmodified.
 *
 * Logging is raw write() only: the V1's glibc 2.19 segfaults on
 * setvbuf()/fflush(), so this file never touches stdio buffering (it uses
 * snprintf/vsnprintf into stack buffers, then write()).
 *
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "guard.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

/* ===================================================================================== *
 *  logging (raw write only)                                                             *
 * ===================================================================================== */
void v1g_log(int fd, const char *fmt, ...) {
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    int needed = vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (needed < 0) return;
    if ((size_t)needed >= sizeof buf) needed = (int)sizeof buf - 1;
    ssize_t written = write(fd, buf, (size_t)needed);
    (void)written;
}

/* ===================================================================================== *
 *  small helpers                                                                        *
 * ===================================================================================== */
static const char *join_path(char *dst, size_t cap, const char *dir, const char *name) {
    int written = snprintf(dst, cap, "%s/%s", dir, name);
    if (written < 0 || (size_t)written >= cap) dst[0] = '\0';
    return dst;
}

static int file_exists(const char *path) {
    struct stat st;
    return (stat(path, &st) == 0 && S_ISREG(st.st_mode));
}

static int float_close(float value_a, float value_b) {
    float diff = value_a - value_b;
    if (diff < 0) diff = -diff;
    return diff < 0.01f;
}

static volatile sig_atomic_t g_v1g_stop = 0;

static void sleep_ms(int ms) {
    struct timespec remaining;
    remaining.tv_sec = ms / 1000;
    remaining.tv_nsec = (long)(ms % 1000) * 1000000L;
    while (nanosleep(&remaining, &remaining) != 0 && errno == EINTR) {
        /* re-sleep the remainder unless we are asked to stop */
        if (g_v1g_stop) return;
    }
}

/* Monotonic milliseconds; used only for throttle/debounce bookkeeping. */
static long now_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return (long)ts.tv_sec * 1000L + (long)(ts.tv_nsec / 1000000L);
}

static uint32_t rd_u32(const unsigned char *bytes) {
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}
static int32_t rd_i32(const unsigned char *bytes) { return (int32_t)rd_u32(bytes); }
static float   rd_f32(const unsigned char *bytes) {
    uint32_t bits = rd_u32(bytes); float value; memcpy(&value, &bits, 4); return value;
}

/* ===================================================================================== *
 *  map frame reader                                                                     *
 * ===================================================================================== */
int v1g_read_map(const char *path, v1g_mapframe_t *out) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    struct stat st;
    if (fstat(fd, &st) != 0 || !S_ISREG(st.st_mode) || st.st_size < 56) { close(fd); return -1; }
    size_t file_size = (size_t)st.st_size;
    unsigned char *data = (unsigned char *)malloc(file_size);
    if (!data) { close(fd); return -1; }
    size_t read_so_far = 0;
    while (read_so_far < file_size) {
        ssize_t nread = read(fd, data + read_so_far, file_size - read_so_far);
        if (nread <= 0) { free(data); close(fd); return -1; }
        read_so_far += (size_t)nread;
    }
    close(fd);
    uint32_t w = rd_u32(data + 32), h = rd_u32(data + 36);
    /* a complete V1 map is exactly 56 + w*h*4 bytes; anything else is treated as
     * "not a usable map" (so a torn/short nav write is never snapshotted).
     * Compute in 64 bits: on 32-bit ARM `w*h*4` can wrap size_t. */
    uint64_t expected_size = 56ull + (uint64_t)w * (uint64_t)h * 4ull;
    if (w == 0 || h == 0 || expected_size != (uint64_t)file_size) { free(data); return -1; }
    memset(out, 0, sizeof *out);
    out->present = 1;
    out->scale = rd_f32(data + 20);
    out->ox = rd_i32(data + 24);
    out->oy = rd_i32(data + 28);
    out->w = w;
    out->h = h;
    out->crc = persist_crc32(data, file_size);
    free(data);
    return 0;
}

int v1g_read_map_stable(const char *path, v1g_mapframe_t *out, int settle_ms) {
    if (v1g_read_map(path, out) != 0) return -1;
    if (settle_ms <= 0) return 0;
    uint32_t first_crc = out->crc;
    sleep_ms(settle_ms);
    v1g_mapframe_t second;
    if (v1g_read_map(path, &second) != 0) return -1;
    if (second.crc != first_crc) return -2;   /* still changing */
    *out = second;
    return 0;
}

/* ===================================================================================== *
 *  policy / hook state                                                                  *
 * ===================================================================================== */
static v1g_rotate_policy_t g_policy = V1G_ROT_HOLD;
static v1g_rotate_hook_fn  g_hook = NULL;
static void               *g_hook_ctx = NULL;
static int                 g_refresh = 1;
static int                 g_settle_ms = 50;
static int                 g_min_refresh_ms = 2000;   /* throttle snapshot refreshes */
static int                 g_min_tick_ms = 250;       /* debounce watcher ticks */
static long                g_last_refresh_ms = -1;

void v1g_set_policy(v1g_rotate_policy_t policy) { g_policy = policy; }
void v1g_set_rotate_hook(v1g_rotate_hook_fn hook, void *context) { g_hook = hook; g_hook_ctx = context; }
void v1g_set_refresh(int on) { g_refresh = on ? 1 : 0; }
void v1g_set_settle_ms(int ms) { g_settle_ms = ms < 0 ? 0 : ms; }
void v1g_set_min_refresh_ms(int ms) { g_min_refresh_ms = ms < 0 ? 0 : ms; }
void v1g_set_min_tick_ms(int ms) { g_min_tick_ms = ms < 0 ? 0 : ms; }
const char *v1g_root(void) { return persist_root(); }

const char *v1g_state_name(v1g_state_t state) {
    switch (state) {
    case V1G_CLEAR:    return "clear-restored";
    case V1G_SEED:     return "seeded";
    case V1G_HEALTHY:  return "healthy";
    case V1G_GROWN:    return "grown-refreshed";
    case V1G_ROTATED:  return "rotated";
    case V1G_NOMAP:    return "no-map";
    case V1G_UNSTABLE: return "unstable";
    case V1G_ERROR:    return "error";
    default:           return "?";
    }
}

const char *v1g_policy_name(v1g_rotate_policy_t policy) {
    switch (policy) {
    case V1G_ROT_HOLD:    return "hold";
    case V1G_ROT_RESAVE:  return "resave";
    case V1G_ROT_RESEG:   return "reseg";
    case V1G_ROT_RESTORE: return "restore";
    default:              return "?";
    }
}

v1g_rotate_policy_t v1g_policy_from_name(const char *name) {
    if (!name) return V1G_ROT_HOLD;
    if (!strcmp(name, "hold"))    return V1G_ROT_HOLD;
    if (!strcmp(name, "resave"))  return V1G_ROT_RESAVE;
    if (!strcmp(name, "reseg"))   return V1G_ROT_RESEG;
    if (!strcmp(name, "restore")) return V1G_ROT_RESTORE;
    return V1G_ROT_HOLD;
}

/* ===================================================================================== *
 *  tmp cleanup + single-writer lock                                                     *
 * ===================================================================================== */
static int g_tmp_stale_sec = 300;
void v1g_set_tmp_stale_sec(int sec) { g_tmp_stale_sec = sec < 0 ? 0 : sec; }

/* A .tmp/.new file is only removed when it is old enough that no live writer can still
 * own it.  last_map.tmp / slam_info.cfg.tmp are written by the nav, which does not take
 * the persist lock, so blindly unlinking them could delete a rename-in-progress. */
static int tmp_is_stale(const char *path, time_t now_sec) {
    struct stat st;
    if (stat(path, &st) != 0) return 0;
    if (!S_ISREG(st.st_mode)) return 0;          /* never follow/remove a directory */
    if (g_tmp_stale_sec <= 0) return 1;          /* 0 disables the age guard */
    return (now_sec - st.st_mtime) >= (time_t)g_tmp_stale_sec;
}

int v1g_cleanup_tmp(void) {
    const char *root = persist_root();
    static const char *root_tmp_names[] = { "last_map.tmp", "slam_info.cfg.tmp", "last_segmap.tmp", NULL };
    static const char *persist_tmp_names[] = { "persist/map0.tmp", "persist/seg0.tmp",
                                               "persist/slam_info0.cfg.tmp", "persist/frame0.tmp",
                                               "persist/map0.new", "persist/seg0.new",
                                               "persist/slam_info0.cfg.new", "persist/frame0.new", NULL };
    char path[PATH_MAX];
    int removed_count = 0;
    time_t now_sec = time(NULL);
    /* Take the shared write lock so a live snapshot writer's staged file is not deleted
     * mid-write (which would make its rename()/write fail).  This is the short-lived
     * .v1persist.lock, not the guard's lifetime .v1guard.lock.  The nav writers do
     * not take it, so those files are additionally protected by the age check above. */
    int lock_fd = persist_write_lock();
    for (int i = 0; root_tmp_names[i]; i++) {
        join_path(path, sizeof path, root, root_tmp_names[i]);
        if (tmp_is_stale(path, now_sec) && unlink(path) == 0) removed_count++;
    }
    for (int i = 0; persist_tmp_names[i]; i++) {
        join_path(path, sizeof path, root, persist_tmp_names[i]);
        if (tmp_is_stale(path, now_sec) && unlink(path) == 0) removed_count++;
    }
    persist_write_unlock(lock_fd);
    return removed_count;
}

static int g_lockfd = -1;

int v1g_lock(void) {
    char path[PATH_MAX];
    join_path(path, sizeof path, persist_root(), ".v1guard.lock");
    int fd = open(path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) return -2;
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        int err = errno;
        close(fd);
        return (err == EWOULDBLOCK || err == EAGAIN) ? -1 : -2;
    }
    g_lockfd = fd;
    return 0;
}

void v1g_unlock(void) {
    if (g_lockfd >= 0) { flock(g_lockfd, LOCK_UN); close(g_lockfd); g_lockfd = -1; }
}

/* ===================================================================================== *
 *  rotation policy                                                                      *
 * ===================================================================================== */
static int apply_rotate(const v1g_mapframe_t *live, const v1g_mapframe_t *snap,
                        char *detail, size_t detail_cap) {
    if (g_hook) {
        int rc = g_hook(g_hook_ctx, live, snap);
        snprintf(detail, detail_cap, "custom hook rc=%d", rc);
        return rc;
    }
    switch (g_policy) {
    case V1G_ROT_RESAVE: {
        int rc = persist_map_save();
        snprintf(detail, detail_cap, "resave (adopt new frame) rc=%d", rc);
        return rc;
    }
    case V1G_ROT_RESEG: {
        char seg_path[PATH_MAX];
        join_path(seg_path, sizeof seg_path, persist_root(), "last_segmap");
        unlink(seg_path);
        int rc = persist_map_save();
        snprintf(detail, detail_cap, "reseg (drop stale last_segmap + resave) rc=%d", rc);
        return rc;
    }
    case V1G_ROT_RESTORE: {
        /* verify-then-restore under a single lock (persist_map_force_restore): if the
         * snapshot is inconsistent the live map is left untouched, rather than being
         * deleted by a reset that a following recover then refuses to undo. */
        int rc = persist_map_force_restore();
        snprintf(detail, detail_cap, "forced restore rc=%d", rc);
        return (rc == 0) ? 0 : -1;
    }
    case V1G_ROT_HOLD:
    default:
        snprintf(detail, detail_cap, "hold: live map left untouched (safe default)");
        return 0;
    }
}

/* ===================================================================================== *
 *  one guard pass                                                                       *
 * ===================================================================================== */
static v1g_state_t finish(v1g_state_t state, const char *text, char *msg, size_t cap) {
    if (msg && cap) snprintf(msg, cap, "%s", text);
    return state;
}

/* 1 when the last snapshot refresh was too recent to refresh again. */
static int refresh_throttled(void) {
    if (g_min_refresh_ms <= 0) return 0;
    long now = now_ms();
    if (g_last_refresh_ms >= 0 && (now - g_last_refresh_ms) < g_min_refresh_ms) return 1;
    return 0;
}

v1g_state_t v1g_run_once(char *msg, size_t cap) {
    char summary[320];
    const char *root = persist_root();
    char lastmap_path[PATH_MAX], snapmap_path[PATH_MAX];
    join_path(lastmap_path, sizeof lastmap_path, root, "last_map");
    join_path(snapmap_path, sizeof snapmap_path, root, "persist/map0");

    int live_exists = file_exists(lastmap_path);
    v1g_mapframe_t live; memset(&live, 0, sizeof live);
    int live_rc = live_exists ? v1g_read_map(lastmap_path, &live) : -1;

    v1g_mapframe_t snap; memset(&snap, 0, sizeof snap);
    int snap_rc = v1g_read_map(snapmap_path, &snap);
    (void)snap_rc;

    /* The snapshot is only "present" when its map half AND its frame0 fingerprint agree.
     * Reading persist/map0 alone reports a healthy snapshot even
     * when frame0 disagrees, which is exactly what makes recover return -3 forever. */
    persist_frame_rec_t snap_rec; memset(&snap_rec, 0, sizeof snap_rec);
    int snap_have_seg = 0;
    int snap_verify = persist_snapshot_verify(&snap_rec, &snap_have_seg);
    int snapshot_consistent = (snap_verify == 0);
    int snapshot_any = (snap_verify == 0) || snap.present;

    uint32_t timestamp = 0;
    int timestamp_ok = (persist_read_slam_timestamp(&timestamp) == 0);
    int timestamp_zero = (timestamp_ok && timestamp == 0);

    /* --- CLEAR: the nav dropped last_map or reset the freshness timestamp --- */
    if (!live_exists || timestamp_zero) {
        if (!snapshot_any) {
            snprintf(summary, sizeof summary, "clear signal (%s, ts=%u) but no snapshot to restore",
                     live_exists ? "ts==0" : "last_map missing", timestamp);
            return finish(V1G_NOMAP, summary, msg, cap);
        }
        int rc = persist_map_recover(0);
        if (rc == 0) {
            snprintf(summary, sizeof summary, "restored snapshot after clear (%s, ts=%u)",
                     live_exists ? "slam ts==0" : "last_map missing", timestamp);
            return finish(V1G_CLEAR, summary, msg, cap);
        }
        snprintf(summary, sizeof summary, "recover after clear failed rc=%d", rc);
        return finish(V1G_ERROR, summary, msg, cap);
    }

    /* live file exists: did it read as a complete map? */
    if (live_rc != 0) {
        snprintf(summary, sizeof summary, "last_map present but incomplete/unreadable (torn write?)");
        return finish(V1G_UNSTABLE, summary, msg, cap);
    }

    /* --- no snapshot yet: seed one from the good live map --- */
    if (!snapshot_any) {
        v1g_mapframe_t stable_frame;
        int stable_rc = v1g_read_map_stable(lastmap_path, &stable_frame, g_settle_ms);
        if (stable_rc == -2) { snprintf(summary, sizeof summary, "live map not quiescent; not seeding"); return finish(V1G_UNSTABLE, summary, msg, cap); }
        if (stable_rc != 0)  { snprintf(summary, sizeof summary, "live map re-read failed"); return finish(V1G_UNSTABLE, summary, msg, cap); }
        int rc = persist_map_save();
        if (rc == 0) {
            g_last_refresh_ms = now_ms();
            snprintf(summary, sizeof summary, "seeded snapshot from live map %ux%u origin=(%d,%d)",
                     stable_frame.w, stable_frame.h, stable_frame.ox, stable_frame.oy);
            return finish(V1G_SEED, summary, msg, cap);
        }
        snprintf(summary, sizeof summary, "seed failed rc=%d", rc);
        return finish(V1G_ERROR, summary, msg, cap);
    }

    /* --- snapshot present but frame0 disagrees with map0: rebuild it from the live map.
     * This is the self-heal that stops a torn snapshot from pinning recover at -3. --- */
    if (!snapshot_consistent) {
        v1g_mapframe_t stable_frame;
        int stable_rc = v1g_read_map_stable(lastmap_path, &stable_frame, g_settle_ms);
        if (stable_rc != 0) {
            snprintf(summary, sizeof summary, "snapshot inconsistent (verify=%d) and live map not quiescent",
                     snap_verify);
            return finish(V1G_UNSTABLE, summary, msg, cap);
        }
        int rc = persist_map_save();
        if (rc == 0) {
            g_last_refresh_ms = now_ms();
            snprintf(summary, sizeof summary, "snapshot inconsistent (verify=%d); re-seeded from live map",
                     snap_verify);
            return finish(V1G_SEED, summary, msg, cap);
        }
        snprintf(summary, sizeof summary, "snapshot inconsistent (verify=%d) and re-save failed rc=%d",
                 snap_verify, rc);
        return finish(V1G_ERROR, summary, msg, cap);
    }

    /* --- both present and consistent --- */
    if (live.crc == snap.crc) {
        snprintf(summary, sizeof summary, "healthy (live == snapshot, crc=%08x)", live.crc);
        return finish(V1G_HEALTHY, summary, msg, cap);
    }

    int same_frame = (live.w == snap.w && live.h == snap.h &&
                      live.ox == snap.ox && live.oy == snap.oy &&
                      float_close(live.scale, snap.scale));
    if (same_frame) {
        /* same frame, different cells: a legitimate growth during a clean.
         * Keep the snapshot current, but only from a quiescent file and no more often
         * than g_min_refresh_ms. */
        if (!g_refresh) {
            snprintf(summary, sizeof summary, "growth detected, snapshot refresh disabled");
            return finish(V1G_HEALTHY, summary, msg, cap);
        }
        if (refresh_throttled()) {
            snprintf(summary, sizeof summary, "growth detected, snapshot refresh throttled (%d ms)",
                     g_min_refresh_ms);
            return finish(V1G_HEALTHY, summary, msg, cap);
        }
        v1g_mapframe_t stable_frame;
        int stable_rc = v1g_read_map_stable(lastmap_path, &stable_frame, g_settle_ms);
        if (stable_rc == -2) { snprintf(summary, sizeof summary, "growth detected but map not quiescent"); return finish(V1G_UNSTABLE, summary, msg, cap); }
        if (stable_rc != 0)  { snprintf(summary, sizeof summary, "growth re-read failed"); return finish(V1G_UNSTABLE, summary, msg, cap); }
        int rc = persist_map_save();
        if (rc == 0) {
            g_last_refresh_ms = now_ms();
            snprintf(summary, sizeof summary, "growth: snapshot refreshed (%ux%u origin=(%d,%d))",
                     stable_frame.w, stable_frame.h, stable_frame.ox, stable_frame.oy);
            return finish(V1G_GROWN, summary, msg, cap);
        }
        snprintf(summary, sizeof summary, "growth refresh failed rc=%d", rc);
        return finish(V1G_ERROR, summary, msg, cap);
    }

    /* --- frame changed: a rotation / full revisit --- */
    char detail[160];
    int rotate_rc = apply_rotate(&live, &snap, detail, sizeof detail);
    snprintf(summary, sizeof summary,
             "frame changed live{%ux%u o=(%d,%d) s=%.2f} snap{%ux%u o=(%d,%d) s=%.2f}: %s",
             live.w, live.h, live.ox, live.oy, live.scale,
             snap.w, snap.h, snap.ox, snap.oy, snap.scale, detail);
    return finish(rotate_rc < 0 ? V1G_ERROR : V1G_ROTATED, summary, msg, cap);
}

/* ===================================================================================== *
 *  loops (standalone binary only)                                                       *
 * ===================================================================================== */
#ifndef V1GUARD_NO_MAIN
static int g_quiet = 0;   /* --quiet: suppress per-tick logging (errors still logged) */

static void on_signal(int sig) { (void)sig; g_v1g_stop = 1; }

static int run_poll(int interval_ms, unsigned long max_ticks, int once) {
    unsigned long ticks = 0;
    while (!g_v1g_stop) {
        char msg[320];
        v1g_state_t state = v1g_run_once(msg, sizeof msg);
        if (!g_quiet || state == V1G_ERROR)
            v1g_log(1, "[v1guard] tick %lu: %s (%s)\n", ticks, v1g_state_name(state), msg);
        ticks++;
        if (once) break;
        if (max_ticks && ticks >= max_ticks) break;
        sleep_ms(interval_ms);
    }
    return 0;
}

static int run_watcher(int interval_ms, unsigned long max_ticks, int once) {
    int ifd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (ifd < 0) {
        v1g_log(1, "[v1guard] inotify unavailable (errno=%d); poll fallback\n", errno);
        return run_poll(interval_ms, max_ticks, once);
    }
    int wd = inotify_add_watch(ifd, persist_root(),
                               IN_CLOSE_WRITE | IN_MOVED_TO | IN_MOVED_FROM | IN_DELETE | IN_CREATE);
    if (wd < 0) {
        v1g_log(1, "[v1guard] inotify_add_watch failed (errno=%d); poll fallback\n", errno);
        close(ifd);
        return run_poll(interval_ms, max_ticks, once);
    }
    v1g_log(1, "[v1guard] watcher active on %s (safety poll %d ms)\n", persist_root(), interval_ms);
    unsigned long ticks = 0;
    long last_tick_ms = -1;
    while (!g_v1g_stop) {
        /* Debounce: coalesce a burst of inotify events (including the guard's own
         * snapshot renames) into at most one pass per g_min_tick_ms. */
        if (g_min_tick_ms > 0 && last_tick_ms >= 0) {
            long elapsed = now_ms() - last_tick_ms;
            if (elapsed < g_min_tick_ms) sleep_ms((int)(g_min_tick_ms - elapsed));
        }
        struct pollfd pfd;
        pfd.fd = ifd; pfd.events = POLLIN; pfd.revents = 0;
        int poll_rc = poll(&pfd, 1, interval_ms);
        if (poll_rc > 0) {
            char buf[4096];
            while (read(ifd, buf, sizeof buf) > 0) { /* drain */ }
        }
        char msg[320];
        v1g_state_t state = v1g_run_once(msg, sizeof msg);
        last_tick_ms = now_ms();
        if (!g_quiet || state == V1G_ERROR)
            v1g_log(1, "[v1guard] tick %lu: %s (%s)\n", ticks, v1g_state_name(state), msg);
        ticks++;
        if (once) break;
        if (max_ticks && ticks >= max_ticks) break;
    }
    inotify_rm_watch(ifd, wd);
    close(ifd);
    return 0;
}

/* ===================================================================================== *
 *  CLI                                                                                  *
 * ===================================================================================== */
static void usage(const char *argv0) {
    v1g_log(2,
        "usage: %s [options]\n"
        "  --data-dir DIR      V1 data dir (default $PERSIST_ROOT or /mnt/data/rockrobo)\n"
        "  --interval-ms N     poll interval / inotify safety timeout (default 2000)\n"
        "  --settle-ms N       double-read settle window before snapshotting (default 50)\n"
        "  --min-refresh-ms N  minimum ms between snapshot refreshes (default 2000)\n"
        "  --min-tick-ms N     minimum ms between watcher passes (debounce, default 250)\n"
        "  --tmp-stale-sec N   only remove *.tmp/*.new older than N s (default 300)\n"
        "  --max-ticks N       stop after N passes (0 = forever)\n"
        "  --once              run a single pass and exit\n"
        "  --on-rotate POLICY  hold|resave|reseg|restore (default hold)\n"
        "  --no-refresh        do not refresh the snapshot on legit cell growth\n"
        "  --watcher           use inotify to react immediately (default: poll)\n"
        "  --quiet             only log errors\n"
        "  -h, --help          this text\n",
        argv0);
}

int main(int argc, char **argv) {
    const char *datadir = NULL;
    int interval_ms = 2000;
    unsigned long max_ticks = 0;
    int once = 0, watcher = 0, quiet = 0;

    for (int i = 1; i < argc; i++) {
        const char *opt = argv[i];
        if (!strcmp(opt, "--data-dir") || !strcmp(opt, "--root")) {
            if (i + 1 >= argc) { usage(argv[0]); return 2; }
            datadir = argv[++i];
        } else if (!strcmp(opt, "--interval-ms")) {
            if (i + 1 >= argc) { return 2; }
            interval_ms = atoi(argv[++i]);
            if (interval_ms < 0) interval_ms = 0;
        } else if (!strcmp(opt, "--settle-ms")) {
            if (i + 1 >= argc) { return 2; }
            v1g_set_settle_ms(atoi(argv[++i]));
        } else if (!strcmp(opt, "--min-refresh-ms")) {
            if (i + 1 >= argc) { return 2; }
            v1g_set_min_refresh_ms(atoi(argv[++i]));
        } else if (!strcmp(opt, "--min-tick-ms")) {
            if (i + 1 >= argc) { return 2; }
            v1g_set_min_tick_ms(atoi(argv[++i]));
        } else if (!strcmp(opt, "--tmp-stale-sec")) {
            if (i + 1 >= argc) { return 2; }
            v1g_set_tmp_stale_sec(atoi(argv[++i]));
        } else if (!strcmp(opt, "--max-ticks")) {
            if (i + 1 >= argc) { return 2; }
            max_ticks = (unsigned long)strtoul(argv[++i], NULL, 10);
        } else if (!strcmp(opt, "--once")) {
            once = 1;
        } else if (!strcmp(opt, "--on-rotate")) {
            if (i + 1 >= argc) { return 2; }
            v1g_set_policy(v1g_policy_from_name(argv[++i]));
        } else if (!strcmp(opt, "--no-refresh")) {
            v1g_set_refresh(0);
        } else if (!strcmp(opt, "--watcher")) {
            watcher = 1;
        } else if (!strcmp(opt, "--quiet")) {
            quiet = 1;
        } else if (!strcmp(opt, "-h") || !strcmp(opt, "--help")) {
            usage(argv[0]);
            return 0;
        } else {
            v1g_log(2, "[v1guard] unknown option: %s\n", opt);
            usage(argv[0]);
            return 2;
        }
    }

    persist_set_root(datadir);          /* NULL -> env -> default */
    if (persist_init() != 0) {
        v1g_log(2, "[v1guard] cannot create persist dir under %s (errno=%d)\n", persist_root(), errno);
        return 1;
    }

    int lock_rc = v1g_lock();
    if (lock_rc == -1) { v1g_log(2, "[v1guard] another guard holds the lock under %s\n", persist_root()); return 1; }
    if (lock_rc < 0)   { v1g_log(2, "[v1guard] lock error under %s (errno=%d)\n", persist_root(), errno); return 1; }

    int removed_tmp = v1g_cleanup_tmp();
    g_quiet = quiet;
    if (!g_quiet)
        v1g_log(1, "[v1guard] start root=%s policy=%s watcher=%d interval=%dms cleaned_tmp=%d\n",
                persist_root(), v1g_policy_name(g_policy), watcher, interval_ms, removed_tmp);

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    if (watcher) run_watcher(interval_ms, max_ticks, once);
    else         run_poll(interval_ms, max_ticks, once);

    if (!g_quiet) v1g_log(1, "[v1guard] stop\n");
    v1g_unlock();
    return 0;
}
#endif /* V1GUARD_NO_MAIN */
