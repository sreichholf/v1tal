/* guard.h -- keep-alive guard for the V1 map, as a standalone process.
 *
 * Reuses the shared snapshot/recover logic of src/persist.c
 * (read-only; not modified) and adds the pieces a *separate process* needs:
 *   * a poll loop / optional inotify watcher;
 *   * detection of the nav's clear triggers (slam_info.cfg timestamp -> 0,
 *     last_map missing) and of a *changed* last_map frame (rotation);
 *   * a rotation / re-anchor policy hook, so the caller decides what to do when
 *     the frame changes (the honest core limit: a file guard cannot undo a
 *     genuine rotation; it can only detect it and refuse / re-snapshot);
 *   * safe single-writer locking and stale-.tmp cleanup.
 *
 * Build/run for the V1 with the bundled arm-linux-gcc; the same source runs on
 * the host.  Logging is raw write() only (the V1's glibc 2.19
 * segfaults on setvbuf/fflush; this file never calls them).
 *
 */
#ifndef V1GUARD_H
#define V1GUARD_H

#include <stddef.h>
#include <stdint.h>

#include "persist.h"   /* persist_* API + PERSIST_SEG_PAD */

#ifdef __cplusplus
extern "C" {
#endif

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif

/* One V1 `last_map` frame: the documented header fields (scale@+20,
 * origin@+24/+28, w@+32, h@+36) plus a CRC32 of the whole file. */
typedef struct {
    int      present;    /* 1 if read successfully from a file of exact size */
    uint32_t w, h;
    int32_t  ox, oy;
    float    scale;
    uint32_t crc;        /* persist_crc32 over the whole file */
} v1g_mapframe_t;

/* Result of one guard pass. */
typedef enum {
    V1G_CLEAR    = 0,   /* nav cleared: last_map gone or slam ts==0 -> restored */
    V1G_SEED     = 1,   /* good live map, no snapshot yet -> snapshotted */
    V1G_HEALTHY  = 2,   /* live map == snapshot (or growth ignored): no action */
    V1G_GROWN    = 3,   /* same frame, cells changed -> snapshot refreshed */
    V1G_ROTATED  = 4,   /* frame changed -> rotation policy applied */
    V1G_NOMAP    = 5,   /* nothing to guard (no live map and no snapshot) */
    V1G_UNSTABLE = 6,   /* live file present but not yet quiescent / unreadable */
    V1G_ERROR    = 7    /* an operation failed */
} v1g_state_t;

/* What to do when last_map is rewritten in a *different* frame (a rotation or a
 * full revisit).  The default (HOLD) never clobbers a live map. */
typedef enum {
    V1G_ROT_HOLD    = 0,  /* default: warn, change nothing (safe) */
    V1G_ROT_RESAVE  = 1,  /* adopt the new frame: re-snapshot the live map */
    V1G_ROT_RESEG   = 2,  /* adopt the new frame + drop stale last_segmap */
    V1G_ROT_RESTORE = 3   /* DESTRUCTIVE: drop the live map, force the snapshot back */
} v1g_rotate_policy_t;

const char *v1g_state_name(v1g_state_t state);
const char *v1g_policy_name(v1g_rotate_policy_t policy);
v1g_rotate_policy_t v1g_policy_from_name(const char *name);   /* HOLD on unknown */

/* Rotation / re-anchor hook.  Called on V1G_ROTATED *instead of* the built-in
 * policy.  Return 0 to report the rotation as handled, <0 to report an error.
 * The hook is the seam for a real re-anchor (dock-vector / occupancy-grid
 * registration) or for the segmentation producer to recompute. */
typedef int (*v1g_rotate_hook_fn)(void *ctx, const v1g_mapframe_t *live,
                                  const v1g_mapframe_t *snap);
void v1g_set_rotate_hook(v1g_rotate_hook_fn hook, void *ctx);
void v1g_set_policy(v1g_rotate_policy_t policy);
void v1g_set_refresh(int on);          /* refresh snapshot on cell growth (default 1) */
void v1g_set_settle_ms(int ms);        /* double-read settle window (default 50) */
/* Minimum wall-clock interval between snapshot refreshes, and between watcher ticks.
 * The standalone binary defaults to 2000 ms / 250 ms so a busy clean cannot hammer the
 * filesystem; 0 disables the respective throttle. */
void v1g_set_min_refresh_ms(int ms);
void v1g_set_min_tick_ms(int ms);
/* Stale-.tmp cleanup age: a "<name>.tmp" is only removed when its mtime is at least
 * this many seconds old, so a live writer that does not take the persist lock is not
 * deleted mid-write.  0 removes regardless of age.  Default 300. */
void v1g_set_tmp_stale_sec(int sec);

/* Raw, unbuffered log line to fd (stdout=1 / stderr=2).  Never uses stdio
 * buffering; safe on the V1's glibc 2.19. */
void v1g_log(int fd, const char *fmt, ...);

/* Read a V1 last_map file and fill *out.  0 ok; -1 missing/short/malformed. */
int v1g_read_map(const char *path, v1g_mapframe_t *out);
/* Like v1g_read_map but reads twice `settle_ms` apart and requires an identical
 * CRC, so a torn nav write is not snapshotted.  0 stable, -1 error, -2 unstable. */
int v1g_read_map_stable(const char *path, v1g_mapframe_t *out, int settle_ms);

/* One guard pass.  Writes a one-line human summary into msg (may be NULL).
 * Returns the state and applies the corresponding action. */
v1g_state_t v1g_run_once(char *msg, size_t cap);

/* Remove any *stale* "<name>.tmp"/"<name>.new" siblings a crashed writer may have
 * left under the data dir and persist/.  A file is only removed when it is older
 * than ~v1g_set_tmp_stale_sec() seconds, so a live writer that does not hold the
 * persist lock (the nav's last_map/slam_info.cfg writers) is not deleted mid-rename.
 * Returns the number removed (>=0). */
int v1g_cleanup_tmp(void);

/* Single-writer, lifetime lock at <root>/.v1guard.lock (flock LOCK_EX|LOCK_NB),
 * held from startup to exit to keep two guards apart.  This is distinct from the
 * short-lived cross-process snapshot write lock <root>/.v1persist.lock that
 * persist_map_save/recover/reset take (see persist.h); the guard never holds that
 * one for its lifetime, so the LD_PRELOAD shim's writes cannot block behind it.
 * Returns 0 when acquired, -1 when another guard holds it, -2 on error. */
int v1g_lock(void);
void v1g_unlock(void);

/* The data-dir root currently in use (persist_set_root / PERSIST_ROOT / default). */
const char *v1g_root(void);

#ifdef __cplusplus
}
#endif
#endif /* V1GUARD_H */
