/* persist.h -- On-V1 persistent-map module.
 *
 * Robot-side logic for the persistent-maps feature (always on; there is no user toggle):
 *   * a protected snapshot of the V1's single `last_map` slot (+ `slam_info.cfg` +
 *     the `last_segmap` segment sidecar), so the map can survive a full clean / relocalise
 *     failure that would otherwise recreate (and rotate) it;
 *   * `recover_map`: reload `last_map` + `last_segmap` together, frame-linked (see below);
 *   * `reset_map`: drop the current map *and* the snapshot, so the guard cannot undo a
 *     deliberate reset.
 *
 * Frame link
 * ----------
 * `last_map` (56 + w*h*4 float32, V1 SLAM format) carries scale@+20, origin@+24/+28,
 * width@+32, height@+36.  `slam_info.cfg` carries the map `timestamp` (0 == "first clean").
 * `last_segmap` (the sidecar produced by src/v1seg.cpp) carries its own copy of the same
 * frame fields + a CRC32.  The sidecar is produced from a *padded* copy of the map grid
 * (v1seg PAD=3), so its dims are either equal to the map's (grid.bin path) or map+2*PAD
 * (--lastmap path).  A sidecar is "linked" to a map when scale and origin agree, the dims
 * agree up to that documented pad, and any nonzero timestamps/map-ids agree.
 *
 * This module is pure libc; it builds for the V1 (arm-linux-gcc, glibc 2.19) and for the
 * host.
 */
#ifndef PERSIST_HANDLERS_H
#define PERSIST_HANDLERS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Default root; override with persist_set_root() or the PERSIST_ROOT env var.
 * This is exactly NAV_DYNAMIC_CONFIG_PATH (/mnt/data/rockrobo, rrwatchdoge.conf). */
#define PERSIST_DEFAULT_ROOT "/mnt/data/rockrobo"

/* v1seg pads the map grid by PAD=3 on each side for the --lastmap input path, so a linked
 * sidecar has dims == map dims + 2*PAD.  A sidecar generated from a bare grid.bin has
 * dims == map dims (pad 0).  Both are accepted. */
#define PERSIST_SEG_PAD 3

/* last_segmap sidecar header (64 B, little-endian), matches src/v1seg.cpp. */
typedef struct {
    uint8_t  magic[8];       /* "RRSEG\0\0\1" */
    uint32_t version;        /* 1 */
    uint32_t map_timestamp;  /* frame link (0 = unknown) */
    uint32_t map_id;
    float    scale;
    int32_t  origin_x;
    int32_t  origin_y;
    uint32_t width;
    uint32_t height;
    int32_t  dock_x;
    int32_t  dock_y;
    uint32_t crc32;          /* over header (crc zeroed) + payload */
    uint32_t reserved[3];
} persist_seg_hdr_t;

/* Snapshot fingerprint file (persist/frame0).  Stores the map frame and the segment frame
 * as captured at `save` time so `recover` can verify the pair is still frame-linked. */
typedef struct {
    uint8_t  magic[8];       /* "RRPFRM\0\1" */
    uint32_t version;        /* 1 */
    /* map (last_map) half */
    uint32_t map_ts;         /* slam_info.cfg timestamp at snapshot time */
    uint32_t map_id;         /* 0 = unknown (the V1 SLAM file carries no map index) */
    uint32_t map_w, map_h;
    float    map_scale;
    int32_t  map_ox, map_oy;
    uint32_t map_crc;        /* crc32 of the last_map file */
    /* segment (last_segmap) half; seg_crc == 0 and seg_file_crc == 0 if no sidecar */
    uint32_t seg_ts, seg_map_id;
    uint32_t seg_w, seg_h;
    float    seg_scale;
    int32_t  seg_ox, seg_oy;
    uint32_t seg_crc;        /* sidecar header crc field (verified) */
    uint32_t seg_file_crc;   /* crc32 of the whole sidecar file */
    int32_t  pad;            /* seg_w - map_w (0 or 2*PERSIST_SEG_PAD) */
    uint32_t has_seg;        /* 1 if a sidecar was present when snapshotted */
    uint32_t crc;            /* crc32 over this struct with crc zeroed */
} persist_frame_rec_t;

/* ---- setup ---------------------------------------------------------------- */
void        persist_set_root(const char *root);   /* NULL -> env -> default */
const char *persist_root(void);
int         persist_init(void);                   /* resolve root + mkdir persist dir */

/* ---- cross-process write lock --------------------------------------------- */
/* Short-lived advisory flock on <persist_root()>/.v1persist.lock, separate from the
 * guard's *lifetime* instance lock (<root>/.v1guard.lock, see guard.h).
 *
 * persist_map_save/recover/reset (and factory_reset) take this lock internally, so
 * the LD_PRELOAD shim (map thread + MIIO thread) and the v1guard process can no
 * longer interleave snapshot writes.  The guard still holds .v1guard.lock for its
 * whole run, but never .v1persist.lock, so a shim write can never block forever
 * behind a running guard.  The lock is never held while taking another lock, so
 * deadlock is impossible; the kernel drops it if a writer dies.
 *
 * Callers that touch the snapshot/map files directly (e.g. the guard's stale-.tmp
 * cleanup) may bracket their work with these.  Returns a handle >= 0, or -1 when
 * the lock file cannot be created (the caller may then proceed best-effort). */
int  persist_write_lock(void);
void persist_write_unlock(int lock_fd);

/* ---- frame helpers -------------------------------------------------------- */
uint32_t persist_crc32(const void *data, size_t len);  /* standard IEEE crc32 */
int persist_read_slam_timestamp(uint32_t *ts);    /* /mnt/data/rockrobo/slam_info.cfg */

/* Reads the map half (last_map + slam_info.cfg).  Returns 0 on success. */
int persist_read_lastmap_frame(persist_frame_rec_t *rec);
/* Reads the seg half (last_segmap, CRC-verified).  Returns 0 on success. */
int persist_read_segmap_frame(persist_frame_rec_t *rec);
/* 1 when the seg frame belongs to the map frame (see header comment). */
int persist_frames_linked(const persist_frame_rec_t *map, const persist_frame_rec_t *seg);

/* ---- command handlers (mirror the miio surface) --------------------------- */
/* `save_map`: snapshot last_map (+ last_segmap + slam_info.cfg).  0 ok, <0 error.
 *
 * The save is staged and committed with a single ordering rule: the new files are
 * written under `*.new` names, then renamed into place with `frame0` renamed LAST
 * as the commit point.  A write failure at any step therefore leaves the previous
 * snapshot (map0/frame0/...) untouched and recoverable.  Every source file is read
 * exactly once into memory; its CRC and the bytes written to the snapshot come from
 * the same buffer, so a concurrent nav rewrite can never make frame0 disagree with
 * map0.  Returns PERSIST_ERR_SLAM_TS_ZERO (and changes nothing) when the live
 * slam_info.cfg carries timestamp 0: that is the nav's "cleared" signal, and
 * snapshotting it would only make the guard restore a freshly-cleared map. */
int persist_map_save(void);
#define PERSIST_ERR_SLAM_TS_ZERO (-6)
/* `recover_map [slot]`: restore the snapshot atomically, frame-linked.
 *   0 ok, -2 no snapshot, -3 snapshot not frame-linked (refused), -4 io error. */
int persist_map_recover(int slot);
/* `reset_map`: drop the current map (last_map + slam_info.cfg + last_segmap) AND the
 * protected snapshot.  Clearing the snapshot is what makes the unconditional guard
 * respect a deliberate reset instead of immediately restoring the map.  0 ok. */
int persist_map_reset(void);
/* Recover the snapshot even when the live map is a *different* frame (rotation
 * re-anchor).  The snapshot is verified first; on a missing snapshot returns -2,
 * on an inconsistent snapshot returns -3 leaving the live map untouched, otherwise
 * the live map is dropped and the snapshot installed under ONE lock (no window in
 * which the live map is gone but the snapshot was not validated). */
int persist_map_force_restore(void);
/* wipe the snapshot too (factory reset).  Does not recreate persist/. */
int persist_factory_reset(void);
/* `get_recover_maps`: fills slots[]/times[] (epoch seconds of the snapshot); returns count. */
int persist_recover_list(uint32_t *slots, uint32_t *times, int max);

/* ---- keep-alive guard ----------------------------------------------------- */
/* Intended to be called by a tiny supervisor; persistence is unconditional.  If the nav
 * has cleared / reset the map (slam_info timestamp == 0, or last_map gone) but a snapshot
 * exists, restore it.  Returns 1 if it restored, 0 otherwise, <0 on error. */
int persist_guard_tick(void);

/* ---- introspection (logging) ---------------------------------------------- */
/* Re-verify the committed snapshot (persist/frame0 against persist/map0) under the
 * write lock.  Returns 0 when consistent, -1 when the snapshot is absent/unreadable,
 * -2 when map0 no longer matches the frame0 fingerprint.  On success fills *rec with
 * the recorded frame and *have_seg with whether the recorded sidecar is still valid.
 * `have_seg` may be NULL. */
int  persist_snapshot_verify(persist_frame_rec_t *rec, int *have_seg);
/* Human-readable one-line description of a frame record into buf. */
void persist_frame_str(const persist_frame_rec_t *rec, char *buf, size_t cap);

#ifdef __cplusplus
}
#endif
#endif /* PERSIST_HANDLERS_H */
