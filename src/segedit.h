/* segedit.h -- room segment editing (merge / split) for the V1 `last_segmap` sidecar.
 *
 * Pure libc, no robot/AppProxy dependencies: this compiles into the AppProxy add-on
 * (src/addon.c) or can be linked independently.  It implements the two
 * `RoborockMapSegmentEditCapability` commands on the byte-per-cell segment grid used
 * by the V1 sidecar (`src/v1seg.cpp`):
 *
 *   merge_segment [idA, idB]
 *       relabel every cell whose id (`byte & 0x3f`) is any listed id to one target id.
 *       Target = the first listed id in 16..31, else the lowest free id in 16..31.
 *       The 0x40 flag bit (and any other non-id bit) is preserved.  Out-of-range ids
 *       are rejected and at least two distinct present ids are required.
 *
 *   split_segment [segId, x1,y1,x2,y2]      (mm, already Y-flipped by Valetudo)
 *       cut the region `segId` along the finite segment pA->pB; keep `segId` on one
 *       side and assign a fresh user id (16..31) to the component(s) the segment
 *       actually cuts off.  A segment that does not separate the region is refused.
 *
 * Ids remain `byte & 0x3f` (auto ids 1..15, user-defined ids 16..31), mirroring the
 * S5 `SegmentMap::GetNextUserDefineSegmentID` convention
 * (TECHNICAL.md §2.6).
 *
 * After an edit the caller rewrites the `RRSEG` sidecar atomically (`.tmp` + fsync +
 * rename) with a rebuilt room table and a fresh CRC32 -- see
 * segedit_sidecar_write_atomic().
 *
 */
#ifndef SEGEDIT_H
#define SEGEDIT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- segment-id bit layout (matches src/addon.c / v1seg.cpp) ---- */
#define SEGEDIT_ID_MASK    0x3fu  /* id  = byte & 0x3f          */
#define SEGEDIT_FLAG_MASK  0x40u  /* S5 flag bit, preserved      */
#define SEGEDIT_KEEP_MASK  0xc0u  /* every non-id bit preserved  */

/* user-defined id range (S5 GetNextUserDefineSegmentID: 0x10..0x1f) */
#define SEGEDIT_USER_MIN   16
#define SEGEDIT_USER_MAX   31
#define SEGEDIT_AUTO_MAX   15

/* ---- return codes (SEGEDIT_OK == 0; all errors are negative) ---- */
enum {
    SEGEDIT_OK       =  0,
    SEGEDIT_EARG     = -1,   /* bad argument (null/bad dims/no ids)     */
    SEGEDIT_ELINE    = -2,   /* degenerate split line (pA == pB)        */
    SEGEDIT_ENOSEG   = -3,   /* listed segment id not present in grid   */
    SEGEDIT_ENOID    = -4,   /* no free user id left in 16..31          */
    SEGEDIT_ENOSPLIT = -5,   /* the line does not split the region      */
    SEGEDIT_EMAGIC   = -6,   /* sidecar: bad magic/size/CRC             */
    SEGEDIT_EIO      = -7    /* sidecar: I/O error                      */
};

/* User-facing text for a SEGEDIT_* code ("ok" for SEGEDIT_OK). */
const char *segedit_strerror(int rc);

/* ---- a byte-per-cell segment grid (row-major, top-first) ---- */
typedef struct {
    int      w, h;
    uint8_t *cells;          /* w*h bytes; id = cells[i] & SEGEDIT_ID_MASK */
} segedit_grid;

/* ---- mm -> cell mapping ----
 * Valetudo's `split_segment` (and `app_zoned_clean`) send millimetres in the
 * 0..51200 frame where one map cell is `mm_per_cell` (50) mm:
 *
 *     mm_x = 50 * shared_grid_col      mm_y = 50 * shared_grid_row
 *
 * The sidecar grid may be a *padded* copy of the map (v1seg PAD=3) and is blitted
 * into the shared-grid-shaped buffer at (dx,dy) (V1SEG_BLIT_OFFSET).  The inverse is
 *
 *     sidecar_col = mm_x / mm_per_cell - dx
 *     sidecar_row = mm_y / mm_per_cell - dy
 *
 * so a padded sidecar (dx=dy=-PAD) maps map cell (0,0) to sidecar (PAD,PAD). */
typedef struct {
    int mm_per_cell;         /* 0 -> default 50 */
    int dx, dy;              /* sidecar -> shared-grid blit offset (V1SEG_BLIT_OFFSET) */
} segedit_mm_map;

/* ---- room table entry (id, area, centroid), as stored in the sidecar ---- */
typedef struct {
    int  id;
    long area;
    int  cx, cy;
} segedit_room_t;

/* ===================== grid helpers ===================== */
void segedit_grid_init(segedit_grid *g);
/* Allocate (or re-allocate) a w*h grid.  The grid MUST have been initialised
 * (or previously used) first: alloc releases any buffer it already holds. */
int  segedit_grid_alloc(segedit_grid *g, int w, int h);   /* 0 ok */
void segedit_grid_free(segedit_grid *g);

static inline int segedit_id_at(const segedit_grid *g, int c, int r) {
    if (!g || !g->cells || c < 0 || r < 0 || c >= g->w || r >= g->h) return 0;
    return g->cells[(size_t)r * g->w + c] & SEGEDIT_ID_MASK;
}

/* lowest free user id in 16..31 (0 when the range is exhausted) */
int segedit_next_user_id(const segedit_grid *g);
/* present ids, ascending; fills up to `max` entries of `out` (when non-NULL) and
 * returns the *true* number of distinct present ids -- which may be larger than `max`
 * (the function only ever writes `max` entries). */
int segedit_present_ids(const segedit_grid *g, int *out, int max);
/* number of cells carrying `id` (byte & 0x3f == id) */
long segedit_count_id(const segedit_grid *g, int id);

/* ===================== edits ===================== */

/* merge_segment: relabel every cell whose id is in ids[] to one target id.
 *   The ids must be raw, distinct segment ids in 1..0x3f: out-of-range values are
 *   rejected (SEGEDIT_EARG) rather than masked, and at least two *distinct present*
 *   ids are required (a single id or a duplicate set is SEGEDIT_EARG; fewer than two
 *   present ids is SEGEDIT_ENOSEG).
 *   *out_target (optional) receives the chosen target id.
 * Returns SEGEDIT_OK, or SEGEDIT_EARG / SEGEDIT_ENOSEG / SEGEDIT_ENOID. */
int segedit_merge(segedit_grid *g, const int *ids, int nids, int *out_target);

/* split_segment in *cell* coordinates (the line may be fractional).
 *   The finite segment pA->pB is used as a barrier: the region `segId` is flood-filled
 *   with the sign of the cross product deciding the keep half (cross >= 0 keeps the
 *   id) and only components that are actually cut off from the keep side receive the
 *   fresh user id.  A segment that does not separate the region is SEGEDIT_ENOSPLIT.
 *   *out_newid (optional) receives the freshly allocated user id.
 * Returns SEGEDIT_OK, or SEGEDIT_EARG / SEGEDIT_ELINE / SEGEDIT_ENOSEG /
 * SEGEDIT_ENOID / SEGEDIT_ENOSPLIT. */
int segedit_split_cells(segedit_grid *g, int segid,
                        double ax, double ay, double bx, double by,
                        int *out_newid);

/* split_segment in millimetres (the Valetudo Y-flipped frame).
 * `opt` may be NULL (defaults: 50 mm/cell, blit 0,0). */
int segedit_split_mm(segedit_grid *g, int segid,
                     long x1, long y1, long x2, long y2,
                     const segedit_mm_map *opt, int *out_newid);

/* convert an mm point to sidecar cell coordinates (fractional). */
void segedit_mm_to_cell(const segedit_mm_map *opt, long x_mm, long y_mm,
                        double *cx, double *cy);

/* ===================== RRSEG sidecar ===================== */
/* 64-byte little-endian header, magic "RRSEG\0\0\1", matching src/v1seg.cpp:
 *   0 magic[8] | 8 u32 version | 12 u32 map_timestamp | 16 u32 map_id
 *   20 f32 scale | 24 i32 origin_x | 28 i32 origin_y | 32 u32 width | 36 u32 height
 *   40 i32 dock_x | 44 i32 dock_y | 48 u32 crc32 (hdr crc-zeroed + payload)
 *   52 u32 reserved[3]
 * payload: u8 cells[w*h]; u32 room_count; room_count * {u8 id;u8 pad[3];u32 area;i32 cx;i32 cy}
 */
#define SEGEDIT_HDR_SIZE 64

typedef struct {
    uint32_t version;
    uint32_t map_timestamp;
    uint32_t map_id;
    float    scale;
    int32_t  origin_x, origin_y;
    uint32_t width, height;
    int32_t  dock_x, dock_y;
    uint32_t reserved[3];
} segedit_meta;

/* Read + CRC-verify a sidecar into a freshly allocated grid.  0 ok. */
int segedit_sidecar_read(const char *path, segedit_grid *g, segedit_meta *m);
/* Serialize a sidecar (recomputing the room table + CRC) and write it atomically
 * (`.tmp` + fsync + rename).  `seed` may be NULL to zero the header extras. */
int segedit_sidecar_write_atomic(const char *path, const segedit_grid *g,
                                 const segedit_meta *seed);

/* Build the room table (ascending id) from a grid; returns the room count,
 * or <0 on error.  `out` may be NULL to only count. */
int segedit_room_table(const segedit_grid *g, segedit_room_t *out, int max);

/* standard IEEE CRC32 (zlib polynomial), init 0 */
uint32_t segedit_crc32(const void *p, size_t n);

#ifdef __cplusplus
}
#endif
#endif /* SEGEDIT_H */
