/* reanchor.h -- Occupancy-grid registration / frame re-anchoring for the V1.
 *
 * Problem
 * -------
 * The V1's SLAM map frame drifts between runs (the gen1 "full clean recreates and
 * rotates the map" behaviour, the original room-segmentation notes).  A stored `last_segmap` sidecar is
 * only meaningful against the map frame it was computed from, so
 * `src/persist.c`'s frame-link check *refuses* a mismatched frame (never mis-cleans,
 * but also never reuses).  This module estimates the rigid (optionally scaled)
 * transform `stored frame -> current frame` from the two occupancy grids, so the
 * stored segment grid can be re-anchored onto the current map and reused.  When the
 * alignment is not confident it abstains (no transform), and the caller falls back
 * to recomputing segmentation.
 *
 * What it works on
 * ----------------
 * Both grids are the same 5 cm/V1 map scale.  A grid is stored as two binary masks:
 *   free[c] = 1  -> known floor   (V1 log-odds < -eps)
 *   obs[c]  = 1  -> wall/obstacle (V1 log-odds > +eps)
 * The registration feature is the *free-space (floorplan) mask*; the free-region
 * boundary is used for the confidence metric.  The transform is
 *     current_cell = A * stored_cell + b,   A = (cur.scale/stored.scale) * s * R(theta)
 * i.e. rotation theta + translation + optional isotropic scale s.
 *
 * Search: coarse binarised rotation sweep (translation located per angle) -> top-K
 * coarse-to-fine pattern search (theta, tx, ty[, s]) on a free-mask Dice objective
 * augmented by a smooth boundary-Chamfer term.  Confidence: boundary Chamfer
 * inlier precision/recall/F1 plus Dice; abstain when below threshold, or when the
 * best candidate at a different rotation (>= min_angle_sep_deg) scores within
 * min_margin of the winner (near-symmetric plan; false-positive alignment is the
 * dangerous case).
 *
 */
#ifndef REANCHOR_H
#define REANCHOR_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- occupancy / segment grids ------------------------------------------- */

typedef struct {
    int      w, h;
    float    scale;      /* map resolution (cells per metre; 20.0 = 5 cm cells); equal for stored+current */
    int      ox, oy;     /* world origin of cell (0,0) (informational) */
    uint8_t *free;       /* w*h, 1 = known floor */
    uint8_t *obs;        /* w*h, 1 = wall/obstacle */
    long     freecount;
    long     obscount;
} reanchor_grid_t;

typedef struct {
    int      w, h;
    uint8_t *cells;      /* w*h, room id (already masked &0x3f) */
} reanchor_seg_t;

/* Grid-file format used by the CLI ("RRG1"):
 *   0  magic[4] "RRG1"   4 u32 version=1   8 u32 w   12 u32 h
 *   16 f32 scale         20 i32 ox         24 i32 oy   28 u32 reserved
 *   32 w*h bytes: 0 = unknown, 1 = free, 2 = obstacle
 * `reanchor_load_lastmap` also accepts the real V1 `last_map`
 * (`56 + w*h*4` float32 log-odds).                                             */

/* ---- options / result ----------------------------------------------------- */

typedef struct {
    double theta_min_deg;   /* sweep start (default 0) */
    double theta_max_deg;   /* sweep end (default 360) */
    double theta_step_deg;  /* coarse sweep step (default 3.0) */
    int    coarse_max;      /* coarse canvas max dim (default 72) */
    double tol_cells;       /* boundary inlier tolerance in cells (default 1.5) */
    double min_dice;        /* abstain below this Dice (default 0.45) */
    double min_edge_f1;     /* abstain below this boundary F1 (default 0.50) */
    double min_edge_each;   /* abstain below this boundary precision OR recall (default 0.35) */
    double min_support;     /* min fraction of stored free inside current bounds (default 0.10) */
    double min_margin;      /* abstain if the best refined score beats the best candidate at a
                             * different orientation by less than this (default 0.05) */
    double min_angle_sep_deg; /* a runner-up must differ from the winner by at least this
                               * rotation (degrees) to count (default 30.0) */
    int    allow_scale;     /* 1 => also search isotropic scale (default 0) */
    int    max_features;    /* cap on stored free cells used for optimisation (default 12000) */
    int    top_k;           /* coarse candidates refined (default 4) */
} reanchor_opts_t;

typedef struct {
    int      ok;            /* 1 = confident transform produced */
    double   theta_rad;
    double   theta_deg;     /* rotation stored -> current */
    double   scale;         /* searched isotropic scale (1.0 if not searched) */
    double   a00, a01, a10, a11;  /* affine: c = A*p + b (cell coords) */
    double   bx, by;
    double   score;         /* free-mask Dice at the solution */
    double   overlap;       /* |A n B| / min(|A|,|B|) */
    double   coverage;      /* fraction of stored free inside current bounds */
    double   edge_precision, edge_recall, edge_f1;
    double   edge_support;  /* min(fwd support, bwd support) */
    double   runner_up_score;      /* best refined score at a different orientation (>= min_angle_sep_deg
                                    * from the winner); -1 when no such candidate was refined */
    double   runner_up_theta_deg;  /* rotation of that runner-up (0 when none) */
    long     n_stored_free, n_cur_free;
    const char *reason;     /* why it abstained (static string), else "ok" */
} reanchor_result_t;

void reanchor_default_opts(reanchor_opts_t *o);

/* Core: estimate the transform aligning `stored` onto `cur`.  Returns 0 on success
 * (a transform was produced) or <0 if it abstained / on error; `res->ok` is the
 * authoritative flag. */
int reanchor_align(const reanchor_grid_t *stored, const reanchor_grid_t *cur,
                   const reanchor_opts_t *opts, reanchor_result_t *res);

/* Map a stored cell coordinate to a current cell coordinate through `res`. */
void reanchor_apply(const reanchor_result_t *res, double sx, double sy,
                    double *cx, double *cy);
/* Inverse of reanchor_apply. */
void reanchor_apply_inv(const reanchor_result_t *res, double cx, double cy,
                        double *sx, double *sy);

/* ---- segment-grid re-anchoring -------------------------------------------- */

/* Transform the stored byte segment grid (ids &0x3f) into the current frame.
 * `stored_pad` = number of pad cells by which the stored seg grid exceeds the
 * stored map grid on each side (v1seg PAD, 0 for a bare grid): seg cell (x,y)
 * corresponds to stored map cell (x-stored_pad, y-stored_pad).
 * The output grid has dims out_w x out_h and is aligned with the *current map*
 * grid cell (0,0); out cells outside the current map bounds are 0.
 * Sampling is nearest-neighbour with a small valid-neighbour fill (radius 1) so
 * labels are not dropped at cell boundaries.  Returns 0 on success. */
int reanchor_transform_seg(const reanchor_seg_t *stored, int stored_pad,
                           const reanchor_result_t *res,
                           int cur_w, int cur_h,
                           int out_w, int out_h, uint8_t *out_cells);

/* As reanchor_transform_seg, but `cur_free` (cur_w*cur_h, may be NULL) marks the
 * current map's known-floor cells.  The radius-1 valid-neighbour fill then only
 * paints known-floor cells, so room ids are never bled onto current walls. */
int reanchor_transform_seg_masked(const reanchor_seg_t *stored, int stored_pad,
                                  const reanchor_result_t *res,
                                  int cur_w, int cur_h,
                                  int out_w, int out_h, uint8_t *out_cells,
                                  const uint8_t *cur_free);

/* ---- I/O ------------------------------------------------------------------ */

int reanchor_load_grid(const char *path, reanchor_grid_t *g);      /* RRG1, or V1 last_map (eps=1.0) */
int reanchor_load_grid_ex(const char *path, float logodds_eps, reanchor_grid_t *g);
int reanchor_save_grid(const char *path, const reanchor_grid_t *g);/* RRG1 */
int reanchor_load_lastmap(const char *path, float logodds_eps, reanchor_grid_t *g);
/* Reads a `last_segmap` (RRSEG v1) sidecar payload; fills seg + the pad relative to
 * `map_w`/`map_h` (must be the map it was produced from). */
int reanchor_load_segmap(const char *path, int map_w, int map_h,
                         reanchor_seg_t *seg, int *pad_out,
                         float *scale_out, int *ox_out, int *oy_out);
/* Writes a `last_segmap` (RRSEG v1) sidecar: 64 B header + cells + room table,
 * computing the room table from the cells.  CRC32-verified on read. */
int reanchor_write_segmap(const char *path, int w, int h, const uint8_t *cells,
                          float scale, int ox, int oy,
                          unsigned ts, unsigned map_id, int dock_x, int dock_y);

void reanchor_grid_free(reanchor_grid_t *g);
void reanchor_seg_free(reanchor_seg_t *g);

#ifdef __cplusplus
}
#endif
#endif /* REANCHOR_H */
