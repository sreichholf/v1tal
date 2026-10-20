/* v1seg.cpp -- ON-V1 room-segmentation producer.
 *
 * This is the project's only C++ translation unit.  It is C elsewhere, but it calls the S5's
 * own segmentation library -- C++ (`rs::segment_gridmap` + OpenCV) -- which can throw
 * `cv::Exception`; the `try/catch` around that call is the sole reason it is built as C++
 * (`arm-linux-g++`) and named `.cpp` (the extension always states the language).  See
 * `TECHNICAL.md` §1 "Language choices".
 *
 *   v1seg <grid.bin|--lastmap <path>> <out_segmap> [minArea] [options]
 *
 * Options:
 *   --model-grid WALL.bin   second occupancy grid fed to segment_gridmap (the WALL-ONLY grid:
 *                           obstacle = wall pixels, unknown = free).  Its output is informative
 *                           (per-wall-piece labels) whereas on the nav's obstacle-inclusive grid
 *                           it is a single constant.  Required for the model cue with grid.bin.
 *   --cue off|door|uniform|stub  cue mode (default: "stub" on the grid path, but **"off"**
 *                           on the --lastmap path, where the S5-tuned "stub" cue misfires).
 *                           "uniform" merges adjacent regions
 *                           whose shared boundary is model-uniform (the literal brief rule).
 *                           "door" merges every boundary except one carrying exactly two model
 *                           labels (the model's doorway signature).  "stub" = "door" plus a
 *                           fragment-absorption refinement of the 2-label boundaries (a short
 *                           flanking wall piece + a small region behind it is an internal neck,
 *                           not a door). The cue is empirical.
 *   --merge-tol N           tolerance for --cue uniform (default 1).
 *   --stub-piece N          "stub" min flanking wall-piece size threshold (default 128).
 *   --stub-area N           "stub" smaller-region area threshold (default 2500).
 *   --door-radius N|auto    core threshold / auto-select (default 7; auto bumps to 8 on
 *                           many-room maps).  Env V1SEG_DOOR_RADIUS.
 *   --min-core N --pad N    geometry params (env V1SEG_MIN_CORE / V1SEG_PAD).
 *
 * Reads a V1 occupancy grid (either the raw grid.bin format
 *   int32 w,h + w*h bytes,  0xFF = obstacle, 0 = free
 * or the real V1 SLAM map format `56 + w*h*4` float32 log-odds), runs the S5
 * room-segmentation function `rs::segment_gridmap` (dlopen libroomseg.so.0), then runs the
 * reconstructed nav post-processing on its raw output to obtain a byte
 * room-id grid (id = byte & 0x3f) and persists it as a sidecar `last_segmap` (atomic
 * .tmp + rename, magic/version/frame-link/CRC32, self-describing room table).
 *
 * The segment_gridmap input ("model grid") and the navpost input ("post grid") may differ:
 *   model grid = obstacle only where a wall is present  -> informative wall-piece labelling
 *   post grid  = obstacle where wall OR unknown         -> the nav's "known floor" free space
 * The `--lastmap` path builds both from the log-odds (occupied / occupied-or-unknown).  The
 * grid.bin path uses the positional grid as the post grid and optionally --model-grid for the
 * model grid; if --model-grid is omitted the two coincide and the cue degrades to off.
 *
 * ABI (recovered from libroomseg; see TECHNICAL.md):
 *   param_1 grid    : { int w; int h; void* data; }   w*h bytes, -1/0xFF = obstacle, 0 = free
 *   param_2/param_3 : std::vector outputs (unused by libroomseg; empty objects)
 *   param_4         : 1
 *   param_5 model   : 0x24-byte struct; +0x0d != 0, +0x14 = model path, +0x20 == 3
 *   param_6 segmap  : { int w; int h; int* out; }     out = w*h int32
 *
 * NOTE on logging: on the V1's glibc 2.19, `setvbuf`/`fflush` on stdio
 * segfault.  All diagnostics therefore go through raw write(); files are
 * written with open()/write()/fsync().  We also _exit() to skip stdio atexit flushing.
 *
 * The file is deliberately written to compile as C or C++ (it is built with arm-linux-g++ so
 * that a throwing libroomseg (cv::Exception ROI assertion) can be caught and the run degrades
 * gracefully to geometry-only segmentation).
 *
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <dlfcn.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "crc32.h"

/* ===================================================================================== *
 *  logging (raw write, never stdio)                                                     *
 * ===================================================================================== */
static void wout(const char *s) { ssize_t nwritten = write(1, s, strlen(s)); (void)nwritten; }
static void woutf(const char *fmt, ...) {
    char buf[1024]; va_list ap; va_start(ap, fmt);
    int length = vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    if (length > 0) { if (length > (int)sizeof buf) length = (int)sizeof buf; ssize_t nwritten = write(1, buf, length); (void)nwritten; }
}

/* ===================================================================================== *
 *  little-endian helpers + crc32                                                        *
 * ===================================================================================== */
static void put_u32(unsigned char *buf, uint32_t value) {
    buf[0] = (unsigned char)(value); buf[1] = (unsigned char)(value >> 8);
    buf[2] = (unsigned char)(value >> 16); buf[3] = (unsigned char)(value >> 24);
}
static uint32_t get_u32(const unsigned char *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}
static int32_t get_i32(const unsigned char *buf) { return (int32_t)get_u32(buf); }
static float get_f32(const unsigned char *buf) { uint32_t bits = get_u32(buf); float value; memcpy(&value, &bits, 4); return value; }
static void put_f32(unsigned char *buf, float value) { uint32_t bits; memcpy(&bits, &value, 4); put_u32(buf, bits); }

static uint32_t crc_table[256];
static void crc_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc_val = i;
        for (int k = 0; k < 8; k++)
            crc_val = (crc_val & 1) ? (CRC32_POLYNOMIAL_REFLECTED ^ (crc_val >> 1))
                                    : (crc_val >> 1);
        crc_table[i] = crc_val;
    }
}
static uint32_t crc32_buf(const void *data, size_t length, uint32_t crc) {
    const unsigned char *bytes = (const unsigned char *)data;
    crc = crc ^ CRC32_INITIAL_VALUE;
    for (size_t i = 0; i < length; i++) crc = crc_table[(crc ^ bytes[i]) & 0xff] ^ (crc >> 8);
    return crc ^ CRC32_FINAL_XOR;
}

/* ===================================================================================== *
 *  rs::segment_gridmap (C++ symbol, called through a C ABI)                              *
 * ===================================================================================== */
#define SEG_SYM "_ZN2rs15segment_gridmapEPKvPSt6vectorIS2_IS2_IN2cv6Point_IiEESaIS5_EESaIS7_EESaIS9_EESC_iPvPKNS_12SegmentMap_tE"
typedef unsigned (*segfn_t)(const void *, void *, void *, int, void *, const void *);

struct GridT   { int w; int h; void *data; };
struct SegMapT { int cols; int rows; void *out; int extra; };
struct VecT    { void *b; void *e; void *c; };   /* libstdc++ empty std::vector layout */

static int call_segment_gridmap(const char *modelpath, const unsigned char *grid, int w, int h,
                                int32_t *out) {
    void *lib = dlopen("libroomseg.so.0", RTLD_NOW | RTLD_GLOBAL);
    if (!lib) {
        woutf("[v1seg] dlopen(libroomseg.so.0) FAILED: %s\n", dlerror());
        return 0;
    }
    segfn_t segment_fn = (segfn_t)dlsym(lib, SEG_SYM);
    if (!segment_fn) {
        woutf("[v1seg] dlsym(rs::segment_gridmap) FAILED: %s\n", dlerror());
        return 0;
    }
    GridT grid_arg = { w, h, (void *)grid };
    SegMapT segmap_arg = { w, h, (void *)out, 0 };
    VecT vec_out_a; vec_out_a.b = vec_out_a.e = vec_out_a.c = 0;
    VecT vec_out_b; vec_out_b.b = vec_out_b.e = vec_out_b.c = 0;

    unsigned char model[64];
    memset(model, 0, sizeof model);
    put_f32(model + 0x00, 0.05f);
    put_f32(model + 0x08, 50.0f);
    model[0x0c] = 1;
    model[0x0d] = 1;
    void *model_path_ptr = (void *)modelpath;
    memcpy(model + 0x14, &model_path_ptr, sizeof model_path_ptr);
    int three = 3;
    memcpy(model + 0x20, &three, sizeof three);

    int ok = 0;
    try {
        unsigned result_code = segment_fn(&grid_arg, &vec_out_a, &vec_out_b, 1, model, &segmap_arg);
        woutf("[v1seg] segment_gridmap returned %u\n", result_code);
        ok = 1;
    } catch (...) {
        wout("[v1seg] segment_gridmap threw (cv/model); falling back to geometry-only\n");
        ok = 0;
    }
    return ok;
}

/* ===================================================================================== *
 *  nav post-processing port (reconstructed from the nav)                                       *
 *    free space -> distance transform -> door cutting -> model-cue merge ->             *
 *    area filter -> sequential byte ids (& 0x3f)                                        *
 * ===================================================================================== */
static int W, H;
static unsigned char *post_grid;      /* post-processing grid: 0xFF obstacle, 0 free */
static int32_t *raw_labels;          /* raw segment_gridmap output (w*h), from the model grid */
static int PAD = 3;

/* Model-cue configuration (overridable by flags / env).
 *   cue_mode 0 = off ; 1 = "door" ; 2 = "uniform" (the brief's literal rule) ;
 *   3 = "stub" (tuned default: "door" + fragment absorption on 2-label boundaries).
 * The wall-only grid fed to segment_gridmap makes its output informative: its nonzero cells are
 * the wall cells and value/100 is a per-wall-piece label.  The cue merges adjacent watershed
 * regions according to the model labels seen on their shared boundary (see model_merge_pass). */
static int cue_mode = 3;        /* CUE_MODE_STUB (tuned default) */
static int model_merge_tol = 1; /* used by cue_mode == 2 (uniform) */
/* "stub" refinement: on a boundary carrying exactly two labels, the opening is treated as an
 * internal neck (merge) when the *shorter* flanking wall piece is small AND the smaller region
 * behind it is a modest fragment rather than a full room.  A real doorway is punched through
 * substantial walls and separates two sizeable rooms on both sides. */
#define STUB_PIECE     128      /* max cell count of the shorter flanking wall piece */
#define STUB_FRAG_AREA 2500     /* max cell count of the smaller of the two regions */
static int stub_piece = STUB_PIECE;
static long stub_area = STUB_FRAG_AREA;
static int auto_door = 0;       /* auto: raise the base door radius on many-room maps */

static const char *cue_name(int on) {
    if (!on) return "off";
    switch (cue_mode) {
        case 2: return "uniform";
        case 3: return "stub";
        default: return "door";
    }
}
#define AUTO_CORE_THRESHOLD 16  /* cores at the base radius that trigger the bump */
#define AUTO_DOOR_BUMP 8

static inline int inside_roi(int x, int y) { return x >= PAD && y >= PAD && x < W - PAD && y < H - PAD; }
static inline int cell_index(int x, int y) { return y * W + x; }
static inline int isfree(int x, int y) { return inside_roi(x, y) && post_grid[cell_index(x, y)] == 0; }

static const int DX4[4] = { 1, -1, 0, 0 };
static const int DY4[4] = { 0, 0, 1, -1 };

static int *dist_map;
static void distance_transform(void) {
    size_t n = (size_t)W * H;
    dist_map = (int *)calloc(n, sizeof(int));
    int *queue = (int *)malloc(n * sizeof(int));
    long head = 0, tail = 0;
    for (int y = PAD; y < H - PAD; y++)
        for (int x = PAD; x < W - PAD; x++) {
            if (post_grid[cell_index(x, y)]) continue;
            int edge = 0;
            for (int k = 0; k < 4; k++) {
                int nx = x + DX4[k], ny = y + DY4[k];
                if (!inside_roi(nx, ny) || post_grid[cell_index(nx, ny)]) { edge = 1; break; }
            }
            if (edge) { dist_map[cell_index(x, y)] = 1; queue[tail++] = cell_index(x, y); }
        }
    while (head < tail) {
        int i = queue[head++], x = i % W, y = i / W, dist = dist_map[i];
        for (int k = 0; k < 4; k++) {
            int nx = x + DX4[k], ny = y + DY4[k];
            if (!inside_roi(nx, ny)) continue;
            int j = cell_index(nx, ny);
            if (post_grid[j] == 0 && dist_map[j] == 0) { dist_map[j] = dist + 1; queue[tail++] = j; }
        }
    }
    free(queue);
}

static int *core_label, core_count;
static int *core_size;
static void find_cores(int doorRadius, int minCore) {
    size_t n = (size_t)W * H;
    if (core_label) { free(core_label); core_label = 0; }
    if (core_size) { free(core_size); core_size = 0; }
    core_label = (int *)calloc(n, sizeof(int));
    core_size = (int *)calloc(n + 2, sizeof(int));
    int *bfs_queue = (int *)malloc(n * sizeof(int));
    core_count = 0;
    for (int y = PAD; y < H - PAD; y++)
        for (int x = PAD; x < W - PAD; x++) {
            int start_index = cell_index(x, y);
            if (core_label[start_index] || post_grid[start_index] || dist_map[start_index] <= doorRadius) continue;
            int core_id = ++core_count;
            long head = 0, tail = 0, size_cells = 0;
            bfs_queue[tail++] = start_index; core_label[start_index] = core_id;
            while (head < tail) {
                int i = bfs_queue[head++]; size_cells++;
                int ix = i % W, iy = i / W;
                for (int k = 0; k < 4; k++) {
                    int nx = ix + DX4[k], ny = iy + DY4[k];
                    if (!inside_roi(nx, ny)) continue;
                    int j = cell_index(nx, ny);
                    if (!core_label[j] && post_grid[j] == 0 && dist_map[j] > doorRadius) {
                        core_label[j] = core_id; bfs_queue[tail++] = j;
                    }
                }
            }
            core_size[core_id] = (int)size_cells;
        }
    for (int core_index = 1; core_index <= core_count; core_index++)
        if (core_size[core_index] < minCore)
            for (size_t i = 0; i < n; i++)
                if (core_label[i] == core_index) core_label[i] = 0;
    free(bfs_queue);
}

static int *label_map;
static void watershed(unsigned char *barrier) {
    size_t n = (size_t)W * H;
    label_map = (int *)calloc(n, sizeof(int));
    int max_dist = 0;
    for (size_t i = 0; i < n; i++) if (dist_map[i] > max_dist) max_dist = dist_map[i];
    int *bucket_count = (int *)calloc(max_dist + 2, sizeof(int));
    for (size_t i = 0; i < n; i++)
        if (post_grid[i] == 0 && dist_map[i] > 0 && !(barrier && barrier[i])) bucket_count[dist_map[i]]++;
    int **bucket = (int **)malloc((max_dist + 2) * sizeof(int *));
    for (int dist = 1; dist <= max_dist; dist++) { bucket[dist] = (int *)malloc((size_t)(bucket_count[dist] + 1) * sizeof(int)); bucket_count[dist] = 0; }
    for (size_t i = 0; i < n; i++)
        if (post_grid[i] == 0 && dist_map[i] > 0 && !(barrier && barrier[i])) {
            int dist = dist_map[i]; bucket[dist][bucket_count[dist]++] = (int)i;
        }
    for (size_t i = 0; i < n; i++) if (core_label[i]) label_map[i] = core_label[i];
    for (int dist = max_dist; dist >= 1; dist--)
        for (int item_index = 0; item_index < bucket_count[dist]; item_index++) {
            int i = bucket[dist][item_index];
            int ix = i % W, iy = i / W;
            int my_label = label_map[i];
            if (!my_label) {
                int best_label = 0, best_dist = -1;
                for (int k = 0; k < 4; k++) {
                    int nx = ix + DX4[k], ny = iy + DY4[k];
                    if (!inside_roi(nx, ny)) continue;
                    int j = cell_index(nx, ny);
                    if (label_map[j] && dist_map[j] > best_dist) { best_dist = dist_map[j]; best_label = label_map[j]; }
                }
                if (best_label) { label_map[i] = best_label; my_label = best_label; }
            }
            if (my_label)
                for (int k = 0; k < 4; k++) {
                    int nx = ix + DX4[k], ny = iy + DY4[k];
                    if (!inside_roi(nx, ny)) continue;
                    int j = cell_index(nx, ny);
                    if (post_grid[j] == 0 && dist_map[j] > 0 && !label_map[j] && !(barrier && barrier[j]))
                        label_map[j] = my_label;
                }
        }
    for (int dist = 1; dist <= max_dist; dist++) free(bucket[dist]);
    free(bucket); free(bucket_count);
}

/* model label field: raw/100 of the nearest labelled cell, propagated through free space.
 * Also fills wall_piece_cells[label] = number of raw cells carrying that label, i.e. the size (in cells)
 * of each model wall piece (used by the "stub" cue mode). */
static int32_t *label_field;
static int *wall_piece_cells;           /* wall_piece_cells[label] = cell count of wall piece `label` */
static int max_wall_label;        /* highest label index present (0 if none) */
static int build_labfield(void) {
    if (!raw_labels) return 0;
    size_t n = (size_t)W * H;
    label_field = (int32_t *)calloc(n, sizeof(int32_t));
    int *queue = (int *)malloc(n * sizeof(int));
    long head = 0, tail = 0;
    int highest_label = 0, capped = 0;
    for (size_t i = 0; i < n; i++)
        if (raw_labels[i]) {
            int32_t label_val = raw_labels[i] / 100;
            if (label_val > 0 && label_val < (1 << 20)) { label_field[i] = label_val; queue[tail++] = (int)i; if (label_val > highest_label) highest_label = label_val; }
            else capped = 1;
        }
    if (capped) wout("[v1seg] model labels capped at 2^20 (some ignored)\n");
    max_wall_label = highest_label;
    wall_piece_cells = (int *)calloc((size_t)max_wall_label + 1, sizeof(int));
    for (size_t i = 0; i < n; i++) {
        int32_t label_val = label_field[i];
        if (label_val > 0 && label_val <= max_wall_label) wall_piece_cells[label_val]++;
    }
    while (head < tail) {
        int i = queue[head++], x = i % W, y = i / W;
        for (int dx = -1; dx <= 1; dx++)
            for (int dy = -1; dy <= 1; dy++) {
                int nx = x + dx, ny = y + dy;
                if (!inside_roi(nx, ny)) continue;
                int j = cell_index(nx, ny);
                if (post_grid[j] == 0 && label_field[j] == 0) { label_field[j] = label_field[i]; queue[tail++] = j; }
            }
    }
    free(queue);
    /* Cue sanity: the raw grid is degenerate over the ROI when segment_gridmap is run
     * on the same obstacle-inclusive grid we segment (it becomes a single constant).  A cue
     * that is uniform over the whole free space carries no boundary information, so using it
     * would merge every region into one room.  Detect that and disable the cue.
     *
     * The caller only distinguishes "<=1 distinct value" from "more than one", so stop as
     * soon as a second distinct value is seen.  (A fixed-size table of every label in the ROI
     * could overflow here on wall-piece-rich maps, which silently disabled the cue.) */
    int32_t first_label = 0;
    int distinct_labels = 0;
    for (int y = PAD; y < H - PAD && distinct_labels < 2; y++)
        for (int x = PAD; x < W - PAD; x++) {
            int i = cell_index(x, y);
            if (post_grid[i]) continue;
            int32_t label_value = label_field[i];
            if (distinct_labels == 0) { first_label = label_value; distinct_labels = 1; }
            else if (label_value != first_label) { distinct_labels = 2; break; }
        }
    return distinct_labels;   /* 0/1 distinct value => uninformative */
}

static int region_slots;
static long *region_area;
static int *region_peak;             /* region_peak[r] = max distance-transform value inside region r */
static void region_stats(void) {
    region_slots = core_count + 2;
    region_area = (long *)calloc(region_slots + 1, sizeof(long));
    region_peak = (int *)calloc(region_slots + 1, sizeof(int));
    for (size_t i = 0; i < (size_t)W * H; i++) {
        int region_id = label_map[i];
        if (region_id && region_id < region_slots) {
            region_area[region_id]++;
            if (dist_map[i] > region_peak[region_id]) region_peak[region_id] = dist_map[i];
        }
    }
}
static void merge_region(int from, int to) {
    for (size_t i = 0; i < (size_t)W * H; i++) if (label_map[i] == from) label_map[i] = to;
    region_area[to] += region_area[from];
    region_area[from] = 0;
    if (region_peak[from] > region_peak[to]) region_peak[to] = region_peak[from];
    region_peak[from] = 0;
}

#define MAXPAIR 8192
/* Model-cue merge.  For each pair of adjacent free-space regions, collect the distinct model
 * labels (raw_labels/100 of the nearest labelled wall cell) carried by the shared boundary.  Then
 * decide, per pair:
 *   cue_mode 2 ("uniform") : merge if 0 < nlab <= model_merge_tol   (boundary is model-uniform)
 *   cue_mode 1 ("door")    : merge unless nlab == 2   (tuned default)
 *   cue_mode 3 ("stub")    : as "door", but a 2-label boundary is also merged when the *shorter*
 *                            flanking wall piece is a stub (<= stub_piece cells) and the smaller
 *                            region behind it is a fragment (area <= stub_area cells).  This
 *                            absorbs the L-shaped room's internal necks (short wall spurs) without
 *                            joining two real rooms (a doorway is punched through substantial walls
 *                            and has sizeable rooms on both sides).
 * A boundary carrying exactly two model labels is the model's signature of a *doorway* between
 * two wall pieces; the "door" mode keeps it, "stub" mode refines it.
 * One merge per pass, larger region absorbs the smaller (mirrors the nav post-processing). */
static void model_merge_pass(void) {
    if (!label_field) return;
    int changed = 1, pass = 0;
    while (changed && pass < 256) {
        changed = 0; pass++;
        int pair_lo[MAXPAIR], pair_hi[MAXPAIR], pair_contacts[MAXPAIR], pair_labels[MAXPAIR];
        static int32_t pair_seen[MAXPAIR][8];
        int pair_count = 0;
        for (int y = PAD; y < H - PAD; y++)
            for (int x = PAD; x < W - PAD; x++) {
                int i = cell_index(x, y);
                int region_a = label_map[i];
                if (!region_a) continue;
                for (int k = 0; k < 2; k++) {   /* right and down to avoid duplicates */
                    int nx = x + (k == 0 ? 1 : 0), ny = y + (k == 1 ? 1 : 0);
                    if (!inside_roi(nx, ny)) continue;
                    int j = cell_index(nx, ny);
                    int region_b = label_map[j];
                    if (!region_b || region_b == region_a) continue;
                    int low_label = region_a < region_b ? region_a : region_b, high_label = region_a < region_b ? region_b : region_a;
                    int pair_index = -1;
                    for (int scan = 0; scan < pair_count; scan++) if (pair_lo[scan] == low_label && pair_hi[scan] == high_label) { pair_index = scan; break; }
                    if (pair_index < 0) {
                        if (pair_count >= MAXPAIR) continue;
                        pair_index = pair_count++; pair_lo[pair_index] = low_label; pair_hi[pair_index] = high_label; pair_contacts[pair_index] = 0; pair_labels[pair_index] = 0;
                        for (int slot = 0; slot < 8; slot++) pair_seen[pair_index][slot] = 0;
                    }
                    pair_contacts[pair_index]++;
                    int32_t cand[2] = { label_field[i], label_field[j] };
                    for (int slot = 0; slot < 2; slot++) {
                        int32_t label_value = cand[slot];
                        if (!label_value) continue;
                        /* Saturate at 8 distinct labels: pair_labels < 0 marks the pair as
                         * "too many labels to be a doorway".  Must skip here, otherwise the
                         * scan below reads -1 iterations (none), `!found` is true and
                         * `-1 < 8` passes, so the next new label writes pair_seen[i][-1]
                         * (out of bounds) and resets the count to 0. */
                        if (pair_labels[pair_index] < 0) continue;
                        int found = 0;
                        for (int scan = 0; scan < pair_labels[pair_index]; scan++) if (pair_seen[pair_index][scan] == label_value) { found = 1; break; }
                        if (!found) { if (pair_labels[pair_index] < 8) pair_seen[pair_index][pair_labels[pair_index]++] = label_value; else pair_labels[pair_index] = -1; }
                    }
                }
            }
        for (int pair_index = 0; pair_index < pair_count && !changed; pair_index++) {
            int label_count = pair_labels[pair_index];
            if (label_count < 0) continue;
            int region_a = pair_lo[pair_index], region_b = pair_hi[pair_index];
            if (!region_area[region_a] || !region_area[region_b]) continue;
            int ok = 0;
            if (cue_mode == 2) {
                ok = (label_count > 0 && label_count <= model_merge_tol);
            } else if (cue_mode == 3) {
                if (label_count != 2) {
                    ok = 1;
                } else {
                    /* fragment absorption: short flanking wall piece + small region behind it */
                    long min_area = region_area[region_a] < region_area[region_b] ? region_area[region_a] : region_area[region_b];
                    int piece0 = (pair_seen[pair_index][0] > 0 && pair_seen[pair_index][0] <= max_wall_label) ? wall_piece_cells[pair_seen[pair_index][0]] : 0;
                    int piece1 = (pair_seen[pair_index][1] > 0 && pair_seen[pair_index][1] <= max_wall_label) ? wall_piece_cells[pair_seen[pair_index][1]] : 0;
                    int min_piece = piece0 < piece1 ? piece0 : piece1;
                    ok = (min_area <= stub_area && min_piece <= stub_piece);
                }
            } else {
                ok = (label_count != 2);
            }
            if (!ok) continue;
            int to = region_area[region_a] >= region_area[region_b] ? region_a : region_b;
            int from = (to == region_a) ? region_b : region_a;
            woutf("[v1seg] model-merge %d -> %d (nlab=%d contact=%d amin=%ld minp=%d)\n",
                  from, to, label_count, pair_contacts[pair_index], (long)(region_area[region_a] < region_area[region_b] ? region_area[region_a] : region_area[region_b]),
                  (label_count == 2 && pair_seen[pair_index][0] > 0 && pair_seen[pair_index][0] <= max_wall_label && pair_seen[pair_index][1] > 0 && pair_seen[pair_index][1] <= max_wall_label)
                      ? (wall_piece_cells[pair_seen[pair_index][0]] < wall_piece_cells[pair_seen[pair_index][1]] ? wall_piece_cells[pair_seen[pair_index][0]] : wall_piece_cells[pair_seen[pair_index][1]]) : -1);
            merge_region(from, to);
            changed = 1;
        }
    }
}

static void area_filter(long minArea) {
    while (1) {
        int best_region = 0; long best_area = 0;
        for (int region_index = 1; region_index <= core_count; region_index++)
            if (region_area[region_index] > 0 && region_area[region_index] < minArea && (best_region == 0 || region_area[region_index] < best_area)) {
                best_region = region_index; best_area = region_area[region_index];
            }
        if (!best_region) break;
        int neighbor_ids[8192]; long neighbor_contacts[8192]; int neighbor_count = 0;
        for (int y = PAD; y < H - PAD; y++)
            for (int x = PAD; x < W - PAD; x++) {
                int i = cell_index(x, y);
                if (label_map[i] != best_region) continue;
                for (int k = 0; k < 4; k++) {
                    int nx = x + DX4[k], ny = y + DY4[k];
                    if (!inside_roi(nx, ny)) continue;
                    int j = cell_index(nx, ny);
                    int neighbor_label = label_map[j];
                    if (neighbor_label == best_region || neighbor_label == 0) continue;
                    int neighbor_index = -1;
                    for (int scan = 0; scan < neighbor_count; scan++) if (neighbor_ids[scan] == neighbor_label) { neighbor_index = scan; break; }
                    if (neighbor_index < 0) { if (neighbor_count < 8192) { neighbor_ids[neighbor_count] = neighbor_label; neighbor_contacts[neighbor_count] = 0; neighbor_index = neighbor_count++; } else continue; }
                    neighbor_contacts[neighbor_index]++;
                }
            }
        int target_region = 0; long target_contacts = -1;
        for (int scan = 0; scan < neighbor_count; scan++) if (neighbor_contacts[scan] > target_contacts) { target_contacts = neighbor_contacts[scan]; target_region = neighbor_ids[scan]; }
        if (!target_region) {
            for (size_t i = 0; i < (size_t)W * H; i++) if (label_map[i] == best_region) label_map[i] = 0;
            region_area[best_region] = 0;
            woutf("[v1seg] area-filter: drop isolated region %d\n", best_region);
        } else {
            woutf("[v1seg] area-filter: merge %d (area %ld) -> %d\n", best_region, region_area[best_region], target_region);
            merge_region(best_region, target_region);
        }
    }
}

static int cmp_region_desc(const void *lhs, const void *rhs) {
    long area_lhs = region_area[*(const int *)lhs], area_rhs = region_area[*(const int *)rhs];
    if (area_lhs != area_rhs) return area_lhs < area_rhs ? 1 : -1;
    return *(const int *)lhs - *(const int *)rhs;
}

/* The RRMap IMAGE pixel stores the segment id in 5 bits: Valetudo's
 * RoborockMapParser.PARSE_IMAGE_BLOCK does `segmentId = (val & 0b11111000) >> 3`
 * (1..31; see TECHNICAL.md §2.3).  The S5 reserves 16..31 for user-defined
 * ids (SegmentMap::GetNextUserDefineSegmentID; TECHNICAL.md §2.6), so
 * automatic room ids must stay in 1..15.  `region_order` is area-descending.
 *
 * If watershed oversegments a map past that cap, merge the smallest surplus
 * region into the adjacent region it shares the most boundary with (preferring a
 * region that is itself kept) so its floor keeps a valid id instead of silently
 * becoming 0.  A surplus region with no neighbour at all cannot be merged and is
 * dropped (its cells become unsegmented floor). */
#define AUTO_ROOM_ID_MAX 15
static void limit_auto_rooms(int *region_order, int *order_count) {
    long *contact = (long *)calloc((size_t)region_slots + 1, sizeof(long));
    while (*order_count > AUTO_ROOM_ID_MAX && contact) {
        int victim = region_order[*order_count - 1];      /* smallest region */
        memset(contact, 0, ((size_t)region_slots + 1) * sizeof(long));
        for (int y = PAD; y < H - PAD; y++)
            for (int x = PAD; x < W - PAD; x++) {
                int i = cell_index(x, y);
                if (label_map[i] != victim) continue;
                for (int k = 0; k < 4; k++) {
                    int nx = x + DX4[k], ny = y + DY4[k];
                    if (!inside_roi(nx, ny)) continue;
                    int neighbour = label_map[cell_index(nx, ny)];
                    if (neighbour && neighbour != victim && neighbour < region_slots) contact[neighbour]++;
                }
            }
        int best = 0, best_retained = 0;
        long best_contacts = -1;
        for (int i = 0; i < *order_count; i++) {
            int candidate = region_order[i];
            if (!contact[candidate]) continue;
            int retained = i < AUTO_ROOM_ID_MAX;
            if (best == 0 || (retained && !best_retained) ||
                (retained == best_retained && contact[candidate] > best_contacts) ||
                (retained == best_retained && contact[candidate] == best_contacts &&
                 region_area[candidate] > region_area[best])) {
                best = candidate; best_retained = retained; best_contacts = contact[candidate];
            }
        }
        if (best) {
            woutf("[v1seg] auto-id cap: merge region %d (area %ld) -> %d\n",
                  victim, region_area[victim], best);
            merge_region(victim, best);
        } else {
            woutf("[v1seg] auto-id cap: drop isolated region %d (area %ld)\n",
                  victim, region_area[victim]);
            for (size_t i = 0; i < (size_t)W * H; i++) if (label_map[i] == victim) label_map[i] = 0;
            region_area[victim] = 0;
        }
        (*order_count)--;
        qsort(region_order, *order_count, sizeof(int), cmp_region_desc);
    }
    free(contact);
}

/* ===================================================================================== *
 *  input readers                                                                        *
 * ===================================================================================== */
/* Sanity caps for an input grid/map.  The shared grid is 1024x1024 and real V1/S5
 * maps are smaller still, so anything larger is a corrupt or hostile header.
 * All size arithmetic below is done in uint64_t: on 32-bit ARM `(size_t)w * h`
 * wraps for large w,h (`w=65536,h=65537` -> 65536), which let an undersized buffer
 * pass a size check and index far past it.  Mirrors the 64-bit + cap +
 * exact-size pattern in segedit.c and guard.c. */
#define MAX_GRID_DIM   4096
#define MAX_GRID_CELLS ((uint64_t)1 << 26)

static int read_file(const char *path, unsigned char **out, size_t *len) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) { woutf("[v1seg] open(%s) failed: %s\n", path, strerror(errno)); return 0; }
    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size <= 0) { woutf("[v1seg] bad size %s\n", path); close(fd); return 0; }
    size_t size = (size_t)st.st_size;
    unsigned char *buf = (unsigned char *)malloc(size);
    size_t received = 0;
    while (received < size) {
        ssize_t nread = read(fd, buf + received, size - received);
        if (nread <= 0) { woutf("[v1seg] read(%s) short\n", path); free(buf); close(fd); return 0; }
        received += (size_t)nread;
    }
    close(fd);
    *out = buf; *len = size;
    return 1;
}

/* grid.bin: int32 w,h + w*h bytes.  Fills *seg (for segment_gridmap) and *post (navpost). */
static int load_grid_bin(const char *path, unsigned char **seg, unsigned char **post,
                         int *pw, int *ph, float *scale, int *ox, int *oy) {
    unsigned char *raw; size_t size;
    if (!read_file(path, &raw, &size)) return 0;
    if (size < 8) { wout("[v1seg] grid.bin too short\n"); free(raw); return 0; }
    int w = get_i32(raw), h = get_i32(raw + 4);
    uint64_t cells = (w > 0 && h > 0) ? (uint64_t)w * (uint64_t)h : 0;
    if (w <= 0 || h <= 0 || w > MAX_GRID_DIM || h > MAX_GRID_DIM ||
        cells == 0 || cells > MAX_GRID_CELLS || (uint64_t)8 + cells != (uint64_t)size) {
        wout("[v1seg] bad grid.bin dims\n"); free(raw); return 0;
    }
    unsigned char *grid = (unsigned char *)malloc((size_t)cells);
    memcpy(grid, raw + 8, (size_t)cells);
    free(raw);
    *seg = grid; *post = grid; *pw = w; *ph = h; *scale = 20.0f; *ox = 0; *oy = 0;
    woutf("[v1seg] loaded grid.bin %dx%d\n", w, h);
    return 1;
}

/* Read a bare occupancy grid (int32 w,h + w*h bytes) into a fresh buffer.
 * Used for the optional model grid (the wall-only grid segment_gridmap wants). */
static int read_bare_grid(const char *path, unsigned char **out, int *pw, int *ph) {
    unsigned char *raw; size_t size;
    if (!read_file(path, &raw, &size)) return 0;
    if (size < 8) { wout("[v1seg] model grid too short\n"); free(raw); return 0; }
    int w = get_i32(raw), h = get_i32(raw + 4);
    uint64_t cells = (w > 0 && h > 0) ? (uint64_t)w * (uint64_t)h : 0;
    if (w <= 0 || h <= 0 || w > MAX_GRID_DIM || h > MAX_GRID_DIM ||
        cells == 0 || cells > MAX_GRID_CELLS || (uint64_t)8 + cells != (uint64_t)size) {
        wout("[v1seg] bad model grid dims\n"); free(raw); return 0;
    }
    unsigned char *grid = (unsigned char *)malloc((size_t)cells);
    memcpy(grid, raw + 8, (size_t)cells);
    free(raw);
    *out = grid; *pw = w; *ph = h;
    return 1;
}

/* V1 SLAM map: 56 byte header + w*h float32 log-odds.
 *   +20 float scale (cells per metre; 20.0 = 5 cm cells), +24/+28 int origin, +32/+36 uint w,h.
 * Occupied = log-odds > 1 ; known-free = log-odds < -1 ; 0 = unknown.
 * Returns two padded grids: seg (obstacle = occupied only -> meaningful model cue) and
 * post (obstacle = occupied OR unknown -> the nav's "known floor" free space). */
static int load_lastmap(const char *path, unsigned char **seg, unsigned char **post,
                        int *pw, int *ph, float *scale, int *ox, int *oy) {
    unsigned char *raw; size_t size;
    if (!read_file(path, &raw, &size)) return 0;
    if (size < 56) { wout("[v1seg] last_map too short\n"); free(raw); return 0; }
    float scale_f = get_f32(raw + 20);
    int32_t origin_x = get_i32(raw + 24), origin_y = get_i32(raw + 28);
    uint32_t w = get_u32(raw + 32), h = get_u32(raw + 36);
    uint64_t cells64 = (uint64_t)w * (uint64_t)h;
    /* exact size: a complete V1 map is 56 + w*h*4 bytes (mirrors guard.c).  The
     * 64-bit math keeps a large w,h from wrapping size_t on 32-bit ARM. */
    if (w == 0 || h == 0 || w > MAX_GRID_DIM || h > MAX_GRID_DIM ||
        cells64 > MAX_GRID_CELLS || (uint64_t)56 + cells64 * 4ull != (uint64_t)size) {
        woutf("[v1seg] bad last_map dims %ux%u (file %zu)\n", w, h, size); free(raw); return 0;
    }
    int pad = PAD;
    int padded_w = (int)w + 2 * pad, padded_h = (int)h + 2 * pad;
    unsigned char *seg_grid = (unsigned char *)calloc((size_t)padded_w * padded_h, 1);            /* border free */
    unsigned char *post_grid_local = (unsigned char *)malloc((size_t)padded_w * padded_h);
    memset(post_grid_local, 0xFF, (size_t)padded_w * padded_h);                                          /* border obstacle */
    const unsigned char *cell_bytes = raw + 56;
    long occupied_count = 0, free_count = 0, unknown_count = 0;
    for (uint32_t y = 0; y < h; y++)
        for (uint32_t x = 0; x < w; x++) {
            float log_odds = get_f32(cell_bytes + ((size_t)y * w + x) * 4);
            int occupied = (log_odds > 1.0f);
            int known_free = (log_odds < -1.0f);
            if (occupied) occupied_count++; else if (known_free) free_count++; else unknown_count++;
            size_t i = (size_t)(y + pad) * padded_w + (x + pad);
            seg_grid[i] = occupied ? 0xFF : 0;                    /* segment_gridmap sees walls only */
            post_grid_local[i] = (occupied || !known_free) ? 0xFF : 0;    /* navpost free space = known floor */
        }
    free(raw);
    *seg = seg_grid; *post = post_grid_local; *pw = padded_w; *ph = padded_h;
    *scale = scale_f; *ox = origin_x; *oy = origin_y;
    /* last_map carries no timestamp/map_id; leave whatever the caller set (V1SEG_TIMESTAMP /
     * V1SEG_MAP_ID, defaulting to 0).  Do NOT clobber them here. */
    woutf("[v1seg] loaded last_map %ux%u scale=%.3f origin=(%d,%d) occ=%ld free=%ld unk=%ld\n",
          w, h, scale_f, origin_x, origin_y, occupied_count, free_count, unknown_count);
    return 1;
}

/* ===================================================================================== *
 *  last_segmap writer (atomic .tmp + rename, LE, crc32)                                 *
 * ===================================================================================== */
/* 64-byte header:
 *   0  magic[8] "RRSEG\0\0\1"
 *   8  u32 version
 *   12 u32 map_timestamp       (frame link; 0 = unknown)
 *   16 u32 map_id
 *   20 f32 scale
 *   24 i32 origin_x
 *   28 i32 origin_y
 *   32 u32 width
 *   36 u32 height
 *   40 i32 dock_x
 *   44 i32 dock_y
 *   48 u32 crc32               (over header with crc zeroed + payload)
 *   52 u32 reserved[3]
 * payload:
 *   u8  seg_cells[width*height]     room id per cell (0 = none; 1..15 auto, 16..31 user)
 *   u32 room_count
 *   room_count x { u8 id; u8 pad[3]; u32 area_cells; i32 cx; i32 cy; }
 */
static int write_segmap(const char *path, int w, int h, const unsigned char *cells,
                        const int *room_ids, const long *room_area, const int *room_cx,
                        const int *room_cy, int room_count, float scale, int ox, int oy,
                        unsigned ts, unsigned map_id) {
    size_t cell_count = (size_t)w * h;
    size_t payload_size = cell_count + 4 + (size_t)room_count * 16;
    unsigned char *payload_buf = (unsigned char *)malloc(payload_size);
    unsigned char *cursor = payload_buf;
    memcpy(cursor, cells, cell_count); cursor += cell_count;
    put_u32(cursor, (uint32_t)room_count); cursor += 4;
    for (int room_index = 0; room_index < room_count; room_index++) {
        cursor[0] = (unsigned char)(room_ids[room_index] & 0xff); cursor[1] = cursor[2] = cursor[3] = 0;
        put_u32(cursor + 4, (uint32_t)room_area[room_index]);
        put_u32(cursor + 8, (uint32_t)room_cx[room_index]);
        put_u32(cursor + 12, (uint32_t)room_cy[room_index]);
        cursor += 16;
    }

    unsigned char header[64];
    memset(header, 0, sizeof header);
    const char magic[8] = { 'R', 'R', 'S', 'E', 'G', 0, 0, 1 };
    memcpy(header, magic, 8);
    put_u32(header + 8, 1u);                       /* version */
    put_u32(header + 12, ts);
    put_u32(header + 16, map_id);
    put_f32(header + 20, scale);
    put_u32(header + 24, (uint32_t)ox);
    put_u32(header + 28, (uint32_t)oy);
    put_u32(header + 32, (uint32_t)w);
    put_u32(header + 36, (uint32_t)h);
    put_u32(header + 40, 0);                       /* dock_x */
    put_u32(header + 44, 0);                       /* dock_y */
    put_u32(header + 48, 0);                       /* crc placeholder */
    put_u32(header + 52, 0); put_u32(header + 56, 0); put_u32(header + 60, 0);

    uint32_t crc = crc32_buf(header, sizeof header, 0);
    crc = crc32_buf(payload_buf, payload_size, crc);
    put_u32(header + 48, crc);

    char tmp_path[4096];
    snprintf(tmp_path, sizeof tmp_path, "%s.tmp", path);
    int fd = open(tmp_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) { woutf("[v1seg] open(%s) failed: %s\n", tmp_path, strerror(errno)); free(payload_buf); return 0; }
    int ok = 1;
    size_t offset = 0; const unsigned char *chunks[2] = { header, payload_buf }; size_t lens[2] = { sizeof header, payload_size };
    for (int chunk = 0; chunk < 2; chunk++) {
        offset = 0;
        while (offset < lens[chunk]) {
            ssize_t nwritten = write(fd, chunks[chunk] + offset, lens[chunk] - offset);
            if (nwritten <= 0) { woutf("[v1seg] write(%s) failed\n", tmp_path); ok = 0; break; }
            offset += (size_t)nwritten;
        }
        if (!ok) break;
    }
    if (ok && fsync(fd) != 0) { ok = 0; }
    close(fd);
    if (ok) {
        if (rename(tmp_path, path) != 0) { woutf("[v1seg] rename failed: %s\n", strerror(errno)); ok = 0; }
    }
    if (!ok) unlink(tmp_path);
    free(payload_buf);
    if (ok) woutf("[v1seg] wrote %s (%zu bytes, %d rooms, crc32=%08x)\n",
                  path, sizeof header + payload_size, room_count, crc);
    return ok;
}

/* ===================================================================================== *
 *  main                                                                                 *
 * ===================================================================================== */
int main(int argc, char **argv) {
    crc_init();
    wout("[v1seg] ON-V1 segment producer\n");

    const char *gridpath = 0, *lastmap = 0, *outpath = 0, *modelgrid = 0;
    long minArea = 400;
    int doorRadius = 7, minCore = 20;
    int cue_given = 0;   /* set by --cue / --no-cue / V1SEG_CUE: caller chose explicitly */
    unsigned ts = 0, map_id = 0;

    const char *env;
    if ((env = getenv("V1SEG_DOOR_RADIUS"))) doorRadius = atoi(env);
    if ((env = getenv("V1SEG_MIN_CORE")))    minCore = atoi(env);
    if ((env = getenv("V1SEG_PAD")))         PAD = atoi(env);
    if ((env = getenv("V1SEG_TIMESTAMP")))   ts = (unsigned)strtoul(env, 0, 0);
    if ((env = getenv("V1SEG_MAP_ID")))      map_id = (unsigned)strtoul(env, 0, 0);
    if ((env = getenv("V1SEG_NO_CUE")))      { cue_mode = atoi(env) ? 0 : 3; cue_given = 1; }
    if ((env = getenv("V1SEG_CUE"))) {
        if (!strcmp(env, "off")) cue_mode = 0;
        else if (!strcmp(env, "uniform")) cue_mode = 2;
        else if (!strcmp(env, "door")) cue_mode = 1;
        else if (!strcmp(env, "stub")) cue_mode = 3;
        cue_given = 1;
    }
    if ((env = getenv("V1SEG_MERGE_TOL")))   model_merge_tol = atoi(env);
    if ((env = getenv("V1SEG_STUB_PIECE")))  stub_piece = atoi(env);
    if ((env = getenv("V1SEG_STUB_AREA")))   stub_area = atol(env);
    if ((env = getenv("V1SEG_AUTO_DOOR")))   auto_door = atoi(env);
    if ((env = getenv("V1SEG_MODEL_GRID")))  modelgrid = env;

    /* options may appear anywhere; positional = <grid.bin> <out_segmap> [minArea] */
    int positional_count = 0; const char *positional[4] = { 0, 0, 0, 0 };
    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        if (!strcmp(arg, "--lastmap") && i + 1 < argc) { lastmap = argv[++i]; continue; }
        if (!strcmp(arg, "--model-grid") && i + 1 < argc) { modelgrid = argv[++i]; continue; }
        if (!strcmp(arg, "--door-radius") && i + 1 < argc) {
            const char *value = argv[++i];
            if (!strcmp(value, "auto")) auto_door = 1; else doorRadius = atoi(value);
            continue;
        }
        if (!strcmp(arg, "--min-core") && i + 1 < argc) { minCore = atoi(argv[++i]); continue; }
        if (!strcmp(arg, "--pad") && i + 1 < argc) { PAD = atoi(argv[++i]); continue; }
        if (!strcmp(arg, "--merge-tol") && i + 1 < argc) { model_merge_tol = atoi(argv[++i]); continue; }
        if (!strcmp(arg, "--stub-piece") && i + 1 < argc) { stub_piece = atoi(argv[++i]); continue; }
        if (!strcmp(arg, "--stub-area") && i + 1 < argc) { stub_area = atol(argv[++i]); continue; }
        if (!strcmp(arg, "--cue") && i + 1 < argc) {
            const char *value = argv[++i];
            if (!strcmp(value, "off")) cue_mode = 0;
            else if (!strcmp(value, "uniform")) cue_mode = 2;
            else if (!strcmp(value, "door")) cue_mode = 1;
            else if (!strcmp(value, "stub")) cue_mode = 3;
            cue_given = 1;
            continue;
        }
        if (!strcmp(arg, "--no-cue")) { cue_mode = 0; cue_given = 1; continue; }
        if (arg[0] == '-') { woutf("[v1seg] unknown option %s\n", arg); return 2; }
        if (positional_count < 4) positional[positional_count++] = arg;
    }
    /* A negative pad makes inside_roi() accept negative indices and the loops start
     * below 0 -> out-of-bounds cell_index() writes.  Reject instead of zipping
     * through (and bound the top end so w + 2*pad cannot overflow int). */
    if (PAD < 0 || PAD > MAX_GRID_DIM) {
        woutf("[v1seg] invalid pad %d (must be 0..%d)\n", PAD, MAX_GRID_DIM);
        return 2;
    }
    /* The "stub" cue default was tuned empirically.  On a real V1 `last_map` the raw
     * watershed ("off") segments better, so default the --lastmap path to off unless the
     * caller asked for a specific cue (via --cue/--no-cue flag or the V1SEG_CUE env). */
    if (lastmap && !cue_given) cue_mode = 0;
    if (lastmap) {
        outpath = positional[0];
        if (positional_count >= 2) minArea = atol(positional[1]);
    } else {
        gridpath = positional[0]; outpath = positional[1];
        if (positional_count >= 3) minArea = atol(positional[2]);
    }
    if (!outpath) {
        wout("usage: v1seg <grid.bin|--lastmap <path>> <out_segmap> [minArea]\n"
             "            [--model-grid WALL.bin] [--cue off|door|uniform|stub] [--merge-tol N]\n"
             "            [--stub-piece N] [--stub-area N] [--door-radius N|auto] [--min-core N] [--pad N]\n");
        return 2;
    }

    const char *model = getenv("ROOMSEG_MODEL");
    if (!model) model = "/opt/rockrobo/cleaner/conf/roomseg.mdl";

    unsigned char *model_grid_buf = 0, *post_grid_buf = 0;
    float scale = 20.0f; int ox = 0, oy = 0;
    if (lastmap) {
        if (!load_lastmap(lastmap, &model_grid_buf, &post_grid_buf, &W, &H, &scale, &ox, &oy)) return 1;
    } else {
        if (!load_grid_bin(gridpath, &model_grid_buf, &post_grid_buf, &W, &H, &scale, &ox, &oy)) return 1;
        if (modelgrid) {
            int model_w = 0, model_h = 0; unsigned char *model_grid = 0;
            if (!read_bare_grid(modelgrid, &model_grid, &model_w, &model_h)) return 1;
            if (model_w != W || model_h != H) {
                woutf("[v1seg] model grid %dx%d != nav grid %dx%d\n", model_w, model_h, W, H);
                free(model_grid); return 1;
            }
            model_grid_buf = model_grid;                       /* seg != post: model grid is separate */
            woutf("[v1seg] model grid %s loaded (wall-only)\n", modelgrid);
        }
    }
    post_grid = post_grid_buf;

    /* ---- (a) raw model output (from the model grid) ---- */
    int32_t *raw_out = (int32_t *)calloc((size_t)W * H, 4);
    int seg_ok = call_segment_gridmap(model, model_grid_buf, W, H, raw_out);
    if (model_grid_buf && model_grid_buf != post_grid_buf) free(model_grid_buf);
    if (!seg_ok) { free(raw_out); raw_out = 0; }
    raw_labels = raw_out;

    /* ---- (b) nav post-processing ---- */
    distance_transform();
    find_cores(doorRadius, minCore);
    if (auto_door && core_count >= AUTO_CORE_THRESHOLD) {
        /* many-room map: the base radius fragments; step up once so real doorways are cut. */
        doorRadius = AUTO_DOOR_BUMP;
        find_cores(doorRadius, minCore);
        woutf("[v1seg] auto-door: raised doorRadius to %d (cores=%d)\n", doorRadius, core_count);
    }
    int cue_labels = build_labfield();
    int cue_on = (cue_mode != 0) && cue_labels >= 2;
    woutf("[v1seg] cores=%d cue_labels=%d cue=%s\n", core_count, cue_labels, cue_name(cue_on));
    watershed(0);
    region_stats();
    if (cue_on) model_merge_pass();
    area_filter(minArea);

    int *region_order = (int *)malloc(((size_t)core_count + 1) * sizeof(int));
    int order_count = 0;
    for (int r = 1; r <= core_count; r++) if (region_area[r] > 0) region_order[order_count++] = r;
    qsort(region_order, order_count, sizeof(int), cmp_region_desc);

    /* Keep automatic ids inside the 5-bit RRMap field's auto range (1..15); merge
     * or drop the surplus so no id can wrap when stamped into a map pixel. */
    limit_auto_rooms(region_order, &order_count);

    int room_count = order_count; if (room_count > AUTO_ROOM_ID_MAX) room_count = AUTO_ROOM_ID_MAX;
    unsigned char *out_cells = (unsigned char *)calloc((size_t)W * H, 1);
    int *remap = (int *)calloc((size_t)core_count + 1, sizeof(int));
    for (int i = 0; i < room_count; i++) remap[region_order[i]] = i + 1;
    for (size_t i = 0; i < (size_t)W * H; i++) {
        int region_label = label_map[i];
        /* ids are 1..15 (auto) so they fit the 5-bit field unmodified -- no masking */
        if (region_label && remap[region_label]) out_cells[i] = (unsigned char)remap[region_label];
    }

    /* room table: area + centroid */
    int *room_ids = (int *)calloc(room_count + 1, sizeof(int));
    long *room_areas = (long *)calloc(room_count + 1, sizeof(long));
    double *sum_x = (double *)calloc(room_count + 1, sizeof(double));
    double *sum_y = (double *)calloc(room_count + 1, sizeof(double));
    for (int i = 0; i < room_count; i++) { room_ids[i] = i + 1; room_areas[i] = region_area[region_order[i]]; }
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            unsigned char cell_id = out_cells[(size_t)y * W + x];
            if (cell_id >= 1 && cell_id <= room_count) { sum_x[cell_id - 1] += x; sum_y[cell_id - 1] += y; }
        }
    int *room_cx = (int *)calloc(room_count + 1, sizeof(int));
    int *room_cy = (int *)calloc(room_count + 1, sizeof(int));
    for (int i = 0; i < room_count; i++) {
        room_cx[i] = room_areas[i] ? (int)(sum_x[i] / room_areas[i]) : 0;
        room_cy[i] = room_areas[i] ? (int)(sum_y[i] / room_areas[i]) : 0;
    }

    woutf("[v1seg] RESULT w=%d h=%d cores=%d rooms=%d minArea=%ld doorRadius=%d cue=%d(%s) tol=%d auto=%d\n",
          W, H, core_count, room_count, minArea, doorRadius, cue_on, cue_name(cue_on), model_merge_tol, auto_door);
    wout("[v1seg] areas:");
    for (int i = 0; i < room_count; i++) woutf(" %ld", room_areas[i]);
    wout("\n");

    /* ---- (c) persist sidecar ---- */
    if (!write_segmap(outpath, W, H, out_cells, room_ids, room_areas, room_cx, room_cy, room_count,
                      scale, ox, oy, ts, map_id)) {
        _exit(1);
    }
    _exit(0);
}
