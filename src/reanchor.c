/* reanchor.c -- occupancy-grid registration / frame re-anchoring for the V1.
 * See reanchor.h for the design.  Pure libc; builds for host and V1 (arm-linux-gcc).
 *
 * Logging goes through raw write() because stdio flushing is broken under
 * the V1's glibc 2.19 (see src/v1seg.cpp).
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "reanchor.h"
#include "crc32.h"

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* ===================================================================================== *
 *  logging (raw write)                                                                  *
 * ===================================================================================== */
#ifndef REANCHOR_NO_CLI
static void wout(const char *s) { ssize_t nwritten = write(1, s, strlen(s)); (void)nwritten; }
static void woutf(const char *fmt, ...) {
    char buf[1024]; va_list ap; va_start(ap, fmt);
    int length = vsnprintf(buf, sizeof buf, fmt, ap); va_end(ap);
    if (length > 0) { if (length > (int)sizeof buf) length = (int)sizeof buf; ssize_t nwritten = write(1, buf, length); (void)nwritten; }
}
#endif

/* ===================================================================================== *
 *  little-endian helpers + crc32                                                        *
 * ===================================================================================== */
static void put_u32(unsigned char *buf, uint32_t value) {
    buf[0] = (uint8_t)value; buf[1] = (uint8_t)(value >> 8); buf[2] = (uint8_t)(value >> 16); buf[3] = (uint8_t)(value >> 24);
}
static uint32_t get_u32(const unsigned char *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}
static int32_t get_i32(const unsigned char *buf) { return (int32_t)get_u32(buf); }
static float get_f32(const unsigned char *buf) { uint32_t bits = get_u32(buf); float value; memcpy(&value, &bits, 4); return value; }
static void put_f32(unsigned char *buf, float value) { uint32_t bits; memcpy(&bits, &value, 4); put_u32(buf, bits); }

static uint32_t crc_table[256];
static int crc_ready = 0;
static void crc_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc_val = i;
        for (int k = 0; k < 8; k++)
            crc_val = (crc_val & 1) ? (CRC32_POLYNOMIAL_REFLECTED ^ (crc_val >> 1))
                                    : (crc_val >> 1);
        crc_table[i] = crc_val;
    }
    crc_ready = 1;
}
static uint32_t crc32_buf(const void *data, size_t length, uint32_t crc) {
    if (!crc_ready) crc_init();
    const unsigned char *bytes = (const unsigned char *)data;
    crc = crc ^ CRC32_INITIAL_VALUE;
    for (size_t i = 0; i < length; i++) crc = crc_table[(crc ^ bytes[i]) & 0xff] ^ (crc >> 8);
    return crc ^ CRC32_FINAL_XOR;
}

/* ===================================================================================== *
 *  grid / seg lifecycle                                                                 *
 * ===================================================================================== */
void reanchor_grid_free(reanchor_grid_t *g) {
    if (!g) return;
    free(g->free); free(g->obs);
    g->free = g->obs = NULL; g->w = g->h = 0; g->freecount = g->obscount = 0;
}
void reanchor_seg_free(reanchor_seg_t *g) {
    if (!g) return;
    free(g->cells); g->cells = NULL; g->w = g->h = 0;
}

static int grid_recensus(reanchor_grid_t *g) {
    long free_count = 0, obs_count = 0;
    size_t cell_count = (g->w > 0 && g->h > 0) ? (size_t)g->w * (size_t)g->h : 0;
    for (size_t i = 0; i < cell_count; i++) {
        if (g->obs && g->obs[i]) obs_count++;
        else if (g->free && g->free[i]) free_count++;
    }
    g->freecount = free_count; g->obscount = obs_count;
    return 0;
}

/* ===================================================================================== *
 *  file I/O                                                                             *
 * ===================================================================================== */
static int read_whole(const char *path, unsigned char **out, size_t *outn) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size <= 0) { close(fd); return -1; }
    size_t size = (size_t)st.st_size;
    unsigned char *buf = (unsigned char *)malloc(size);
    if (!buf) { close(fd); return -1; }
    size_t offset = 0;
    while (offset < size) {
        ssize_t nread = read(fd, buf + offset, size - offset);
        if (nread <= 0) { free(buf); close(fd); return -1; }
        offset += (size_t)nread;
    }
    close(fd); *out = buf; *outn = size; return 0;
}

static int write_atomic(const char *path, const void *data, size_t n) {
    size_t tmp_size = strlen(path) + sizeof ".tmp";   /* ".tmp" + NUL */
    char *tmp_path = (char *)malloc(tmp_size);
    if (!tmp_path) return -1;
    snprintf(tmp_path, tmp_size, "%s.tmp", path);
    int result = -1;
    int fd = open(tmp_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) goto out;
    const unsigned char *cursor = (const unsigned char *)data;
    size_t offset = 0;
    while (offset < n) {
        ssize_t nwritten = write(fd, cursor + offset, n - offset);
        if (nwritten <= 0) { close(fd); unlink(tmp_path); goto out; }
        offset += (size_t)nwritten;
    }
    if (fsync(fd) != 0) { close(fd); unlink(tmp_path); goto out; }
    close(fd);
    if (rename(tmp_path, path) != 0) { unlink(tmp_path); goto out; }
    result = 0;
out:
    free(tmp_path);
    return result;
}

#define RRG1_HDR 32
/* Sanity caps for grid/map headers.  The shared V1 grid is 1024x1024 and real maps
 * are smaller, so anything larger is a corrupt/hostile header.  All size arithmetic
 * below is done in uint64_t: on 32-bit ARM `(size_t)w * h` wraps for large w,h
 * (w=65536,h=65537 -> 65536), which let an undersized buffer pass a size
 * check and index far past it.  Mirrors segedit.c / guard.c / v1seg.cpp. */
#define REANCHOR_MAX_GRID_DIM   4096
#define REANCHOR_MAX_GRID_CELLS ((uint64_t)1 << 26)
int reanchor_save_grid(const char *path, const reanchor_grid_t *g) {
    if (!g || !g->free || g->w <= 0 || g->h <= 0 ||
        g->w > REANCHOR_MAX_GRID_DIM || g->h > REANCHOR_MAX_GRID_DIM) return -1;
    uint64_t cells64 = (uint64_t)(uint32_t)g->w * (uint64_t)(uint32_t)g->h;
    if (cells64 > REANCHOR_MAX_GRID_CELLS) return -1;
    size_t size = RRG1_HDR + (size_t)cells64;
    unsigned char *buf = (unsigned char *)calloc(1, size);
    if (!buf) return -1;
    memcpy(buf, "RRG1", 4);
    put_u32(buf + 4, 1);
    put_u32(buf + 8, (uint32_t)g->w);
    put_u32(buf + 12, (uint32_t)g->h);
    put_f32(buf + 16, g->scale);
    put_u32(buf + 20, (uint32_t)g->ox);
    put_u32(buf + 24, (uint32_t)g->oy);
    put_u32(buf + 28, 0);
    size_t cell_count = (size_t)cells64;
    for (size_t i = 0; i < cell_count; i++)
        buf[RRG1_HDR + i] = g->obs && g->obs[i] ? 2 : (g->free && g->free[i] ? 1 : 0);
    int rc = write_atomic(path, buf, size);
    free(buf);
    return rc;
}

int reanchor_load_grid_ex(const char *path, float logodds_eps, reanchor_grid_t *g) {
    unsigned char *buf = NULL; size_t size = 0;
    if (read_whole(path, &buf, &size) != 0) return -2;
    memset(g, 0, sizeof *g);
    if (size >= 4 && memcmp(buf, "RRG1", 4) == 0) {
        if (size < RRG1_HDR) { free(buf); return -2; }
        uint32_t w32 = get_u32(buf + 8), h32 = get_u32(buf + 12);
        uint64_t cells64 = (uint64_t)w32 * (uint64_t)h32;
        if (w32 == 0 || h32 == 0 || w32 > REANCHOR_MAX_GRID_DIM || h32 > REANCHOR_MAX_GRID_DIM ||
            cells64 > REANCHOR_MAX_GRID_CELLS || (uint64_t)RRG1_HDR + cells64 != (uint64_t)size) {
            free(buf); return -2;
        }
        int w = (int)w32, h = (int)h32;
        g->w = w; g->h = h;
        g->scale = get_f32(buf + 16);
        g->ox = get_i32(buf + 20); g->oy = get_i32(buf + 24);
        g->free = (uint8_t *)calloc(1, (size_t)cells64);
        g->obs  = (uint8_t *)calloc(1, (size_t)cells64);
        if (!g->free || !g->obs) { free(buf); reanchor_grid_free(g); return -1; }
        uint64_t cell_count = cells64;
        for (uint64_t i = 0; i < cell_count; i++) {
            uint8_t cell = buf[RRG1_HDR + i];
            if (cell == 2) g->obs[i] = 1; else if (cell == 1) g->free[i] = 1;
        }
        free(buf); grid_recensus(g);
        return (g->freecount > 0) ? 0 : -3;
    }
    free(buf);
    return reanchor_load_lastmap(path, logodds_eps, g);
}

int reanchor_load_grid(const char *path, reanchor_grid_t *g) {
    return reanchor_load_grid_ex(path, 1.0f, g);
}

int reanchor_load_lastmap(const char *path, float logodds_eps, reanchor_grid_t *g) {
    unsigned char *buf = NULL; size_t size = 0;
    if (read_whole(path, &buf, &size) != 0) return -2;
    if (size < 56) { free(buf); return -2; }
    uint32_t w32 = get_u32(buf + 32), h32 = get_u32(buf + 36);
    uint64_t cells64 = (uint64_t)w32 * (uint64_t)h32;
    /* a complete V1 map is exactly 56 + w*h*4 bytes (mirrors guard.c) */
    if (w32 == 0 || h32 == 0 || w32 > REANCHOR_MAX_GRID_DIM || h32 > REANCHOR_MAX_GRID_DIM ||
        cells64 > REANCHOR_MAX_GRID_CELLS || (uint64_t)56 + cells64 * 4ull != (uint64_t)size) {
        free(buf); return -2;
    }
    memset(g, 0, sizeof *g);
    int w = (int)w32, h = (int)h32;
    g->w = w; g->h = h;
    g->scale = get_f32(buf + 20);
    g->ox = get_i32(buf + 24); g->oy = get_i32(buf + 28);
    g->free = (uint8_t *)calloc(1, (size_t)cells64);
    g->obs  = (uint8_t *)calloc(1, (size_t)cells64);
    if (!g->free || !g->obs) { free(buf); reanchor_grid_free(g); return -1; }
    uint64_t cell_count = cells64;
    for (uint64_t i = 0; i < cell_count; i++) {
        float log_odds = get_f32(buf + 56 + i * 4);
        if (log_odds > logodds_eps) g->obs[i] = 1;
        else if (log_odds < -logodds_eps) g->free[i] = 1;
    }
    free(buf); grid_recensus(g);
    return (g->freecount > 0) ? 0 : -3;
}

int reanchor_load_segmap(const char *path, int map_w, int map_h,
                         reanchor_seg_t *seg, int *pad_out,
                         float *scale_out, int *ox_out, int *oy_out) {
    unsigned char *buf = NULL; size_t size = 0;
    if (read_whole(path, &buf, &size) != 0) return -2;
    if (size < 64 || memcmp(buf, "RRSEG", 5) != 0) { free(buf); return -2; }
    uint32_t w32 = get_u32(buf + 32), h32 = get_u32(buf + 36);
    uint64_t cells64 = (uint64_t)w32 * (uint64_t)h32;
    if (w32 == 0 || h32 == 0 || w32 > REANCHOR_MAX_GRID_DIM || h32 > REANCHOR_MAX_GRID_DIM ||
        cells64 > REANCHOR_MAX_GRID_CELLS || (uint64_t)size < 64ull + cells64) {
        free(buf); return -2;
    }
    int w = (int)w32, h = (int)h32;
    uint32_t stored_crc = get_u32(buf + 48);
    unsigned char saved_crc[4];
    memcpy(saved_crc, buf + 48, 4);
    put_u32(buf + 48, 0);
    uint32_t computed_crc = crc32_buf(buf, 64, 0);
    computed_crc = crc32_buf(buf + 64, size - 64, computed_crc);
    memcpy(buf + 48, saved_crc, 4);
    if (stored_crc != computed_crc) { free(buf); return -2; }

    memset(seg, 0, sizeof *seg);
    seg->w = w; seg->h = h;
    seg->cells = (uint8_t *)malloc((size_t)cells64);
    if (!seg->cells) { free(buf); return -1; }
    uint64_t cell_count = cells64;
    for (uint64_t i = 0; i < cell_count; i++) seg->cells[i] = (uint8_t)(buf[64 + i] & 0x3f);
    if (scale_out) *scale_out = get_f32(buf + 20);
    if (ox_out) *ox_out = get_i32(buf + 24);
    if (oy_out) *oy_out = get_i32(buf + 28);
    if (pad_out) {
        if (map_w <= 0 || map_h <= 0) {
            *pad_out = 0;   /* bare-grid sidecar: no map to compare against */
        } else {
            int dim_diff_w = w - map_w, dim_diff_h = h - map_h;
            /* A sidecar produced from `map` exceeds it by the same even pad on both
             * axes.  Anything else means the sidecar does not belong to this map:
             * silently falling back to pad=0 shifts every room label. */
            if (dim_diff_w < 0 || dim_diff_h < 0 || dim_diff_w != dim_diff_h || (dim_diff_w % 2) != 0) {
                free(buf); reanchor_seg_free(seg); return -2;
            }
            *pad_out = dim_diff_w / 2;
        }
    }
    free(buf);
    return 0;
}

int reanchor_write_segmap(const char *path, int w, int h, const uint8_t *cells,
                          float scale, int ox, int oy,
                          unsigned ts, unsigned map_id, int dock_x, int dock_y) {
    /* room table */
    long area_cells[64]; long sum_x[64], sum_y[64];
    for (int i = 0; i < 64; i++) { area_cells[i] = 0; sum_x[i] = 0; sum_y[i] = 0; }
    for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
        int id = cells[(long)y * w + x] & 0x3f;
        if (!id) continue;
        area_cells[id]++; sum_x[id] += x; sum_y[id] += y;
    }
    int room_count = 0;
    for (int i = 1; i < 64; i++) if (area_cells[i]) room_count++;

    size_t payload_size = (size_t)w * h + 4 + (size_t)room_count * 16;
    unsigned char *payload_buf = (unsigned char *)malloc(payload_size);
    unsigned char *header = (unsigned char *)calloc(1, 64);
    if (!payload_buf || !header) { free(payload_buf); free(header); return -1; }
    memcpy(payload_buf, cells, (size_t)w * h);
    unsigned char *cursor = payload_buf + (size_t)w * h;
    put_u32(cursor, (uint32_t)room_count); cursor += 4;
    for (int i = 1; i < 64; i++) {
        if (!area_cells[i]) continue;
        cursor[0] = (uint8_t)i; cursor[1] = cursor[2] = cursor[3] = 0;
        put_u32(cursor + 4, (uint32_t)area_cells[i]);
        put_u32(cursor + 8, (uint32_t)(sum_x[i] / area_cells[i]));
        put_u32(cursor + 12, (uint32_t)(sum_y[i] / area_cells[i]));
        cursor += 16;
    }
    const char magic[8] = { 'R', 'R', 'S', 'E', 'G', 0, 0, 1 };
    memcpy(header, magic, 8);
    put_u32(header + 8, 1);
    put_u32(header + 12, ts);
    put_u32(header + 16, map_id);
    put_f32(header + 20, scale);
    put_u32(header + 24, (uint32_t)ox);
    put_u32(header + 28, (uint32_t)oy);
    put_u32(header + 32, (uint32_t)w);
    put_u32(header + 36, (uint32_t)h);
    put_u32(header + 40, (uint32_t)dock_x);
    put_u32(header + 44, (uint32_t)dock_y);
    uint32_t crc = crc32_buf(header, 64, 0);
    crc = crc32_buf(payload_buf, payload_size, crc);
    put_u32(header + 48, crc);

    size_t total = 64 + payload_size;
    unsigned char *crc_stream = (unsigned char *)malloc(total);
    if (!crc_stream) { free(payload_buf); free(header); return -1; }
    memcpy(crc_stream, header, 64);
    memcpy(crc_stream + 64, payload_buf, payload_size);
    int result_code = write_atomic(path, crc_stream, total);
    free(crc_stream); free(payload_buf); free(header);
    return result_code;
}

/* ===================================================================================== *
 *  grid analysis: centroid, boundary edge mask, distance transform                      *
 * ===================================================================================== */
static void free_centroid(const reanchor_grid_t *g, double *cx, double *cy) {
    double sum_x = 0, sum_y = 0; long count = 0;
    for (int y = 0; y < g->h; y++) for (int x = 0; x < g->w; x++)
        if (g->free[(long)y * g->w + x]) { sum_x += x; sum_y += y; count++; }
    if (!count) { *cx = 0; *cy = 0; return; }
    *cx = sum_x / count; *cy = sum_y / count;
}

/* free-region boundary: a free cell with a non-free / out-of-bounds 4-neighbour */
static uint8_t *free_edge(const reanchor_grid_t *g, long *out_n) {
    long cell_count = (long)g->w * g->h;
    uint8_t *edge_mask = (uint8_t *)calloc(1, cell_count);
    long edge_count = 0;
    if (!edge_mask) { *out_n = 0; return NULL; }
    for (int y = 0; y < g->h; y++) for (int x = 0; x < g->w; x++) {
        if (g->obs && g->obs[(long)y * g->w + x]) continue;
        if (!g->free[(long)y * g->w + x]) continue;
        int is_boundary = 0;
        if (x == 0 || x == g->w - 1 || y == 0 || y == g->h - 1) is_boundary = 1;
        else {
            long i = (long)y * g->w + x;
            if (!g->free[i - 1] || !g->free[i + 1] || !g->free[i - g->w] || !g->free[i + g->w]) is_boundary = 1;
        }
        if (is_boundary) { edge_mask[(long)y * g->w + x] = 1; edge_count++; }
    }
    *out_n = edge_count;
    return edge_mask;
}

/* two-pass chamfer distance transform (orthogonal 1, diagonal sqrt2) */
static float *dist_transform(int w, int h, const uint8_t *mask) {
    long cell_count = (long)w * h;
    float *dist = (float *)malloc(sizeof(float) * cell_count);
    if (!dist) return NULL;
    const float INF = 1e9f, D1 = 1.0f, D2 = 1.41421356f;
    for (long i = 0; i < cell_count; i++) dist[i] = mask[i] ? 0.f : INF;
    for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
        long i = (long)y * w + x;
        float best_dist = dist[i];
        if (x > 0 && dist[i - 1] + D1 < best_dist) best_dist = dist[i - 1] + D1;
        if (y > 0) {
            if (dist[i - w] + D1 < best_dist) best_dist = dist[i - w] + D1;
            if (x > 0 && dist[i - w - 1] + D2 < best_dist) best_dist = dist[i - w - 1] + D2;
            if (x < w - 1 && dist[i - w + 1] + D2 < best_dist) best_dist = dist[i - w + 1] + D2;
        }
        dist[i] = best_dist;
    }
    for (int y = h - 1; y >= 0; y--) for (int x = w - 1; x >= 0; x--) {
        long i = (long)y * w + x;
        float best_dist = dist[i];
        if (x < w - 1 && dist[i + 1] + D1 < best_dist) best_dist = dist[i + 1] + D1;
        if (y < h - 1) {
            if (dist[i + w] + D1 < best_dist) best_dist = dist[i + w] + D1;
            if (x < w - 1 && dist[i + w + 1] + D2 < best_dist) best_dist = dist[i + w + 1] + D2;
            if (x > 0 && dist[i + w - 1] + D2 < best_dist) best_dist = dist[i + w - 1] + D2;
        }
        dist[i] = best_dist;
    }
    return dist;
}

/* majority downsample of a mask into a new grid (k x k blocks).  Returns 0 on
 * success, -1 if the destination could not be allocated (dst is left empty). */
static int downsample_grid(const reanchor_grid_t *src, int block, reanchor_grid_t *dst) {
    int coarse_w = (src->w + block - 1) / block, coarse_h = (src->h + block - 1) / block;
    memset(dst, 0, sizeof *dst);
    dst->w = coarse_w; dst->h = coarse_h;
    dst->scale = src->scale; dst->ox = src->ox; dst->oy = src->oy;
    size_t cell_count = (size_t)coarse_w * (size_t)coarse_h;
    dst->free = (uint8_t *)calloc(1, cell_count);
    dst->obs  = (uint8_t *)calloc(1, cell_count);
    if (!dst->free || !dst->obs) { reanchor_grid_free(dst); return -1; }
    for (int cy = 0; cy < coarse_h; cy++) for (int cx = 0; cx < coarse_w; cx++) {
        int free_hits = 0, obs_hits = 0, total = 0;
        for (int dy = 0; dy < block; dy++) for (int dx = 0; dx < block; dx++) {
            int x = cx * block + dx, y = cy * block + dy;
            if (x >= src->w || y >= src->h) continue;
            total++;
            long i = (long)y * src->w + x;
            if (src->obs && src->obs[i]) obs_hits++;
            else if (src->free && src->free[i]) free_hits++;
        }
        long out_index = (long)cy * coarse_w + cx;
        if (total && free_hits * 2 >= total) dst->free[out_index] = 1;
        if (total && obs_hits * 2 >= total) dst->obs[out_index] = 1;
    }
    grid_recensus(dst);
    return 0;
}

/* ===================================================================================== *
 *  transform parameterisation and scoring                                               *
 * ===================================================================================== */
/* params p = { theta_deg, tx, ty [, s] }; t is the residual translation (current
 * cells) after centroid alignment; A = ratio_base * s * R(theta). */
static void params_to_affine(const double *params, int param_count, double ratio_base,
                             double stored_cx, double stored_cy, double current_cx, double current_cy,
                             double *A00, double *A01, double *A10, double *A11,
                             double *bx, double *by) {
    double scale_factor = (param_count >= 4) ? params[3] : 1.0;
    double factor = ratio_base * scale_factor;
    double theta = params[0] * (M_PI / 180.0);
    double cos_th = cos(theta), sin_th = sin(theta);
    *A00 = factor * cos_th; *A01 = -factor * sin_th; *A10 = factor * sin_th; *A11 = factor * cos_th;
    *bx = current_cx + params[1] - (*A00 * stored_cx + *A01 * stored_cy);
    *by = current_cy + params[2] - (*A10 * stored_cx + *A11 * stored_cy);
}

/* free-mask Dice after transforming stored by (A,b).  Iterates stored free cells
 * (subsampled by `stride`) and scatters into the current mask. */
static double dice_eval(const reanchor_grid_t *stored_grid, const reanchor_grid_t *current_grid,
                        double A00, double A01, double A10, double A11,
                        double bx, double by, int stride,
                        long *out_mapped, long *out_overlap, long *out_covered) {
    long overlap_count = 0, mapped_count = 0, free_index = 0;
    for (int y = 0; y < stored_grid->h; y++) for (int x = 0; x < stored_grid->w; x++) {
        if (!stored_grid->free[(long)y * stored_grid->w + x]) continue;
        if (stride > 1 && (free_index++ % stride)) continue;
        double cur_x = A00 * x + A01 * y + bx;
        double cur_y = A10 * x + A11 * y + by;
        int ix = (int)floor(cur_x), iy = (int)floor(cur_y);
        if (ix < 0 || iy < 0 || ix >= current_grid->w || iy >= current_grid->h) continue;
        mapped_count++;
        if (current_grid->free[(long)iy * current_grid->w + ix]) overlap_count++;
    }
    /* Subsample the current free count with the same rule as the stored cells so the
     * Dice is unbiased: with a bare `current_grid->freecount` denominator a *perfect*
     * match subsampled by `stride` scored ~2/(stride+1) ≈ 0.4. */
    long current_count = current_grid->freecount;
    if (stride > 1) {
        current_count = 0;
        long current_index = 0;
        size_t current_cells = (size_t)current_grid->w * (size_t)current_grid->h;
        for (size_t i = 0; i < current_cells; i++)
            if (current_grid->free[i] && (current_index++ % stride) == 0) current_count++;
    }
    if (out_mapped) *out_mapped = mapped_count;
    if (out_overlap) *out_overlap = overlap_count;
    if (out_covered) *out_covered = stored_grid->freecount ? mapped_count : 0;
    double denom = (double)current_count + (double)mapped_count;
    return denom > 0 ? (2.0 * overlap_count) / denom : 0.0;
}

typedef struct edge_ctx {
    const uint8_t *stored_edge, *cur_edge;      /* boundary masks */
    const float   *stored_dist, *cur_dist;    /* distance transforms */
    int   stored_w, stored_h, cur_w, cur_h;
    long  stored_edge_count, cur_edge_count;
    double tol, weight;               /* tolerance (cells) and weight in the objective */
} edge_ctx;

typedef struct {
    const reanchor_grid_t *stored_grid, *current_grid;
    double ratio_base, stored_cx, stored_cy, current_cx, current_cy;
    int param_count, stride;
    const edge_ctx *edge;        /* optional smooth boundary term */
} score_ctx;

/* soft symmetric boundary inlier score in [0,1]: sum of (1 - d/tol)+ normalised so
 * that a perfect overlap is 1.  Smooth in the transform (unlike a hard threshold). */
static double edge_soft_f1(const edge_ctx *edge, double A00, double A01, double A10,
                           double A11, double bx, double by) {
    double det = A00 * A11 - A01 * A10;
    if (det == 0) return 0.0;
    double sum_forward = 0, sum_backward = 0; long forward_total = 0, backward_total = 0;
    for (int y = 0; y < edge->stored_h; y++) for (int x = 0; x < edge->stored_w; x++) {
        if (!edge->stored_edge[(long)y * edge->stored_w + x]) continue;
        double cur_x = A00 * x + A01 * y + bx, cur_y = A10 * x + A11 * y + by;
        int ix = (int)floor(cur_x + 0.5), iy = (int)floor(cur_y + 0.5);
        if (ix < 0 || iy < 0 || ix >= edge->cur_w || iy >= edge->cur_h) continue;
        forward_total++;
        double dist = edge->cur_dist[(long)iy * edge->cur_w + ix];
        if (dist < edge->tol) sum_forward += 1.0 - dist / edge->tol;
    }
    for (int y = 0; y < edge->cur_h; y++) for (int x = 0; x < edge->cur_w; x++) {
        if (!edge->cur_edge[(long)y * edge->cur_w + x]) continue;
        double delta_x = x - bx, delta_y = y - by;
        double stored_x = (A11 * delta_x - A01 * delta_y) / det;
        double stored_y = (-A10 * delta_x + A00 * delta_y) / det;
        int ix = (int)floor(stored_x + 0.5), iy = (int)floor(stored_y + 0.5);
        if (ix < 0 || iy < 0 || ix >= edge->stored_w || iy >= edge->stored_h) continue;
        backward_total++;
        double dist = edge->stored_dist[(long)iy * edge->stored_w + ix];
        if (dist < edge->tol) sum_backward += 1.0 - dist / edge->tol;
    }
    double precision = forward_total ? sum_forward / forward_total : 0.0;
    double recall = backward_total ? sum_backward / backward_total : 0.0;
    return (precision + recall > 0) ? 2 * precision * recall / (precision + recall) : 0.0;
}

static double score_params(const score_ctx *ctx, const double *params) {
    double A00, A01, A10, A11, bx, by;
    params_to_affine(params, ctx->param_count, ctx->ratio_base, ctx->stored_cx, ctx->stored_cy,
                     ctx->current_cx, ctx->current_cy, &A00, &A01, &A10, &A11, &bx, &by);
    double dice = dice_eval(ctx->stored_grid, ctx->current_grid, A00, A01, A10, A11, bx, by, ctx->stride,
                            NULL, NULL, NULL);
    if (ctx->edge)
        dice += ctx->edge->weight * edge_soft_f1(ctx->edge, A00, A01, A10, A11, bx, by);
    return dice;
}

static void pattern_search(const score_ctx *ctx, double *params,
                           const double *step0, int levels) {
    double step[4];
    for (int i = 0; i < ctx->param_count; i++) step[i] = step0[i];
    double best = score_params(ctx, params);
    for (int level = 0; level < levels; level++) {
        int improved = 1, guard = 0;
        while (improved && guard++ < 40) {
            improved = 0;
            for (int i = 0; i < ctx->param_count; i++) {
                for (int sgn = -1; sgn <= 1; sgn += 2) {
                    double candidate[4]; memcpy(candidate, params, sizeof candidate);
                    candidate[i] += sgn * step[i];
                    if (ctx->param_count >= 4 && candidate[3] < 0.5) continue;
                    if (i == 0) { /* keep theta in [0,360) */
                        if (candidate[0] < 0) candidate[0] += 360.0;
                        if (candidate[0] >= 360.0) candidate[0] -= 360.0;
                    }
                    double score = score_params(ctx, candidate);
                    if (score > best + 1e-12) { best = score; memcpy(params, candidate, sizeof candidate); improved = 1; }
                }
            }
        }
        int stop = 1;
        for (int i = 0; i < ctx->param_count; i++) {
            step[i] *= 0.5;
            double floor_ = (i == 0) ? 0.05 : (i == 3 ? 0.002 : 0.2);
            if (step[i] > floor_) stop = 0;
        }
        if (stop) break;
    }
}

/* translation search at a fixed theta/scale: full probe grid then refine */
static double translation_search(const score_ctx *ctx, double theta_deg, double s_scale,
                                 double range, double step0, double *btx, double *bty) {
    double params[4] = { theta_deg, 0.0, 0.0, s_scale };
    double best = -1.0, best_tx = 0, best_ty = 0;
    for (double tx = -range; tx <= range + 1e-9; tx += step0) {
        for (double ty = -range; ty <= range + 1e-9; ty += step0) {
            params[1] = tx; params[2] = ty;
            double score = score_params(ctx, params);
            if (score > best) { best = score; best_tx = tx; best_ty = ty; }
        }
    }
    /* refine translation */
    params[1] = best_tx; params[2] = best_ty;
    for (double step = step0 * 0.5; step > 0.24; step *= 0.5) {
        int improved = 1, guard = 0;
        while (improved && guard++ < 10) {
            improved = 0;
            for (int i = 1; i <= 2; i++) for (int sgn = -1; sgn <= 1; sgn += 2) {
                double candidate[4]; memcpy(candidate, params, sizeof candidate);
                candidate[i] += sgn * step;
                double score = score_params(ctx, candidate);
                if (score > best) { best = score; params[1] = candidate[1]; params[2] = candidate[2]; improved = 1; }
            }
        }
    }
    *btx = params[1]; *bty = params[2];
    return best;
}

/* ===================================================================================== *
 *  core alignment                                                                       *
 * ===================================================================================== */
void reanchor_default_opts(reanchor_opts_t *opts) {
    memset(opts, 0, sizeof *opts);
    opts->theta_min_deg = 0.0;
    opts->theta_max_deg = 360.0;
    opts->theta_step_deg = 3.0;
    opts->coarse_max = 72;
    opts->tol_cells = 1.5;
    opts->min_dice = 0.45;
    opts->min_edge_f1 = 0.50;
    opts->min_edge_each = 0.35;
    opts->min_support = 0.10;
    opts->min_margin = 0.05;
    opts->min_angle_sep_deg = 30.0;
    opts->allow_scale = 0;
    opts->max_features = 12000;
    opts->top_k = 4;
}

static void edge_metrics(const reanchor_grid_t *stored_grid, const reanchor_grid_t *current_grid,
                         const reanchor_result_t *res, double tol,
                         const uint8_t *stored_edge, long stored_edge_count,
                         const uint8_t *cur_edge, long cur_edge_count,
                         const float *stored_dist, const float *cur_dist,
                         double *precision, double *recall, double *f1, double *support) {
    double A00 = res->a00, A01 = res->a01, A10 = res->a10, A11 = res->a11;
    double bx = res->bx, by = res->by;
    double det = A00 * A11 - A01 * A10;
    long forward_hits = 0, forward_total = 0, backward_hits = 0, backward_total = 0;
    for (int y = 0; y < stored_grid->h; y++) for (int x = 0; x < stored_grid->w; x++) {
        if (!stored_edge[(long)y * stored_grid->w + x]) continue;
        double cur_x = A00 * x + A01 * y + bx, cur_y = A10 * x + A11 * y + by;
        int ix = (int)floor(cur_x + 0.5), iy = (int)floor(cur_y + 0.5);
        if (ix < 0 || iy < 0 || ix >= current_grid->w || iy >= current_grid->h) continue;
        forward_total++;
        if (cur_dist[(long)iy * current_grid->w + ix] <= tol) forward_hits++;
    }
    for (int y = 0; y < current_grid->h; y++) for (int x = 0; x < current_grid->w; x++) {
        if (!cur_edge[(long)y * current_grid->w + x]) continue;
        if (det == 0) break;
        double delta_x = x - bx, delta_y = y - by;
        double stored_x = (A11 * delta_x - A01 * delta_y) / det;
        double stored_y = (-A10 * delta_x + A00 * delta_y) / det;
        int ix = (int)floor(stored_x + 0.5), iy = (int)floor(stored_y + 0.5);
        if (ix < 0 || iy < 0 || ix >= stored_grid->w || iy >= stored_grid->h) continue;
        backward_total++;
        if (stored_dist[(long)iy * stored_grid->w + ix] <= tol) backward_hits++;
    }
    *precision = forward_total ? (double)forward_hits / forward_total : 0.0;
    *recall  = backward_total ? (double)backward_hits / backward_total : 0.0;
    *f1   = (*precision + *recall > 0) ? 2 * (*precision) * (*recall) / (*precision + *recall) : 0.0;
    double forward_support = stored_edge_count ? (double)forward_total / stored_edge_count : 0.0;
    double backward_support = cur_edge_count ? (double)backward_total / cur_edge_count : 0.0;
    *support = forward_support < backward_support ? forward_support : backward_support;
}

int reanchor_align(const reanchor_grid_t *stored, const reanchor_grid_t *cur,
                   const reanchor_opts_t *opts_in, reanchor_result_t *res) {
    reanchor_opts_t opts;
    if (opts_in) opts = *opts_in; else reanchor_default_opts(&opts);
    memset(res, 0, sizeof *res);
    res->ok = 0; res->reason = "init";
    res->scale = 1.0; res->a00 = 1; res->a11 = 1;
    res->runner_up_score = -1.0;

    /* Reject options that would divide by zero / produce UB.  The CLI validates
     * them too, but library callers must be safe as well: `coarse_max == 0` divides by
     * zero and `theta_step == 0` computes floor(inf) -> (int) UB. */
    if (opts.coarse_max <= 0) { res->reason = "bad-coarse-max"; return -1; }
    if (opts.theta_step_deg <= 0.0) { res->reason = "bad-theta-step"; return -1; }
    if (opts.max_features <= 0) { res->reason = "bad-max-features"; return -1; }
    if (opts.top_k <= 0) { res->reason = "bad-top-k"; return -1; }
    if (opts.min_margin < 0.0) { res->reason = "bad-min-margin"; return -1; }
    if (opts.min_angle_sep_deg <= 0.0 || opts.min_angle_sep_deg > 180.0) { res->reason = "bad-min-angle-sep"; return -1; }

    const reanchor_grid_t *stored_grid = stored, *current_grid = cur;
    if (!stored_grid->free || !current_grid->free) { res->reason = "no-grid"; return -1; }
    if (stored_grid->freecount < 4 || current_grid->freecount < 4) { res->reason = "empty"; return -1; }
    res->n_stored_free = stored_grid->freecount;
    res->n_cur_free = current_grid->freecount;

    /* `scale` is the V1 map resolution in cells/metre, so a stored cell maps to
     * stored_cell * (cur.scale/stored.scale) cells in the current frame. */
    double ratio_base = (current_grid->scale > 0 && stored_grid->scale > 0) ? (double)current_grid->scale / stored_grid->scale : 1.0;

    /* fine centroid + stride cap */
    double fine_stored_cx, fine_stored_cy, fine_cur_cx, fine_cur_cy;
    free_centroid(stored_grid, &fine_stored_cx, &fine_stored_cy);
    free_centroid(current_grid, &fine_cur_cx, &fine_cur_cy);
    int feature_stride = (int)((stored_grid->freecount + opts.max_features - 1) / opts.max_features);
    if (feature_stride < 1) feature_stride = 1;

    /* ---- coarse resolution ---- */
    int max_dim = stored_grid->w > stored_grid->h ? stored_grid->w : stored_grid->h;
    if (current_grid->w > max_dim) max_dim = current_grid->w;
    if (current_grid->h > max_dim) max_dim = current_grid->h;
    int downsample_k = (max_dim + opts.coarse_max - 1) / opts.coarse_max;
    if (downsample_k < 1) downsample_k = 1;

    reanchor_grid_t coarse_stored = {0}, coarse_cur = {0};
    const reanchor_grid_t *active_stored = stored_grid, *active_cur = current_grid;
    double coarse_stored_cx = fine_stored_cx, coarse_stored_cy = fine_stored_cy, coarse_cur_cx = fine_cur_cx, coarse_cur_cy = fine_cur_cy;
    if (downsample_k > 1) {
        if (downsample_grid(stored_grid, downsample_k, &coarse_stored) != 0 ||
            downsample_grid(current_grid, downsample_k, &coarse_cur) != 0) {
            reanchor_grid_free(&coarse_stored); reanchor_grid_free(&coarse_cur);
            res->reason = "oom"; return -1;
        }
        if (coarse_stored.freecount < 4 || coarse_cur.freecount < 4) {
            /* degenerate downsample: fall back to fine-only */
            reanchor_grid_free(&coarse_stored); reanchor_grid_free(&coarse_cur);
            downsample_k = 1;
        } else {
            active_stored = &coarse_stored; active_cur = &coarse_cur;
            free_centroid(&coarse_stored, &coarse_stored_cx, &coarse_stored_cy);
            free_centroid(&coarse_cur, &coarse_cur_cx, &coarse_cur_cy);
        }
    }

    /* candidate scales */
    double scales[5]; int scale_count = 1;
    if (opts.allow_scale) { scales[0]=0.94; scales[1]=0.97; scales[2]=1.0; scales[3]=1.03; scales[4]=1.06; scale_count=5; }
    else scales[0] = 1.0;

    /* ---- coarse rotation sweep + translation search ---- */
    int angle_count = (int)floor((opts.theta_max_deg - opts.theta_min_deg) / opts.theta_step_deg);
    if (angle_count < 1) angle_count = 1;
    if (angle_count > 720) angle_count = 720;
    double coarse_range = 0.5 * (active_stored->w > active_stored->h ? active_stored->w : active_stored->h);
    double coarse_step  = (active_stored->w < active_stored->h ? active_stored->w : active_stored->h) / 8.0;
    if (coarse_step < 1.0) coarse_step = 1.0;

    typedef struct { double theta, tx, ty, s, score; } candidate_t;
    /* keep the best per (theta,s) then the top-K overall */
    int candidate_capacity = angle_count * scale_count;
    candidate_t *candidates = (candidate_t *)malloc(sizeof(candidate_t) * (size_t)candidate_capacity);
    if (!candidates) {
        if (downsample_k > 1) { reanchor_grid_free(&coarse_stored); reanchor_grid_free(&coarse_cur); }
        res->reason = "oom"; return -1;
    }
    int candidate_count = 0;

    for (int scale_index = 0; scale_index < scale_count; scale_index++) {
        score_ctx coarse_ctx = { active_stored, active_cur, ratio_base, coarse_stored_cx, coarse_stored_cy, coarse_cur_cx, coarse_cur_cy,
                           opts.allow_scale ? 4 : 3, 1, NULL };
        for (int angle_index = 0; angle_index < angle_count; angle_index++) {
            double theta = opts.theta_min_deg + angle_index * opts.theta_step_deg;
            double tx, ty;
            double score = translation_search(&coarse_ctx, theta, scales[scale_index], coarse_range, coarse_step, &tx, &ty);
            candidates[candidate_count].theta = theta; candidates[candidate_count].tx = tx; candidates[candidate_count].ty = ty;
            candidates[candidate_count].s = scales[scale_index]; candidates[candidate_count].score = score;
            candidate_count++;
        }
    }

    /* ---- fine boundary features (also the smooth refinement objective) ---- */
    long stored_edge_count = 0, cur_edge_count = 0;
    uint8_t *stored_edge = free_edge(stored_grid, &stored_edge_count);
    uint8_t *cur_edge = free_edge(current_grid, &cur_edge_count);
    float *stored_dist = stored_edge ? dist_transform(stored_grid->w, stored_grid->h, stored_edge) : NULL;
    float *cur_dist = cur_edge ? dist_transform(current_grid->w, current_grid->h, cur_edge) : NULL;
    edge_ctx edge_var;
    int have_edge = (stored_edge && cur_edge && stored_dist && cur_dist && stored_edge_count > 0 && cur_edge_count > 0);
    if (have_edge) {
        edge_var.stored_edge = stored_edge; edge_var.cur_edge = cur_edge; edge_var.stored_dist = stored_dist; edge_var.cur_dist = cur_dist;
        edge_var.stored_w = stored_grid->w; edge_var.stored_h = stored_grid->h; edge_var.cur_w = current_grid->w; edge_var.cur_h = current_grid->h;
        edge_var.stored_edge_count = stored_edge_count; edge_var.cur_edge_count = cur_edge_count;
        edge_var.tol = opts.tol_cells;
        edge_var.weight = 0.5;   /* boundary term weight in the (dice + w*f1) objective */
    }

    /* pick top-K with angular separation */
    double min_sep = opts.theta_step_deg * 2.0; if (min_sep < 12.0) min_sep = 12.0;
    int picked_count = 0;
    char *taken = (char *)calloc((size_t)candidate_count, 1);
    if (!taken) {
        free(stored_edge); free(cur_edge); free(stored_dist); free(cur_dist);
        free(candidates);
        if (downsample_k > 1) { reanchor_grid_free(&coarse_stored); reanchor_grid_free(&coarse_cur); }
        res->reason = "oom"; return -1;
    }
    double best_score = -1; double best_params[4] = {0,0,0,1};
    score_ctx fine_ctx = { stored_grid, current_grid, ratio_base, fine_stored_cx, fine_stored_cy, fine_cur_cx, fine_cur_cy,
                       opts.allow_scale ? 4 : 3, feature_stride, have_edge ? &edge_var : NULL };
    /* Refined (score, theta) of every picked candidate.  Kept so the runner-up at a
     * *different* orientation can be compared with the winner: on a near-symmetric
     * plan a 180/90-degree alternative scores almost as well and the winner may be the
     * wrong orientation.  Capacity is bounded by the coarse candidate count. */
    typedef struct { double theta, score; } refined_candidate_t;
    int refined_capacity = opts.top_k < candidate_count ? opts.top_k : candidate_count;
    refined_candidate_t *refined = (refined_candidate_t *)calloc((size_t)refined_capacity, sizeof *refined);
    if (!refined) {
        free(taken); free(candidates);
        free(stored_edge); free(cur_edge); free(stored_dist); free(cur_dist);
        if (downsample_k > 1) { reanchor_grid_free(&coarse_stored); reanchor_grid_free(&coarse_cur); }
        res->reason = "oom"; return -1;
    }
    for (int pick = 0; pick < opts.top_k && pick < candidate_count; pick++) {
        int best_index = -1; double best_value = -1;
        for (int i = 0; i < candidate_count; i++) {
            if (taken[i]) continue;
            int clash = 0;
            for (int j = 0; j < candidate_count; j++) {
                if (!taken[j] || j == i) continue;
                double angle_diff = fabs(candidates[i].theta - candidates[j].theta);
                if (angle_diff > 180) angle_diff = 360 - angle_diff;
                if (angle_diff < min_sep && fabs(candidates[i].s - candidates[j].s) < 1e-9) { clash = 1; break; }
            }
            if (clash) continue;
            if (candidates[i].score > best_value) { best_value = candidates[i].score; best_index = i; }
        }
        if (best_index < 0) break;
        taken[best_index] = 1; picked_count++;

        double params[4] = { candidates[best_index].theta, candidates[best_index].tx * downsample_k, candidates[best_index].ty * downsample_k, candidates[best_index].s };
        double step0[4] = { 2.5, (double)(downsample_k > 1 ? 2.0 * downsample_k : 2.0), (double)(downsample_k > 1 ? 2.0 * downsample_k : 2.0), 0.02 };
        if (opts.allow_scale) pattern_search(&fine_ctx, params, step0, 40);
        else {
            score_ctx fine_ctx3 = fine_ctx; fine_ctx3.param_count = 3;
            pattern_search(&fine_ctx3, params, step0, 40);
        }
        double score = score_params(&fine_ctx, params);
        refined[picked_count - 1].theta = params[0];
        refined[picked_count - 1].score = score;
        if (score > best_score) { best_score = score; memcpy(best_params, params, sizeof best_params); }
    }
    free(taken); free(candidates);
    if (downsample_k > 1) { reanchor_grid_free(&coarse_stored); reanchor_grid_free(&coarse_cur); }
    if (picked_count == 0) { free(refined); free(stored_edge); free(cur_edge); free(stored_dist); free(cur_dist); res->reason = "no-candidate"; return -1; }

    /* ---- runner-up at a different orientation ---- */
    for (int i = 0; i < picked_count; i++) {
        double angle_diff = fabs(refined[i].theta - best_params[0]);
        if (angle_diff > 180.0) angle_diff = 360.0 - angle_diff;
        if (angle_diff < opts.min_angle_sep_deg) continue;
        if (refined[i].score > res->runner_up_score) {
            res->runner_up_score = refined[i].score;
            res->runner_up_theta_deg = refined[i].theta;
        }
    }
    free(refined);

    /* ---- final transform + metrics at full resolution ---- */
    double A00, A01, A10, A11, bx, by;
    params_to_affine(best_params, fine_ctx.param_count, ratio_base, fine_stored_cx, fine_stored_cy, fine_cur_cx, fine_cur_cy,
                     &A00, &A01, &A10, &A11, &bx, &by);
    long mapped_count = 0, overlap_count = 0, coverage_count = 0;
    double dice = dice_eval(stored_grid, current_grid, A00, A01, A10, A11, bx, by, 1, &mapped_count, &overlap_count, &coverage_count);

    res->a00 = A00; res->a01 = A01; res->a10 = A10; res->a11 = A11;
    res->bx = bx; res->by = by;
    res->theta_deg = best_params[0];
    res->theta_rad = best_params[0] * (M_PI / 180.0);
    res->scale = (fine_ctx.param_count >= 4) ? best_params[3] : 1.0;
    res->score = dice;
    res->coverage = stored_grid->freecount ? (double)mapped_count / stored_grid->freecount : 0.0;
    double min_above = current_grid->freecount < mapped_count ? current_grid->freecount : mapped_count;
    res->overlap = min_above ? (double)overlap_count / min_above : 0.0;

    /* ---- boundary (free-edge) confidence ---- */
    if (have_edge) {
        double pr, rc, f1, sup;
        edge_metrics(stored_grid, current_grid, res, opts.tol_cells, stored_edge, stored_edge_count, cur_edge, cur_edge_count, stored_dist, cur_dist,
                     &pr, &rc, &f1, &sup);
        res->edge_precision = pr; res->edge_recall = rc;
        res->edge_f1 = f1; res->edge_support = sup;
    }
    free(stored_edge); free(cur_edge); free(stored_dist); free(cur_dist);

    /* ---- decision (abstain is the safe default) ---- */
    if (res->score < opts.min_dice) { res->reason = "low-dice"; return -2; }
    if (res->edge_f1 < opts.min_edge_f1) { res->reason = "low-edge-f1"; return -2; }
    if (res->edge_precision < opts.min_edge_each) { res->reason = "low-edge-prec"; return -2; }
    if (res->edge_recall < opts.min_edge_each) { res->reason = "low-edge-rec"; return -2; }
    if (res->coverage < opts.min_support) { res->reason = "low-support"; return -2; }
    /* a close runner-up at a different rotation means the winner may be the wrong
     * orientation (near-symmetric plan).  Abstaining is mandatory here: applying the
     * transform would swap stored segment labels / clean the wrong room. */
    if (res->runner_up_score >= 0.0 && (best_score - res->runner_up_score) < opts.min_margin) {
        res->reason = "ambiguous"; return -2;
    }
    res->ok = 1;
    res->reason = "ok";
    return 0;
}

/* ===================================================================================== *
 *  transform application + segment re-anchoring                                         *
 * ===================================================================================== */
void reanchor_apply(const reanchor_result_t *res, double sx, double sy, double *cx, double *cy) {
    *cx = res->a00 * sx + res->a01 * sy + res->bx;
    *cy = res->a10 * sx + res->a11 * sy + res->by;
}
void reanchor_apply_inv(const reanchor_result_t *res, double cx, double cy, double *sx, double *sy) {
    double det = res->a00 * res->a11 - res->a01 * res->a10;
    if (det == 0) { *sx = *sy = 0; return; }
    double delta_x = cx - res->bx, delta_y = cy - res->by;
    *sx = ( res->a11 * delta_x - res->a01 * delta_y) / det;
    *sy = (-res->a10 * delta_x + res->a00 * delta_y) / det;
}

int reanchor_transform_seg_masked(const reanchor_seg_t *stored, int stored_pad,
                                  const reanchor_result_t *res,
                                  int cur_w, int cur_h,
                                  int out_w, int out_h, uint8_t *out_cells,
                                  const uint8_t *cur_free) {
    if (!stored || !stored->cells || !out_cells) return -1;
    memset(out_cells, 0, (size_t)out_w * out_h);
    int pad_x = (out_w - cur_w) / 2;   /* out grid is current map padded by padx */
    int pad_y = (out_h - cur_h) / 2;
    if (pad_x < 0) pad_x = 0;
    if (pad_y < 0) pad_y = 0;

    for (int out_y = 0; out_y < out_h; out_y++) {
        for (int out_x = 0; out_x < out_w; out_x++) {
            int cur_x = out_x - pad_x, cur_y = out_y - pad_y;
            if (cur_x < 0 || cur_y < 0 || cur_x >= cur_w || cur_y >= cur_h) continue;
            double stored_x, stored_y;
            reanchor_apply_inv(res, (double)cur_x, (double)cur_y, &stored_x, &stored_y);
            int stored_cx = (int)floor(stored_x + 0.5) + stored_pad;
            int stored_cy = (int)floor(stored_y + 0.5) + stored_pad;
            uint8_t cell_id = 0;
            if (stored_cx >= 0 && stored_cy >= 0 && stored_cx < stored->w && stored_cy < stored->h)
                cell_id = stored->cells[(long)stored_cy * stored->w + stored_cx] & 0x3f;
            if (!cell_id) {
                /* valid-neighbour fill (radius 1): nearest nonzero label.  When the
                 * current free mask is known, only paint a known-floor cell so a room
                 * id is never bled onto a wall. */
                int fillable = !cur_free || cur_free[(long)cur_y * cur_w + cur_x];
                int best_dist = 99; uint8_t best_id = 0;
                if (fillable) {
                    for (int dy = -1; dy <= 1; dy++) for (int dx = -1; dx <= 1; dx++) {
                        if (!dx && !dy) continue;
                        int ix = stored_cx + dx, iy = stored_cy + dy;
                        if (ix < 0 || iy < 0 || ix >= stored->w || iy >= stored->h) continue;
                        uint8_t neighbor_id = stored->cells[(long)iy * stored->w + ix] & 0x3f;
                        if (!neighbor_id) continue;
                        int dist = dx * dx + dy * dy;
                        if (dist < best_dist) { best_dist = dist; best_id = neighbor_id; }
                    }
                }
                cell_id = best_id;
            }
            out_cells[(long)out_y * out_w + out_x] = cell_id & 0x3f;
        }
    }
    return 0;
}

int reanchor_transform_seg(const reanchor_seg_t *stored, int stored_pad,
                           const reanchor_result_t *res,
                           int cur_w, int cur_h,
                           int out_w, int out_h, uint8_t *out_cells) {
    return reanchor_transform_seg_masked(stored, stored_pad, res, cur_w, cur_h, out_w, out_h, out_cells, NULL);
}

/* ===================================================================================== *
 *  CLI                                                                                  *
 * ===================================================================================== */
#ifndef REANCHOR_NO_CLI
static void usage(void) {
    wout("usage: reanchor align <stored_map> <cur_map> [opts]\n");
    wout("       reanchor transform <stored_map> <stored_segmap> <cur_map> <out_segmap> [opts]\n");
    wout("opts: --theta-step D --top-k N --coarse-max N --tol C --min-dice D --min-edge-f1 D\n");
    wout("      --min-margin D --min-angle-sep D --allow-scale --eps E\n");
}

static int parse_opts(reanchor_opts_t *opts, int argc, char **argv, int start) {
    for (int i = start; i < argc; i++) {
        if (!strcmp(argv[i], "--theta-step") && i + 1 < argc) opts->theta_step_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--top-k") && i + 1 < argc) opts->top_k = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--coarse-max") && i + 1 < argc) opts->coarse_max = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--tol") && i + 1 < argc) opts->tol_cells = atof(argv[++i]);
        else if (!strcmp(argv[i], "--min-dice") && i + 1 < argc) opts->min_dice = atof(argv[++i]);
        else if (!strcmp(argv[i], "--min-edge-f1") && i + 1 < argc) opts->min_edge_f1 = atof(argv[++i]);
        else if (!strcmp(argv[i], "--min-support") && i + 1 < argc) opts->min_support = atof(argv[++i]);
        else if (!strcmp(argv[i], "--min-margin") && i + 1 < argc) opts->min_margin = atof(argv[++i]);
        else if (!strcmp(argv[i], "--min-angle-sep") && i + 1 < argc) opts->min_angle_sep_deg = atof(argv[++i]);
        else if (!strcmp(argv[i], "--allow-scale")) opts->allow_scale = 1;
    }
    /* Values that would divide by zero / produce UB downstream, plus the
     * ambiguity gate bounds (margin >= 0, 0 < angle separation <= 180). */
    if (opts->theta_step_deg <= 0.0 || opts->coarse_max <= 0 || opts->top_k <= 0 || opts->max_features <= 0 ||
        opts->min_margin < 0.0 || opts->min_angle_sep_deg <= 0.0 || opts->min_angle_sep_deg > 180.0)
        return -1;
    return 0;
}

static void print_result(const reanchor_result_t *res) {
    woutf("REANCHOR ok=%d theta_deg=%.4f dx=%.4f dy=%.4f scale=%.5f dice=%.4f "
          "overlap=%.4f coverage=%.4f edge_f1=%.4f prec=%.4f rec=%.4f "
          "runner_up=%.4f runner_up_theta=%.4f "
          "s_free=%ld c_free=%ld reason=%s\n",
          res->ok, res->theta_deg, res->bx, res->by, res->scale, res->score,
          res->overlap, res->coverage, res->edge_f1, res->edge_precision, res->edge_recall,
          res->runner_up_score, res->runner_up_theta_deg,
          res->n_stored_free, res->n_cur_free, res->reason);
}

int main(int argc, char **argv) {
    crc_init();
    if (argc < 4) { usage(); return 2; }
    reanchor_opts_t opts; reanchor_default_opts(&opts);
    float eps = 1.0f;

    if (!strcmp(argv[1], "align")) {
        if (argc < 4) { usage(); return 2; }
        const char *stored_path = argv[2], *cur_path = argv[3];
        /* optional --eps before opts parsing */
        for (int i = 4; i < argc; i++) if (!strcmp(argv[i], "--eps") && i + 1 < argc) eps = (float)atof(argv[i + 1]);
        if (parse_opts(&opts, argc, argv, 4) != 0) {
            wout("REANCHOR error=bad-options (--theta-step/--coarse-max/--top-k > 0; --min-margin >= 0; 0 < --min-angle-sep <= 180)\n"); return 2;
        }
        reanchor_grid_t stored_grid = {0}, cur_grid = {0};
        int load_stored_rc = reanchor_load_grid_ex(stored_path, eps, &stored_grid);
        int load_cur_rc = reanchor_load_grid_ex(cur_path, eps, &cur_grid);
        if (load_stored_rc != 0) {
            woutf("REANCHOR error=stored-load(%d)\n", load_stored_rc);
            reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid);
            return 2;
        }
        if (load_cur_rc != 0) {
            woutf("REANCHOR error=cur-load(%d)\n", load_cur_rc);
            reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid);
            return 2;
        }
        reanchor_result_t res;
        int align_rc = reanchor_align(&stored_grid, &cur_grid, &opts, &res);
        print_result(&res);
        reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid);
        return align_rc == 0 ? 0 : 3;
    } else if (!strcmp(argv[1], "transform")) {
        if (argc < 6) { usage(); return 2; }
        const char *smp = argv[2], *ssp = argv[3], *cmp_ = argv[4], *outp = argv[5];
        if (parse_opts(&opts, argc, argv, 6) != 0) {
            wout("REANCHOR error=bad-options (--theta-step/--coarse-max/--top-k > 0; --min-margin >= 0; 0 < --min-angle-sep <= 180)\n"); return 2;
        }
        reanchor_grid_t stored_grid = {0}, cur_grid = {0};
        int load_stored_rc = reanchor_load_grid(smp, &stored_grid);
        int load_cur_rc = reanchor_load_grid(cmp_, &cur_grid);
        if (load_stored_rc != 0) {
            woutf("REANCHOR error=stored-load(%d)\n", load_stored_rc);
            reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid);
            return 2;
        }
        if (load_cur_rc != 0) {
            woutf("REANCHOR error=cur-load(%d)\n", load_cur_rc);
            reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid);
            return 2;
        }
        reanchor_seg_t seg; int pad = 0; float seg_scale = 0; int seg_ox = 0, seg_oy = 0;
        int load_seg_rc = reanchor_load_segmap(ssp, stored_grid.w, stored_grid.h, &seg, &pad, &seg_scale, &seg_ox, &seg_oy);
        if (load_seg_rc != 0) { woutf("REANCHOR error=seg-load(%d)\n", load_seg_rc); reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid); return 2; }
        reanchor_result_t res;
        int align_rc = reanchor_align(&stored_grid, &cur_grid, &opts, &res);
        print_result(&res);
        if (align_rc != 0) {
            woutf("REANCHOR abstain -> no segmap written (recompute expected)\n");
            reanchor_seg_free(&seg); reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid);
            return 3;
        }
        int out_w = cur_grid.w + 2 * pad, out_h = cur_grid.h + 2 * pad;
        uint8_t *out = (uint8_t *)calloc(1, (size_t)out_w * out_h);
        if (!out) { reanchor_seg_free(&seg); reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid); return 2; }
        reanchor_transform_seg_masked(&seg, pad, &res, cur_grid.w, cur_grid.h, out_w, out_h, out, cur_grid.free);
        int write_rc = reanchor_write_segmap(outp, out_w, out_h, out, cur_grid.scale, cur_grid.ox, cur_grid.oy, 0, 0, 0, 0);
        woutf("REANCHOR wrote=%s %dx%d pad=%d rc=%d\n", outp, out_w, out_h, pad, write_rc);
        free(out); reanchor_seg_free(&seg); reanchor_grid_free(&stored_grid); reanchor_grid_free(&cur_grid);
        return write_rc == 0 ? 0 : 2;
    }
    usage();
    return 2;
}
#endif /* REANCHOR_NO_CLI */
