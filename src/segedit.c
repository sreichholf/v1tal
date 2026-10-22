/* segedit.c -- see segedit.h.  Pure libc; no robot/AppProxy dependencies.
 *
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "segedit.h"
#include "crc32.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

/* ===================================================================================== *
 *  crc32 (IEEE / zlib) + little-endian helpers                                          *
 * ===================================================================================== */
static uint32_t crc_tab[256];
static int crc_ready = 0;
static void crc_init(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++)
            c = (c & 1) ? (CRC32_POLYNOMIAL_REFLECTED ^ (c >> 1)) : (c >> 1);
        crc_tab[i] = c;
    }
    crc_ready = 1;
}
uint32_t segedit_crc32(const void *data, size_t length) {
    if (!crc_ready) crc_init();
    const unsigned char *bytes = (const unsigned char *)data;
    uint32_t crc = CRC32_INITIAL_VALUE;
    for (size_t i = 0; i < length; i++) crc = crc_tab[(crc ^ bytes[i]) & 0xff] ^ (crc >> 8);
    return crc ^ CRC32_FINAL_XOR;
}

static uint32_t rd_u32(const unsigned char *buf) {
    return (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
}
static int32_t rd_i32(const unsigned char *buf) { return (int32_t)rd_u32(buf); }
static float rd_f32(const unsigned char *buf) { uint32_t bits = rd_u32(buf); float value; memcpy(&value, &bits, 4); return value; }
static void wr_u32(unsigned char *buf, uint32_t value) {
    buf[0] = (unsigned char)value; buf[1] = (unsigned char)(value >> 8);
    buf[2] = (unsigned char)(value >> 16); buf[3] = (unsigned char)(value >> 24);
}
static void wr_i32(unsigned char *buf, int32_t value) { wr_u32(buf, (uint32_t)value); }
static void wr_f32(unsigned char *buf, float value) { uint32_t bits; memcpy(&bits, &value, 4); wr_u32(buf, bits); }

const char *segedit_strerror(int rc) {
    switch (rc) {
    case SEGEDIT_OK:       return "ok";
    case SEGEDIT_EARG:     return "invalid segment edit arguments";
    case SEGEDIT_ELINE:    return "the split line has zero length";
    case SEGEDIT_ENOSEG:   return "the segment is not in the current map";
    case SEGEDIT_ENOID:    return "no free user segment id (16..31) left";
    case SEGEDIT_ENOSPLIT: return "the line does not split the segment in two";
    case SEGEDIT_EMAGIC:   return "the segment map is missing or corrupt";
    case SEGEDIT_EIO:      return "reading or writing the segment map failed";
    default:               return "segment edit failed";
    }
}

/* ===================================================================================== *
 *  grid helpers                                                                         *
 * ===================================================================================== */
void segedit_grid_init(segedit_grid *g) {
    if (g) { g->w = g->h = 0; g->cells = NULL; }
}

int segedit_grid_alloc(segedit_grid *g, int w, int h) {
    if (!g || w <= 0 || h <= 0) return SEGEDIT_EARG;
    segedit_grid_free(g);
    size_t cell_count = (size_t)w * (size_t)h;
    uint8_t *cell_buf = (uint8_t *)calloc(cell_count, 1);
    if (!cell_buf) return SEGEDIT_EIO;
    g->w = w; g->h = h; g->cells = cell_buf;
    return SEGEDIT_OK;
}

void segedit_grid_free(segedit_grid *g) {
    if (!g) return;
    free(g->cells);
    g->cells = NULL; g->w = g->h = 0;
}

int segedit_next_user_id(const segedit_grid *g) {
    if (!g || !g->cells) return 0;
    int used_ids[64];
    memset(used_ids, 0, sizeof used_ids);
    size_t cell_count = (size_t)g->w * g->h;
    for (size_t i = 0; i < cell_count; i++) {
        int id = g->cells[i] & SEGEDIT_ID_MASK;
        if (id < 64) used_ids[id] = 1;
    }
    for (int id = SEGEDIT_USER_MIN; id <= SEGEDIT_USER_MAX; id++)
        if (!used_ids[id]) return id;
    return 0;
}

int segedit_present_ids(const segedit_grid *g, int *out, int max) {
    if (!g || !g->cells) return 0;
    int used_ids[64]; memset(used_ids, 0, sizeof used_ids);
    size_t cell_count = (size_t)g->w * g->h;
    for (size_t i = 0; i < cell_count; i++) { int id = g->cells[i] & SEGEDIT_ID_MASK; if (id) used_ids[id] = 1; }
    int count = 0;
    for (int id = 1; id < 64; id++) {
        if (!used_ids[id]) continue;
        if (out && count < max) out[count] = id;
        count++;
    }
    return count;
}

long segedit_count_id(const segedit_grid *g, int id) {
    if (!g || !g->cells) return 0;
    long count = 0;
    size_t cell_count = (size_t)g->w * g->h;
    for (size_t i = 0; i < cell_count; i++) if ((int)(g->cells[i] & SEGEDIT_ID_MASK) == id) count++;
    return count;
}

/* ===================================================================================== *
 *  merge_segment                                                                        *
 * ===================================================================================== */
int segedit_merge(segedit_grid *g, const int *ids, int nids, int *out_target) {
    if (!g || !g->cells || !ids || nids <= 0) return SEGEDIT_EARG;
    /* A merge combines at least two segments: a single id or a duplicate set is a
     * no-op rename request, not a merge. */
    if (nids < 2) return SEGEDIT_EARG;

    /* validate + de-duplicate the listed ids (reject anything that is not a raw
     * 1..0x3f segment id instead of silently masking it) */
    int wanted_ids[64]; memset(wanted_ids, 0, sizeof wanted_ids);
    int distinct_count = 0;
    for (int i = 0; i < nids; i++) {
        int id = ids[i];
        if (id < 1 || id > (int)SEGEDIT_ID_MASK) return SEGEDIT_EARG;
        if (!wanted_ids[id]) { wanted_ids[id] = 1; distinct_count++; }
    }
    if (distinct_count < 2) return SEGEDIT_EARG;   /* single id / all duplicates */

    /* which ids are actually present?  A merge needs >= 2 distinct present ids. */
    int present_flags[64]; memset(present_flags, 0, sizeof present_flags);
    size_t cell_count = (size_t)g->w * g->h;
    for (size_t i = 0; i < cell_count; i++) {
        int id = g->cells[i] & SEGEDIT_ID_MASK;
        present_flags[id] = 1;
    }
    int present_count = 0;
    for (int id = 1; id <= (int)SEGEDIT_ID_MASK; id++)
        if (wanted_ids[id] && present_flags[id]) present_count++;
    if (present_count < 2) return SEGEDIT_ENOSEG;  /* "No segment data to merge" */

    /* target: first listed id in 16..31, else the lowest free user id */
    int target = 0;
    for (int i = 0; i < nids && !target; i++) {
        int id = ids[i];
        if (id >= SEGEDIT_USER_MIN && id <= SEGEDIT_USER_MAX) target = id;
    }
    if (!target) {
        target = segedit_next_user_id(g);
        if (!target) return SEGEDIT_ENOID;      /* "user defined segment id is out" */
    }

    for (size_t i = 0; i < cell_count; i++) {
        uint8_t cell_byte = g->cells[i];
        int id = cell_byte & SEGEDIT_ID_MASK;
        if (wanted_ids[id]) g->cells[i] = (uint8_t)((cell_byte & SEGEDIT_KEEP_MASK) | (target & SEGEDIT_ID_MASK));
    }
    if (out_target) *out_target = target;
    return SEGEDIT_OK;
}

/* ===================================================================================== *
 *  split_segment                                                                        *
 * ===================================================================================== */
static double segedit_orient(double ax, double ay, double bx, double by,
                             double px, double py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

static int segedit_on_segment(double ax, double ay, double bx, double by,
                              double px, double py) {
    const double eps = 1e-9;
    double lo_x = ax < bx ? ax : bx, hi_x = ax > bx ? ax : bx;
    double lo_y = ay < by ? ay : by, hi_y = ay > by ? ay : by;
    return px >= lo_x - eps && px <= hi_x + eps && py >= lo_y - eps && py <= hi_y + eps;
}

/* Do the two (finite) segments AB and CD intersect?  Touching at an endpoint or a
 * collinear overlap counts as an intersection. */
static int segedit_segments_intersect(double ax, double ay, double bx, double by,
                                      double cx, double cy, double dx, double dy) {
    const double eps = 1e-9;
    double d1 = segedit_orient(ax, ay, bx, by, cx, cy);
    double d2 = segedit_orient(ax, ay, bx, by, dx, dy);
    double d3 = segedit_orient(cx, cy, dx, dy, ax, ay);
    double d4 = segedit_orient(cx, cy, dx, dy, bx, by);
    if (((d1 > eps && d2 < -eps) || (d1 < -eps && d2 > eps)) &&
        ((d3 > eps && d4 < -eps) || (d3 < -eps && d4 > eps)))
        return 1;
    if (d1 >= -eps && d1 <= eps && segedit_on_segment(ax, ay, bx, by, cx, cy)) return 1;
    if (d2 >= -eps && d2 <= eps && segedit_on_segment(ax, ay, bx, by, dx, dy)) return 1;
    if (d3 >= -eps && d3 <= eps && segedit_on_segment(cx, cy, dx, dy, ax, ay)) return 1;
    if (d4 >= -eps && d4 <= eps && segedit_on_segment(cx, cy, dx, dy, bx, by)) return 1;
    return 0;
}

/* Split `segid` by the finite segment pA->pB.  The sign of
 *   cross = (bx-ax)*(y-ay) - (by-ay)*(x-ax)
 * at the cell centre (c+0.5, r+0.5) classifies cells: cross >= 0 keeps `segid`,
 * cross < 0 is a candidate for the fresh id.  But the sign is taken against the
 * *infinite* line, so a segment that only grazes the region (or a short cut across
 * one arm of a U-shaped region) must not relabel everything on that half-plane.
 *
 * The drawn segment is therefore used as a barrier between 4-adjacent cells: an edge
 * is removed when the finite segment crosses the segment joining the two cell centres.
 * The region is then flood-filled; only components that contain *no* keep cell are
 * relabelled.  If the segment does not actually separate the region (no component is
 * fully cut off, or one half is empty) the edit is refused with SEGEDIT_ENOSPLIT. */
int segedit_split_cells(segedit_grid *g, int segid,
                        double ax, double ay, double bx, double by,
                        int *out_newid) {
    if (!g || !g->cells) return SEGEDIT_EARG;
    segid &= SEGEDIT_ID_MASK;
    if (segid <= 0) return SEGEDIT_EARG;
    if (ax == bx && ay == by) return SEGEDIT_ELINE;
    if (segedit_count_id(g, segid) == 0) return SEGEDIT_ENOSEG;

    size_t width = (size_t)g->w, height = (size_t)g->h;
    size_t cell_count = width * height;
    uint8_t *visited = (uint8_t *)calloc(cell_count, 1);
    uint8_t *reassign = (uint8_t *)calloc(cell_count, 1);
    int *queue = (int *)malloc(sizeof(int) * cell_count);
    if (!visited || !reassign || !queue) {
        free(visited); free(reassign); free(queue);
        return SEGEDIT_EIO;
    }

    int have_keep_component = 0, have_cut_component = 0;
    static const int dc[4] = { 1, -1, 0,  0 };
    static const int dr[4] = { 0,  0, 1, -1 };
    for (size_t start = 0; start < cell_count; start++) {
        if (visited[start]) continue;
        if ((int)(g->cells[start] & SEGEDIT_ID_MASK) != segid) continue;

        size_t qhead = 0, qtail = 0;
        queue[qtail++] = (int)start;
        visited[start] = 1;
        int component_keeps = 0;
        while (qhead < qtail) {
            int cell = queue[qhead++];
            int row = (int)((size_t)cell / width);
            int col = (int)((size_t)cell % width);
            double x = (double)col + 0.5, y = (double)row + 0.5;
            double cross = (bx - ax) * (y - ay) - (by - ay) * (x - ax);
            if (cross >= 0.0) component_keeps = 1;   /* on-line cells keep segid */

            for (int k = 0; k < 4; k++) {
                int nrow = row + dr[k], ncol = col + dc[k];
                if (nrow < 0 || ncol < 0 || nrow >= (int)height || ncol >= (int)width) continue;
                int neighbor = nrow * (int)width + ncol;
                if (visited[neighbor]) continue;
                if ((int)(g->cells[neighbor] & SEGEDIT_ID_MASK) != segid) continue;
                /* the drawn segment cuts the edge between the two cell centres */
                if (segedit_segments_intersect(ax, ay, bx, by,
                                               x, y, (double)ncol + 0.5, (double)nrow + 0.5))
                    continue;
                visited[neighbor] = 1;
                queue[qtail++] = neighbor;
            }
        }
        if (component_keeps) {
            have_keep_component = 1;
        } else {
            have_cut_component = 1;
            for (size_t t = 0; t < qtail; t++) reassign[queue[t]] = 1;
        }
    }

    if (!have_keep_component || !have_cut_component) {
        free(visited); free(reassign); free(queue);
        return SEGEDIT_ENOSPLIT;
    }

    int newid = segedit_next_user_id(g);
    if (!newid) {
        free(visited); free(reassign); free(queue);
        return SEGEDIT_ENOID;
    }
    for (size_t i = 0; i < cell_count; i++) {
        if (!reassign[i]) continue;
        g->cells[i] = (uint8_t)((g->cells[i] & SEGEDIT_KEEP_MASK) | (newid & SEGEDIT_ID_MASK));
    }
    free(visited); free(reassign); free(queue);
    if (out_newid) *out_newid = newid;
    return SEGEDIT_OK;
}

void segedit_mm_to_cell(const segedit_mm_map *opt, long x_mm, long y_mm,
                        double *cx, double *cy) {
    int scale_mm = (opt && opt->mm_per_cell > 0) ? opt->mm_per_cell : 50;
    int dx = opt ? opt->dx : 0;
    int dy = opt ? opt->dy : 0;
    if (cx) *cx = (double)x_mm / (double)scale_mm - (double)dx;
    if (cy) *cy = (double)y_mm / (double)scale_mm - (double)dy;
}

int segedit_split_mm(segedit_grid *g, int segid,
                     long x1, long y1, long x2, long y2,
                     const segedit_mm_map *opt, int *out_newid) {
    double ax, ay, bx, by;
    segedit_mm_to_cell(opt, x1, y1, &ax, &ay);
    segedit_mm_to_cell(opt, x2, y2, &bx, &by);
    return segedit_split_cells(g, segid, ax, ay, bx, by, out_newid);
}

/* ===================================================================================== *
 *  RRSEG sidecar                                                                        *
 * ===================================================================================== */
static const unsigned char SEG_MAGIC[8] = { 'R', 'R', 'S', 'E', 'G', 0, 0, 1 };

int segedit_sidecar_read(const char *path, segedit_grid *g, segedit_meta *m) {
    if (!path || !g) return SEGEDIT_EARG;
    segedit_grid_init(g);
    int fd = open(path, O_RDONLY);
    if (fd < 0) return SEGEDIT_EIO;
    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size < SEGEDIT_HDR_SIZE) { close(fd); return SEGEDIT_EMAGIC; }
    size_t file_size = (size_t)st.st_size;
    unsigned char *data = (unsigned char *)malloc(file_size);
    if (!data) { close(fd); return SEGEDIT_EIO; }
    size_t received = 0;
    while (received < file_size) {
        ssize_t nread = read(fd, data + received, file_size - received);
        if (nread <= 0) { free(data); close(fd); return SEGEDIT_EIO; }
        received += (size_t)nread;
    }
    close(fd);

    if (memcmp(data, SEG_MAGIC, 8) != 0) { free(data); return SEGEDIT_EMAGIC; }
    uint32_t w = rd_u32(data + 32), h = rd_u32(data + 36);
    if (w == 0 || h == 0 || (uint64_t)w * h > (uint64_t)1 << 26 ||
        file_size < SEGEDIT_HDR_SIZE + (size_t)w * h + 4) { free(data); return SEGEDIT_EMAGIC; }

    /* CRC over header (crc field zeroed) + payload */
    unsigned char *crc_copy = (unsigned char *)malloc(file_size);
    if (!crc_copy) { free(data); return SEGEDIT_EIO; }
    memcpy(crc_copy, data, file_size);
    uint32_t stored_crc = rd_u32(data + 48);
    memset(crc_copy + 48, 0, 4);
    uint32_t computed_crc = segedit_crc32(crc_copy, file_size);
    if (computed_crc != stored_crc) { free(crc_copy); free(data); return SEGEDIT_EMAGIC; }
    free(crc_copy);

    int result_code = segedit_grid_alloc(g, (int)w, (int)h);
    if (result_code != SEGEDIT_OK) { free(data); return result_code; }
    memcpy(g->cells, data + SEGEDIT_HDR_SIZE, (size_t)w * h);

    if (m) {
        m->version       = rd_u32(data + 8);
        m->map_timestamp = rd_u32(data + 12);
        m->map_id        = rd_u32(data + 16);
        m->scale         = rd_f32(data + 20);
        m->origin_x      = rd_i32(data + 24);
        m->origin_y      = rd_i32(data + 28);
        m->width         = w;
        m->height        = h;
        m->dock_x        = rd_i32(data + 40);
        m->dock_y        = rd_i32(data + 44);
        m->reserved[0]   = rd_u32(data + 52);
        m->reserved[1]   = rd_u32(data + 56);
        m->reserved[2]   = rd_u32(data + 60);
    }
    free(data);
    return SEGEDIT_OK;
}

int segedit_room_table(const segedit_grid *g, segedit_room_t *out, int max) {
    if (!g || !g->cells) return SEGEDIT_EARG;
    int used_ids[64]; memset(used_ids, 0, sizeof used_ids);
    long area_cells[64]; memset(area_cells, 0, sizeof area_cells);
    double sum_x[64], sum_y[64]; memset(sum_x, 0, sizeof sum_x); memset(sum_y, 0, sizeof sum_y);
    size_t cell_count = (size_t)g->w * g->h;
    for (size_t i = 0; i < cell_count; i++) {
        int id = g->cells[i] & SEGEDIT_ID_MASK;
        if (!id) continue;
        used_ids[id] = 1; area_cells[id]++;
        sum_x[id] += (double)(int)(i % (size_t)g->w);
        sum_y[id] += (double)(int)(i / (size_t)g->w);
    }
    int count = 0;
    for (int id = 1; id < 64; id++) {
        if (!used_ids[id]) continue;
        if (out && count < max) {
            out[count].id = id;
            out[count].area = area_cells[id];
            out[count].cx = area_cells[id] ? (int)(sum_x[id] / (double)area_cells[id]) : 0;
            out[count].cy = area_cells[id] ? (int)(sum_y[id] / (double)area_cells[id]) : 0;
        }
        count++;
    }
    return count;
}

int segedit_sidecar_write_atomic(const char *path, const segedit_grid *g,
                                 const segedit_meta *seed) {
    if (!path || !g || !g->cells || g->w <= 0 || g->h <= 0) return SEGEDIT_EARG;
    int room_count = segedit_room_table(g, NULL, 0);
    if (room_count < 0) return room_count;

    size_t cell_count = (size_t)g->w * g->h;
    size_t payload_size = cell_count + 4 + (size_t)room_count * 16;
    unsigned char *payload_buf = (unsigned char *)malloc(payload_size);
    if (!payload_buf) return SEGEDIT_EIO;
    unsigned char *cursor = payload_buf;
    /* store only the id + preserved high bits (the grid is already normalized) */
    memcpy(cursor, g->cells, cell_count); cursor += cell_count;
    wr_u32(cursor, (uint32_t)room_count); cursor += 4;

    segedit_room_t room_table[64];
    room_count = segedit_room_table(g, room_table, 64);
    for (int room_index = 0; room_index < room_count; room_index++) {
        cursor[0] = (unsigned char)(room_table[room_index].id & 0xff);
        cursor[1] = cursor[2] = cursor[3] = 0;
        wr_u32(cursor + 4, (uint32_t)room_table[room_index].area);
        wr_i32(cursor + 8, room_table[room_index].cx);
        wr_i32(cursor + 12, room_table[room_index].cy);
        cursor += 16;
    }

    unsigned char header[SEGEDIT_HDR_SIZE];
    memset(header, 0, sizeof header);
    memcpy(header, SEG_MAGIC, 8);
    wr_u32(header + 8, seed ? seed->version : 1u);
    wr_u32(header + 12, seed ? seed->map_timestamp : 0u);
    wr_u32(header + 16, seed ? seed->map_id : 0u);
    wr_f32(header + 20, seed ? seed->scale : 0.0f);
    wr_i32(header + 24, seed ? seed->origin_x : 0);
    wr_i32(header + 28, seed ? seed->origin_y : 0);
    wr_u32(header + 32, (uint32_t)g->w);
    wr_u32(header + 36, (uint32_t)g->h);
    wr_i32(header + 40, seed ? seed->dock_x : 0);
    wr_i32(header + 44, seed ? seed->dock_y : 0);
    wr_u32(header + 48, 0);
    wr_u32(header + 52, seed ? seed->reserved[0] : 0u);
    wr_u32(header + 56, seed ? seed->reserved[1] : 0u);
    wr_u32(header + 60, seed ? seed->reserved[2] : 0u);

    /* CRC over header||payload as one stream (crc field zeroed) */
    uint32_t crc;
    {
        unsigned char *crc_stream = (unsigned char *)malloc(sizeof header + payload_size);
        if (!crc_stream) { free(payload_buf); return SEGEDIT_EIO; }
        memcpy(crc_stream, header, sizeof header);
        memcpy(crc_stream + sizeof header, payload_buf, payload_size);
        crc = segedit_crc32(crc_stream, sizeof header + payload_size);
        free(crc_stream);
    }
    wr_u32(header + 48, crc);

    char tmp_path[4096];
    snprintf(tmp_path, sizeof tmp_path, "%s.tmp", path);
    int fd = open(tmp_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) { free(payload_buf); return SEGEDIT_EIO; }
    int ok = 1;
    const unsigned char *chunks[2] = { header, payload_buf };
    size_t lens[2] = { sizeof header, payload_size };
    for (int chunk = 0; chunk < 2 && ok; chunk++) {
        size_t offset = 0;
        while (offset < lens[chunk]) {
            ssize_t nwritten = write(fd, chunks[chunk] + offset, lens[chunk] - offset);
            if (nwritten <= 0) { ok = 0; break; }
            offset += (size_t)nwritten;
        }
    }
    if (ok && fsync(fd) != 0) ok = 0;
    if (close(fd) != 0) ok = 0;
    if (ok && rename(tmp_path, path) != 0) ok = 0;
    if (!ok) unlink(tmp_path);
    free(payload_buf);
    return ok ? SEGEDIT_OK : SEGEDIT_EIO;
}
