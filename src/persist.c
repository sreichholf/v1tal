/* persist.c -- see persist.h.  Pure libc, no robot deps.
 *
 * All writes are crash-safe: a `.tmp` sibling is written, fsync()'d, then rename()d over the
 * destination.  Nothing here calls into the nav or AppProxy; the integration points are named
 * in TECHNICAL.md / README.md.
 *
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include "persist.h"
#include "crc32.h"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#if !defined(PATH_MAX)
#define PATH_MAX 4096
#endif

/* ===================================================================================== *
 *  paths                                                                                *
 * ===================================================================================== */
static char g_root[PATH_MAX];
static int  g_root_set = 0;

void persist_set_root(const char *root) {
    if (root && *root) {
        snprintf(g_root, sizeof g_root, "%s", root);
    } else {
        const char *env = getenv("PERSIST_ROOT");
        if (env && *env) snprintf(g_root, sizeof g_root, "%s", env);
        else             snprintf(g_root, sizeof g_root, "%s", PERSIST_DEFAULT_ROOT);
    }
    g_root_set = 1;
}

const char *persist_root(void) {
    if (!g_root_set) persist_set_root(NULL);
    return g_root;
}

static const char *join_root_path(char *buf, size_t cap, const char *name) {
    size_t root_len = strlen(persist_root());
    size_t name_len = strlen(name);
    if (root_len + name_len + 2 <= cap) {
        memcpy(buf, persist_root(), root_len);
        buf[root_len] = '/';
        memcpy(buf + root_len + 1, name, name_len + 1);
    } else {
        buf[0] = '\0';
    }
    return buf;
}
#define P_LAST_MAP      "last_map"
#define P_SLAM_INFO     "slam_info.cfg"
#define P_LAST_SEGMAP   "last_segmap"
#define P_DIR           "persist"
#define P_MAP0          "persist/map0"
#define P_SEG0          "persist/seg0"
#define P_SLAM0         "persist/slam_info0.cfg"
#define P_FRAME0        "persist/frame0"
#define P_PERSIST_LOCK  ".v1persist.lock"

/* Staging names used by persist_map_save_locked(): the new generation is written
 * here first and only renamed over the P_* names (frame0 last) once every write has
 * succeeded.  A crash or write error before the final rename leaves the previous
 * snapshot fully intact. */
#define P_MAP0_NEW      "persist/map0.new"
#define P_SEG0_NEW      "persist/seg0.new"
#define P_SLAM0_NEW     "persist/slam_info0.cfg.new"
#define P_FRAME0_NEW    "persist/frame0.new"

/* Sanity cap on a V1 SLAM map header.  read_map_frame_at()/map_frame_from_bytes()
 * require an *exact* size (mirroring guard.c's v1g_read_map) computed in 64 bits and
 * reject absurd dimensions before any arithmetic can wrap. */
#define PERSIST_MAP_MAX_DIM    16384u
#define PERSIST_MAP_MAX_CELLS  (16u * 1024u * 1024u)

int persist_init(void) {
    if (!g_root_set) persist_set_root(NULL);
    char dir[PATH_MAX];
    join_root_path(dir, sizeof dir, P_DIR);
    if (mkdir(dir, 0755) != 0 && errno != EEXIST) return -1;
    return 0;
}

/* ===================================================================================== *
 *  cross-process write lock (.v1persist.lock)                                           *
 * ===================================================================================== */
/* A separate, short-lived advisory lock from the guard's lifetime .v1guard.lock.
 * Every snapshot writer takes it: the LD_PRELOAD shim's save/reset/recover handlers
 * and the guard's own snapshot/rotate writes both enter through the persist_map_*()
 * API below.  Because the guard does NOT hold this lock for its lifetime, a shim
 * write can never block behind a running guard (the failure mode a shared
 * .v1guard.lock would create).  flock() is released by the kernel if a writer dies,
 * and the lock is only ever taken and released around a single bounded write with no
 * nested acquisition, so deadlock is impossible.  Returns an fd >= 0, or -1 when the
 * lock file cannot be created (callers then proceed best-effort, exactly as before
 * this lock existed). */
int persist_write_lock(void) {
    char path[PATH_MAX];
    join_root_path(path, sizeof path, P_PERSIST_LOCK);
    int fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (fd < 0) return -1;
    if (flock(fd, LOCK_EX) != 0) { close(fd); return -1; }
    return fd;
}

void persist_write_unlock(int lock_fd) {
    if (lock_fd >= 0) { flock(lock_fd, LOCK_UN); close(lock_fd); }
}

/* ===================================================================================== *
 *  crc32 (IEEE, same polynomial as zlib.crc32)                *
 * ===================================================================================== */
/* The CRC table is built into a function-local array on every call.  persist_crc32()
 * is reached from the shim's map thread *and* the MIIO thread (and from the
 * single-threaded guard), so a shared lazily-filled static table is a data race: on
 * ARM a torn read of the table yields a wrong checksum and a spurious snapshot
 * rejection.  A per-call table has no shared state to tear, and
 * 256*8 cheap ops are negligible next to hashing a map.  This keeps persist.c free
 * of any threading/pthread dependency (the guard does not link -lpthread). */
uint32_t persist_crc32(const void *data, size_t len) {
    uint32_t table[256];
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t crc_value = i;
        for (int bit = 0; bit < 8; bit++)
            crc_value = (crc_value & 1) ? (CRC32_POLYNOMIAL_REFLECTED ^ (crc_value >> 1))
                                        : (crc_value >> 1);
        table[i] = crc_value;
    }
    const unsigned char *bytes = (const unsigned char *)data;
    uint32_t crc = CRC32_INITIAL_VALUE;
    for (size_t i = 0; i < len; i++) crc = table[(crc ^ bytes[i]) & 0xff] ^ (crc >> 8);
    return crc ^ CRC32_FINAL_XOR;
}

/* ===================================================================================== *
 *  little-endian accessors                                                              *
 * ===================================================================================== */
static uint32_t rd_u32(const unsigned char *bytes) {
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}
static int32_t rd_i32(const unsigned char *bytes) { return (int32_t)rd_u32(bytes); }
static float   rd_f32(const unsigned char *bytes) {
    uint32_t bits = rd_u32(bytes); float value; memcpy(&value, &bits, 4); return value;
}
static void wr_u32(unsigned char *bytes, uint32_t value) {
    bytes[0] = (unsigned char)value; bytes[1] = (unsigned char)(value >> 8);
    bytes[2] = (unsigned char)(value >> 16); bytes[3] = (unsigned char)(value >> 24);
}
static void wr_i32(unsigned char *bytes, int32_t value) { wr_u32(bytes, (uint32_t)value); }
static void wr_f32(unsigned char *bytes, float value) {
    uint32_t bits; memcpy(&bits, &value, 4); wr_u32(bytes, bits);
}

/* ===================================================================================== *
 *  file helpers                                                                         *
 * ===================================================================================== */
static int read_all(const char *path, unsigned char **out, size_t *len) {
    int fd = open(path, O_RDONLY);
    if (fd < 0) return -1;
    struct stat st;
    if (fstat(fd, &st) != 0 || st.st_size < 0) { close(fd); return -1; }
    size_t file_size = (size_t)st.st_size;
    unsigned char *buf = (unsigned char *)malloc(file_size ? file_size : 1);
    if (!buf) { close(fd); return -1; }
    size_t got = 0;
    while (got < file_size) {
        ssize_t nread = read(fd, buf + got, file_size - got);
        if (nread <= 0) { free(buf); close(fd); return -1; }
        got += (size_t)nread;
    }
    close(fd);
    *out = buf; *len = file_size;
    return 0;
}

/* fsync the directory holding `path` so that the rename() that published a file is
 * itself durable across a power cut.  Best effort: a failure here
 * is not fatal, the file content is already safe. */
static void fsync_parent_dir(const char *path) {
    char dir[PATH_MAX];
    const char *slash = strrchr(path, '/');
    if (!slash) {
        dir[0] = '.'; dir[1] = '\0';
    } else {
        size_t len = (size_t)(slash - path);
        if (len == 0) { dir[0] = '/'; dir[1] = '\0'; }
        else {
            if (len >= sizeof dir) len = sizeof dir - 1;
            memcpy(dir, path, len);
            dir[len] = '\0';
        }
    }
    int dir_fd = open(dir, O_RDONLY | O_DIRECTORY);
    if (dir_fd < 0) return;
    (void)fsync(dir_fd);
    close(dir_fd);
}

static int write_atomic(const char *path, const void *data, size_t len) {
    char tmp[PATH_MAX];
    snprintf(tmp, sizeof tmp, "%s.tmp", path);
    int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return -1;
    const unsigned char *src = (const unsigned char *)data;
    size_t off = 0;
    while (off < len) {
        ssize_t nwritten = write(fd, src + off, len - off);
        if (nwritten <= 0) { close(fd); unlink(tmp); return -1; }
        off += (size_t)nwritten;
    }
    if (fsync(fd) != 0) { close(fd); unlink(tmp); return -1; }
    if (close(fd) != 0) { unlink(tmp); return -1; }
    if (rename(tmp, path) != 0) { unlink(tmp); return -1; }
    fsync_parent_dir(path);
    return 0;
}

static int copy_file(const char *src, const char *dst) {
    unsigned char *data; size_t len;
    if (read_all(src, &data, &len) != 0) return -1;
    int rc = write_atomic(dst, data, len);
    free(data);
    return rc;
}

/* Write `data` to `path` in place (create/truncate), fsync it, and return 0/-1.
 * Unlike write_atomic() this publishes nothing by itself: callers use it only for
 * the persist/save staging names (P_*_NEW), which are renamed into place once every
 * staged write has succeeded, with frame0 last acting as the commit point. */
static int write_file_durable(const char *path, const void *data, size_t len) {
    int fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
    if (fd < 0) return -1;
    const unsigned char *src = (const unsigned char *)data;
    size_t off = 0;
    while (off < len) {
        ssize_t nwritten = write(fd, src + off, len - off);
        if (nwritten <= 0) { close(fd); unlink(path); return -1; }
        off += (size_t)nwritten;
    }
    if (fsync(fd) != 0) { close(fd); unlink(path); return -1; }
    if (close(fd) != 0) { unlink(path); return -1; }
    return 0;
}

static int file_exists(const char *path) {
    struct stat st;
    return (stat(path, &st) == 0 && S_ISREG(st.st_mode));
}

/* ===================================================================================== *
 *  slam_info.cfg timestamp                                                              *
 * ===================================================================================== */
/* Parse the `timestamp` value from an in-memory slam_info.cfg.  Returns 0 and sets
 * *ts when a value was found, -1 otherwise.  Used both by the path reader and by the
 * snapshot writer (which reads the file exactly once). */
static int slam_ts_from_bytes(const unsigned char *data, size_t len, uint32_t *ts) {
    const char *key = "timestamp";
    size_t key_len = strlen(key);
    uint32_t value = 0; int found = 0;
    for (size_t i = 0; i + key_len <= len; i++) {
        if (memcmp(data + i, key, key_len) != 0) continue;
        size_t cursor = i + key_len;
        while (cursor < len && data[cursor] != '=' && data[cursor] != ':') cursor++;
        if (cursor < len) cursor++;
        while (cursor < len && (data[cursor] == ' ' || data[cursor] == '\t')) cursor++;
        unsigned long accumulator = 0; int saw_digit = 0;
        while (cursor < len && data[cursor] >= '0' && data[cursor] <= '9') {
            accumulator = accumulator * 10 + (unsigned)(data[cursor] - '0'); cursor++; saw_digit = 1;
        }
        if (saw_digit) { value = (uint32_t)accumulator; found = 1; }
        break;
    }
    if (!found) return -1;
    *ts = value;
    return 0;
}

int persist_read_slam_timestamp(uint32_t *ts) {
    char path[PATH_MAX];
    join_root_path(path, sizeof path, P_SLAM_INFO);
    unsigned char *data; size_t len;
    if (read_all(path, &data, &len) != 0) return -1;
    int rc = slam_ts_from_bytes(data, len, ts);
    free(data);
    return rc;
}

/* ===================================================================================== *
 *  frame readers                                                                        *
 * ===================================================================================== */
/* V1 SLAM map: 56 + w*h*4 bytes, float32 cells.  scale@+20 origin@+24/+28 w@+32 h@+36.
 * The dims are validated exactly the way guard.c's v1g_read_map() does: a complete map
 * is exactly 56 + w*h*4 bytes (computed in 64 bits so 32-bit ARM cannot wrap), and
 * absurd dims are rejected before any allocation/multiplication. */
static int map_frame_from_bytes(const unsigned char *data, size_t len, persist_frame_rec_t *rec) {
    if (len < 56) return -1;
    uint32_t w = rd_u32(data + 32), h = rd_u32(data + 36);
    if (w == 0 || h == 0 || w > PERSIST_MAP_MAX_DIM || h > PERSIST_MAP_MAX_DIM) return -1;
    uint64_t cells = (uint64_t)w * (uint64_t)h;
    if (cells > (uint64_t)PERSIST_MAP_MAX_CELLS) return -1;
    uint64_t expected_size = 56ull + cells * 4ull;
    if (expected_size != (uint64_t)len) return -1;
    rec->map_scale = rd_f32(data + 20);
    rec->map_ox    = rd_i32(data + 24);
    rec->map_oy    = rd_i32(data + 28);
    rec->map_w = w; rec->map_h = h;
    rec->map_crc = persist_crc32(data, len);
    return 0;
}

static int read_map_frame_at(const char *path, persist_frame_rec_t *rec) {
    unsigned char *data; size_t len;
    if (read_all(path, &data, &len) != 0) return -1;
    memset(rec, 0, sizeof *rec);
    int rc = map_frame_from_bytes(data, len, rec);
    free(data);
    return rc;
}

/* verify the sidecar CRC over header (crc field zeroed) + payload. */
static int seg_crc_ok(const unsigned char *data, size_t len, uint32_t stored_crc) {
    if (len < 64) return 0;
    unsigned char *copy = (unsigned char *)malloc(len);
    if (!copy) return 0;
    memcpy(copy, data, len);
    memset(copy + 48, 0, 4);
    uint32_t computed = persist_crc32(copy, len);
    free(copy);
    return (computed == stored_crc);
}

/* Parse a last_segmap sidecar from an in-memory buffer.  Returns 0, -1 (bad magic), or
 * -2 (CRC mismatch); fills the seg half of *rec.  Computing seg_file_crc here means the
 * bytes that were CRC-checked are the same bytes the caller writes to seg0. */
static int seg_frame_from_bytes(const unsigned char *data, size_t len, persist_frame_rec_t *rec) {
    static const unsigned char MAGIC[8] = { 'R', 'R', 'S', 'E', 'G', 0, 0, 1 };
    if (len < 64 || memcmp(data, MAGIC, 8) != 0) return -1;
    uint32_t stored_crc = rd_u32(data + 48);
    if (!seg_crc_ok(data, len, stored_crc)) return -2;   /* corrupted sidecar */
    rec->seg_ts     = rd_u32(data + 12);
    rec->seg_map_id = rd_u32(data + 16);
    rec->seg_scale  = rd_f32(data + 20);
    rec->seg_ox     = rd_i32(data + 24);
    rec->seg_oy     = rd_i32(data + 28);
    rec->seg_w      = rd_u32(data + 32);
    rec->seg_h      = rd_u32(data + 36);
    rec->seg_crc    = stored_crc;
    rec->seg_file_crc = persist_crc32(data, len);
    return 0;
}

static int read_seg_frame_at(const char *path, persist_frame_rec_t *rec) {
    unsigned char *data; size_t len;
    if (read_all(path, &data, &len) != 0) return -1;
    memset(rec, 0, sizeof *rec);
    int rc = seg_frame_from_bytes(data, len, rec);
    free(data);
    return rc;
}

int persist_read_lastmap_frame(persist_frame_rec_t *rec) {
    char path[PATH_MAX];
    join_root_path(path, sizeof path, P_LAST_MAP);
    if (read_map_frame_at(path, rec) != 0) return -1;
    uint32_t timestamp = 0;
    if (persist_read_slam_timestamp(&timestamp) == 0) rec->map_ts = timestamp;
    return 0;
}

int persist_read_segmap_frame(persist_frame_rec_t *rec) {
    char path[PATH_MAX];
    join_root_path(path, sizeof path, P_LAST_SEGMAP);
    return read_seg_frame_at(path, rec);
}

static int float_close_enough(float value_a, float value_b) {
    float diff = value_a - value_b; if (diff < 0) diff = -diff;
    return diff < 0.01f;
}

int persist_frames_linked(const persist_frame_rec_t *map, const persist_frame_rec_t *seg) {
    if (!map || !seg) return 0;
    if (seg->seg_w == 0 || seg->seg_h == 0) return 0;
    if (!float_close_enough(map->map_scale, seg->seg_scale)) return 0;
    if (map->map_ox != seg->seg_ox || map->map_oy != seg->seg_oy) return 0;
    /* dims agree up to the documented v1seg pad (0 or 2*PAD on each axis) */
    long width_delta = (long)seg->seg_w - (long)map->map_w;
    long height_delta = (long)seg->seg_h - (long)map->map_h;
    if (width_delta != height_delta) return 0;
    if (!(width_delta == 0 || width_delta == 2 * PERSIST_SEG_PAD)) return 0;
    /* nonzero timestamps / map-ids must agree; zero means "unknown" and is tolerated */
    if (map->map_ts && seg->seg_ts && map->map_ts != seg->seg_ts) return 0;
    if (map->map_id && seg->seg_map_id && map->map_id != seg->seg_map_id) return 0;
    return 1;
}

/* ===================================================================================== *
 *  frame record serialization                                                           *
 * ===================================================================================== */
#define FRAME_REC_SIZE 128
#define FRAME_CRC_OFF  88
static const unsigned char FRAME_MAGIC[8] = { 'R', 'R', 'P', 'F', 'R', 'M', 0, 1 };

static void frame_pack(unsigned char buf[FRAME_REC_SIZE], const persist_frame_rec_t *rec) {
    memset(buf, 0, FRAME_REC_SIZE);
    memcpy(buf, FRAME_MAGIC, 8);
    wr_u32(buf + 8, 1);
    wr_u32(buf + 12, rec->map_ts);
    wr_u32(buf + 16, rec->map_id);
    wr_u32(buf + 20, rec->map_w);
    wr_u32(buf + 24, rec->map_h);
    wr_f32(buf + 28, rec->map_scale);
    wr_i32(buf + 32, rec->map_ox);
    wr_i32(buf + 36, rec->map_oy);
    wr_u32(buf + 40, rec->map_crc);
    wr_u32(buf + 44, rec->seg_ts);
    wr_u32(buf + 48, rec->seg_map_id);
    wr_u32(buf + 52, rec->seg_w);
    wr_u32(buf + 56, rec->seg_h);
    wr_f32(buf + 60, rec->seg_scale);
    wr_i32(buf + 64, rec->seg_ox);
    wr_i32(buf + 68, rec->seg_oy);
    wr_u32(buf + 72, rec->seg_crc);
    wr_u32(buf + 76, rec->seg_file_crc);
    wr_i32(buf + 80, rec->pad);
    wr_u32(buf + 84, rec->has_seg);
    uint32_t crc = persist_crc32(buf, FRAME_REC_SIZE);
    wr_u32(buf + FRAME_CRC_OFF, crc);
}

static int frame_unpack(const unsigned char *buf, size_t len, persist_frame_rec_t *rec) {
    if (len < FRAME_REC_SIZE || memcmp(buf, FRAME_MAGIC, 8) != 0) return -1;
    unsigned char copy[FRAME_REC_SIZE];
    memcpy(copy, buf, FRAME_REC_SIZE);
    uint32_t stored_crc = rd_u32(copy + FRAME_CRC_OFF);
    wr_u32(copy + FRAME_CRC_OFF, 0);
    if (persist_crc32(copy, FRAME_REC_SIZE) != stored_crc) return -1;
    uint32_t version = rd_u32(buf + 8);
    if (version != 1) return -1;   /* only the documented record layout is accepted */
    memset(rec, 0, sizeof *rec);
    memcpy(rec->magic, buf, 8);
    rec->version = version;
    rec->map_ts = rd_u32(buf + 12);
    rec->map_id = rd_u32(buf + 16);
    rec->map_w = rd_u32(buf + 20); rec->map_h = rd_u32(buf + 24);
    rec->map_scale = rd_f32(buf + 28);
    rec->map_ox = rd_i32(buf + 32); rec->map_oy = rd_i32(buf + 36);
    rec->map_crc = rd_u32(buf + 40);
    rec->seg_ts = rd_u32(buf + 44); rec->seg_map_id = rd_u32(buf + 48);
    rec->seg_w = rd_u32(buf + 52); rec->seg_h = rd_u32(buf + 56);
    rec->seg_scale = rd_f32(buf + 60);
    rec->seg_ox = rd_i32(buf + 64); rec->seg_oy = rd_i32(buf + 68);
    rec->seg_crc = rd_u32(buf + 72);
    rec->seg_file_crc = rd_u32(buf + 76);
    rec->pad = rd_i32(buf + 80);
    rec->has_seg = rd_u32(buf + 84);
    rec->crc = stored_crc;
    return 0;
}

void persist_frame_str(const persist_frame_rec_t *rec, char *buf, size_t cap) {
    snprintf(buf, cap,
             "map{ts=%u id=%u %ux%u scale=%.3f origin=(%d,%d) crc=%08x} "
             "seg{%ux%u scale=%.3f origin=(%d,%d) ts=%u id=%u crc=%08x pad=%d has_seg=%u}",
             rec->map_ts, rec->map_id, rec->map_w, rec->map_h, rec->map_scale, rec->map_ox, rec->map_oy, rec->map_crc,
             rec->seg_w, rec->seg_h, rec->seg_scale, rec->seg_ox, rec->seg_oy, rec->seg_ts,
             rec->seg_map_id, rec->seg_crc, rec->pad, rec->has_seg);
}

/* ===================================================================================== *
 *  snapshot / recover / reset                                                           *
 * ===================================================================================== */
static int snapshot_exists(void) {
    char path[PATH_MAX];
    return file_exists(join_root_path(path, sizeof path, P_MAP0)) &&
           file_exists(join_root_path(path, sizeof path, P_FRAME0));
}

/* The actual save body; callers hold the .v1persist.lock write lock.
 *
 * Single-read + staged commit:
 *   * every source (last_map, last_segmap, slam_info.cfg) is read exactly ONCE into
 *     memory, and the CRC stored in frame0 is computed over the very same buffer that
 *     is written to the snapshot, so a concurrent nav write can never make frame0
 *     disagree with map0;
 *   * the new generation is written under `*.new` staging names and only renamed over
 *     the live snapshot names once every staged write has succeeded, with frame0
 *     renamed LAST as the commit point.  A failure at any earlier step leaves the
 *     previous map0/frame0/... untouched, so recover still succeeds.
 *
 * slam_info.cfg timestamp 0 means the nav cleared the map; saving
 * it verbatim would just make the guard restore a freshly-cleared map on every tick,
 * so the save is refused instead (the previous snapshot is kept). */
static void unlink_new_staged(const char *map_new, const char *seg_new,
                              const char *slam_new, const char *frame_new) {
    unlink(map_new); unlink(seg_new); unlink(slam_new); unlink(frame_new);
}

static int persist_map_save_locked(void) {
    char last_map_path[PATH_MAX], last_segmap_path[PATH_MAX], slam_info_path[PATH_MAX];
    char snap_map_path[PATH_MAX], snap_seg_path[PATH_MAX];
    char snap_slam_path[PATH_MAX], snap_frame_path[PATH_MAX];
    char stage_map_path[PATH_MAX], stage_seg_path[PATH_MAX];
    char stage_slam_path[PATH_MAX], stage_frame_path[PATH_MAX];
    join_root_path(last_map_path, sizeof last_map_path, P_LAST_MAP);
    join_root_path(snap_map_path, sizeof snap_map_path, P_MAP0);
    join_root_path(last_segmap_path, sizeof last_segmap_path, P_LAST_SEGMAP);
    join_root_path(snap_seg_path, sizeof snap_seg_path, P_SEG0);
    join_root_path(slam_info_path, sizeof slam_info_path, P_SLAM_INFO);
    join_root_path(snap_slam_path, sizeof snap_slam_path, P_SLAM0);
    join_root_path(snap_frame_path, sizeof snap_frame_path, P_FRAME0);
    join_root_path(stage_map_path, sizeof stage_map_path, P_MAP0_NEW);
    join_root_path(stage_seg_path, sizeof stage_seg_path, P_SEG0_NEW);
    join_root_path(stage_slam_path, sizeof stage_slam_path, P_SLAM0_NEW);
    join_root_path(stage_frame_path, sizeof stage_frame_path, P_FRAME0_NEW);

    /* --- read the live map once --- */
    unsigned char *map_bytes = NULL; size_t map_len = 0;
    if (read_all(last_map_path, &map_bytes, &map_len) != 0) return -1;   /* nothing to save yet */
    persist_frame_rec_t rec;
    memset(&rec, 0, sizeof rec);
    if (map_frame_from_bytes(map_bytes, map_len, &rec) != 0) { free(map_bytes); return -1; }

    /* --- slam_info.cfg: read once; refuse a cleared (ts == 0) map --- */
    unsigned char *slam_bytes = NULL; size_t slam_len = 0;
    int have_slam = (read_all(slam_info_path, &slam_bytes, &slam_len) == 0);
    if (have_slam) {
        uint32_t timestamp = 0;
        if (slam_ts_from_bytes(slam_bytes, slam_len, &timestamp) == 0 && timestamp == 0) {
            free(map_bytes); free(slam_bytes);
            return PERSIST_ERR_SLAM_TS_ZERO;
        }
        if (slam_ts_from_bytes(slam_bytes, slam_len, &rec.map_ts) != 0) rec.map_ts = 0;
    }

    /* --- last_segmap: read once; merge only when CRC-clean AND frame-linked --- */
    unsigned char *seg_bytes = NULL; size_t seg_len = 0;
    int have_seg = 0;
    if (read_all(last_segmap_path, &seg_bytes, &seg_len) == 0) {
        persist_frame_rec_t seg_frame;
        memset(&seg_frame, 0, sizeof seg_frame);
        if (seg_frame_from_bytes(seg_bytes, seg_len, &seg_frame) == 0 &&
            persist_frames_linked(&rec, &seg_frame)) {
            rec.seg_ts = seg_frame.seg_ts; rec.seg_map_id = seg_frame.seg_map_id;
            rec.seg_w = seg_frame.seg_w; rec.seg_h = seg_frame.seg_h;
            rec.seg_scale = seg_frame.seg_scale;
            rec.seg_ox = seg_frame.seg_ox; rec.seg_oy = seg_frame.seg_oy;
            rec.seg_crc = seg_frame.seg_crc; rec.seg_file_crc = seg_frame.seg_file_crc;
            rec.pad = (int32_t)((long)rec.seg_w - (long)rec.map_w);
            rec.has_seg = 1;
            have_seg = 1;
        } else {
            /* sidecar corrupt or not linked to this map: snapshot the map alone */
            free(seg_bytes); seg_bytes = NULL; seg_len = 0;
        }
    }

    /* --- stage the whole new generation before touching anything live --- */
    unsigned char frame_buf[FRAME_REC_SIZE];
    frame_pack(frame_buf, &rec);
    if (write_file_durable(stage_map_path, map_bytes, map_len) != 0) goto stage_fail;
    if (have_seg  && write_file_durable(stage_seg_path, seg_bytes, seg_len) != 0) goto stage_fail;
    if (have_slam && write_file_durable(stage_slam_path, slam_bytes, slam_len) != 0) goto stage_fail;
    if (write_file_durable(stage_frame_path, frame_buf, sizeof frame_buf) != 0) goto stage_fail;

    /* --- commit: rename into place, frame0 LAST as the commit point --- */
    if (rename(stage_map_path, snap_map_path) != 0) goto stage_fail;
    if (have_seg) {
        if (rename(stage_seg_path, snap_seg_path) != 0) goto stage_fail;
    } else {
        unlink(snap_seg_path);                       /* never keep a sidecar for another frame */
    }
    if (have_slam) {
        if (rename(stage_slam_path, snap_slam_path) != 0) goto stage_fail;
    } else {
        unlink(snap_slam_path);
    }
    if (rename(stage_frame_path, snap_frame_path) != 0) goto stage_fail;   /* commit point */
    fsync_parent_dir(snap_frame_path);

    free(map_bytes); free(slam_bytes); free(seg_bytes);
    return 0;

stage_fail:
    unlink_new_staged(stage_map_path, stage_seg_path, stage_slam_path, stage_frame_path);
    free(map_bytes); free(slam_bytes); free(seg_bytes);
    return -1;
}

/* save_map under the shared write lock (shim MIIO thread or guard process). */
int persist_map_save(void) {
    if (persist_init() != 0) return -1;
    int lock_fd = persist_write_lock();
    int rc = persist_map_save_locked();
    persist_write_unlock(lock_fd);
    return rc;
}

/* Re-verify the snapshot on disk against its frame record.  Returns 0 when consistent,
 * -1 when the map half is missing/unreadable, -2 when the map half's crc changed, and
 * sets *rec to the recorded frame. */
static int verify_snapshot(persist_frame_rec_t *rec, int *have_seg) {
    int local_have_seg = 0;
    if (!have_seg) have_seg = &local_have_seg;
    char snap_frame_path[PATH_MAX], snap_map_path[PATH_MAX], snap_seg_path[PATH_MAX];
    join_root_path(snap_frame_path, sizeof snap_frame_path, P_FRAME0);
    join_root_path(snap_map_path, sizeof snap_map_path, P_MAP0);
    join_root_path(snap_seg_path, sizeof snap_seg_path, P_SEG0);
    unsigned char *frame_buf; size_t frame_len;
    if (read_all(snap_frame_path, &frame_buf, &frame_len) != 0) return -1;
    int unpack_rc = frame_unpack(frame_buf, frame_len, rec);
    free(frame_buf);
    if (unpack_rc != 0) return -1;

    persist_frame_rec_t map_frame;
    if (read_map_frame_at(snap_map_path, &map_frame) != 0) return -1;
    /* the snapshot map must still have the frame (and crc) we recorded */
    if (map_frame.map_crc != rec->map_crc) return -2;
    if (map_frame.map_w != rec->map_w || map_frame.map_h != rec->map_h ||
        map_frame.map_ox != rec->map_ox || map_frame.map_oy != rec->map_oy ||
        !float_close_enough(map_frame.map_scale, rec->map_scale)) return -2;
    /* map frame stays authoritative; carry its ts for the link check */
    map_frame.map_ts = rec->map_ts;
    map_frame.map_id = rec->map_id;

    *have_seg = 0;
    if (rec->has_seg && file_exists(snap_seg_path)) {
        persist_frame_rec_t seg_frame;
        if (read_seg_frame_at(snap_seg_path, &seg_frame) == 0 &&
            seg_frame.seg_file_crc == rec->seg_file_crc &&
            persist_frames_linked(&map_frame, &seg_frame)) {
            *have_seg = 1;
        }
        /* else: the recorded sidecar is gone/corrupt/unlinked -> recover the map alone
         * (install_snapshot drops the sidecar, so segments are recomputed). */
    }
    return 0;
}

static int write_synthesized_slam(const char *slam_info_path) {
    /* Synthesise a minimal slam_info with a nonzero timestamp so the next
     * MapManager::LoadMap() actually loads the recovered map, and so a recovered
     * snapshot can never re-trigger the "timestamp == 0" restore loop. */
    char buf[96];
    uint32_t timestamp = (uint32_t)time(NULL);
    if (timestamp == 0) timestamp = 1;
    int written = snprintf(buf, sizeof buf,
                           "timestamp = %u;\npose: { x = -1; y = -1; angle = 0.0; };\n", timestamp);
    if (written < 0) return -1;
    return write_atomic(slam_info_path, buf, (size_t)written);
}

static int install_snapshot(int with_seg) {
    char snap_map_path[PATH_MAX], last_map_path[PATH_MAX], snap_seg_path[PATH_MAX];
    char last_segmap_path[PATH_MAX], snap_slam_path[PATH_MAX], slam_info_path[PATH_MAX];
    join_root_path(snap_map_path, sizeof snap_map_path, P_MAP0);
    join_root_path(last_map_path, sizeof last_map_path, P_LAST_MAP);
    join_root_path(snap_seg_path, sizeof snap_seg_path, P_SEG0);
    join_root_path(last_segmap_path, sizeof last_segmap_path, P_LAST_SEGMAP);
    join_root_path(snap_slam_path, sizeof snap_slam_path, P_SLAM0);
    join_root_path(slam_info_path, sizeof slam_info_path, P_SLAM_INFO);
    if (copy_file(snap_map_path, last_map_path) != 0) return -4;
    if (with_seg) {
        if (file_exists(snap_seg_path)) {
            if (copy_file(snap_seg_path, last_segmap_path) != 0) return -4;
        } else {
            unlink(last_segmap_path);
        }
    } else {
        /* map-only restore: never leave a sidecar that may belong to another frame, so the
         * producer recomputes segments rather than the robot cleaning stale ids. */
        unlink(last_segmap_path);
    }
    if (file_exists(snap_slam_path)) {
        if (copy_file(snap_slam_path, slam_info_path) != 0) return -4;
        /* A legacy snapshot may hold timestamp 0;
         * stamping a nonzero ts here means the guard cannot loop on "ts == 0". */
        uint32_t timestamp = 0;
        if (persist_read_slam_timestamp(&timestamp) != 0 || timestamp == 0) {
            if (write_synthesized_slam(slam_info_path) != 0) return -4;
        }
    } else {
        if (write_synthesized_slam(slam_info_path) != 0) return -4;
    }
    return 0;
}

/* Public, lock-guarded snapshot consistency check for the guard. */
int persist_snapshot_verify(persist_frame_rec_t *rec, int *have_seg) {
    int lock_fd = persist_write_lock();
    persist_frame_rec_t local_rec;
    int rc = verify_snapshot(rec ? rec : &local_rec, have_seg);
    persist_write_unlock(lock_fd);
    return rc;
}

/* The recover body.  Callers must hold the .v1persist.lock write lock so that the
 * snapshot_exists() -> verify_snapshot() check+read cannot race a concurrent save
 * (the check/read TOCTOU this lock closes). */
static int persist_map_reset_locked(void);
static int persist_map_recover_locked(int slot) {
    (void)slot;   /* single slot 0 */
    if (!snapshot_exists()) return -2;

    persist_frame_rec_t rec;
    int have_seg = 0;
    int verify_rc = verify_snapshot(&rec, &have_seg);
    if (verify_rc == -1) return -4;
    if (verify_rc == -2) return -3;

    /* Refuse to clobber a *different* live map (e.g. one built by a new full clean) unless the
     * live map was cleared (absent, or slam_info timestamp == 0).  A deliberate user reset
     * clears both the live map and the snapshot, so a rotation/re-anchor is the only path
     * that should override a different live frame (persist_map_force_restore). */
    char last_map_path[PATH_MAX];
    join_root_path(last_map_path, sizeof last_map_path, P_LAST_MAP);
    if (file_exists(last_map_path)) {
        persist_frame_rec_t live_frame;
        if (read_map_frame_at(last_map_path, &live_frame) == 0) {
            uint32_t timestamp = 0;
            int timestamp_zero = (persist_read_slam_timestamp(&timestamp) == 0 && timestamp == 0);
            int same_frame = (live_frame.map_w == rec.map_w && live_frame.map_h == rec.map_h &&
                              live_frame.map_ox == rec.map_ox && live_frame.map_oy == rec.map_oy &&
                              float_close_enough(live_frame.map_scale, rec.map_scale));
            if (!timestamp_zero && live_frame.map_crc != rec.map_crc && !same_frame) return -3;
        }
    }
    return install_snapshot(have_seg);
}

/* recover_map under the shared write lock. */
int persist_map_recover(int slot) {
    int lock_fd = persist_write_lock();
    int rc = persist_map_recover_locked(slot);
    persist_write_unlock(lock_fd);
    return rc;
}

/* Forced restore for the rotation policy.  Unlike recover_map this
 * is allowed to clobber a *different* live frame, but it verifies the snapshot BEFORE
 * dropping anything and does the reset + install under a single lock, so a failed
 * verification can never leave the robot with its live map deleted. */
int persist_map_force_restore(void) {
    int lock_fd = persist_write_lock();
    persist_frame_rec_t rec;
    int have_seg = 0;
    int verify_rc = verify_snapshot(&rec, &have_seg);
    int rc;
    if (verify_rc == -1)      rc = -2;    /* no snapshot */
    else if (verify_rc == -2) rc = -3;    /* inconsistent snapshot: leave live map alone */
    else {
        persist_map_reset_locked();
        rc = install_snapshot(have_seg);
    }
    persist_write_unlock(lock_fd);
    return rc;
}

/* The reset body.  Callers hold the .v1persist.lock write lock.  This drops only the
 * live map; persist_map_force_restore() relies on that to make room for the snapshot it
 * is about to install. */
static int persist_map_reset_locked(void) {
    char path[PATH_MAX];
    unlink(join_root_path(path, sizeof path, P_LAST_MAP));
    unlink(join_root_path(path, sizeof path, P_SLAM_INFO));
    unlink(join_root_path(path, sizeof path, P_LAST_SEGMAP));
    return 0;
}

/* Drop the protected snapshot (committed + staged halves).  Callers hold the write lock. */
static int persist_snapshot_wipe_locked(void) {
    char path[PATH_MAX];
    unlink(join_root_path(path, sizeof path, P_MAP0));
    unlink(join_root_path(path, sizeof path, P_SEG0));
    unlink(join_root_path(path, sizeof path, P_SLAM0));
    unlink(join_root_path(path, sizeof path, P_FRAME0));
    /* also clear any staged leftovers */
    unlink(join_root_path(path, sizeof path, P_MAP0_NEW));
    unlink(join_root_path(path, sizeof path, P_SEG0_NEW));
    unlink(join_root_path(path, sizeof path, P_SLAM0_NEW));
    unlink(join_root_path(path, sizeof path, P_FRAME0_NEW));
    return 0;
}

/* `reset_map` is the user's escape hatch.  Dropping the snapshot as well is what makes the
 * unconditional guard leave the reset alone instead of restoring the map on its next tick. */
int persist_map_reset(void) {
    int lock_fd = persist_write_lock();
    int rc = persist_map_reset_locked();
    persist_snapshot_wipe_locked();
    persist_write_unlock(lock_fd);
    return rc;
}

int persist_factory_reset(void) {
    int lock_fd = persist_write_lock();
    persist_map_reset_locked();
    persist_snapshot_wipe_locked();
    char dir[PATH_MAX];
    join_root_path(dir, sizeof dir, P_DIR);
    rmdir(dir);
    persist_write_unlock(lock_fd);
    return 0;
}

int persist_recover_list(uint32_t *slots, uint32_t *times, int max) {
    if (max <= 0) return 0;
    if (!snapshot_exists()) return 0;
    char snap_frame_path[PATH_MAX];
    join_root_path(snap_frame_path, sizeof snap_frame_path, P_FRAME0);
    struct stat file_stat;
    if (stat(snap_frame_path, &file_stat) != 0) return 0;
    if (slots) slots[0] = 0;
    if (times) times[0] = (uint32_t)file_stat.st_mtime;
    return 1;
}

int persist_guard_tick(void) {
    if (!snapshot_exists()) return 0;
    char last_map_path[PATH_MAX];
    int map_missing = !file_exists(join_root_path(last_map_path, sizeof last_map_path, P_LAST_MAP));
    uint32_t timestamp = 0;
    int timestamp_zero = (persist_read_slam_timestamp(&timestamp) == 0 && timestamp == 0);
    if (!map_missing && !timestamp_zero) return 0;        /* nothing to undo */
    int rc = persist_map_recover(0);
    return (rc == 0) ? 1 : rc;
}
