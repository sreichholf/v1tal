/* addon.c -- V1 "AppProxy room-segment addon".
 *
 * A single-file ARM LD_PRELOAD shim that, on load, patches the *live* text of
 * the V1 cleaner's `AppProxy` so that it
 *
 *   (A) emits a 28-header (segmented) RRMap IMAGE block instead of the stock
 *       24-header one, stamping `out = (segId<<3) | (in&7)` from the
 *       per-cell segment sidecar `/mnt/data/rockrobo/last_segmap`, and
 *   (B) answers the standard Roborock segment/persistence commands that stock
 *       Valetudo sends (`app_segment_clean`, `get_room_mapping`,
 *       `save_map`, `reset_map`, `get_recover_maps`,
 *       `recover_map`, plus a best-effort `name_segment`).
 *
 * It performs no ELF surgery: every AppProxy touch point is either a 2/4-byte
 * in-place instruction edit or a 12-byte "movw/movt/bx" branch to a Thumb gate
 * that lives in this shared object (see gates.S).  The whole thing is
 * reverted by simply not setting LD_PRELOAD.
 *
 * Addresses/offsets are for AppProxy sha256
 *   df9b124c324842394d9f74ace99cb8eee387798922c991e755736e604376f4cc
 * (see TECHNICAL.md §4).  They are all resolved via a base found in
 * /proc/self/maps; every edit is validated against the expected original
 * bytes before it is applied, and skipped (with a log line) otherwise.
 *
 * The pure logic (sidecar parse, blob stamping, request parse, zone mapping)
 * is deliberately free of ARM/AppProxy dependencies and can be exercised on the host.
 *
 */
#define _GNU_SOURCE
#include <ctype.h>
#include <elf.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "persist.h"
#include "segedit.h"   /* merge_segment / split_segment grid edits (pure libc) */

/* ===================================================================================== *
 *  AppProxy build constants (TECHNICAL.md §4)                                           *
 * ===================================================================================== */
#define APP_SHA256 "df9b124c324842394d9f74ace99cb8eee387798922c991e755736e604376f4cc"

/* .text of the exe: file off = vaddr - 0x10000 */
#define TEXT_VADDR_START 0x267a8u
#define TEXT_VADDR_END   0xa5300u

/* MapData constructor: the LIVE scratch header_length initializer.
 *
 * `MapData::InitGridHdr` @0x3dba4 (its `movs r2,#24`) is DEAD CODE in the pinned
 * AppProxy: nothing branches to it (no bl/blx and no tail b.w anywhere in the
 * image), no code has a pointer/table reference to its address (it appears only
 * as its own dynamic-symbol export), and no other ELF in the firmware references the
 * symbol -- editing it is inert.  The initializer that actually
 * runs is the MapData constructor:
 *   0x3f0d8  mov.w  r8,#24           (Thumb bytes 4f f0 18 08)
 *   0x3f0e8  strh.w r8,[r4,#0x85e]   (r4 = this+0x900000 -> scratch header_length)
 * Patch that `mov.w` immediate 24 -> 28 (4 bytes: 4f f0 18 08 -> 4f f0 1c 08).
 * The emitter forces 28 into the served block regardless (see
 * v1seg_write_image_header), so this keeps the scratch consistent with it. */
#define OFF_MAPDATA_CTOR_HL 0x3f0d8u

/* MapData::GenerateMapData */
#define OFF_GEN_HDR_SITE  0x3e84au   /* 40-byte IMAGE-header block */
#define OFF_GEN_HDR_CONT  0x3e872u
#define OFF_GEN_COPY_SITE 0x3e89eu   /* ~80-byte row-copy loop */
#define OFF_GEN_COPY_CONT 0x3e8eeu
#define OFF_GEN_PIXOFF    0x3e830u   /* add.w r4,r7,#72 -> #76 */
/* GenerateMapData reads the "image already in the buffer" byte at +0x9ec and
 * skips the row-copy loop when it is set.  A map loaded via Deserialize sets it,
 * so the first map request after an AppProxy restart served the loaded (raw)
 * pixels without our stamping/clear -> phantom segment 31 / no rooms.  NOP the
 * branch: every request rasterises and stamps.  (The byte is then irrelevant;
 * save/DB paths that call GenerateMapData are covered too.) */
#define OFF_GEN_SKIP_SITE 0x3e87au   /* bne.n 0x3e950 -> nop */
#define OFF_GEN_DATALEN   0x3e8f4u   /* sub.w r3,sl,#24 -> #28 */
#define OFF_GEN_LITERAL_POOL_END 0x3e9f8u /* (informational) end of the copies */

/* MapData::Deserialize IMAGE block read.
 *
 * The block walk (3ee76..3ee80) must advance by `header_length + data_length`
 * -- the RRMap rule (TECHNICAL.md §2.3, and what Valetudo's
 * RoborockMapParser does).  The stock code hard-codes 24 (because the V1 only
 * ever wrote 24); a fixed 28 mis-walks a stock/older 24-header blob by 4 bytes.
 * The substitution below reads the block's *own* header_length into r2 and
 * computes the step `r3 = data_length + header_length`, so both forms align:
 *
 *   3ee76  ldr   r3,[r5,#4]      -> ldrh r2,[r5,#2]   (header_length)
 *   3ee78  subs  r4,r4,r3        -> ldr  r3,[r5,#4]   (data_length)
 *   3ee7a  adds  r3,#24          -> add  r3,r3,r2     (step = dl + hl)
 *   3ee7c  subs  r4,#24          -> subs r4,r4,r3     (r4 -= step)
 *   3ee7e  add   r5,r3                (unchanged; r5 += step)
 *   3ee80  b.n   3ed8e                (unchanged)
 *
 * All five are 16-bit Thumb, so the edit stays two bytes per site (no cave).
 * r2/r3/r4 are dead at 3ee76 (r2 was clobbered by the LogPrint at 3ee6e and is
 * reloaded at the loop head 3ed56; r3 is reloaded at 3ed58; the flags set by
 * the subs at the new 3ee7c are not read before the branch).  The `cmp r4,#23`
 * guard at 3ee72 is left untouched: every valid block has
 * remaining >= header_length >= 24, so `> 23` is correct for 24 and 28 alike. */
#define OFF_DESER_LDRH     0x3ee76u  /* ldr  r3,[r5,#4]  -> ldrh r2,[r5,#2] */
#define OFF_DESER_LDR      0x3ee78u  /* subs r4,r4,r3    -> ldr  r3,[r5,#4] */
#define OFF_DESER_ADD      0x3ee7au  /* adds r3,#24      -> add  r3,r3,r2   */
#define OFF_DESER_SUB      0x3ee7cu  /* subs r4,#24      -> subs r4,r4,r3   */
#define OFF_DESER_IMG_SITE 0x3ef4au  /* 6-word scratch read */
#define OFF_DESER_IMG_CONT 0x3ee76u

/* AppProxyServer::RcvMessageFromMIIO(char const*) -- the entry of the real
 * single-message path: MiioRobotClient::receive_message() tail-calls it for
 * every message read from the miio_client.  It parses a single JSON message and
 * hands it to RcvSingleMessageFromMIIO(rapidjson::Document&); the char* overload
 * of RcvSingleMessageFromMIIO is only reached for batched/unparseable buffers,
 * so gating that overload (the original 0x37d08 site) missed every ordinary
 * command.  Gate the entry instead, where the raw request text is still intact. */
#define OFF_RCV_SITE          0x37f00u
#define OFF_RCV_CONT          0x37f0cu   /* after movw r3,#0xc4a8 / movt / stmdb */
#define OFF_RCV_PROLOGUE_LEN  12
/* The per-message dispatch inside RcvMessageFromMIIO's split loop: when one
 * datagram carries several JSON objects, the stock splitter calls this char*
 * overload once per object.  The entry gate defers such datagrams, so this
 * second gate handles each message of a batch. */
#define OFF_RCV2_SITE         0x37d08u
#define OFF_RCV2_CONT         0x37d14u

/* MapData scratch layout (this+0x900000) */
#define MD_SCRATCH   0x900000u
#define MD_GRID_PTR  8u
#define MD_IMAGE_HDR (48u)
#define MD_SCR_TYPE_HL (0x85cu)
#define MD_SCR_DATALEN (0x860u)
#define MD_SCR_TOP     (0x864u)
#define MD_SCR_LEFT    (0x868u)
#define MD_SCR_HEIGHT  (0x86cu)
#define MD_SCR_WIDTH   (0x870u)

#define GRID_STRIDE 1024
#define GRID_CELLS  (1024 * 1024)
#define IMAGE_HDR_28 28u

/* MiioRobotClient (the channel the stock handlers answer the app on) */
#define OFF_MIIO_GETINSTANCE 0x41bd8u
#define OFF_MIIO_SEND        0x41e28u

/* AppProxyServer::HandleZonedClean (0x333f4) packs the zones into a 64-byte
 * payload, a u32 count plus five 12-byte zones (0x33574..0x3358a copy exactly 64
 * bytes), and never checks the count: a 6th zone falls outside the packet and
 * later ones overrun its stack frame. */
#define V1_ZONED_CLEAN_MAX_ZONES 5

/* `error.code` of the miio error replies the shim sends.  Valetudo's MiioSocket
 * rejects a request only when the reply carries an `error` member; a
 * `result:["error"]` reply resolves as success. */
#define V1SEG_MIIO_ERROR_CODE (-10000)

/* The RRMap IMAGE pixel stores the segment id in 5 bits: Valetudo's
 * RoborockMapParser.PARSE_IMAGE_BLOCK does `segmentId = (val & 0b11111000) >> 3`,
 * so the valid range is 1..31 (1..15 automatic, 16..31 user-defined; S5
 * convention, see TECHNICAL.md §2.6).  That single 5-bit mask is the
 * id mask used everywhere in this file.  The sidecar stores the id in the low 6
 * bits of a byte (bit 5 = SEG_ID_OVERFLOW must be clear; bit 6 is the S5 0x40
 * flag), but an id above 31 cannot be stamped into a map pixel, so the sidecar
 * readers range-check and reject it instead of folding it with the mask. */
#define SEG_ID_MASK     0x1f
#define SEG_ID_OVERFLOW 0x20   /* sidecar byte bit 5 set => id > 31 (unrepresentable) */

/* ===================================================================================== *
 *  logging / config                                                                     *
 * ===================================================================================== */
static int g_verbose = 0;
static FILE *g_log = NULL;

#if defined(__arm__)
#define VLOG(...) do { if (g_verbose) { fprintf(g_log ? g_log : stderr, "[v1seg] " __VA_ARGS__); fflush(g_log ? g_log : stderr); } } while (0)
#else
#define VLOG(...) do { if (g_verbose) { fprintf(stderr, "[v1seg] " __VA_ARGS__); } } while (0)
#endif
#define WARN(...) do { fprintf(g_log ? g_log : stderr, "[v1seg][warn] " __VA_ARGS__); fflush(g_log ? g_log : stderr); } while (0)
/* Always-on logger for the dry-run clean path: NOT gated by V1SEG_VERBOSE.  A
 * dry-run must leave a durable, correlatable trace of exactly what the rewrite
 * would have sent even when verbose logging is off, so the first on-robot run
 * can be an observation.  Same sink/tag as VLOG, minus the gate. */
#if defined(__arm__)
#define LOG(...) do { fprintf(g_log ? g_log : stderr, "[v1seg] " __VA_ARGS__); fflush(g_log ? g_log : stderr); } while (0)
#else
#define LOG(...) do { fprintf(stderr, "[v1seg] " __VA_ARGS__); } while (0)
#endif

typedef struct {
    char root[512];
    char sidecar[640];
    char names[640];
    int  dx, dy;          /* V1SEG_BLIT_OFFSET: manual blit offset (absolute override when set) */
    int  blit_override;   /* V1SEG_BLIT_OFFSET was set explicitly -> use dx/dy verbatim */
    int  max_zones;       /* zones per app_zoned_clean packet (<= V1_ZONED_CLEAN_MAX_ZONES) */
    int  patch_deser;     /* patch Deserialize too (1) */
    int  dryrun_clean;    /* V1SEG_DRYRUN_CLEAN: log zones, reply ["ok"], never move */
    int  zone_endpoint_inclusive; /* V1SEG_ZONE_ENDPOINT=inclusive: x2 = 50*max_col (default outer edge) */
    long ttl_ms;          /* sidecar re-stat interval */
} v1seg_cfg_t;

static v1seg_cfg_t g_cfg = {
    .root = PERSIST_DEFAULT_ROOT,
    .dx = 0, .dy = 0, .max_zones = V1_ZONED_CLEAN_MAX_ZONES, .patch_deser = 1, .ttl_ms = 1000,
};

static void cfg_init(void) __attribute__((unused));
static void cfg_init(void) {
    const char *env;
    if ((env = getenv("V1SEG_ROOT")) && *env) snprintf(g_cfg.root, sizeof g_cfg.root, "%s", env);
    else if ((env = getenv("PERSIST_ROOT")) && *env) snprintf(g_cfg.root, sizeof g_cfg.root, "%s", env);
    snprintf(g_cfg.sidecar, sizeof g_cfg.sidecar, "%s/last_segmap", g_cfg.root);
    snprintf(g_cfg.names, sizeof g_cfg.names, "%s/room_names.cfg", g_cfg.root);
    if ((env = getenv("V1SEG_SIDECAR")) && *env) snprintf(g_cfg.sidecar, sizeof g_cfg.sidecar, "%s", env);
    if ((env = getenv("V1SEG_NAMES")) && *env) snprintf(g_cfg.names, sizeof g_cfg.names, "%s", env);
    if ((env = getenv("V1SEG_BLIT_OFFSET")) && *env) { sscanf(env, "%d,%d", &g_cfg.dx, &g_cfg.dy); g_cfg.blit_override = 1; }
    if ((env = getenv("V1SEG_MAX_ZONES")) && *env) g_cfg.max_zones = atoi(env);
    if ((env = getenv("V1SEG_PATCH_DESER")) && *env) g_cfg.patch_deser = atoi(env);
    if ((env = getenv("V1SEG_DRYRUN_CLEAN")) && *env) g_cfg.dryrun_clean = atoi(env);
    if ((env = getenv("V1SEG_ZONE_ENDPOINT")) && *env)
        g_cfg.zone_endpoint_inclusive = (strcmp(env, "inclusive") == 0);
    if ((env = getenv("V1SEG_TTL_MS")) && *env) g_cfg.ttl_ms = atol(env);
    /* time() has 1-second resolution, so a TTL below 1000 ms rounds to 0 and the
     * throttle never engages (re-read on every call).  Floor it at 1 s. */
    if (g_cfg.ttl_ms < 1000) g_cfg.ttl_ms = 1000;
    if (g_cfg.max_zones < 1) g_cfg.max_zones = 1;
    if (g_cfg.max_zones > V1_ZONED_CLEAN_MAX_ZONES) g_cfg.max_zones = V1_ZONED_CLEAN_MAX_ZONES;
    if ((env = getenv("V1SEG_VERBOSE")) && *env) g_verbose = atoi(env);
    if ((env = getenv("V1SEG_LOG")) && *env) g_log = fopen(env, "a");
}

static void cfg_fini(void) __attribute__((unused));
static void cfg_fini(void) { if (g_log) { fclose(g_log); g_log = NULL; } }

/* ===================================================================================== *
 *  segment sidecar + room table state                                                   *
 * ===================================================================================== */
#define MAX_ROOMS 64
typedef struct { int id; char name[64]; } v1seg_room_t;

/* Proven upper bound for the `get_room_mapping` JSON reply.  Each room renders as
 * "%s[%d,\"%s\"]": an optional comma, '[', up to 11 digits of `int`, ',"', the
 * escaped name, '"]'.  json_escape() never emits more than ROOM_NAME_ESCAPED_MAX-1
 * bytes into its ROOM_NAME_ESCAPED_MAX buffer (it stops once out_len+7 >= cap).
 * Plus the two outer '[' ']' and the NUL.  A fixed 4096-byte reply buffer
 * truncated MAX_ROOMS * long names into invalid JSON. */
#define ROOM_NAME_ESCAPED_MAX 128
#define ROOM_ENTRY_MAX (1 + 1 + 11 + 2 + (ROOM_NAME_ESCAPED_MAX - 1) + 2)
#define ROOM_MAPPING_MAX (2 + MAX_ROOMS * ROOM_ENTRY_MAX + 1)

static pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;
static uint8_t *g_segmap = NULL;      /* 1 MiB, GRID_STRIDE stride, same space as shared grid */
static int      g_seg_count = 0;      /* maxSegmentId + 1 (RRMap segment_count) */
static v1seg_room_t g_rooms[MAX_ROOMS];
static int      g_nrooms = 0;

/* Raw sidecar cells + the map extents they must be aligned against.
 *
 * A `v1seg --lastmap` sidecar is the SLAM grid padded by P cells on every side, so
 * sidecar cell (a,b) is map cell (a-Py, b-Px) and belongs at shared-grid
 * (top + a - Py, left + b - Px).  The map extents (top,left,h,w) live in the
 * GenerateMapData scratch and are only known at map-generation time, so the
 * sidecar->shared-grid blit offset is derived there.  A FIXED offset can never be
 * correct: even the PAD-only case is (left-PAD, top-PAD), and top/left move with the
 * robot.  `g_cfg.dx/dy` (V1SEG_BLIT_OFFSET), when set explicitly, remains an absolute
 * override for rare cases; with no extents yet, the provided offset is used verbatim. */
static uint8_t *g_sidecar_cells = NULL;   /* raw sidecar cells, sidecar frame */
static int      g_sidecar_w = 0, g_sidecar_h = 0;
static int      g_sidecar_fb_dx = 0, g_sidecar_fb_dy = 0;  /* fallback when extents unknown */
static int      g_map_known = 0;          /* g_map_* holds a real (top,left,height,width) */
static int      g_map_top = 0, g_map_left = 0, g_map_w = 0, g_map_h = 0;

static uint32_t rd_u32(const uint8_t *bytes) {
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}
static uint64_t rd_u64(const uint8_t *bytes) __attribute__((unused));
static uint64_t rd_u64(const uint8_t *bytes) {
    return (uint64_t)rd_u32(bytes) | ((uint64_t)rd_u32(bytes + 4) << 32);
}

/* g_segmap is a process-lifetime singleton: it is allocated once, shared by every hook,
 * and deliberately never freed (freeing it mid-run would be a use-after-free).  -fanalyzer
 * cannot see the "never freed on purpose" intent, so the leak report is annotated here.
 * (Guarded: the ARM toolchain's GCC 6.4 has no -Wanalyzer-* and would warn on the pragma.) */
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 10
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wanalyzer-malloc-leak"
#endif
static void segmap_alloc(void) {
    if (!g_segmap) g_segmap = (uint8_t *)calloc(1, GRID_CELLS);
}
#if defined(__GNUC__) && !defined(__clang__) && __GNUC__ >= 10
#pragma GCC diagnostic pop
#endif

/* Copy an (w*h) byte cell grid into the 1 MiB shared-grid-shaped buffer at
 * (dx,dy).  Cells outside the buffer are dropped.  Returns the number of ids. */
int v1seg_blit_grid(const uint8_t *cells, int w, int h, int dx, int dy) {
    if (!g_segmap) segmap_alloc();
    if (!g_segmap || !cells || w <= 0 || h <= 0) return 0;
    int seen[256]; memset(seen, 0, sizeof seen);
    int max_id = -1, id_count = 0, out_of_range = 0;
    for (int r = 0; r < h; r++) {
        int blit_row = dy + r;
        if (blit_row < 0 || blit_row >= GRID_STRIDE) continue;
        for (int c = 0; c < w; c++) {
            int blit_col = dx + c;
            if (blit_col < 0 || blit_col >= GRID_STRIDE) continue;
            uint8_t cell = cells[(size_t)r * w + c];
            if (cell & SEG_ID_OVERFLOW) { cell = 0; out_of_range++; }   /* id > 31: drop to floor */
            g_segmap[(size_t)blit_row * GRID_STRIDE + blit_col] = cell;
            int id = cell & SEG_ID_MASK;
            if (id) {
                if (!seen[id]) { seen[id] = 1; id_count++; }
                if (id > max_id) max_id = id;
            }
        }
    }
    if (out_of_range)
        WARN("blit: %d cell(s) carry a segment id > %d (outside the 5-bit RRMap field); dropped\n",
             out_of_range, SEG_ID_MASK);
    g_seg_count = (max_id >= 0) ? max_id + 1 : 0;
    return id_count;
}

/* Derive the sidecar->shared-grid blit offset.  Caller holds g_lock (or is
 * single-threaded).  With the map extents known it uses them and adds the fallback
 * delta (g_sidecar_fb_dx/dy, nonzero only for an explicit override); otherwise it
 * falls back to that offset verbatim, preserving the pre-extents behaviour. */
static void v1seg_offset_core(int *out_dx, int *out_dy) {
    /* An explicit V1SEG_BLIT_OFFSET is an absolute override (the operator asked for a
     * specific placement); otherwise derive from the map extents. */
    if (g_cfg.blit_override) { *out_dx = g_cfg.dx; *out_dy = g_cfg.dy; return; }
    int dx = g_sidecar_fb_dx, dy = g_sidecar_fb_dy;
    if (g_map_known) {
        /* A v1seg --lastmap sidecar pads BOTH axes by the same P (the offset is derived the same way:
         * dw == dh and even).  Only then is it a padded copy;
         * anything else is treated as an origin-aligned (unpadded) sidecar. */
        int dw = g_sidecar_w - g_map_w, dh = g_sidecar_h - g_map_h;
        if (dw >= 0 && dw == dh && (dw & 1) == 0) {
            int pad = dw / 2;
            dx = g_map_left - pad + g_sidecar_fb_dx;
            dy = g_map_top  - pad + g_sidecar_fb_dy;
        } else {
            dx = g_map_left + g_sidecar_fb_dx;
            dy = g_map_top  + g_sidecar_fb_dy;
        }
    }
    *out_dx = dx; *out_dy = dy;
}

/* Locking wrapper for callers outside the segment-state critical section (the
 * segedit split converter must use the very same offset the blitter used). */
static void v1seg_effective_offset(int *out_dx, int *out_dy) {
    pthread_mutex_lock(&g_lock);
    v1seg_offset_core(out_dx, out_dy);
    pthread_mutex_unlock(&g_lock);
}

/* Rebuild g_segmap from the stored raw sidecar at the derived offset.  Does not
 * touch the room table.  Caller holds g_lock (or is single-threaded). */
static void v1seg_place_segmap_locked(void) {
    if (!g_segmap) segmap_alloc();
    if (g_segmap) memset(g_segmap, 0, GRID_CELLS);
    g_seg_count = 0;
    if (!g_sidecar_cells || g_sidecar_w <= 0 || g_sidecar_h <= 0) return;
    int dx, dy;
    v1seg_offset_core(&dx, &dy);
    v1seg_blit_grid(g_sidecar_cells, g_sidecar_w, g_sidecar_h, dx, dy);
}

/* Reset the segment grid to "no rooms" (used when the sidecar is missing). */
void v1seg_reset_segmap(void) {
    if (!g_segmap) segmap_alloc();
    if (g_segmap) memset(g_segmap, 0, GRID_CELLS);
    g_seg_count = 0;
    g_nrooms = 0;
    free(g_sidecar_cells); g_sidecar_cells = NULL;
    g_sidecar_w = g_sidecar_h = 0;
}

/* room_names.cfg: one "id=name" (or "id name") per line; '#' comments. */
static void load_room_names(void) {
    FILE *file = fopen(g_cfg.names, "r");
    if (!file) return;
    char line[256];
    while (fgets(line, sizeof line, file)) {
        char *cursor = line;
        while (*cursor == ' ' || *cursor == '\t') cursor++;
        if (*cursor == '#' || *cursor == '\n' || !*cursor) continue;
        char *sep = strpbrk(cursor, "=\t ");
        if (!sep) continue;
        *sep = 0;
        int id = atoi(cursor);
        if (id <= 0 || id > 255) continue;
        char *room_name = sep + 1;
        while (*room_name == '=' || *room_name == ' ' || *room_name == '\t') room_name++;
        char *newline = strpbrk(room_name, "\r\n");
        if (newline) *newline = 0;
        if (!*room_name) continue;
        int found_existing = 0;
        for (int i = 0; i < g_nrooms; i++) {
            if (g_rooms[i].id == id) {
                snprintf(g_rooms[i].name, sizeof g_rooms[i].name, "%s", room_name);
                found_existing = 1;
                break;
            }
        }
        if (!found_existing && g_nrooms < MAX_ROOMS) {
            g_rooms[g_nrooms].id = id;
            snprintf(g_rooms[g_nrooms].name, sizeof g_rooms[g_nrooms].name, "%s", room_name);
            g_nrooms++;
        }
    }
    fclose(file);
}

/* Add a default "Room N" entry for every room id present in the grid. */
static void ensure_room_names(const v1seg_room_t *ids, int id_count) {
    for (int i = 0; i < id_count; i++) {
        int found = 0;
        for (int j = 0; j < g_nrooms; j++)
            if (g_rooms[j].id == ids[i].id) { found = 1; break; }
        if (!found && g_nrooms < MAX_ROOMS) {
            g_rooms[g_nrooms].id = ids[i].id;
            snprintf(g_rooms[g_nrooms].name, sizeof g_rooms[g_nrooms].name, "Room %d", ids[i].id);
            g_nrooms++;
        }
    }
}

/* Verify the RRSEG CRC32 exactly as persist.c's seg_crc_ok does: over the whole
 * sidecar (header + payload) with the u32 crc field at offset 48 zeroed.  A
 * non-atomic writer (power loss / torn last_segmap) otherwise blits bogus room
 * ids; reject it instead.  Computed on a temporary copy so the
 * caller's buffer is never modified. */
static int seg_crc_ok(const uint8_t *buf, size_t len, uint32_t stored_crc) {
    if (len < 64) return 0;
    uint8_t *copy = (uint8_t *)malloc(len);
    if (!copy) return 0;                    /* cannot verify -> reject (conservative) */
    memcpy(copy, buf, len);
    memset(copy + 48, 0, 4);
    uint32_t computed = persist_crc32(copy, len);
    free(copy);
    return computed == stored_crc;
}

/* Parse a `last_segmap` sidecar (src/v1seg.cpp writer) into the state.
 * Layout: 64-byte header ("RRSEG\0\0\1", w@32 h@36, crc32@48) + w*h cell bytes +
 * u32 room_count + room_count * { u8 id; pad[3]; u32 area; i32 cx; i32 cy; }.
 * The CRC is verified before any state is touched, so a torn sidecar leaves the
 * current in-memory rooms/grid unchanged. */
int v1seg_parse_sidecar(const uint8_t *buf, size_t len, int dx, int dy, char *err, size_t err_cap) {
    if (len < 64 || memcmp(buf, "RRSEG", 5) != 0) {
        if (err) snprintf(err, err_cap, "bad magic/size (%zu bytes)", len);
        return -1;
    }
    uint32_t w = rd_u32(buf + 32), h = rd_u32(buf + 36);
    if (w == 0 || h == 0 || (uint64_t)w * h > (uint64_t)1 << 26 ||
        len < 64 + (size_t)w * h + 4) {
        if (err) snprintf(err, err_cap, "bad dims %ux%u for %zu bytes", w, h, len);
        return -1;
    }
    uint32_t stored_crc = rd_u32(buf + 48);
    if (!seg_crc_ok(buf, len, stored_crc)) {
        if (err) snprintf(err, err_cap, "crc mismatch (stored %08x)", stored_crc);
        return -1;
    }
    v1seg_room_t ids[MAX_ROOMS];
    int id_count = 0;
    memset(ids, 0, sizeof ids);
    uint8_t *cells = (uint8_t *)malloc((size_t)w * h);
    if (!cells) { if (err) snprintf(err, err_cap, "oom"); return -1; }
    memcpy(cells, buf + 64, (size_t)w * h);

    pthread_mutex_lock(&g_lock);
    /* Keep the raw cells: the blit offset depends on the map extents, which are only
     * known at GenerateMapData time, so the grid is (re)placed from them there. */
    free(g_sidecar_cells);
    g_sidecar_cells = cells;
    g_sidecar_w = (int)w; g_sidecar_h = (int)h;
    g_sidecar_fb_dx = dx; g_sidecar_fb_dy = dy;
    v1seg_place_segmap_locked();
    /* collect ids in ascending order for deterministic room tables */
    int seen[256]; memset(seen, 0, sizeof seen);
    int out_of_range = 0;
    for (size_t i = 0; i < (size_t)w * h; i++) {
        if (cells[i] & SEG_ID_OVERFLOW) { out_of_range++; continue; }   /* id > 31: ignore */
        int id = cells[i] & SEG_ID_MASK;
        if (id && !seen[id]) { seen[id] = 1; if (id_count < MAX_ROOMS) ids[id_count++].id = id; }
    }
    if (out_of_range)
        WARN("sidecar %s: %d cell(s) carry a segment id > %d (outside the 5-bit RRMap field); ignored\n",
             g_cfg.sidecar, out_of_range, SEG_ID_MASK);
    for (int i = 0; i < id_count; i++)
        for (int j = i + 1; j < id_count; j++)
            if (ids[j].id < ids[i].id) { v1seg_room_t swap = ids[i]; ids[i] = ids[j]; ids[j] = swap; }
    g_nrooms = 0;
    ensure_room_names(ids, id_count);
    load_room_names();
    pthread_mutex_unlock(&g_lock);
    return id_count;
}

/* Sidecar stat cache.  v1seg_refresh_sidecar() runs from both the map thread
 * (GenerateMapData) and the MIIO thread (merge/split -> force refresh), so these
 * are read/written without g_lock.  They are a few words of a monotonically
 * refreshed cache; a torn read would at worst cause one extra or one skipped
 * re-read.  The interleaving cannot be forced deterministically, so the race is
 * deliberately left unlocked. */
static struct { time_t mtime; off_t size; } g_last_stat;
static time_t g_last_check;
static int g_sidecar_seen = 0;

/* Load / reload the sidecar if it appeared/changed (stat-cached, TTL-throttled). */
int v1seg_refresh_sidecar(int force) {
    struct stat file_stat;
    if (stat(g_cfg.sidecar, &file_stat) != 0) {
        if (g_sidecar_seen || force) {
            pthread_mutex_lock(&g_lock);
            v1seg_reset_segmap();
            pthread_mutex_unlock(&g_lock);
            g_sidecar_seen = 0;
            VLOG("sidecar %s absent -> emitting segment-less 28-header map\n", g_cfg.sidecar);
        }
        return 0;
    }
    time_t now = time(NULL);
    /* Fast path: the same mtime and size means the file has not changed
     * since the last successful load, so serve the already-loaded state without
     * re-reading.  Both writers (the producer and segedit) publish via rename(),
     * which updates mtime, so a real change is always seen. */
    if (!force && g_sidecar_seen && file_stat.st_mtime == g_last_stat.mtime &&
        file_stat.st_size == g_last_stat.size) {
        return g_seg_count;
    }
    /* A *changed* file is read at most once per (clamped >= 1 s) TTL so a burst of
     * rewrites cannot thrash parse + CRC + blit. */
    if (!force && g_sidecar_seen && now - g_last_check < (time_t)(g_cfg.ttl_ms / 1000)) {
        return g_seg_count;
    }
    g_last_check = now;
    int fd = open(g_cfg.sidecar, O_RDONLY);
    if (fd < 0) return g_seg_count;
    size_t file_size = (size_t)file_stat.st_size;
    uint8_t *file_buf = (uint8_t *)malloc(file_size ? file_size : 1);
    if (!file_buf) { close(fd); return g_seg_count; }
    ssize_t nread = read(fd, file_buf, file_size);
    close(fd);
    char err[128] = "";
    if (nread < 0 || (size_t)nread != file_size) {
        WARN("short read on %s\n", g_cfg.sidecar);
        free(file_buf);
        return g_seg_count;
    }
    int room_count = v1seg_parse_sidecar(file_buf, file_size, g_cfg.dx, g_cfg.dy, err, sizeof err);
    free(file_buf);
    if (room_count < 0) {
        WARN("sidecar %s rejected: %s\n", g_cfg.sidecar, err);
        return g_seg_count;
    }
    g_sidecar_seen = 1;
    g_last_stat.mtime = file_stat.st_mtime;
    g_last_stat.size = file_stat.st_size;
    VLOG("sidecar loaded: %d rooms, segment_count=%d, blit=(%d,%d)\n",
         room_count, g_seg_count, g_cfg.dx, g_cfg.dy);
    return room_count;
}

/* ===================================================================================== *
 *  blob encoder core                                                                     *
 * ===================================================================================== */

/* Splice the 6-word scratch IMAGE header into this+48 as 28 bytes, with the
 * (single) extra segment_count word.  Matches the RRMap IMAGE block layout.
 *
 * The block *type* is read from scratch, but header_length is FORCED to 28 here
 * rather than copied from the scratch {type,header_length} word.  This makes the
 * served format unconditional: the pinned MapData ctor stores 28 (see
 * OFF_MAPDATA_CTOR_HL), but a legacy/loaded 24-header blob can still leave 24 in
 * scratch, and copying that verbatim would declare header_length=24 while writing
 * the 28-byte layout below (segment_count @+8, extents @+12..+24, pixels @+76).
 * Stock Valetudo's RoborockMapParser.PARSE_IMAGE_BLOCK switches on header_length,
 * so a 24-declared/28-laid-out block reads permuted extents, a 4-byte-shifted
 * pixel array, and no rooms.  Forcing 28 guarantees the declared format matches
 * the bytes. */
void v1seg_write_image_header(void *mapdata) {
    if (!mapdata) return;
    uint8_t *map_data = (uint8_t *)mapdata;
    uint8_t *scratch = map_data + MD_SCRATCH;
    uint8_t *header_block = map_data + MD_IMAGE_HDR;

    /* The map's placement in the shared grid is only known here (the nav fills the
     * scratch before GenerateMapData emits the header).  Record it and, when it moved,
     * re-derive the sidecar alignment; top/left change per map, so a fixed blit offset
     * cannot track them (and a PAD=3 sidecar needs (left-PAD, top-PAD), not (0,0)). */
    int32_t top, left, height, width;
    memcpy(&top, scratch + MD_SCR_TOP, 4);
    memcpy(&left, scratch + MD_SCR_LEFT, 4);
    memcpy(&height, scratch + MD_SCR_HEIGHT, 4);
    memcpy(&width, scratch + MD_SCR_WIDTH, 4);
    pthread_mutex_lock(&g_lock);
    if (!g_map_known || top != g_map_top || left != g_map_left ||
        height != g_map_h || width != g_map_w) {
        g_map_top = top; g_map_left = left; g_map_h = height; g_map_w = width;
        g_map_known = 1;
        if (g_sidecar_cells) v1seg_place_segmap_locked();
    }
    pthread_mutex_unlock(&g_lock);

    v1seg_refresh_sidecar(0);   /* picks up a sidecar edit; re-places if the file changed */

    pthread_mutex_lock(&g_lock);                             /* g_seg_count is rewritten on reload */
    uint32_t seg_count = (uint32_t)g_seg_count;
    pthread_mutex_unlock(&g_lock);
    uint16_t scratch_type;                                   /* block type comes from scratch */
    memcpy(&scratch_type, scratch + MD_SCR_TYPE_HL, 2);
    uint16_t header_length_28 = (uint16_t)IMAGE_HDR_28;      /* but the format is always 28 */
    memcpy(header_block + 0, &scratch_type, 2);              /* IMAGE block type */
    memcpy(header_block + 2, &header_length_28, 2);          /* force header_length=28 */
    memcpy(header_block + 4, scratch + MD_SCR_DATALEN, 4);   /* data_length (recomputed later) */
    memcpy(header_block + 8, &seg_count, 4);                 /* segment_count  <-- the added word */
    memcpy(header_block + 12, scratch + MD_SCR_TOP, 16);     /* top,left,height,width */
}

/* Copy the shared-grid rows into `dst` while stamping room ids:
 *   sid == 0            -> keep walls (type 1) and dropped pixels (type 0) verbatim;
 *                          clear the high bits of floor (type>=2) so a 28-header
 *                          blob does not invent a phantom "segment 31";
 *   sid != 0, type == 1 -> keep the wall byte (parser ignores segment bits);
 *   sid != 0, type <  2 -> promote to floor (type 7, as the S5 does);
 *   sid != 0            -> out = (sid & 0x1f) << 3 | type.
 * Returns the new write pointer (dst + width*height), i.e. the value the
 * original GenerateMapData loop would have left in r4. */
uint8_t *v1seg_stamp_copy(void *mapdata, uint8_t *dst) {
    if (!mapdata || !dst) return dst;
    uint8_t *map_data = (uint8_t *)mapdata;
    uint8_t *scratch = map_data + MD_SCRATCH;
    const uint8_t *grid = *(uint8_t **)(map_data + MD_GRID_PTR);
    int32_t top, left, height, width;
    memcpy(&top, scratch + MD_SCR_TOP, 4);
    memcpy(&left, scratch + MD_SCR_LEFT, 4);
    memcpy(&height, scratch + MD_SCR_HEIGHT, 4);
    memcpy(&width, scratch + MD_SCR_WIDTH, 4);
    if (width < 0) width = 0;
    if (height < 0) height = 0;
    /* Optional cost probe: the always-rasterise edit (OFF_GEN_SKIP_SITE) gives up
     * the "image already present" cache.  With V1SEG_VERBOSE this logs the actual
     * raster duration so the robot can decide whether the cache-aware variant is
     * worth its invalidation complexity.  Gated so normal runs pay nothing. */
    struct timespec raster_start, raster_end;
    if (g_verbose) clock_gettime(CLOCK_MONOTONIC, &raster_start);
    /* Hold the lock for the whole raster: a concurrent sidecar reload must not rewrite
     * the grid mid-frame.  `segmap` is the snapshot every row of this frame stamps from. */
    pthread_mutex_lock(&g_lock);
    const uint8_t *segmap = g_segmap;
    for (int32_t y = 0; y < height; y++) {
        int32_t r = top + y, col_base = left;
        const uint8_t *src_row = NULL, *seg_row = NULL;
        if (grid && r >= 0 && r < GRID_STRIDE && col_base >= 0 && col_base + width <= GRID_STRIDE)
            src_row = grid + (size_t)r * GRID_STRIDE + col_base;
        if (segmap && r >= 0 && r < GRID_STRIDE && col_base >= 0 && col_base + width <= GRID_STRIDE)
            seg_row = segmap + (size_t)r * GRID_STRIDE + col_base;
        for (int32_t x = 0; x < width; x++) {
            uint8_t src = src_row ? src_row[x] : 0;
            uint8_t seg_id = seg_row ? (uint8_t)(seg_row[x] & SEG_ID_MASK) : 0;
            uint8_t out;
            if (!seg_id) {
                uint8_t cell_type = src & 7;
                out = (cell_type >= 2 && (src & 0xF8)) ? (src & 7) : src;
            } else {
                uint8_t cell_type = src & 7;
                if (cell_type == 1) out = src;
                else {
                    if (cell_type < 2) cell_type = 7;
                    out = (uint8_t)((seg_id << 3) | cell_type);
                }
            }
            dst[x] = out;
        }
        dst += width;
    }
    pthread_mutex_unlock(&g_lock);
    if (g_verbose) {
        clock_gettime(CLOCK_MONOTONIC, &raster_end);
        long long elapsed_us = (long long)(raster_end.tv_sec - raster_start.tv_sec) * 1000000LL
                             + (long long)(raster_end.tv_nsec - raster_start.tv_nsec) / 1000LL;
        VLOG("raster: %dx%d (%lld cells) %lld us\n", width, height,
             (long long)width * (long long)height, elapsed_us);
    }
    return dst;
}

/* Rewrite the 6-word IMAGE scratch read in Deserialize, tolerating both the 24-
 * and 28-header forms (header_length is the 3rd u16 of the block).  The block
 * walk (3ee76..3ee80) is patched to advance by the block's own
 * header_length + data_length, so both forms stay aligned (see OFF_DESER_*). */
void v1seg_deser_image(void *mapdata, const uint8_t *block) {
    if (!mapdata || !block) return;
    uint8_t *map_data = (uint8_t *)mapdata;
    uint8_t *scratch = map_data + MD_SCRATCH;
    uint32_t type_and_hdrlen = rd_u32(block + 0);   /* {type, header_length} */
    uint32_t data_length = rd_u32(block + 4);
    uint16_t header_length = (uint16_t)((type_and_hdrlen >> 16) & 0xffff);
    uint32_t top, left, height, width;
    if (header_length >= IMAGE_HDR_28) {
        top = rd_u32(block + 12); left = rd_u32(block + 16);
        height = rd_u32(block + 20); width = rd_u32(block + 24);
    } else {
        top = rd_u32(block + 8); left = rd_u32(block + 12);
        height = rd_u32(block + 16); width = rd_u32(block + 20);
    }
    memcpy(scratch + MD_SCR_TYPE_HL, &type_and_hdrlen, 4);
    memcpy(scratch + MD_SCR_DATALEN, &data_length, 4);
    memcpy(scratch + MD_SCR_TOP, &top, 4);
    memcpy(scratch + MD_SCR_LEFT, &left, 4);
    memcpy(scratch + MD_SCR_HEIGHT, &height, 4);
    memcpy(scratch + MD_SCR_WIDTH, &width, 4);
    uint32_t flags;
    memcpy(&flags, scratch + 0x8d8, 4);
    flags |= 1u;
    memcpy(scratch + 0x8d8, &flags, 4);
    scratch[0x9ec] = 1;
}

/* ===================================================================================== *
 *  minimal request scanner (no rapidjson dependency)                                    *
 * ===================================================================================== */
/* One entry of an upstream `name_segment` payload: [{miRoomId:<name>, robotRoomId:<id>}, …]. */
#define V1SEG_MAX_RENAMES 64
typedef struct { int id; char name[64]; } v1seg_rename_t;

typedef struct {
    int have_id; unsigned long long id;
    int have_method; char method[64];
    int nseg; long long seg[64];
    int have_repeat; long long repeat;
    int have_mapid; long long mapid;
    int have_segment_id; long long segment_id;
    int have_name; char name[64];
    int nparams; long long params[8];   /* merge_segment / split_segment args */
    int nrenames;                       /* name_segment: upstream {miRoomId,robotRoomId} array */
    v1seg_rename_t renames[V1SEG_MAX_RENAMES];
} v1seg_req_t;

static const char *skip_ws(const char *cursor) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') cursor++;
    return cursor;
}

/* Locate `"key"` used as an object key (preceded by '{', ',' or '['). */
static const char *find_key(const char *text, const char *key) {
    size_t key_len = strlen(key);
    const char *scan = text;
    while ((scan = strchr(scan, '"')) != NULL) {
        if (strncmp(scan + 1, key, key_len) == 0 && scan[1 + key_len] == '"') {
            const char *probe = scan;
            while (probe > text && (probe[-1] == ' ' || probe[-1] == '\t' ||
                                    probe[-1] == '\n' || probe[-1] == '\r')) probe--;
            if (probe == text || probe[-1] == '{' || probe[-1] == ',' || probe[-1] == '[')
                return scan + 1 + key_len + 1;
        }
        scan++;
    }
    return NULL;
}

static int get_num_after(const char *text, const char *key, long long *out) {
    const char *scan = find_key(text, key);
    if (!scan) return 0;
    scan = skip_ws(scan);
    if (*scan != ':') return 0;
    scan = skip_ws(scan + 1);
    char *end_ptr;
    long long value = strtoll(scan, &end_ptr, 10);
    if (end_ptr == scan) return 0;
    *out = value;
    return 1;
}

static int get_str_after(const char *text, const char *key, char *out, size_t cap) {
    const char *scan = find_key(text, key);
    if (!scan) return 0;
    scan = skip_ws(scan);
    if (*scan != ':') return 0;
    scan = skip_ws(scan + 1);
    if (*scan != '"') return 0;
    scan++;
    size_t i = 0;
    while (*scan && *scan != '"' && i + 1 < cap) {
        if (*scan == '\\' && scan[1]) scan++;
        out[i++] = *scan++;
    }
    out[i] = 0;
    return 1;
}

static int get_int_array_after(const char *text, const char *key, long long *out, int max) {
    const char *scan = find_key(text, key);
    if (!scan) return 0;
    scan = skip_ws(scan);
    if (*scan != ':') return 0;
    scan = skip_ws(scan + 1);
    if (*scan != '[') return 0;
    scan++;
    int count = 0;
    while (*scan && *scan != ']') {
        scan = skip_ws(scan);
        if (*scan == ']') break;
        if (*scan == '"') {
            scan++;
            while (*scan && *scan != '"') { if (*scan == '\\' && scan[1]) scan++; scan++; }
            if (*scan) scan++;
        } else {
            char *end_ptr;
            long long value = strtoll(scan, &end_ptr, 10);
            if (end_ptr == scan) { scan++; continue; }
            if (count < max) out[count++] = value;
            scan = end_ptr;
        }
        scan = skip_ws(scan);
        if (*scan == ',') scan++;
    }
    return count;
}

/* Upstream `RoborockMapSegmentRenameCapability` sends
 *   name_segment params:[{miRoomId:<name>, robotRoomId:<id>}, …all rooms]
 * (the whole table, not one room; see TECHNICAL.md §3).  Walk the array of
 * objects and pick out the (robotRoomId -> miRoomId) pairs; the object is copied
 * into a bounded buffer so the per-object key scanner cannot run past its end. */
static int parse_rename_array(const char *text, v1seg_rename_t *out, int max) {
    const char *scan = find_key(text, "params");
    if (!scan) return 0;
    scan = skip_ws(scan);
    if (*scan != ':') return 0;
    scan = skip_ws(scan + 1);
    if (*scan != '[') return 0;
    scan++;
    int count = 0;
    while (*scan) {
        scan = skip_ws(scan);
        if (*scan == ']' || *scan == 0) break;
        if (*scan == ',') { scan++; continue; }
        if (*scan != '{') { scan++; continue; }
        char object[256];
        size_t n = 0;
        int depth = 0, in_string = 0;
        const char *cursor = scan;
        while (*cursor && n + 2 < sizeof object) {
            char ch = *cursor;
            if (in_string) {
                object[n++] = ch;
                if (ch == '\\' && cursor[1]) { object[n++] = cursor[1]; cursor += 2; continue; }
                if (ch == '"') in_string = 0;
                cursor++;
                continue;
            }
            if (ch == '"') { in_string = 1; object[n++] = ch; cursor++; continue; }
            if (ch == '{') depth++;
            else if (ch == '}') depth--;
            object[n++] = ch;
            cursor++;
            if (depth == 0) break;             /* object complete */
        }
        object[n] = 0;
        scan = cursor;
        long long id;
        char name[64];
        if (get_num_after(object, "robotRoomId", &id) &&
            get_str_after(object, "miRoomId", name, sizeof name)) {
            if (count < max) {
                out[count].id = (int)id;
                snprintf(out[count].name, sizeof out[count].name, "%s", name);
                count++;
            }
        }
    }
    return count;
}

int v1seg_parse_request(const char *text, v1seg_req_t *req) {
    memset(req, 0, sizeof *req);
    req->repeat = 1;
    long long value;
    if (get_num_after(text, "id", &value)) { req->have_id = 1; req->id = (unsigned long long)value; }
    if (get_str_after(text, "method", req->method, sizeof req->method)) req->have_method = 1;
    req->nseg = get_int_array_after(text, "segments", req->seg, 64);
    if (get_num_after(text, "repeat", &value)) { req->have_repeat = 1; req->repeat = value; }
    if (get_num_after(text, "map_id", &value)) { req->have_mapid = 1; req->mapid = value; }
    if (get_num_after(text, "segment_id", &value)) { req->have_segment_id = 1; req->segment_id = value; }
    get_str_after(text, "name", req->name, sizeof req->name);
    req->have_name = req->name[0] != 0;
    /* merge_segment [idA,idB] / split_segment [id,x1,y1,x2,y2]: top-level params array */
    req->nparams = get_int_array_after(text, "params", req->params, 8);
    /* name_segment: the upstream array-of-objects form */
    req->nrenames = parse_rename_array(text, req->renames, V1SEG_MAX_RENAMES);
    return 1;
}

/* ===================================================================================== *
 *  segment id -> zone rectangles                                                        *
 * ===================================================================================== */
#define MM_PER_CELL 50
typedef struct { int x1, y1, x2, y2, iterations; } v1seg_zone_t;

/* Bounding box (cells, inclusive) of one id in the 1 MiB grid.  0 if absent. */
int v1seg_seg_bbox(int id, int *out_min_col, int *out_min_row, int *out_max_col, int *out_max_row) {
    int min_col = 1 << 30, min_row = 1 << 30, max_col = -1, max_row = -1;
    if (id <= 0 || id > SEG_ID_MASK) return 0;   /* outside the 5-bit field: no cells can match */
    pthread_mutex_lock(&g_lock);         /* snapshot the grid for bbox scan */
    const uint8_t *segmap = g_segmap;
    if (!segmap) { pthread_mutex_unlock(&g_lock); return 0; }
    for (int r = 0; r < GRID_STRIDE; r++) {
        const uint8_t *row = segmap + (size_t)r * GRID_STRIDE;
        for (int x = 0; x < GRID_STRIDE; x++) {
            if ((row[x] & SEG_ID_MASK) == id) {
                if (x < min_col) min_col = x;
                if (x > max_col) max_col = x;
                if (r < min_row) min_row = r;
                if (r > max_row) max_row = r;
            }
        }
    }
    pthread_mutex_unlock(&g_lock);
    if (max_col < 0) return 0;
    *out_min_col = min_col; *out_min_row = min_row;
    *out_max_col = max_col; *out_max_row = max_row;
    return 1;
}

/* Translate requested segment ids into `app_zoned_clean` zones, in the
 * 0..51200 mm frame that AppProxyServer::HandleZonedClean expects (it then
 * subtracts 25600 internally).  Returns the number of zones. */
int v1seg_segments_to_zones(const long long *segs, int count, int repeat,
                            v1seg_zone_t *zones, int max_zones) {
    int zone_count = 0;
    for (int i = 0; i < count && zone_count < max_zones; i++) {
        int id = (int)segs[i];
        if (id <= 0) continue;
        int min_col, min_row, max_col, max_row;
        if (!v1seg_seg_bbox(id, &min_col, &min_row, &max_col, &max_row)) continue;
        int iterations = repeat > 0 ? repeat : 1;
        /* The bbox is an inclusive cell range [min..max].  The default emits the
         * rect's *outer* edge, 50*(max+1) (the long-standing behaviour; the live nav
         * then reads w = x2-x1+1 as one cell past max).  V1SEG_ZONE_ENDPOINT=inclusive
         * emits 50*max instead, so that same inclusive nav arithmetic covers exactly
         * the (max-min+1) bbox cells.  Default is unchanged. */
        int max_col_endpoint = g_cfg.zone_endpoint_inclusive ? max_col : (max_col + 1);
        int max_row_endpoint = g_cfg.zone_endpoint_inclusive ? max_row : (max_row + 1);
        zones[zone_count].x1 = MM_PER_CELL * min_col;
        zones[zone_count].y1 = MM_PER_CELL * min_row;
        zones[zone_count].x2 = MM_PER_CELL * max_col_endpoint;
        zones[zone_count].y2 = MM_PER_CELL * max_row_endpoint;
        zones[zone_count].iterations = iterations;
        zone_count++;
    }
    return zone_count;
}

/* ===================================================================================== *
 *  JSON replies / rewritten requests                                                    *
 * ===================================================================================== */
static void json_escape(const char *in, char *out, size_t cap) {
    size_t out_len = 0;
    for (const unsigned char *ch = (const unsigned char *)in; *ch && out_len + 7 < cap; ch++) {
        if (*ch == '"' || *ch == '\\') { out[out_len++] = '\\'; out[out_len++] = (char)*ch; }
        else if (*ch < 0x20) { out_len += (size_t)snprintf(out + out_len, cap - out_len, "\\u%04x", *ch); }
        else out[out_len++] = (char)*ch;
    }
    out[out_len] = 0;
}

/* Zones for an app_segment_clean request.  Plans one zone past g_cfg.max_zones so
 * the caller can tell "too many rooms" (returns max_zones + 1) from a request
 * that fits; returns 0 when no requested id has cells. */
static int plan_segment_clean(const v1seg_req_t *req, v1seg_zone_t zones[V1_ZONED_CLEAN_MAX_ZONES + 1]) {
    int repeat_count = req->have_repeat ? (int)req->repeat : 1;
    return v1seg_segments_to_zones(req->seg, req->nseg, repeat_count, zones, g_cfg.max_zones + 1);
}

static char *build_zoned_clean_request(const v1seg_req_t *req, const v1seg_zone_t *zones, int zone_count) {
    size_t capacity = 256 + (size_t)zone_count * 96;
    char *json = (char *)malloc(capacity);
    if (!json) return NULL;
    size_t out_len = 0;
    out_len += (size_t)snprintf(json + out_len, capacity - out_len,
                                "{\"id\":%llu,\"method\":\"app_zoned_clean\",\"params\":[", req->id);
    for (int i = 0; i < zone_count; i++)
        out_len += (size_t)snprintf(json + out_len, capacity - out_len, "%s[%d,%d,%d,%d,%d]", i ? "," : "",
                                    zones[i].x1, zones[i].y1, zones[i].x2, zones[i].y2, zones[i].iterations);
    snprintf(json + out_len, capacity - out_len, "]}");
    return json;
}

/* V1SEG_DRYRUN_CLEAN: log exactly what the app_zoned_clean rewrite *would* have
 * sent -- one line per zone -- then let the caller answer ["ok"] without driving
 * the robot.  This is the observation tool for the first live run, not a
 * behaviour change: with the env unset nothing here runs.  The zone math is the
 * same v1seg_segments_to_zones + g_cfg.max_zones the real rewrite uses, and the map
 * extents are the values the blob header carries, so the logged rects can be
 * checked against the room's rendered bbox before a wheel turns. */
static void dryrun_log_clean(const v1seg_req_t *req, const v1seg_zone_t *zones, int zone_count) {
    int repeat_count = req->have_repeat ? (int)req->repeat : 1;

    int map_known, map_top, map_left, map_w, map_h;
    pthread_mutex_lock(&g_lock);
    map_known = g_map_known;
    map_top = g_map_top; map_left = g_map_left; map_w = g_map_w; map_h = g_map_h;
    pthread_mutex_unlock(&g_lock);
    int offset_dx = 0, offset_dy = 0;
    v1seg_effective_offset(&offset_dx, &offset_dy);   /* takes g_lock itself */

    /* Requested ids, comma-separated, bounded and always NUL-terminated. */
    char ids[256];
    size_t pos = 0;
    ids[0] = 0;
    for (int i = 0; i < req->nseg && pos < sizeof ids - 1; i++) {
        int n = snprintf(ids + pos, sizeof ids - pos, "%s%lld", i ? "," : "", req->seg[i]);
        if (n < 0) break;
        pos += (size_t)n;
        if (pos > sizeof ids - 1) pos = sizeof ids - 1;   /* truncated: stop cleanly */
    }

    LOG("dry-run app_segment_clean: id=%llu requested=[%s] zones=%d repeat=%d "
        "map known=%d top=%d left=%d w=%d h=%d offset=(%d,%d)\n",
        req->id, ids, zone_count, repeat_count, map_known, map_top, map_left, map_w, map_h,
        offset_dx, offset_dy);

    /* Walk the requested ids in order: the k-th id that has cells is zones[k]
     * (v1seg_segments_to_zones keeps that order and skips ids without cells). */
    int emitted = 0;
    for (int i = 0; i < req->nseg; i++) {
        int id = (int)req->seg[i];
        if (id <= 0) { LOG("dry-run   id=%d: ignored (id <= 0)\n", id); continue; }
        int min_col, min_row, max_col, max_row;
        if (!v1seg_seg_bbox(id, &min_col, &min_row, &max_col, &max_row)) {
            LOG("dry-run   id=%d: no cells in the segment grid\n", id);
            continue;
        }
        if (emitted < zone_count && emitted < g_cfg.max_zones) {
            const v1seg_zone_t *zone = &zones[emitted];
            LOG("dry-run   id=%d: cells c[%d..%d] r[%d..%d] rect [%d,%d,%d,%d] it=%d\n",
                id, min_col, max_col, min_row, max_row,
                zone->x1, zone->y1, zone->x2, zone->y2, zone->iterations);
        } else {
            LOG("dry-run   id=%d: cells c[%d..%d] r[%d..%d] - over the max_zones=%d cap, "
                "request refused\n", id, min_col, max_col, min_row, max_row, g_cfg.max_zones);
        }
        emitted++;
    }
}

/* Build the `get_room_mapping` JSON reply into a fresh heap buffer sized to the
 * proven bound, so it can never be truncated into invalid JSON.  Returns a
 * NUL-terminated malloc'd string (caller frees) or NULL on allocation failure. */
static char *build_room_mapping(void) {
    char *buf = (char *)malloc(ROOM_MAPPING_MAX);
    if (!buf) return NULL;
    size_t out_len = 0;
    out_len += (size_t)snprintf(buf + out_len, ROOM_MAPPING_MAX - out_len, "[");
    pthread_mutex_lock(&g_lock);   /* room table is rewritten by v1seg_parse_sidecar */
    for (int i = 0; i < g_nrooms; i++) {
        char escaped_name[ROOM_NAME_ESCAPED_MAX];
        json_escape(g_rooms[i].name, escaped_name, sizeof escaped_name);
        out_len += (size_t)snprintf(buf + out_len, ROOM_MAPPING_MAX - out_len, "%s[%d,\"%s\"]",
                                    i ? "," : "", g_rooms[i].id, escaped_name);
    }
    pthread_mutex_unlock(&g_lock);
    snprintf(buf + out_len, ROOM_MAPPING_MAX - out_len, "]");
    return buf;
}

/* `get_recover_maps` -> [[slot, time], ...] (Valetudo's expected shape). */
static void build_recover_maps(char *buf, size_t cap) {
    uint32_t slots[16], times[16];
    int count = persist_recover_list(slots, times, 16);
    if (count < 0) count = 0;
    size_t out_len = 0;
    out_len += (size_t)snprintf(buf + out_len, cap - out_len, "[");
    for (int i = 0; i < count; i++)
        out_len += (size_t)snprintf(buf + out_len, cap - out_len, "%s[%u,%u]",
                                    i ? "," : "", slots[i], times[i]);
    snprintf(buf + out_len, cap - out_len, "]");
}

/* ===================================================================================== *
 *  AppProxy call-outs (resolved from the exe load base)                                 *
 * ===================================================================================== */
static void *g_exe_base = NULL;
static int   g_exe_base_valid = 0;   /* set by patch_appproxy; the bias is legitimately 0 here */
static void *exe_ptr(uint32_t off) { return (uint8_t *)g_exe_base + off; }

/* The MIIO call-outs are Thumb functions resolved from the exe: mark bit0 so the
 * indirect call stays in Thumb state (an even target would enter ARM state and fault). */
static void *exe_call_ptr(uint32_t off) {
    return (void *)((uintptr_t)exe_ptr(off) | 1u);
}

/* When set, replies are delivered here instead of to AppProxy. */
static void (*g_reply_sink)(unsigned long long id, const char *result_json) = NULL;

/* Send `{"id":<id>,"<member>":<value_json>}` to the app; `member` is "result" or "error". */
static void send_reply(unsigned long long id, const char *member, const char *value_json) {
    if (g_reply_sink) { g_reply_sink(id, value_json); return; }
    if (!g_exe_base_valid) return;
    void *(*get_instance)(void) = (void *(*)(void))exe_call_ptr(OFF_MIIO_GETINSTANCE);
    void (*send_message)(void *, const char *) =
        (void (*)(void *, const char *))exe_call_ptr(OFF_MIIO_SEND);
    void *client = get_instance();
    if (!client) return;
    /* Size the envelope to the payload: a room mapping can exceed a fixed
     * 8192-byte buffer, and truncating it would produce an unparsable reply.
     * send_message() consumes the text synchronously (passing a stack
     * buffer it could not retain), so freeing after the call is safe. */
    size_t need = 64 + strlen(member) + strlen(value_json) + 1;
    char *buf = (char *)malloc(need);
    if (!buf) return;
    snprintf(buf, need, "{\"id\":%llu,\"%s\":%s}", id, member, value_json);
    VLOG("reply %s\n", buf);
    send_message(client, buf);
    free(buf);
}

static void reply_to_app(void *server, unsigned long long id, const char *result_json) {
    (void)server;
    send_reply(id, "result", result_json);
}

/* A miio error reply, the only shape Valetudo surfaces as a failed command. */
static void reply_error_to_app(void *server, unsigned long long id, const char *message) {
    (void)server;
    char escaped[ROOM_NAME_ESCAPED_MAX * 2];
    json_escape(message, escaped, sizeof escaped);
    char error_json[sizeof escaped + 48];
    snprintf(error_json, sizeof error_json, "{\"code\":%d,\"message\":\"%s\"}",
             V1SEG_MIIO_ERROR_CODE, escaped);
    send_reply(id, "error", error_json);
}

/* ===================================================================================== *
 *  room-segment editing (merge_segment / split_segment) — segedit.c                     *
 * ===================================================================================== */
/* Apply one edit to the `last_segmap` RRSEG sidecar and rewrite it atomically
 * (rebuilt room table + fresh CRC32).  `op` 0 = merge, 1 = split.  `*io` receives
 * the chosen target id (merge) / the fresh user id (split) on success.
 *
 * The split line arrives in Valetudo's mm + Y-flip frame (the shared-grid frame), so
 * the converter must use the *same* blit offset as the sidecar->shared-grid blitter --
 * then sidecar_col = mm/50 - offset.  That offset is now derived from the map extents
 * and the sidecar padding (v1seg_effective_offset), not the fixed V1SEG_BLIT_OFFSET.
 * See TECHNICAL.md. */
static int segedit_apply(int op, const long long *params, int param_count, int *out_id) {
    segedit_grid grid; segedit_meta meta;
    int rc = segedit_sidecar_read(g_cfg.sidecar, &grid, &meta);
    if (rc != SEGEDIT_OK) return rc;
    if (op == 0) {                       /* merge_segment [idA,idB,...] */
        if (param_count < 1) { segedit_grid_free(&grid); return SEGEDIT_EARG; }
        int merge_ids[8], id_count = 0;
        for (int i = 0; i < param_count && id_count < 8; i++) merge_ids[id_count++] = (int)params[i];
        int target_id = 0;
        rc = segedit_merge(&grid, merge_ids, id_count, &target_id);
        if (rc == SEGEDIT_OK && out_id) *out_id = target_id;
    } else {                             /* split_segment [id,x1,y1,x2,y2] */
        if (param_count < 5) { segedit_grid_free(&grid); return SEGEDIT_EARG; }
        /* The mm frame is the shared-grid frame, so the converter must subtract the
         * same offset the sidecar was blitted with -- now derived from the map extents
         * (top/left and the PAD inside the sidecar), not the fixed V1SEG_BLIT_OFFSET. */
        int eff_dx, eff_dy;
        v1seg_effective_offset(&eff_dx, &eff_dy);
        segedit_mm_map mm_map = { 50, eff_dx, eff_dy };
        int new_id = 0;
        rc = segedit_split_mm(&grid, (int)params[0], (long)params[1], (long)params[2],
                              (long)params[3], (long)params[4], &mm_map, &new_id);
        if (rc == SEGEDIT_OK && out_id) *out_id = new_id;
    }
    if (rc == SEGEDIT_OK)
        rc = segedit_sidecar_write_atomic(g_cfg.sidecar, &grid, &meta);
    segedit_grid_free(&grid);
    return rc;
}

/* ===================================================================================== *
 *  name_segment: apply an upstream rename payload to the room table                      *
 * ===================================================================================== */
/* Atomic whole-file write with the errors propagated (tmp + fsync + rename).  A
 * short write / failed fsync / failed rename is reported instead of being lost,
 * so the caller can reply with an error rather than a silent `["ok"]`. */
static int write_file_atomic(const char *path, const char *data, size_t len) {
    char tmp[PATH_MAX];
    if (snprintf(tmp, sizeof tmp, "%s.tmp", path) >= (int)sizeof tmp) return -1;
    int fd = open(tmp, O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return -1;
    size_t off = 0;
    int ok = 1;
    while (off < len) {
        ssize_t nwritten = write(fd, data + off, len - off);
        if (nwritten <= 0) { ok = 0; break; }
        off += (size_t)nwritten;
    }
    if (ok && fsync(fd) != 0) ok = 0;
    if (close(fd) != 0) ok = 0;
    if (!ok) { unlink(tmp); return -1; }
    if (rename(tmp, path) != 0) { unlink(tmp); return -1; }
    return 0;
}

/* Apply `req`'s renames to the room table and rewrite room_names.cfg atomically.
 * Accepts both the upstream array form ({miRoomId,robotRoomId} entries) and the
 * legacy single {segment_id,name} form.
 *
 * Refuses to run when no rooms are known: falling through would rewrite the file
 * EMPTY and wipe every stored name.  The file is written from a copy of the room
 * table with g_lock released, because the fsync can take a while on flash and the
 * map thread needs g_lock for every raster.  The new names are committed by room
 * id once the write succeeded, so a failed write leaves memory matching the file. */
#define RENAME_ERR_NO_RENAMES (-1)
#define RENAME_ERR_NO_ROOMS   (-2)
#define RENAME_ERR_WRITE      (-3)
static int apply_room_renames(const v1seg_req_t *req) {
    if (req->nrenames <= 0 && !(req->have_segment_id && req->have_name)) return RENAME_ERR_NO_RENAMES;
    v1seg_room_t candidate[MAX_ROOMS];
    pthread_mutex_lock(&g_lock);
    int room_count = g_nrooms;
    memcpy(candidate, g_rooms, (size_t)room_count * sizeof candidate[0]);
    pthread_mutex_unlock(&g_lock);
    if (room_count <= 0) return RENAME_ERR_NO_ROOMS;

    if (req->nrenames > 0) {
        for (int i = 0; i < req->nrenames; i++)
            for (int j = 0; j < room_count; j++)
                if (candidate[j].id == req->renames[i].id) {
                    snprintf(candidate[j].name, sizeof candidate[j].name, "%s", req->renames[i].name);
                    break;
                }
    } else {
        for (int j = 0; j < room_count; j++)
            if (candidate[j].id == (int)req->segment_id) {
                snprintf(candidate[j].name, sizeof candidate[j].name, "%s", req->name);
                break;
            }
    }
    char content[MAX_ROOMS * 80];
    size_t out_len = 0;
    for (int i = 0; i < room_count && out_len < sizeof content; i++)
        out_len += (size_t)snprintf(content + out_len, sizeof content - out_len,
                                    "%d=%s\n", candidate[i].id, candidate[i].name);
    if (write_file_atomic(g_cfg.names, content, out_len) != 0) return RENAME_ERR_WRITE;

    pthread_mutex_lock(&g_lock);
    for (int i = 0; i < room_count; i++)
        for (int j = 0; j < g_nrooms; j++)
            if (g_rooms[j].id == candidate[i].id) {
                snprintf(g_rooms[j].name, sizeof g_rooms[j].name, "%s", candidate[i].name);
                break;
            }
    pthread_mutex_unlock(&g_lock);
    return 0;
}

/* ===================================================================================== *
 *  the request hook (called for every single MIIO message)                              *
 * ===================================================================================== */
/* True when `text` carries more than one top-level JSON object (a batch).
 * Strings/escapes are tracked so braces inside params do not count. */
static int v1seg_has_multiple_objects(const char *text) {
    int depth = 0, objects = 0, in_string = 0, escape = 0;
    for (const char *p = text; *p; p++) {
        char c = *p;
        if (in_string) {
            if (escape) escape = 0;
            else if (c == '\\') escape = 1;
            else if (c == '"') in_string = 0;
            continue;
        }
        if (c == '"') { in_string = 1; continue; }
        if (c == '{') {
            if (depth == 0 && ++objects > 1) return 1;
            depth++;
        } else if (c == '}' && depth > 0) {
            depth--;
        }
    }
    return 0;
}

/* Returns: the text to hand to the original AppProxyServer method, or NULL to
 * suppress it (we already answered). */
const char *v1seg_rcv_hook(void *server, const char *text) {
    if (!text) return text;
    /* A batch: let the stock splitter call our second gate per message instead
     * of handling only the first id and suppressing the rest. */
    if (v1seg_has_multiple_objects(text)) return text;
    /* cheap reject */
    if (!strstr(text, "\"method\"")) return text;

    v1seg_req_t req;
    v1seg_parse_request(text, &req);
    if (!req.have_method) return text;

    v1seg_refresh_sidecar(0);

    if (strcmp(req.method, "app_segment_clean") == 0) {
        v1seg_zone_t zones[V1_ZONED_CLEAN_MAX_ZONES + 1];
        int zone_count = plan_segment_clean(&req, zones);
        if (g_cfg.dryrun_clean) {
            /* Observation-only: log the zones the rewrite would have produced and
             * answer as the real path would; the app_zoned_clean text is never built
             * or returned, so no motion is triggered.  See V1SEG_DRYRUN_CLEAN. */
            dryrun_log_clean(&req, zones, zone_count);
            LOG("app_segment_clean: dry-run, rewrite suppressed\n");
        }
        if (zone_count == 0) {
            VLOG("app_segment_clean: no zones for ids (n=%d)\n", req.nseg);
            reply_error_to_app(server, req.id, "none of the requested rooms has cells in the current map");
            return NULL;
        }
        if (zone_count > g_cfg.max_zones) {
            char message[96];
            snprintf(message, sizeof message, "too many rooms: the V1 cleans at most %d per request",
                     g_cfg.max_zones);
            reply_error_to_app(server, req.id, message);
            return NULL;
        }
        if (g_cfg.dryrun_clean) {
            reply_to_app(server, req.id, "[\"ok\"]");
            return NULL;
        }
        char *rewritten = build_zoned_clean_request(&req, zones, zone_count);
        if (!rewritten) {
            reply_error_to_app(server, req.id, "out of memory");
            return NULL;
        }
        VLOG("app_segment_clean -> app_zoned_clean: %s\n", rewritten);
        /* The caller (AppProxyServer) borrows this text for the duration of the call, so keep
         * it reachable in a static (bounded to one buffer) and free the previous one here --
         * by the time the next request arrives it has been consumed.  This avoids both freeing
         * too early (use-after-free) and never freeing (unbounded leak). */
        static char *pending_clean_request;
        free(pending_clean_request);
        pending_clean_request = rewritten;
        return rewritten;
    }
    if (strcmp(req.method, "get_room_mapping") == 0) {
        char *res = build_room_mapping();
        if (res) reply_to_app(server, req.id, res);
        else reply_error_to_app(server, req.id, "out of memory");
        free(res);
        return NULL;
    }
    if (strcmp(req.method, "save_map") == 0) {
        int rc = persist_map_save();
        VLOG("save_map -> %d\n", rc);
        if (rc == PERSIST_ERR_SLAM_TS_ZERO) reply_error_to_app(server, req.id, "the map was just cleared; nothing to save");
        else if (rc < 0) reply_error_to_app(server, req.id, "saving the map snapshot failed");
        else reply_to_app(server, req.id, "[\"ok\"]");
        return NULL;
    }
    if (strcmp(req.method, "reset_map") == 0) {
        int rc = persist_map_reset();
        VLOG("reset_map -> %d\n", rc);
        if (rc < 0) reply_error_to_app(server, req.id, "resetting the map failed");
        else reply_to_app(server, req.id, "[\"ok\"]");
        return NULL;
    }
    if (strcmp(req.method, "get_recover_maps") == 0) {
        char res[512];
        build_recover_maps(res, sizeof res);
        reply_to_app(server, req.id, res);
        return NULL;
    }
    if (strcmp(req.method, "recover_map") == 0) {
        /* Upstream RoborockMapSnapshotCapability sends a bare params array,
         * `recover_map [slot]` (TECHNICAL.md §3); the "map_id"
         * object key is only a legacy/other-client form.  Fall back to the parsed
         * params[0] so a real request selects the intended snapshot slot. */
        int slot;
        if (req.have_mapid) slot = (int)req.mapid;
        else if (req.nparams > 0) slot = (int)req.params[0];
        else slot = 0;
        int rc = persist_map_recover(slot);
        VLOG("recover_map %d -> %d\n", slot, rc);
        if (rc == -2) reply_error_to_app(server, req.id, "no saved map");
        else if (rc == -3) reply_error_to_app(server, req.id, "the saved map does not match the current map");
        else if (rc < 0) reply_error_to_app(server, req.id, "restoring the saved map failed");
        else reply_to_app(server, req.id, "[\"ok\"]");
        return NULL;
    }
    if (strcmp(req.method, "name_segment") == 0) {
        /* Upstream RoborockMapSegmentRenameCapability sends the whole table as
         * [{miRoomId:<name>, robotRoomId:<id>}, …]; parse_rename_array() has it in
         * req.renames.  A write failure is reported as an error, and an empty room
         * list is refused instead of wiping room_names.cfg. */
        int rc = apply_room_renames(&req);
        VLOG("name_segment -> %d (%d renames)\n", rc, req.nrenames);
        if (rc == RENAME_ERR_NO_RENAMES) reply_error_to_app(server, req.id, "no room names in the request");
        else if (rc == RENAME_ERR_NO_ROOMS) reply_error_to_app(server, req.id, "no rooms are known yet");
        else if (rc != 0) reply_error_to_app(server, req.id, "writing room_names.cfg failed");
        else reply_to_app(server, req.id, "[\"ok\"]");
        return NULL;
    }
    if (strcmp(req.method, "merge_segment") == 0) {
        int target = 0;
        int rc = segedit_apply(0, req.params, req.nparams, &target);
        VLOG("merge_segment -> %d (target=%d)\n", rc, target);
        v1seg_refresh_sidecar(1);
        if (rc != SEGEDIT_OK) reply_error_to_app(server, req.id, segedit_strerror(rc));
        else reply_to_app(server, req.id, "[\"ok\"]");
        return NULL;
    }
    if (strcmp(req.method, "split_segment") == 0) {
        int new_id = 0;
        int rc = segedit_apply(1, req.params, req.nparams, &new_id);
        VLOG("split_segment -> %d (newid=%d)\n", rc, new_id);
        v1seg_refresh_sidecar(1);
        if (rc != SEGEDIT_OK) reply_error_to_app(server, req.id, segedit_strerror(rc));
        else reply_to_app(server, req.id, "[\"ok\"]");
        return NULL;
    }
    return text;
}

/* ===================================================================================== *
 *  runtime text patching                                                                *
 * ===================================================================================== */
extern void v1_gate_hdr(void);
extern void v1_gate_copy(void);
extern void v1_gate_deser(void);
extern void v1_gate_rcv(void);
extern void v1_gate_rcv2(void);
extern void *v1_hook_hdr;
extern void *v1_cont_hdr;
extern void *v1_hook_copy;
extern void *v1_cont_copy;
extern void *v1_hook_deser;
extern void *v1_cont_deser;
extern void *v1_hook_rcv;
extern void *v1_rcv_tramp;
extern void *v1_rcv_tramp2;

/* =========================================================================
 * byte-level patch primitives.  They emit Thumb-2 (so the *semantics* are
 * ARM), but the byte arithmetic is arch-independent: a host build can construct
 * an in-memory copy of AppProxy and apply exactly these functions.
 * ========================================================================= */
/* Thumb-2 halfword base encodings (ARMv7-M ARM A7.7.75 MOVW / A7.7.76 MOVT).
 * Both share the layout and differ only in the 0x240 / 0x2c0 opcode base. */
#define THUMB2_MOVW_BASE 0xf240u
#define THUMB2_MOVT_BASE 0xf2c0u

/* Thumb-2 movw/movt encoders, validated against the cross assembler. */
static void enc_mov_half(uint8_t *out, int is_movt, int rd, uint16_t imm) {
    uint16_t imm_bits_12_15 = (uint16_t)((imm >> 12) & 0xf);
    uint16_t imm_bit_11 = (uint16_t)((imm >> 11) & 1);
    uint16_t imm_bits_8_10 = (uint16_t)((imm >> 8) & 7);
    uint16_t imm_bits_0_7 = (uint16_t)(imm & 0xff);
    uint16_t first_halfword =
        (uint16_t)((is_movt ? THUMB2_MOVT_BASE : THUMB2_MOVW_BASE) | (imm_bit_11 << 10) | imm_bits_12_15);
    uint16_t second_halfword = (uint16_t)((imm_bits_8_10 << 12) | ((rd & 0xf) << 8) | imm_bits_0_7);
    out[0] = (uint8_t)(first_halfword & 0xff); out[1] = (uint8_t)(first_halfword >> 8);
    out[2] = (uint8_t)(second_halfword & 0xff); out[3] = (uint8_t)(second_halfword >> 8);
}

/* Write the 12-byte "movw r12,#lo / movt r12,#hi / bx r12 / nop" branch. */
static void emit_site_branch(uint8_t *site, uint32_t target) {
    enc_mov_half(site + 0, 0, 12, (uint16_t)(target & 0xffff));
    enc_mov_half(site + 4, 1, 12, (uint16_t)((target >> 16) & 0xffff));
    site[8] = 0x60; site[9] = 0x47;     /* bx r12 */
    site[10] = 0x00; site[11] = 0xbf;   /* nop    */
}

/* Build the RcvMessageFromMIIO trampoline: the overwritten prologue
 * bytes followed by "movw/movt r12,#cont / bx r12". */
static void *build_rcv_tramp(uint8_t *entry, uint32_t cont);
static uint8_t *g_rwx = NULL;
static size_t g_rwx_used = 0;

static void *rwx_alloc(size_t size) {
    size = (size + 7) & ~(size_t)7;
    if (!g_rwx) {
        /* Mapped RW for population; the page is set R-X once its trampoline is
         * written (never W^X at the same time).  See patch_appproxy(). */
        g_rwx = (uint8_t *)mmap(NULL, 4096, PROT_READ | PROT_WRITE,
                                MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
        if (g_rwx == MAP_FAILED) { g_rwx = NULL; return NULL; }
        g_rwx_used = 0;
    }
    if (g_rwx_used + size > 4096) return NULL;
    void *result = g_rwx + g_rwx_used;
    g_rwx_used += size;
    return result;
}

static void *build_rcv_tramp(uint8_t *entry, uint32_t continuation) {
    uint8_t *tramp = (uint8_t *)rwx_alloc(OFF_RCV_PROLOGUE_LEN + 10);
    if (!tramp) return NULL;
    memcpy(tramp, entry, OFF_RCV_PROLOGUE_LEN);
    enc_mov_half(tramp + OFF_RCV_PROLOGUE_LEN + 0, 0, 12, (uint16_t)(continuation & 0xffff));
    enc_mov_half(tramp + OFF_RCV_PROLOGUE_LEN + 4, 1, 12, (uint16_t)((continuation >> 16) & 0xffff));
    tramp[OFF_RCV_PROLOGUE_LEN + 8] = 0x60;
    tramp[OFF_RCV_PROLOGUE_LEN + 9] = 0x47;
    __builtin___clear_cache((char *)tramp, (char *)tramp + OFF_RCV_PROLOGUE_LEN + 10);
    return tramp;
}

/* ---------------------------------------------------------------- preflight */
static int edit_bytes(uint8_t *base, uint32_t off, const uint8_t *want,
                      const uint8_t *expect, size_t len, const char *what) {
    uint8_t *site = base + off;
    if (expect && memcmp(site, expect, len) != 0) {
        WARN("%s @0x%x: unexpected original bytes %02x %02x %02x %02x - not patched\n",
             what, off, site[0], site[1], site[2], site[3]);
        return -1;
    }
    memcpy(site, want, len);
    VLOG("patched %s @0x%x\n", what, off);
    return 0;
}

static int edit_site(uint8_t *base, uint32_t off, void *gate, const char *what) {
    uint8_t *site = base + off;
    emit_site_branch(site, (uint32_t)(uintptr_t)gate);
    VLOG("gated %s @0x%x -> %p\n", what, off, gate);
    return 0;
}

/* Original bytes at every patch site / edit.  The runtime patcher refuses to
 * touch anything else.
 * The gate sites are checked as their full 12 bytes because emit_site_branch()
 * overwrites all 12: a build whose first few bytes matched but whose tail
 * differed would otherwise be accepted and patched at a wrong binary. */
static const uint8_t ORIG_MAPDATA_CTOR_HL[4]  = { 0x4f, 0xf0, 0x18, 0x08 };
static const uint8_t PATCHED_MAPDATA_CTOR_HL[4] = { 0x4f, 0xf0, 0x1c, 0x08 };
static const uint8_t ORIG_GEN_PIXOFF[4]       = { 0x07, 0xf1, 0x48, 0x04 };
static const uint8_t PATCHED_GEN_PIXOFF[4]    = { 0x07, 0xf1, 0x4c, 0x04 };
static const uint8_t ORIG_GEN_DATALEN[4]      = { 0xaa, 0xf1, 0x18, 0x03 };
static const uint8_t PATCHED_GEN_DATALEN[4]   = { 0xaa, 0xf1, 0x1c, 0x03 };
static const uint8_t ORIG_GEN_SKIP[2]         = { 0x69, 0xd1 };
static const uint8_t PATCHED_GEN_SKIP[2]      = { 0x00, 0xbf };
static const uint8_t ORIG_DESER_LDRH[2]       = { 0x6b, 0x68 };
static const uint8_t PATCHED_DESER_LDRH[2]    = { 0x6a, 0x88 };
static const uint8_t ORIG_DESER_LDR[2]        = { 0xe4, 0x1a };
static const uint8_t PATCHED_DESER_LDR[2]     = { 0x6b, 0x68 };
static const uint8_t ORIG_DESER_ADD[2]        = { 0x18, 0x33 };
static const uint8_t PATCHED_DESER_ADD[2]     = { 0x13, 0x44 };
static const uint8_t ORIG_DESER_SUB[2]        = { 0x18, 0x3c };
static const uint8_t PATCHED_DESER_SUB[2]     = { 0xe4, 0x1a };
static const uint8_t ORIG_GEN_HDR_SITE[12]    = { 0xd5, 0xf8, 0x5c, 0x18, 0xd5, 0xf8, 0x60, 0x28, 0xd5, 0xf8, 0x64, 0x68 };
static const uint8_t ORIG_GEN_COPY_SITE[12]   = { 0x86, 0x42, 0x25, 0xda, 0x05, 0xf6, 0x6c, 0x0b, 0x05, 0xf5, 0x07, 0x63 };
static const uint8_t ORIG_DESER_IMG_SITE[12]  = { 0x28, 0x68, 0x07, 0xf5, 0x10, 0x03, 0x69, 0x68, 0x4f, 0xf0, 0x01, 0x0e };
/* The full 12-byte prologue copied by build_rcv_tramp: movw r3,#0xc4a8 /
 * movt r3,#0xc / stmdb sp!,{r4-r9,sl,fp,lr}.  Checking all 12 bytes (not just
 * the movw) keeps a different build from being patched and then executing a
 * stale prologue from the trampoline. */
static const uint8_t ORIG_RCV_SITE[12]        = { 0x4c, 0xf2, 0xa8, 0x43, 0xc0, 0xf2, 0x0c, 0x03, 0x2d, 0xe9, 0xf0, 0x4f };
/* RcvSingleMessageFromMIIO(char const*) prologue: stmdb sp!,{r4-r9,lr} /
 * movw r6,#0xc4a8 / movt r6,#0xc. */
static const uint8_t ORIG_RCV2_SITE[12]       = { 0x2d, 0xe9, 0xf0, 0x43, 0x4c, 0xf2, 0xa8, 0x46, 0xc0, 0xf2, 0x0c, 0x06 };

static int match_at(const uint8_t *base, uint32_t off, const uint8_t *want, size_t len) {
    return memcmp(base + off, want, len) == 0;
}

/* Verify the whole patch table before writing anything.  0 = safe to patch. */
int v1seg_check_sites(const uint8_t *base, int patch_deser) {
    int ok = 1;
#define NEED(off, arr) do { if (!match_at(base, (off), (arr), sizeof(arr))) { \
        WARN("preflight: %s @0x%x does not match this AppProxy build\n", #off, (unsigned)(off)); ok = 0; } } while (0)
    NEED(OFF_MAPDATA_CTOR_HL, ORIG_MAPDATA_CTOR_HL);
    NEED(OFF_GEN_PIXOFF, ORIG_GEN_PIXOFF);
    NEED(OFF_GEN_DATALEN, ORIG_GEN_DATALEN);
    NEED(OFF_GEN_SKIP_SITE, ORIG_GEN_SKIP);
    NEED(OFF_GEN_HDR_SITE, ORIG_GEN_HDR_SITE);
    NEED(OFF_GEN_COPY_SITE, ORIG_GEN_COPY_SITE);
    NEED(OFF_RCV_SITE, ORIG_RCV_SITE);
    NEED(OFF_RCV2_SITE, ORIG_RCV2_SITE);
    if (patch_deser) {
        NEED(OFF_DESER_LDRH, ORIG_DESER_LDRH);
        NEED(OFF_DESER_LDR, ORIG_DESER_LDR);
        NEED(OFF_DESER_ADD, ORIG_DESER_ADD);
        NEED(OFF_DESER_SUB, ORIG_DESER_SUB);
        NEED(OFF_DESER_IMG_SITE, ORIG_DESER_IMG_SITE);
    }
#undef NEED
    return ok ? 0 : -1;
}

int v1seg_apply_edits(uint8_t *base, int patch_deser) {
    int applied = 0;
    applied += edit_bytes(base, OFF_MAPDATA_CTOR_HL, PATCHED_MAPDATA_CTOR_HL, ORIG_MAPDATA_CTOR_HL, 4, "MapData ctor grid hl=28") == 0;
    applied += edit_bytes(base, OFF_GEN_PIXOFF,  PATCHED_GEN_PIXOFF, ORIG_GEN_PIXOFF, 4, "GenerateMapData pixels +76") == 0;
    applied += edit_bytes(base, OFF_GEN_DATALEN, PATCHED_GEN_DATALEN, ORIG_GEN_DATALEN, 4, "GenerateMapData data_length -28") == 0;
    applied += edit_bytes(base, OFF_GEN_SKIP_SITE, PATCHED_GEN_SKIP, ORIG_GEN_SKIP, 2, "GenerateMapData always rasterise (skip branch nop)") == 0;
    if (patch_deser) {
        applied += edit_bytes(base, OFF_DESER_LDRH, PATCHED_DESER_LDRH, ORIG_DESER_LDRH, 2, "Deserialize ldrh header_length") == 0;
        applied += edit_bytes(base, OFF_DESER_LDR,  PATCHED_DESER_LDR,  ORIG_DESER_LDR,  2, "Deserialize ldr data_length") == 0;
        applied += edit_bytes(base, OFF_DESER_ADD,  PATCHED_DESER_ADD,  ORIG_DESER_ADD,  2, "Deserialize add header_length") == 0;
        applied += edit_bytes(base, OFF_DESER_SUB,  PATCHED_DESER_SUB,  ORIG_DESER_SUB,  2, "Deserialize subs step") == 0;
    }
    return applied;
}

int v1seg_apply_sites(uint8_t *base, int patch_deser,
                      void *gate_hdr, void *gate_copy, void *gate_deser,
                      void *gate_rcv, void *gate_rcv2) {
    int applied = 0;
    applied += edit_site(base, OFF_GEN_HDR_SITE,  gate_hdr,  "GenerateMapData header") == 0;
    applied += edit_site(base, OFF_GEN_COPY_SITE, gate_copy, "GenerateMapData copy") == 0;
    if (patch_deser) applied += edit_site(base, OFF_DESER_IMG_SITE, gate_deser, "Deserialize image block") == 0;
    /* The RCV gate branches through the trampoline built by build_rcv_tramp().  If that could
     * not be allocated there is no valid continuation, so patching this site would send a later
     * MIIO message through a NULL pointer -- leave the receive path unpatched instead.  The map
     * patches above still apply; only request interception (segment clean / room mapping) is off. */
    if (gate_rcv) applied += edit_site(base, OFF_RCV_SITE, gate_rcv, "RcvMessageFromMIIO") == 0;
    if (gate_rcv2) applied += edit_site(base, OFF_RCV2_SITE, gate_rcv2,
                                        "RcvSingleMessageFromMIIO (batched)") == 0;
    return applied;
}

/* True when addr..addr+len is covered by some mapping in /proc/self/maps. */
static int range_mapped(uintptr_t addr, size_t len) {
    FILE *maps_file = fopen("/proc/self/maps", "r");
    if (!maps_file) return 0;
    char line[1024];
    int mapped = 0;
    while (fgets(line, sizeof line, maps_file)) {
        unsigned long map_start, map_end;
        if (sscanf(line, "%lx-%lx", &map_start, &map_end) != 2) continue;
        if (map_start <= addr && addr + len <= map_end) { mapped = 1; break; }
    }
    fclose(maps_file);
    return mapped;
}

/* Log the effective /proc/self/maps permissions of [addr, addr+len) so a debugger
 * can prove .text/trampoline are not left writable+executable. */
static void log_region_perms(const char *what, uintptr_t addr, size_t len) {
    FILE *maps_file = fopen("/proc/self/maps", "r");
    if (!maps_file) return;
    char line[1024];
    while (fgets(line, sizeof line, maps_file)) {
        unsigned long map_start, map_end;
        char perms[8] = "";
        if (sscanf(line, "%lx-%lx %7s", &map_start, &map_end, perms) != 3) continue;
        if (map_start <= addr && addr + len <= map_end) { VLOG("%s perms: %s\n", what, perms); break; }
    }
    fclose(maps_file);
}

/* Only ever patch a process whose executable is literally named AppProxy. */
static int exe_is_appproxy(void) {
    char exe_path[512] = "";
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof exe_path - 1);
    if (len <= 0) return 0;
    exe_path[len] = 0;
    const char *basename = strrchr(exe_path, '/');
    basename = basename ? basename + 1 : exe_path;
    return strcmp(basename, "AppProxy") == 0;
}

/* Find the ELF *load bias* of the running executable and store it in *out_bias.
 * Returns 0 on success, -1 if it cannot be determined.
 *
 * The bias is the value added to a link-time vaddr to obtain its runtime
 * address.  This must be the bias, not the mapping start: AppProxy is a non-PIE
 * ET_EXEC whose first PT_LOAD has p_vaddr = map_start (both 0x10000), so the
 * bias is 0 and every OFF_* (a link-time vaddr) is used directly as base+OFF.
 * Returning the mapping start here would shift every site by 0x10000 -- which
 * is exactly the bug that made preflight reject the real binary.  (Because a
 * zero bias is valid, the result is a status + out-param, not a NULL pointer.) */
static int find_exe_base(void **out_bias) {
    char exe_path[512] = "";
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof exe_path - 1);
    if (len <= 0) return -1;
    exe_path[len] = 0;
    FILE *maps_file = fopen("/proc/self/maps", "r");
    if (!maps_file) return -1;
    char line[1024];
    uintptr_t map_start = 0;
    while (fgets(line, sizeof line, maps_file)) {
        unsigned long start_addr, end_addr, file_off;
        char perms[8];
        if (sscanf(line, "%lx-%lx %7s %lx", &start_addr, &end_addr, perms, &file_off) != 4) continue;
        if (file_off == 0 && strstr(line, exe_path)) { map_start = (uintptr_t)start_addr; break; }
    }
    fclose(maps_file);
    if (!map_start) return -1;

    /* Parse the in-memory ELF program headers for the lowest PT_LOAD vaddr (the
     * ELF header sits at file offset 0, i.e. at map_start in the first R-E LOAD). */
    const Elf32_Ehdr *ehdr = (const Elf32_Ehdr *)map_start;
    if (memcmp(ehdr->e_ident, ELFMAG, SELFMAG) != 0) return -1;
    uintptr_t lowest_vaddr = 0;
    int have_load = 0;
    for (int i = 0; i < ehdr->e_phnum; i++) {
        const Elf32_Phdr *phdr = (const Elf32_Phdr *)(map_start + ehdr->e_phoff + (uintptr_t)i * ehdr->e_phentsize);
        if (phdr->p_type != PT_LOAD) continue;
        if (!have_load || phdr->p_vaddr < lowest_vaddr) { lowest_vaddr = phdr->p_vaddr; have_load = 1; }
    }
    if (!have_load) return -1;
    *out_bias = (void *)(map_start - lowest_vaddr);
    return 0;
}

#if defined(__arm__) && !defined(V1SEG_TEST_BUILD)
static void patch_appproxy(void) {
    if (find_exe_base(&g_exe_base) != 0) { WARN("could not find AppProxy base in /proc/self/maps\n"); return; }
    g_exe_base_valid = 1;   /* the load bias is 0 for this non-PIE ET_EXEC, so this flag -- not
                             * the pointer value -- is what reply_to_app must test. */
    VLOG("AppProxy load bias = %p\n", g_exe_base);
    uint8_t *base = (uint8_t *)g_exe_base;

    if (!range_mapped((uintptr_t)base + OFF_RCV_SITE, 16) ||
        !range_mapped((uintptr_t)base + OFF_RCV2_SITE, 16) ||
        !range_mapped((uintptr_t)base + OFF_GEN_DATALEN, 8) ||
        !range_mapped((uintptr_t)base + OFF_GEN_HDR_SITE, 4)) {
        WARN("AppProxy image looks smaller than expected; not patching\n");
        return;
    }

    if (v1seg_check_sites(base, g_cfg.patch_deser) != 0) {
        WARN("refusing to patch: this does not look like the expected AppProxy "
             "(sha256 %s)\n", APP_SHA256);
        return;
    }
    /* make .text writable */
    uintptr_t page_size = 4096;
    uintptr_t text_start = ((uintptr_t)base + TEXT_VADDR_START) & ~(page_size - 1);
    uintptr_t text_end = ((uintptr_t)base + TEXT_VADDR_END + page_size - 1) & ~(page_size - 1);
    if (mprotect((void *)text_start, text_end - text_start, PROT_READ | PROT_WRITE | PROT_EXEC) != 0) {
        WARN("mprotect(.text) failed: %s\n", strerror(errno));
        return;
    }

    v1seg_apply_edits(base, g_cfg.patch_deser);

    v1_hook_hdr  = (void *)v1seg_write_image_header;
    v1_hook_copy = (void *)v1seg_stamp_copy;
    v1_hook_deser = (void *)v1seg_deser_image;
    /* The continuation words hold *link-time vaddrs* (raw arithmetic, not C function
     * pointers), and AppProxy is Thumb code: `bx`/`blx` to an even address switches the
     * core to ARM state, which faults on the next instruction.  Set the Thumb bit (bit 0)
     * explicitly on every address we branch to with bx/blx.  (The gate entry points are
     * fine: those come from C function pointers, whose low bit the compiler already sets.) */
    v1_cont_hdr  = (void *)((uintptr_t)(base + OFF_GEN_HDR_CONT) | 1u);
    v1_cont_copy = (void *)((uintptr_t)(base + OFF_GEN_COPY_CONT) | 1u);
    v1_cont_deser = (void *)((uintptr_t)(base + OFF_DESER_IMG_CONT) | 1u);
    v1_hook_rcv  = (void *)v1seg_rcv_hook;
    uint8_t *tramp_code = build_rcv_tramp(base + OFF_RCV_SITE,
                                          (uint32_t)((uintptr_t)(base + OFF_RCV_CONT) | 1u));
    v1_rcv_tramp = tramp_code ? (void *)((uintptr_t)tramp_code | 1u) : NULL;  /* Thumb bit for blx */
    uint8_t *tramp_code2 = build_rcv_tramp(base + OFF_RCV2_SITE,
                                           (uint32_t)((uintptr_t)(base + OFF_RCV2_CONT) | 1u));
    v1_rcv_tramp2 = tramp_code2 ? (void *)((uintptr_t)tramp_code2 | 1u) : NULL;
    void *gate_rcv = (void *)v1_gate_rcv;
    void *gate_rcv2 = (void *)v1_gate_rcv2;
    /* The trampoline is written once (build_rcv_tramp) and only executed after
     * that, so its page does not need to stay writable: drop W to R-X. */
    if (g_rwx) {
        if (mprotect(g_rwx, 4096, PROT_READ | PROT_EXEC) != 0)
            WARN("mprotect(trampoline) R-X failed: %s\n", strerror(errno));
        log_region_perms("tramp", (uintptr_t)g_rwx, OFF_RCV_PROLOGUE_LEN + 10);
    }
    if (!v1_rcv_tramp) {
        /* No trampoline -> v1_gate_rcv would `blx` to NULL.  Rather than risk crashing the
         * cleaner on the next MIIO message, skip the receive-site patch entirely. */
        WARN("could not build the Rcv trampoline; leaving the MIIO receive path unpatched\n");
        gate_rcv = NULL;
    }
    if (!v1_rcv_tramp2) {
        WARN("could not build the batched-message trampoline; batches stay unhandled\n");
        gate_rcv2 = NULL;
    }
    /* Every one of these must be odd (Thumb): bx/blx to an even address faults.  Logged so
     * a debugger can assert it -- this is the class of bug a byte-level check misses. */
    VLOG("branch targets: hdr=%p copy=%p deser=%p rcv=%p rcv2=%p\n",
         v1_cont_hdr, v1_cont_copy, v1_cont_deser, v1_rcv_tramp, v1_rcv_tramp2);
    /* Call-out words so a debugger can set breakpoints / corrupt a word without hardcoding
     * offsets into the shim: the addresses of the two data words and their current values. */
    VLOG("rcv words: hook_addr=%p tramp_addr=%p hook=%p tramp=%p tramp2_addr=%p tramp2=%p\n",
         (void *)&v1_hook_rcv, (void *)&v1_rcv_tramp, v1_hook_rcv, v1_rcv_tramp,
         (void *)&v1_rcv_tramp2, v1_rcv_tramp2);
    /* MIIO call-out targets (Thumb code in AppProxy).  Logged so a debugger can assert bit0
     * is set on every indirect call target -- a raw offset does not get it automatically the
     * way a C function pointer does. */
    VLOG("miio callouts: get_instance=%p send_message=%p\n",
         exe_call_ptr(OFF_MIIO_GETINSTANCE), exe_call_ptr(OFF_MIIO_SEND));

    v1seg_apply_sites(base, g_cfg.patch_deser,
                      (void *)v1_gate_hdr, (void *)v1_gate_copy,
                      (void *)v1_gate_deser, gate_rcv, gate_rcv2);

    __builtin___clear_cache((char *)text_start, (char *)text_end);
    /* Every edit is in memory and the caches are coherent: .text is executed only,
     * so restore it to R-X instead of leaving it writable+executable. */
    if (mprotect((void *)text_start, text_end - text_start, PROT_READ | PROT_EXEC) != 0)
        WARN("mprotect(.text) restore R-X failed: %s\n", strerror(errno));
    log_region_perms("text", text_start, text_end - text_start);
    VLOG("AppProxy patched\n");
}
#endif /* __arm__ && !V1SEG_TEST_BUILD */

/* ===================================================================================== *
 *  constructor                                                                          *
 * ===================================================================================== */
#if defined(__arm__) && !defined(V1SEG_TEST_BUILD)
__attribute__((constructor)) static void v1seg_ctor(void) {
    /* LD_PRELOAD is inherited by every process AppProxy starts; those must not
     * open the log, allocate the grid, create persist/ or read the sidecar. */
    if (!exe_is_appproxy()) return;
    cfg_init();
    if (getenv("V1SEG_DISABLE")) { VLOG("disabled via V1SEG_DISABLE\n"); return; }
    VLOG("loading (AppProxy sha256 %s)\n", APP_SHA256);
    segmap_alloc();
    persist_set_root(g_cfg.root);
    persist_init();
    v1seg_refresh_sidecar(1);
    patch_appproxy();
}

__attribute__((destructor)) static void v1seg_dtor(void) { cfg_fini(); }
#endif

/* ===================================================================================== *
 *  host/debug entry points                                                              *
 * ===================================================================================== */

/* Exported (despite -fvisibility=hidden) so a loader can dlopen-check us. */
__attribute__((visibility("default"))) const char *v1seg_addon_version(void) {
    return "v1seg-addon/1.0 (blob 24->28 + app_segment_clean/get_room_mapping/merge_segment/split_segment/...)";
}
__attribute__((visibility("default"))) int v1seg_addon_seg_count(void) { return g_seg_count; }

int v1seg_test_load_sidecar_file(const char *path, int dx, int dy) {
    struct stat file_stat;
    if (stat(path, &file_stat) != 0 || file_stat.st_size <= 0) return -2;
    uint8_t *file_buf = (uint8_t *)malloc((size_t)file_stat.st_size);
    if (!file_buf) return -3;
    FILE *file = fopen(path, "rb");
    if (!file) { free(file_buf); return -4; }
    size_t nread = fread(file_buf, 1, (size_t)file_stat.st_size, file);
    fclose(file);
    char err[128] = "";
    int room_count = v1seg_parse_sidecar(file_buf, nread, dx, dy, err, sizeof err);
    free(file_buf);
    if (room_count < 0) { fprintf(stderr, "sidecar: %s\n", err); return -1; }
    return room_count;
}

void v1seg_test_config(const char *root) {
    if (root && *root) {
        snprintf(g_cfg.root, sizeof g_cfg.root, "%s", root);
        snprintf(g_cfg.sidecar, sizeof g_cfg.sidecar, "%s/last_segmap", root);
        snprintf(g_cfg.names, sizeof g_cfg.names, "%s/room_names.cfg", root);
    }
}
void v1seg_test_set_reply_sink(void (*fn)(unsigned long long, const char *)) { g_reply_sink = fn; }
void v1seg_test_set_dryrun(int on) { g_cfg.dryrun_clean = on ? 1 : 0; }
void v1seg_test_set_zone_endpoint(int inclusive) { g_cfg.zone_endpoint_inclusive = inclusive ? 1 : 0; }
const char *v1seg_test_handle(void *server, const char *text) { return v1seg_rcv_hook(server, text); }

int v1seg_test_room_count(void) { return g_nrooms; }
const char *v1seg_test_room_name(int i) { return (i >= 0 && i < g_nrooms) ? g_rooms[i].name : ""; }
int v1seg_test_room_id(int i) { return (i >= 0 && i < g_nrooms) ? g_rooms[i].id : -1; }
int v1seg_test_seg_count(void) { return g_seg_count; }
uint8_t *v1seg_test_segmap(void) { return g_segmap; }
void v1seg_test_set_state(int nrooms, int seg_count) { g_nrooms = nrooms; g_seg_count = seg_count; }
int v1seg_test_parse_request(const char *text, v1seg_req_t *req) { return v1seg_parse_request(text, req); }
int v1seg_test_build_clean(const char *request_json, char *out, size_t cap) {
    v1seg_req_t req;
    v1seg_parse_request(request_json, &req);
    v1seg_zone_t zones[V1_ZONED_CLEAN_MAX_ZONES + 1];
    int zone_count = plan_segment_clean(&req, zones);
    if (zone_count <= 0 || zone_count > g_cfg.max_zones) return -1;
    char *rewritten = build_zoned_clean_request(&req, zones, zone_count);
    if (!rewritten) return -1;
    snprintf(out, cap, "%s", rewritten);
    free(rewritten);
    return 0;
}
/* Debug seam: returns the malloc'd room-mapping JSON (caller frees) so it can be
 * inspected without truncation. */
char *v1seg_test_build_room_mapping(void) { return build_room_mapping(); }
uint8_t *v1seg_test_write_image_header(void *mapdata) {
    v1seg_write_image_header(mapdata);
    return (uint8_t *)mapdata + MD_IMAGE_HDR;
}
uint8_t *v1seg_test_stamp_copy(void *mapdata, uint8_t *dst) { return v1seg_stamp_copy(mapdata, dst); }
void v1seg_test_deser_image(void *mapdata, const uint8_t *block) { v1seg_deser_image(mapdata, block); }
int v1seg_test_segments_to_zones(const long long *segs, int count, int repeat_count, int *out, int max_zones) {
    v1seg_zone_t zones[8];
    if (max_zones > 8) max_zones = 8;
    int zone_count = v1seg_segments_to_zones(segs, count, repeat_count, zones, max_zones);
    for (int i = 0; i < zone_count; i++) {
        out[i * 5 + 0] = zones[i].x1; out[i * 5 + 1] = zones[i].y1;
        out[i * 5 + 2] = zones[i].x2; out[i * 5 + 3] = zones[i].y2;
        out[i * 5 + 4] = zones[i].iterations;
    }
    return zone_count;
}
void v1seg_test_blit_grid(const uint8_t *cells, int w, int h, int dx, int dy) {
    v1seg_blit_grid(cells, w, h, dx, dy);
}
void v1seg_test_reset(void) { v1seg_reset_segmap(); }
#if defined(__arm__)
/* expose the encoder for the on-target encoding self-check */
void v1seg_test_emit_site(uint8_t *site, uint32_t target) { emit_site_branch(site, target); }
#endif

