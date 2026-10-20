/* v1appd.c -- runtime driver for the segment producer.
 *
 * v1appd watches last_map and runs `v1seg --lastmap ...` as a non-blocking child
 * when the map changes. Failed or timed-out passes are retried with backoff.
 *
 * It also makes one AppProxy decision at startup: adopt a shim-preloaded
 * instance, launch one when none is running, or leave a stock instance alone
 * and warn. Resident supervision is opt-in with --supervise-appproxy; when
 * enabled it keeps a preloaded AppProxy alive, prunes duplicates, rate-limits
 * restarts, and never presents two AppProxy processes.
 *
 * Logging uses write() only; the V1's glibc 2.19 is fragile around stdio buffering.
 *
 * Usage:
 *   v1appd [--root DIR] [--map FILE] [--out FILE] [--producer PATH] [--libdir DIR]
 *          [--shim PATH] [--appproxy PATH] [--min-area N] [--settle-ms N]
 *          [--poll-ms N] [--supervise-ms N] [--producer-timeout-ms N]
 *          [--initial-wait-ms N]
 *          [--restart-min-ms N] [--restart-max N] [--restart-window-ms N]
 *          [--log FILE] [--once] [--force] [--producer-only] [--supervise-appproxy]
 *          [--dry-run] [--verbose]
 */
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
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
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#ifndef PATH_MAX
#define PATH_MAX 4096
#endif
#ifndef NAME_MAX
#define NAME_MAX 255
#endif

#define MAP_WATCH_MASK (IN_CLOSE_WRITE | IN_MOVED_TO | IN_MOVED_FROM | IN_DELETE | IN_CREATE | \
                        IN_DELETE_SELF | IN_MOVE_SELF)

#define MAX_TRACKED_APPROXY 64         /* stock AppProxies we will terminate per scan */
#define ENVIRON_READ_MAX 65536         /* /proc/<pid>/environ read cap */
#define PRODUCER_KILL_GRACE_MS 2000    /* SIGTERM -> SIGKILL grace for the producer */
#define APPROXY_KILL_GRACE_MS 2000     /* SIGTERM -> SIGKILL grace for a stock AppProxy */
#define STARTUP_GATE_POLL_MS 250       /* startup-gate poll cadence while waiting for AppProxy */
#define PRODUCER_RETRY_BASE_MS 1000    /* first producer-failure retry delay (doubles) */
#define PRODUCER_RETRY_MAX_MS 60000    /* ceiling for the producer-failure backoff */
#define APPROXY_HEALTHY_MIN_MS 60000   /* same preloaded pid up this long => forgive the budget */

/* Child OOM policy.  Our daemons run at `oom score 1000` (upstart), ahead of the
 * navigation stack/Valetudo at stock; the transient producer is the memory hog and
 * is retried with backoff, so it must stay at 1000 and be the FIRST victim.  The
 * shimmed AppProxy (the Valetudo bridge) must never be the victim, so it is reset
 * to 0. */
#define OOM_SCORE_ADJ_PRODUCER 1000
#define OOM_SCORE_ADJ_BRIDGE   0

/* ===================================================================================== *
 *  logging / signals                                                                    *
 * ===================================================================================== */
static int g_log_fd = STDERR_FILENO;
static int g_verbose = 0;
static volatile sig_atomic_t g_stop = 0;

/* Raw-write logging (never stdio): see the file header. */
static void log_msg(const char *fmt, ...) {
    char message[2048];
    va_list ap;
    va_start(ap, fmt);
    int length = vsnprintf(message, sizeof message, fmt, ap);
    va_end(ap);
    if (length < 0) return;
    if ((size_t)length >= sizeof message) length = (int)sizeof message - 1;
    ssize_t written = write(g_log_fd, message, (size_t)length);
    (void)written;
}

static void handle_stop_signal(int signo) { (void)signo; g_stop = 1; }

/* ===================================================================================== *
 *  small helpers                                                                        *
 * ===================================================================================== */
/* Bounded, warning-free join (never overflows; truncates if the pair cannot fit). */
static void join_path(char *buf, size_t cap, const char *dir, const char *name) {
    if (cap == 0) return;
    size_t pos = 0;
    for (const char *cursor = dir; *cursor && pos + 1 < cap; cursor++) buf[pos++] = *cursor;
    if (pos + 1 < cap) buf[pos++] = '/';
    for (const char *cursor = name; *cursor && pos + 1 < cap; cursor++) buf[pos++] = *cursor;
    buf[pos] = '\0';
}

/* dst = the directory component of `path` ("." if there is none, "/" for "/name"). */
static void dir_name(char *dst, size_t cap, const char *path) {
    snprintf(dst, cap, "%s", path);
    char *slash = strrchr(dst, '/');
    if (!slash) { snprintf(dst, cap, "%s", "."); return; }
    if (slash == dst) { dst[1] = '\0'; return; }
    *slash = '\0';
}

/* dst = the final component of `path` (truncated to fit; warning-free). */
static void base_name(char *dst, size_t cap, const char *path) {
    const char *slash = strrchr(path, '/');
    const char *name = slash ? slash + 1 : path;
    if (cap == 0) return;
    size_t length = strlen(name);
    if (length >= cap) length = cap - 1;
    memcpy(dst, name, length);
    dst[length] = '\0';
}

static int is_all_digits(const char *text) {
    if (!*text) return 0;
    for (const char *cursor = text; *cursor; cursor++)
        if (*cursor < '0' || *cursor > '9') return 0;
    return 1;
}

static long long monotonic_ms(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (long long)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static void sleep_ms(int milliseconds) {
    if (milliseconds <= 0) return;
    struct timespec remaining;
    remaining.tv_sec = milliseconds / 1000;
    remaining.tv_nsec = (long)(milliseconds % 1000) * 1000000L;
    while (nanosleep(&remaining, &remaining) != 0 && errno == EINTR) {
        if (g_stop) return;   /* re-sleep the remainder unless asked to stop */
    }
}

/* Set this process's OOM bias (best effort).  The upstart job sets `oom score
 * 1000` on v1appd, and a forked child inherits it; each child then applies its
 * own policy here before exec.  A kernel refusal (e.g. raising the value without
 * privilege) is ignored -- the child keeps whatever it inherited. */
static void set_oom_score_adj(int value) {
    char text[16];
    int length = snprintf(text, sizeof text, "%d", value);
    if (length <= 0 || (size_t)length >= sizeof text) return;
    int oom_fd = open("/proc/self/oom_score_adj", O_WRONLY);
    if (oom_fd < 0) return;
    ssize_t written = write(oom_fd, text, (size_t)length);
    (void)written;
    close(oom_fd);
}

/* ---------------------------------------------------------------- config */
struct config {
    char root[PATH_MAX];
    char map_path[PATH_MAX];
    char map_dir[PATH_MAX];
    char out_path[PATH_MAX];
    char lock_path[PATH_MAX];
    char producer_path[PATH_MAX];
    char lib_dir[PATH_MAX];
    char shim_path[PATH_MAX];
    char appproxy_path[PATH_MAX];
    char log_path[PATH_MAX];
    long min_area;
    int  settle_ms;
    int  poll_ms;
    int  supervise_ms;
    int  producer_timeout_ms;
    int  initial_wait_ms;
    int  restart_min_ms;
    int  restart_max;
    int  restart_window_ms;
    int  run_once;
    int  force_produce;
    int  producer_only;
    int  supervise_appproxy;
    int  dry_run;
};

static void config_defaults(struct config *settings) {
    memset(settings, 0, sizeof *settings);
    snprintf(settings->root, sizeof settings->root, "%s", "/mnt/data/rockrobo");
    snprintf(settings->producer_path, sizeof settings->producer_path, "%s",
             "/opt/rockrobo/cleaner/bin/v1seg");
    snprintf(settings->lib_dir, sizeof settings->lib_dir, "%s",
             "/opt/rockrobo/cleaner/lib");
    snprintf(settings->shim_path, sizeof settings->shim_path, "%s",
             "/mnt/data/rockrobo/rrtest/libv1seg_addon.so");
    snprintf(settings->appproxy_path, sizeof settings->appproxy_path, "%s",
             "/opt/rockrobo/cleaner/bin/AppProxy");
    settings->min_area = 500;
    settings->settle_ms = 500;
    settings->poll_ms = 5000;
    settings->supervise_ms = 1000;
    settings->producer_timeout_ms = 120000;
    settings->initial_wait_ms = 30000;
    settings->restart_min_ms = 2000;
    settings->restart_max = 5;
    settings->restart_window_ms = 120000;
}

static void config_paths(struct config *settings) {
    if (!settings->map_path[0])
        join_path(settings->map_path, sizeof settings->map_path, settings->root, "last_map");
    if (!settings->out_path[0])
        join_path(settings->out_path, sizeof settings->out_path, settings->root, "last_segmap");
    if (!settings->lock_path[0])
        join_path(settings->lock_path, sizeof settings->lock_path, settings->root, ".v1appd.lock");
    dir_name(settings->map_dir, sizeof settings->map_dir, settings->map_path);
}

/* ===================================================================================== *
 *  producer half                                                                         *
 * ===================================================================================== */
static pid_t g_producer_pid = -1;          /* the transient v1seg child (or -1) */
static struct stat g_producer_map_stat;    /* map revision captured at launch */
static int  g_producer_have_stat = 0;
static long long g_producer_deadline_ms = 0;

static struct stat g_last_produced_map_stat;
static int g_have_last_stat = 0;

/* Producer failure backoff: a revision that produced nothing must not be retried
 * every loop.  We remember the last *attempted* revision and, after each failure,
 * suppress a retry of that same revision until an exponentially growing delay has
 * elapsed.  A *changed* map is never suppressed. */
static struct stat g_attempted_map_stat;
static int g_have_attempt_stat = 0;
static int g_producer_fail_streak = 0;
static long long g_producer_retry_after_ms = 0;

static int stat_path(const char *path, struct stat *st) { return stat(path, st) == 0; }

/* 1 if `older`'s modification time is strictly before `newer`'s (nanosecond aware). */
static int mtime_is_older(const struct stat *older, const struct stat *newer) {
    if (older->st_mtime != newer->st_mtime) return older->st_mtime < newer->st_mtime;
    return older->st_mtim.tv_nsec < newer->st_mtim.tv_nsec;
}

/* 1 if both stats describe the same map revision (mtime incl. nanoseconds, and size).
 * st_mtime alone is second-granular, so two saves inside one second would look equal. */
static int same_map_revision(const struct stat *left, const struct stat *right) {
    return left->st_mtime == right->st_mtime
        && left->st_mtim.tv_nsec == right->st_mtim.tv_nsec
        && left->st_size == right->st_size;
}

/* 1 if `out` is missing or older than the map. */
static int out_is_stale(const struct config *settings, const struct stat *map_stat) {
    struct stat out_stat;
    if (!stat_path(settings->out_path, &out_stat)) return 1;
    return mtime_is_older(&out_stat, map_stat);
}

/* Launch the producer (non-blocking).  Returns 0 once forked, -1 on failure. */
static int launch_producer(const struct config *settings, const struct stat *map_stat) {
    char min_area_arg[32];
    snprintf(min_area_arg, sizeof min_area_arg, "%ld", settings->min_area);

    /* argv points into memory the forked child owns (COW copy), so it stays valid
     * until execv() replaces the image; execv() does not search $PATH. */
    const char *producer_argv[] = {
        settings->producer_path, "--lastmap", settings->map_path, settings->out_path,
        min_area_arg, "--door-radius", "auto", NULL,
    };

    log_msg("[v1appd] producing %s from %s (min-area %s)\n",
            settings->out_path, settings->map_path, min_area_arg);
    pid_t child_pid = fork();
    if (child_pid < 0) {
        log_msg("[v1appd] fork failed: %s\n", strerror(errno));
        return -1;
    }
    if (child_pid == 0) {
        /* Policy: daemons AND the producer sit at 1000, ahead of nav/Valetudo at
         * stock.  The producer is the memory hog and is retried with backoff
         * (see note_producer_failure), so it is deliberately the first victim. */
        set_oom_score_adj(OOM_SCORE_ADJ_PRODUCER);
        if (settings->lib_dir[0]) setenv("LD_LIBRARY_PATH", settings->lib_dir, 1);
        execv(settings->producer_path, (char *const *)producer_argv);
        log_msg("[v1appd] exec %s failed: %s\n", settings->producer_path, strerror(errno));
        _exit(127);
    }
    g_producer_pid = child_pid;
    g_producer_map_stat = *map_stat;
    g_producer_have_stat = 1;
    g_producer_deadline_ms = monotonic_ms() + settings->producer_timeout_ms;
    return 0;
}

/* The producer succeeded: advance the produced revision and clear any backoff. */
static void note_producer_success(void) {
    if (g_producer_have_stat) {
        g_last_produced_map_stat = g_producer_map_stat;
        g_have_last_stat = 1;
    }
    g_producer_fail_streak = 0;
    g_producer_retry_after_ms = 0;
}

/* The producer failed or timed out: remember the revision it was launched for and
 * suppress an immediate retry of *that* revision with an exponentially growing
 * delay, so a persistently failing producer is retried a bounded number of times
 * rather than once every supervise/poll interval. */
static void note_producer_failure(void) {
    if (g_producer_have_stat) {
        g_attempted_map_stat = g_producer_map_stat;
        g_have_attempt_stat = 1;
    }
    g_producer_fail_streak++;
    long long delay = PRODUCER_RETRY_BASE_MS;
    for (int step = 1; step < g_producer_fail_streak && delay < PRODUCER_RETRY_MAX_MS; step++)
        delay *= 2;
    if (delay > PRODUCER_RETRY_MAX_MS) delay = PRODUCER_RETRY_MAX_MS;
    g_producer_retry_after_ms = monotonic_ms() + delay;
    log_msg("[v1appd] producer failed; retrying this map in %lldms (failure streak %d)\n",
            delay, g_producer_fail_streak);
}

/* Record the outcome of a finished producer and advance the produced revision. */
static void finish_producer(int child_status) {
    if (WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0) {
        log_msg("[v1appd] producer ok\n");
        note_producer_success();
    } else {
        log_msg("[v1appd] producer failed (status=%d)\n", child_status);
        note_producer_failure();
    }
    g_producer_pid = -1;
}

/* SIGTERM a process, wait `grace_ms`, then SIGKILL; reap it if it is our child. */
static void terminate_process(pid_t pid, int grace_ms) {
    if (pid <= 0) return;
    (void)kill(pid, SIGTERM);
    long long deadline = monotonic_ms() + grace_ms;
    for (;;) {
        int child_status;
        pid_t reaped = waitpid(pid, &child_status, WNOHANG);
        if (reaped == pid) return;
        if (reaped < 0 && errno == ECHILD) {
            if (kill(pid, 0) != 0 && errno == ESRCH) return;   /* not our child; gone */
        }
        if (monotonic_ms() >= deadline) break;
        sleep_ms(50);
    }
    (void)kill(pid, SIGKILL);
    for (int attempt = 0; attempt < 50; attempt++) {
        int child_status;
        pid_t reaped = waitpid(pid, &child_status, WNOHANG);
        if (reaped == pid) return;
        if (reaped < 0 && errno == ECHILD) {
            if (kill(pid, 0) != 0 && errno == ESRCH) return;
        }
        sleep_ms(20);
    }
}

static void reap_producer(void) {
    if (g_producer_pid < 0) return;
    int child_status;
    pid_t reaped = waitpid(g_producer_pid, &child_status, WNOHANG);
    if (reaped == g_producer_pid) finish_producer(child_status);
    else if (reaped < 0 && errno == ECHILD) g_producer_pid = -1;
}

static void check_producer_timeout(const struct config *settings) {
    if (g_producer_pid < 0) return;
    if (monotonic_ms() < g_producer_deadline_ms) return;
    log_msg("[v1appd] producer pid=%d exceeded %dms; killing it\n",
            (int)g_producer_pid, settings->producer_timeout_ms);
    terminate_process(g_producer_pid, PRODUCER_KILL_GRACE_MS);
    note_producer_failure();   /* treat a timeout like a failure: back off, do not spin */
    g_producer_pid = -1;
}

/* Wait for the producer to finish (blocking); used by --once.  EINTR aware. */
static void wait_producer_blocking(void) {
    if (g_producer_pid < 0) return;
    pid_t producer_pid = g_producer_pid;
    for (;;) {
        int child_status;
        pid_t reaped = waitpid(producer_pid, &child_status, 0);
        if (reaped == producer_pid) { finish_producer(child_status); return; }
        if (reaped < 0 && errno == EINTR) {
            if (g_stop) { terminate_process(producer_pid, 1000); g_producer_pid = -1; return; }
            continue;
        }
        g_producer_pid = -1;
        return;
    }
}

/* 1 when a retry of `map_stat`'s exact revision is being held back by the failure
 * backoff.  A changed revision (or --force) is never withheld. */
static int producer_retry_withheld(const struct config *settings, const struct stat *map_stat) {
    if (settings->force_produce || g_producer_retry_after_ms == 0) return 0;
    if (!g_have_attempt_stat || !same_map_revision(map_stat, &g_attempted_map_stat)) return 0;
    if (monotonic_ms() < g_producer_retry_after_ms) return 1;
    g_producer_retry_after_ms = 0;   /* backoff elapsed: allow exactly one retry */
    return 0;
}

/* Produce (if needed) and return non-zero when a producer was launched. */
static int produce_if_needed(struct config *settings) {
    if (g_producer_pid >= 0) {
        if (g_verbose) log_msg("[v1appd] producer already running; not launching another\n");
        return 0;
    }
    struct stat map_stat;
    if (!stat_path(settings->map_path, &map_stat)) {
        if (g_verbose) log_msg("[v1appd] no map yet (%s)\n", settings->map_path);
        return 0;
    }
    int changed = settings->force_produce
                || out_is_stale(settings, &map_stat)
                || (g_have_last_stat && !same_map_revision(&map_stat, &g_last_produced_map_stat));
    if (!changed) {
        if (g_verbose) log_msg("[v1appd] map unchanged; nothing to do\n");
        return 0;
    }
    if (producer_retry_withheld(settings, &map_stat)) {
        if (g_verbose) log_msg("[v1appd] producer retry for %s is backed off; waiting\n",
                               settings->map_path);
        return 0;
    }
    settings->force_produce = 0;   /* --force triggers exactly one produce */
    if (settings->dry_run) {
        log_msg("[v1appd] dry-run: would produce %s from %s\n",
                settings->out_path, settings->map_path);
        return 0;
    }
    return launch_producer(settings, &map_stat) == 0;
}

/* ===================================================================================== *
 *  AppProxy supervisor                                                                  *
 * ===================================================================================== */
static pid_t g_appproxy_pid = -1;          /* the AppProxy child we launched (or -1) */
static long long g_last_supervise_ms = 0;
static long long g_last_restart_ms = 0;
static long long g_restart_window_start_ms = 0;
static int g_restart_window_count = 0;
static int g_restart_given_up = 0;
static pid_t g_healthy_candidate_pid = -1; /* preloaded pid whose lifetime we are timing */
static long long g_healthy_candidate_since_ms = 0;

/* 1 when `/proc/<pid>/environ` has an `LD_PRELOAD=` entry naming `shim_basename`. */
static int pid_environ_has_preload(pid_t pid, const char *shim_basename) {
    char environ_path[64];
    snprintf(environ_path, sizeof environ_path, "/proc/%d/environ", (int)pid);
    int environ_fd = open(environ_path, O_RDONLY);
    if (environ_fd < 0) return 0;
    char *buffer = (char *)malloc(ENVIRON_READ_MAX);
    if (!buffer) { close(environ_fd); return 0; }
    size_t total = 0;
    for (;;) {
        ssize_t nread = read(environ_fd, buffer + total, ENVIRON_READ_MAX - 1 - total);
        if (nread <= 0) break;
        total += (size_t)nread;
        if (total >= ENVIRON_READ_MAX - 1) break;
    }
    close(environ_fd);
    buffer[total] = '\0';
    int found = 0;
    size_t offset = 0;
    while (offset < total) {
        const char *entry = buffer + offset;
        size_t entry_len = strnlen(entry, total - offset);
        if (strncmp(entry, "LD_PRELOAD=", 11) == 0) {
            const char *preload_value = entry + 11;
            if (shim_basename[0] && strstr(preload_value, shim_basename)) { found = 1; break; }
        }
        offset += entry_len + 1;
    }
    free(buffer);
    return found;
}

/* Scan /proc for AppProxy-named processes; classify by whether they carry our shim.
 * `stock_pids` collects processes that are NOT preloaded (up to max_stock).
 * Returns the first preloaded pid, or -1 when none is found. */
static pid_t find_appproxy_processes(const struct config *settings,
                                     pid_t *stock_pids, int max_stock, int *stock_count) {
    char wanted_basename[NAME_MAX + 1];
    char shim_basename[NAME_MAX + 1];
    base_name(wanted_basename, sizeof wanted_basename, settings->appproxy_path);
    base_name(shim_basename, sizeof shim_basename, settings->shim_path);
    *stock_count = 0;
    pid_t preloaded_pid = -1;

    DIR *proc_dir = opendir("/proc");
    if (!proc_dir) return -1;
    struct dirent *entry;
    while ((entry = readdir(proc_dir)) != NULL) {
        if (!is_all_digits(entry->d_name)) continue;
        pid_t candidate_pid = (pid_t)atoi(entry->d_name);
        if (candidate_pid <= 0) continue;
        char exe_link[64], exe_target[PATH_MAX];
        snprintf(exe_link, sizeof exe_link, "/proc/%d/exe", (int)candidate_pid);
        ssize_t target_len = readlink(exe_link, exe_target, sizeof exe_target - 1);
        if (target_len <= 0) continue;
        exe_target[target_len] = '\0';
        char exe_basename[NAME_MAX + 1];
        base_name(exe_basename, sizeof exe_basename, exe_target);
        if (strcmp(exe_basename, wanted_basename) != 0) continue;
        if (pid_environ_has_preload(candidate_pid, shim_basename)) {
            if (preloaded_pid < 0) preloaded_pid = candidate_pid;
        } else if (*stock_count < max_stock) {
            stock_pids[(*stock_count)++] = candidate_pid;
        }
    }
    closedir(proc_dir);
    return preloaded_pid;
}

/* Runtime context captured from a stock AppProxy before we replace it.  The real
 * AppProxy reads RR_ROOT/RR_UDATA/RR_RESERVE and libNavDriver reads
 * NAV_CONFIG_PATH/NAV_DYNAMIC_CONFIG_PATH, and the watchdog chdirs to $RR_ROOT, so
 * the replacement must inherit the stock process's environment and cwd -- not just
 * receive LD_PRELOAD. */
struct appproxy_inherit {
    char  *environ;      /* malloc'd NUL-separated env block (or NULL) */
    size_t environ_len;  /* size of the block in bytes */
    char   cwd[PATH_MAX];/* captured working directory (when have_cwd) */
    int    have_cwd;
};

/* Capture `pid`'s environment (bounded by ENVIRON_READ_MAX) and cwd.  Returns 0 when
 * at least one was captured; -1 (leaving the struct zeroed) so the caller inherits. */
static int capture_process_context(pid_t pid, struct appproxy_inherit *inherit) {
    memset(inherit, 0, sizeof *inherit);

    char environ_path[64];
    snprintf(environ_path, sizeof environ_path, "/proc/%d/environ", (int)pid);
    int environ_fd = open(environ_path, O_RDONLY);
    if (environ_fd >= 0) {
        char *buffer = (char *)malloc(ENVIRON_READ_MAX);
        if (buffer) {
            size_t total = 0;
            for (;;) {
                ssize_t nread = read(environ_fd, buffer + total, ENVIRON_READ_MAX - 1 - total);
                if (nread <= 0) break;
                total += (size_t)nread;
                if (total >= ENVIRON_READ_MAX - 1) break;
            }
            if (total > 0) {
                buffer[total] = '\0';
                inherit->environ = buffer;
                inherit->environ_len = total;
            } else {
                free(buffer);
            }
        }
        close(environ_fd);
    }

    char cwd_link[64], cwd_target[PATH_MAX];
    snprintf(cwd_link, sizeof cwd_link, "/proc/%d/cwd", (int)pid);
    ssize_t cwd_len = readlink(cwd_link, cwd_target, sizeof cwd_target - 1);
    if (cwd_len > 0) {
        cwd_target[cwd_len] = '\0';
        if (cwd_target[0] == '/') {   /* a deleted cwd reads back with a " (deleted)" suffix */
            snprintf(inherit->cwd, sizeof inherit->cwd, "%s", cwd_target);
            inherit->have_cwd = 1;
        }
    }
    return (inherit->environ || inherit->have_cwd) ? 0 : -1;
}

/* Re-apply every well-formed KEY=VALUE from a captured environ block (skip malformed
 * entries).  LD_PRELOAD is deliberately set by the caller afterwards so it wins. */
static void apply_inherited_environ(const struct appproxy_inherit *inherit) {
    if (!inherit || !inherit->environ) return;
    size_t offset = 0;
    while (offset < inherit->environ_len) {
        const char *entry = inherit->environ + offset;
        size_t entry_len = strnlen(entry, inherit->environ_len - offset);
        const char *equals = (const char *)memchr(entry, '=', entry_len);
        if (equals && equals != entry) {
            char key[NAME_MAX + 1];
            size_t key_len = (size_t)(equals - entry);
            if (key_len < sizeof key) {
                memcpy(key, entry, key_len);
                key[key_len] = '\0';
                setenv(key, equals + 1, 1);
            }
        }
        offset += entry_len + 1;
    }
}

/* Relaunch AppProxy under our shim, detached, with stdout/stderr on the rrlog file.
 * `inherit` (may be NULL) carries a replaced stock instance's env/cwd. */
static pid_t launch_appproxy(const struct config *settings,
                             const struct appproxy_inherit *inherit) {
    char log_dir[PATH_MAX], log_path[PATH_MAX];
    join_path(log_dir, sizeof log_dir, settings->root, "rrlog");
    if (mkdir(log_dir, 0755) != 0 && errno != EEXIST)
        log_msg("[v1appd] cannot create %s: %s\n", log_dir, strerror(errno));
    join_path(log_path, sizeof log_path, log_dir, "AppProxy.log");
    int appproxy_log_fd = open(log_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
    if (appproxy_log_fd < 0)
        log_msg("[v1appd] cannot open %s: %s\n", log_path, strerror(errno));

    pid_t child_pid = fork();
    if (child_pid < 0) {
        log_msg("[v1appd] fork for AppProxy failed: %s\n", strerror(errno));
        if (appproxy_log_fd >= 0) close(appproxy_log_fd);
        return -1;
    }
    if (child_pid == 0) {
        /* Policy: the shimmed bridge must NEVER be the OOM victim, so drop the
         * inherited daemon bias back to stock priority. */
        set_oom_score_adj(OOM_SCORE_ADJ_BRIDGE);
        setsid();
        if (inherit && inherit->have_cwd && chdir(inherit->cwd) != 0)
            log_msg("[v1appd] chdir %s failed: %s\n", inherit->cwd, strerror(errno));
        apply_inherited_environ(inherit);
        setenv("LD_PRELOAD", settings->shim_path, 1);   /* last so ours wins */
        int dev_null = open("/dev/null", O_RDONLY);
        if (dev_null >= 0) {
            dup2(dev_null, STDIN_FILENO);
            if (dev_null > STDERR_FILENO) close(dev_null);
        }
        if (appproxy_log_fd >= 0) {
            dup2(appproxy_log_fd, STDOUT_FILENO);
            dup2(appproxy_log_fd, STDERR_FILENO);
            if (appproxy_log_fd > STDERR_FILENO) close(appproxy_log_fd);
        }
        execl(settings->appproxy_path, settings->appproxy_path, (char *)NULL);
        log_msg("[v1appd] exec %s failed: %s\n", settings->appproxy_path, strerror(errno));
        _exit(127);
    }
    if (appproxy_log_fd >= 0) close(appproxy_log_fd);
    return child_pid;
}

static void reap_appproxy(void) {
    if (g_appproxy_pid < 0) return;
    int child_status;
    pid_t reaped = waitpid(g_appproxy_pid, &child_status, WNOHANG);
    if (reaped == g_appproxy_pid || (reaped < 0 && errno == ECHILD)) {
        if (reaped == g_appproxy_pid && g_verbose)
            log_msg("[v1appd] supervised AppProxy pid=%d exited (status=%d)\n",
                    (int)g_appproxy_pid, child_status);
        g_appproxy_pid = -1;
    }
}

/* Reset the restart budget once a healthy (preloaded) AppProxy is observed. */
static void reset_restart_budget(void) {
    g_restart_given_up = 0;
    g_restart_window_count = 0;
    g_restart_window_start_ms = 0;
    g_last_restart_ms = 0;
}

/* (Re)launch AppProxy, subject to the minimum-interval + per-window cap policy. */
static void maybe_launch_appproxy(struct config *settings,
                                  const struct appproxy_inherit *inherit) {
    long long now = monotonic_ms();
    if (g_restart_given_up) return;
    if (g_restart_window_start_ms == 0) g_restart_window_start_ms = now;
    if (now - g_restart_window_start_ms >= settings->restart_window_ms) {
        g_restart_window_count = 0;
        g_restart_window_start_ms = now;
    }
    if (g_restart_window_count >= settings->restart_max) {
        g_restart_given_up = 1;
        log_msg("[v1appd] AppProxy restart cap reached (%d in %dms); NOT restarting it again "
                "(fix the cause and restart v1appd)\n",
                settings->restart_max, settings->restart_window_ms);
        return;
    }
    if (g_last_restart_ms != 0 && now - g_last_restart_ms < settings->restart_min_ms) return;
    if (settings->dry_run) {
        log_msg("[v1appd] dry-run: would launch AppProxy %s with LD_PRELOAD=%s\n",
                settings->appproxy_path, settings->shim_path);
        return;
    }
    pid_t child_pid = launch_appproxy(settings, inherit);
    if (child_pid > 0) {
        g_appproxy_pid = child_pid;
        g_restart_window_count++;
        g_last_restart_ms = now;
        log_msg("[v1appd] launched AppProxy pid=%d (restart %d/%d in window)\n",
                (int)child_pid, g_restart_window_count, settings->restart_max);
    }
}

/* One supervision pass: leave ours alone, replace stock, relaunch when absent. */
static void supervise_appproxy(struct config *settings) {
    long long now = monotonic_ms();
    if (g_last_supervise_ms != 0 && now - g_last_supervise_ms < settings->supervise_ms) return;
    g_last_supervise_ms = now;

    pid_t stock_pids[MAX_TRACKED_APPROXY];
    int stock_count = 0;
    pid_t preloaded_pid = find_appproxy_processes(settings, stock_pids, MAX_TRACKED_APPROXY, &stock_count);

    if (preloaded_pid > 0) {
        /* Only forgive the restart budget once the SAME preloaded pid has stayed up
         * long enough to count as healthy.  The instance we just launched carries
         * LD_PRELOAD too, so a naive reset here would reset the budget on every pass
         * and a shimmed AppProxy that dies after a few seconds could be restarted
         * forever (and could keep killing a stock instance WatchDoge started). */
        if (preloaded_pid != g_healthy_candidate_pid) {
            g_healthy_candidate_pid = preloaded_pid;
            g_healthy_candidate_since_ms = now;
        } else if (now - g_healthy_candidate_since_ms >= APPROXY_HEALTHY_MIN_MS) {
            reset_restart_budget();
            g_healthy_candidate_since_ms = now;   /* forgive at most once per healthy period */
        }
        if (g_verbose)
            log_msg("[v1appd] AppProxy pid=%d already preloaded; leaving it alone\n",
                    (int)preloaded_pid);
        /* A boot-time race can leave a stock AppProxy beside ours (two bridges): the
         * stock instances are duplicates, so terminate them -- but keep ours and do
         * not relaunch.  find_appproxy_processes never puts a preloaded pid in
         * stock_pids, so a preloaded instance is never killed here. */
        for (int index = 0; index < stock_count; index++) {
            if (settings->dry_run) {
                log_msg("[v1appd] dry-run: would terminate duplicate stock AppProxy pid=%d\n",
                        (int)stock_pids[index]);
            } else {
                log_msg("[v1appd] duplicate stock AppProxy pid=%d (preloaded pid=%d present); "
                        "terminating it\n", (int)stock_pids[index], (int)preloaded_pid);
                terminate_process(stock_pids[index], APPROXY_KILL_GRACE_MS);
            }
        }
        return;
    }
    /* Once the restart budget is spent, stop interfering with AppProxy entirely:
     * do not kill a stock process we would not be allowed to replace. */
    if (g_restart_given_up) return;
    struct appproxy_inherit inherit;
    memset(&inherit, 0, sizeof inherit);
    int have_inherit = 0;
    if (stock_count > 0) {
        if (settings->dry_run) {
            for (int index = 0; index < stock_count; index++)
                log_msg("[v1appd] dry-run: would terminate stock AppProxy pid=%d\n",
                        (int)stock_pids[index]);
            log_msg("[v1appd] dry-run: would relaunch AppProxy %s under LD_PRELOAD=%s\n",
                    settings->appproxy_path, settings->shim_path);
            return;
        }
        /* Capture the stock instance's env/cwd BEFORE terminating it so the shimmed
         * replacement runs with the same runtime variables and working directory. */
        have_inherit = capture_process_context(stock_pids[0], &inherit) == 0;
        for (int index = 0; index < stock_count; index++) {
            log_msg("[v1appd] stock AppProxy pid=%d (no %s in its env); terminating it\n",
                    (int)stock_pids[index], settings->shim_path);
            terminate_process(stock_pids[index], APPROXY_KILL_GRACE_MS);
        }
    }
    maybe_launch_appproxy(settings, have_inherit ? &inherit : NULL);
    free(inherit.environ);
}

/* Startup gate: never race the watchdog into two AppProxy processes.  WatchDoge
 * starts the stock AppProxy itself (matched by name), so if we launched ours first
 * the two could coexist and its `killall -9 AppProxy` could take out both.  When no
 * AppProxy is running yet, wait (poll STARTUP_GATE_POLL_MS) up to --initial-wait-ms
 * for one to appear -- i.e. let the watchdog start the initial stock instance -- and
 * only then let the caller act on it (supervise: capture its env/cwd, replace it;
 * off-mode: adopt/replace/warn once).  The gate never launches anything itself so
 * both modes make exactly one launch decision afterwards.  --initial-wait-ms 0
 * disables the gate (act immediately). */
static void startup_gate(struct config *settings) {
    if (settings->initial_wait_ms <= 0) return;

    pid_t stock_pids[MAX_TRACKED_APPROXY];
    int stock_count = 0;
    pid_t preloaded_pid =
        find_appproxy_processes(settings, stock_pids, MAX_TRACKED_APPROXY, &stock_count);
    if (preloaded_pid > 0 || stock_count > 0) {
        if (g_verbose)
            log_msg("[v1appd] startup gate: AppProxy already present (preloaded=%d stock=%d); "
                    "not waiting\n", (int)preloaded_pid, stock_count);
        return;
    }

    log_msg("[v1appd] startup gate: no AppProxy yet; waiting up to %dms for the watchdog\n",
            settings->initial_wait_ms);
    long long deadline_ms = monotonic_ms() + settings->initial_wait_ms;
    while (!g_stop && monotonic_ms() < deadline_ms) {
        sleep_ms(STARTUP_GATE_POLL_MS);
        stock_count = 0;
        preloaded_pid =
            find_appproxy_processes(settings, stock_pids, MAX_TRACKED_APPROXY, &stock_count);
        if (preloaded_pid > 0 || stock_count > 0) {
            log_msg("[v1appd] startup gate: AppProxy appeared (preloaded=%d stock=%d)\n",
                    (int)preloaded_pid, stock_count);
            return;
        }
    }
    if (g_stop) return;
    log_msg("[v1appd] startup gate: timed out after %dms with no AppProxy\n",
            settings->initial_wait_ms);
}

/* Off-mode (supervision DISABLED) one-shot AppProxy decision, made once after the
 * startup gate.  The default install must never replace an AppProxy the operator did
 * not opt into, so this never terminates anything:
 *   - a preloaded (shimmed) instance is adopted as-is;
 *   - when none is running we launch our own preloaded one;
 *   - a stock (un-shimmed) instance is left running with a LOUD warning that segment
 *     injection is inactive and a reboot is required.
 * After this single pass resident_loop does producer duty only. */
static void one_shot_appproxy(struct config *settings) {
    pid_t stock_pids[MAX_TRACKED_APPROXY];
    int stock_count = 0;
    pid_t preloaded_pid =
        find_appproxy_processes(settings, stock_pids, MAX_TRACKED_APPROXY, &stock_count);

    if (preloaded_pid > 0) {
        log_msg("[v1appd] AppProxy supervision off: adopting preloaded AppProxy pid=%d\n",
                (int)preloaded_pid);
        return;
    }
    if (stock_count > 0) {
        log_msg("[v1appd] WARNING: a stock AppProxy is running (pid=%d) and the shim is NOT "
                "active; segment injection is unavailable. Reboot the robot to start the "
                "shim-preloaded AppProxy. "
                "NOT killing the running AppProxy (supervision is off)\n",
                (int)stock_pids[0]);
        return;
    }
    log_msg("[v1appd] AppProxy supervision off: no AppProxy running; launching ours (preloaded)\n");
    /* No watchdog instance to inherit from: use v1appd's own (job-mirrored) environment. */
    maybe_launch_appproxy(settings, NULL);
}

/* ===================================================================================== *
 *  map watch (inotify)                                                                  *
 * ===================================================================================== */
static int g_inotify_fd = -1;
static int g_map_watch = -1;

struct event_scan {
    int relevant;    /* events naming the map file */
    int overflow;    /* the inotify queue overflowed -> caller must rescan */
    int watch_gone;  /* the watch is no longer valid (IN_IGNORED / _SELF) -> re-arm */
};

/* Drain every queued inotify event, classifying map-file hits, queue overflow and a lost
 * watch.  Our buffer is at least one inotify_event + NAME_MAX, which 4096 satisfies. */
static struct event_scan scan_inotify_events(int inotify_fd, const char *map_path) {
    struct event_scan scan = { 0, 0, 0 };
    char event_buffer[4096];
    const char *map_basename = strrchr(map_path, '/');
    map_basename = map_basename ? map_basename + 1 : map_path;

    for (;;) {
        ssize_t bytes_read = read(inotify_fd, event_buffer, sizeof event_buffer);
        if (bytes_read <= 0) break;
        for (ssize_t cursor = 0;
             cursor + (ssize_t)sizeof(struct inotify_event) <= bytes_read; ) {
            const struct inotify_event *event =
                (const struct inotify_event *)(event_buffer + cursor);
            cursor += (ssize_t)sizeof(struct inotify_event) + event->len;
            if (event->mask & IN_Q_OVERFLOW) { scan.overflow = 1; continue; }
            if (event->mask & (IN_IGNORED | IN_DELETE_SELF | IN_MOVE_SELF)) {
                scan.watch_gone = 1; continue;
            }
            if (event->len == 0) continue;
            if (strcmp(event->name, map_basename) == 0) scan.relevant++;
        }
    }
    return scan;
}

static void setup_map_watch(const struct config *settings) {
    g_inotify_fd = inotify_init1(IN_NONBLOCK | IN_CLOEXEC);
    if (g_inotify_fd < 0) {
        log_msg("[v1appd] inotify_init1 failed: %s; poll fallback\n", strerror(errno));
        return;
    }
    g_map_watch = inotify_add_watch(g_inotify_fd, settings->map_dir, MAP_WATCH_MASK);
    if (g_map_watch < 0)
        log_msg("[v1appd] inotify_add_watch(%s) failed: %s; poll fallback\n",
                settings->map_dir, strerror(errno));
    else
        log_msg("[v1appd] watching %s\n", settings->map_dir);
}

static void rearm_map_watch(const struct config *settings) {
    if (g_inotify_fd < 0 || g_map_watch >= 0) return;
    g_map_watch = inotify_add_watch(g_inotify_fd, settings->map_dir, MAP_WATCH_MASK);
    if (g_map_watch >= 0) log_msg("[v1appd] re-armed watch on %s\n", settings->map_dir);
}

/* Wait up to `timeout_ms` for a relevant map event.  Returns 1 when the caller should settle. */
static int wait_for_map_events(const struct config *settings, int timeout_ms) {
    if (g_inotify_fd < 0 || g_map_watch < 0) { sleep_ms(timeout_ms); return 0; }
    struct pollfd watch_pollfd = { g_inotify_fd, POLLIN, 0 };
    int poll_rc = poll(&watch_pollfd, 1, timeout_ms);
    if (poll_rc <= 0) return 0;
    if (!(watch_pollfd.revents & POLLIN)) return 0;
    struct event_scan scan = scan_inotify_events(g_inotify_fd, settings->map_path);
    int map_event = scan.relevant > 0 || scan.overflow;
    if (scan.watch_gone) { g_map_watch = -1; map_event = 1; }
    return map_event;
}

/* After a relevant event, wait `settle_ms` of quiet (the nav writes the map in
 * chunks), then absorb trailing events so a burst coalesces into one produce. */
static void settle_and_drain(const struct config *settings) {
    sleep_ms(settings->settle_ms);
    while (!g_stop) {
        struct event_scan more = scan_inotify_events(g_inotify_fd, settings->map_path);
        if (more.watch_gone) { g_map_watch = -1; break; }
        if (more.relevant <= 0 && !more.overflow) break;
        sleep_ms(50);
    }
}

/* ===================================================================================== *
 *  loops                                                                                *
 * ===================================================================================== */
static int resident_loop(struct config *settings) {
    setup_map_watch(settings);
    if (!settings->producer_only) {
        startup_gate(settings);
        /* Off-mode makes exactly one AppProxy decision, then does producer duty only;
         * the resident keep-alive loop below is entered only with --supervise-appproxy. */
        if (!settings->supervise_appproxy) one_shot_appproxy(settings);
    }

    while (!g_stop) {
        rearm_map_watch(settings);
        reap_producer();
        reap_appproxy();
        /* --producer-only reliably means "never touch AppProxy", so it also wins over
         * --supervise-appproxy: the resident supervisor is entered only for a genuine
         * supervision run. */
        if (!settings->producer_only && settings->supervise_appproxy) supervise_appproxy(settings);
        check_producer_timeout(settings);
        produce_if_needed(settings);
        if (g_stop) break;

        /* Wake at least every supervise interval so supervision stays responsive.  With
         * supervision off we wake only on the map poll (never busy-spin). */
        int wait_ms = settings->poll_ms;
        if (settings->supervise_appproxy && (wait_ms <= 0 || settings->supervise_ms < wait_ms))
            wait_ms = settings->supervise_ms;
        if (wait_ms <= 0) wait_ms = 50;   /* never busy-spin */
        if (wait_for_map_events(settings, wait_ms)) settle_and_drain(settings);
    }

    log_msg("[v1appd] stopping\n");
    if (g_producer_pid >= 0) terminate_process(g_producer_pid, PRODUCER_KILL_GRACE_MS);
    /* The supervised AppProxy is deliberately NOT signalled: it is setsid()-detached and
     * must outlive v1appd so the shim-preloaded bridge keeps serving the robot while
     * v1appd is down (an upstart respawn, or a manual stop) -- the "headless-safe"
     * contract in TECHNICAL.md sections 1 and 5.  A restarted v1appd finds it already
     * preloaded and adopts it; the uninstall path removes it explicitly. */
    if (g_appproxy_pid >= 0)
        log_msg("[v1appd] leaving supervised AppProxy pid=%d running (headless-safe)\n",
                (int)g_appproxy_pid);
    if (g_inotify_fd >= 0 && g_map_watch >= 0) inotify_rm_watch(g_inotify_fd, g_map_watch);
    if (g_inotify_fd >= 0) close(g_inotify_fd);
    return 0;
}

static int once_loop(struct config *settings) {
    produce_if_needed(settings);
    wait_producer_blocking();
    return 0;
}

/* ===================================================================================== *
 *  CLI                                                                                  *
 * ===================================================================================== */
static void usage(void) {
    log_msg("usage: v1appd [options]\n"
            "  --root DIR              V1 data dir (default /mnt/data/rockrobo)\n"
            "  --map FILE              SLAM map to watch (default <root>/last_map)\n"
            "  --out FILE              sidecar to write   (default <root>/last_segmap)\n"
            "  --producer PATH         producer binary    (default /opt/rockrobo/cleaner/bin/v1seg)\n"
            "  --libdir DIR            LD_LIBRARY_PATH for the producer (default /opt/rockrobo/cleaner/lib)\n"
            "  --shim PATH             LD_PRELOAD shim for AppProxy (default .../rrtest/libv1seg_addon.so)\n"
            "  --appproxy PATH         AppProxy to supervise (default /opt/rockrobo/cleaner/bin/AppProxy)\n"
            "  --min-area N            producer min-area  (default 500)\n"
            "  --settle-ms N           map-write settle window (default 500)\n"
            "  --poll-ms N             inotify safety poll (default 5000)\n"
            "  --supervise-ms N        AppProxy supervision interval (only with --supervise-appproxy;\n"
            "                          default 1000)\n"
            "  --producer-timeout-ms N kill a producer older than N ms (default 120000)\n"
            "  --initial-wait-ms N     at startup wait up to N ms for the watchdog's AppProxy\n"
            "                          before adopting or launching; 0 disables (default 30000)\n"
            "  --restart-min-ms N      minimum ms between AppProxy restarts (default 2000)\n"
            "  --restart-max N         max AppProxy restarts per window (default 5)\n"
            "  --restart-window-ms N   restart budget window (default 120000)\n"
            "  --log FILE              append diagnostics to FILE (default stderr)\n"
            "  --once                  produce (if needed) and exit\n"
            "  --force                 ignore the change check for exactly one produce\n"
            "  --producer-only         do NOT supervise AppProxy (producer half only)\n"
            "  --supervise-appproxy    keep AppProxy supervision ON: keep-alive, replace stock,\n"
            "                          prune duplicates, restart budget (DEFAULT OFF: run the\n"
            "                          startup gate once, then adopt/launch/warn and produce only)\n"
            "  --dry-run               log what would be done; never signal/exec\n"
            "  --verbose               log the quiet decisions too\n"
            "  -h, --help              this text\n");
}

int main(int argc, char **argv) {
    struct config settings;
    config_defaults(&settings);

    for (int i = 1; i < argc; i++) {
        const char *option = argv[i];
#define NEXT_VALUE() (i + 1 < argc ? argv[++i] : NULL)
#define TAKE_STR(field) do { const char *value = NEXT_VALUE(); if (value) snprintf((field), sizeof(field), "%s", value); } while (0)
#define TAKE_INT(field) do { const char *value = NEXT_VALUE(); if (value) (field) = atoi(value); } while (0)
        if (!strcmp(option, "--root"))               { TAKE_STR(settings.root); }
        else if (!strcmp(option, "--map"))           { TAKE_STR(settings.map_path); }
        else if (!strcmp(option, "--out"))           { TAKE_STR(settings.out_path); }
        else if (!strcmp(option, "--producer"))      { TAKE_STR(settings.producer_path); }
        else if (!strcmp(option, "--libdir"))        { TAKE_STR(settings.lib_dir); }
        else if (!strcmp(option, "--shim"))          { TAKE_STR(settings.shim_path); }
        else if (!strcmp(option, "--appproxy"))      { TAKE_STR(settings.appproxy_path); }
        else if (!strcmp(option, "--log"))           { TAKE_STR(settings.log_path); }
        else if (!strcmp(option, "--min-area"))      { TAKE_INT(settings.min_area); }
        else if (!strcmp(option, "--settle-ms"))     { TAKE_INT(settings.settle_ms); if (settings.settle_ms < 0) settings.settle_ms = 0; }
        else if (!strcmp(option, "--poll-ms"))       { TAKE_INT(settings.poll_ms); if (settings.poll_ms < 0) settings.poll_ms = 0; }
        else if (!strcmp(option, "--supervise-ms"))  { TAKE_INT(settings.supervise_ms); if (settings.supervise_ms < 0) settings.supervise_ms = 0; }
        else if (!strcmp(option, "--producer-timeout-ms")) { TAKE_INT(settings.producer_timeout_ms); if (settings.producer_timeout_ms <= 0) settings.producer_timeout_ms = 120000; }
        else if (!strcmp(option, "--initial-wait-ms"))   { TAKE_INT(settings.initial_wait_ms); if (settings.initial_wait_ms < 0) settings.initial_wait_ms = 0; }
        else if (!strcmp(option, "--restart-min-ms"))    { TAKE_INT(settings.restart_min_ms); if (settings.restart_min_ms < 0) settings.restart_min_ms = 0; }
        else if (!strcmp(option, "--restart-max"))       { TAKE_INT(settings.restart_max); if (settings.restart_max < 1) settings.restart_max = 1; }
        else if (!strcmp(option, "--restart-window-ms")) { TAKE_INT(settings.restart_window_ms); if (settings.restart_window_ms < 1) settings.restart_window_ms = 1; }
        else if (!strcmp(option, "--once"))          { settings.run_once = 1; }
        else if (!strcmp(option, "--force"))         { settings.force_produce = 1; }
        else if (!strcmp(option, "--producer-only")) { settings.producer_only = 1; }
        else if (!strcmp(option, "--supervise-appproxy")) { settings.supervise_appproxy = 1; }
        else if (!strcmp(option, "--dry-run"))       { settings.dry_run = 1; }
        else if (!strcmp(option, "--verbose"))       { g_verbose = 1; }
        else if (!strcmp(option, "-h") || !strcmp(option, "--help")) { usage(); return 0; }
        else {
            log_msg("[v1appd] unknown option %s\n", option);
            return 2;
        }
#undef TAKE_STR
#undef TAKE_INT
#undef NEXT_VALUE
    }
    config_paths(&settings);

    if (settings.log_path[0]) {
        int log_fd = open(settings.log_path, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0644);
        if (log_fd >= 0) g_log_fd = log_fd;
        else log_msg("[v1appd] cannot open log %s: %s\n", settings.log_path, strerror(errno));
    }

    /* singleton */
    int lock_fd = open(settings.lock_path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (lock_fd < 0) {
        log_msg("[v1appd] cannot open %s: %s\n", settings.lock_path, strerror(errno));
        return 1;
    }
    if (flock(lock_fd, LOCK_EX | LOCK_NB) != 0) {
        log_msg("[v1appd] another instance holds %s; exiting\n", settings.lock_path);
        close(lock_fd);
        return 0;
    }

    struct sigaction stop_action;
    memset(&stop_action, 0, sizeof stop_action);
    stop_action.sa_handler = handle_stop_signal;
    sigaction(SIGTERM, &stop_action, NULL);
    sigaction(SIGINT, &stop_action, NULL);
    signal(SIGPIPE, SIG_IGN);

    log_msg("[v1appd] start root=%s map=%s out=%s producer=%s min-area=%ld settle=%dms "
            "poll=%dms supervise=%dms initial-wait=%dms appproxy=%s shim=%s producer-only=%d "
            "supervise-appproxy=%d dry-run=%d\n",
            settings.root, settings.map_path, settings.out_path, settings.producer_path,
            settings.min_area, settings.settle_ms, settings.poll_ms, settings.supervise_ms,
            settings.initial_wait_ms, settings.appproxy_path, settings.shim_path,
            settings.producer_only, settings.supervise_appproxy, settings.dry_run);

    if (settings.run_once) once_loop(&settings);
    else                  resident_loop(&settings);

    flock(lock_fd, LOCK_UN);
    close(lock_fd);
    if (g_log_fd != STDERR_FILENO) close(g_log_fd);
    return 0;
}
