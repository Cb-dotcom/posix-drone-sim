// Targets process.
// Generates targets over time and sends them periodically to bb_server
// via an anonymous pipe. bb_server stores them in WorldState and uses them
// for drawing / scoring / scoring.
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <string.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_params.h"
#include "sim_log.h"
#include "sim_const.h"

volatile sig_atomic_t g_current_code_area = 0;

// Flag set by the SIGINT handler to request a clean shutdown
static volatile sig_atomic_t running = 1;
static pid_t g_wd_pid = -1;

static void wd_client_ping_handler(int sig, siginfo_t *info, void *ctx)
{
    (void)sig; (void)ctx;
    if (g_wd_pid > 1) {
        union sigval sv;
        sv.sival_int = (int)g_current_code_area;
        sigqueue(g_wd_pid, SIGUSR2, sv);
    }
}
static void wd_client_init(void)
{
    const char *s = getenv("SIM_WD_PID");
    if (!s) return;

    g_wd_pid = (pid_t)strtol(s, NULL, 10);
    if (g_wd_pid <= 1) return;

    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = wd_client_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_RESTART;

    // Watchdog pings us with SIGUSR1
    (void)sigaction(SIGUSR1, &sa, NULL);
}


static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// keep target generation in one place
static void generate_random_target(Target *t, const SimParams *params, double radius, int id)
{
    double margin = radius;

    double x_range = (double)params->world_width  - 2.0 * margin;
    double y_range = (double)params->world_height - 2.0 * margin;
    if (x_range < 0.0) x_range = 0.0;
    if (y_range < 0.0) y_range = 0.0;

    double rx = (double)rand() / (double)RAND_MAX;
    double ry = (double)rand() / (double)RAND_MAX;

    t->x      = margin + rx * x_range;
    t->y      = margin + ry * y_range;
    t->radius = radius;
    t->id     = id;
    t->active = 1;
    clock_gettime(CLOCK_REALTIME, &t->time_created);
}

int main(int argc, char *argv[])
{
    sim_log_init("targets");
    sim_process_register("targets", getpid());

    signal(SIGINT, handle_sigint);
    wd_client_init();


    if (argc < 2) {
        sim_log_info("targets: usage error: expected fd_targets_out argument");
        return EXIT_FAILURE;
    }

    int fd_tgt_out = atoi(argv[SIM_ARG_TGT_OUT]);

    // Load runtime parameters for targets
    if (sim_params_load(NULL) != 0) {
        sim_log_info("targets: warning: could not load '%s', using built-in defaults",
                     SIM_PARAMS_DEFAULT_PATH);
    }

    const SimParams *params = sim_params_get();

    // IMPORTANT:
    // In this codebase the hard cap is stored in params->num_targets.
    // Your config key "max_targets" is parsed into num_targets in sim_params.c.
    int max_targets = params->num_targets;
    if (max_targets < 0) max_targets = 0;
    if (max_targets > SIM_MAX_TARGETS) max_targets = SIM_MAX_TARGETS;

    // how many we start with
    int active_count = params->initial_targets;
    if (active_count < 0) active_count = 0;
    if (active_count > max_targets) active_count = max_targets;

    sim_log_info("targets: started (world=%dx%d, initial=%d, max=%d, spawn_interval=%.2f)",
                 params->world_width,
                 params->world_height,
                 active_count,
                 max_targets,
                 params->target_spawn_interval);

    // If we have no capacity at all, just exit quietly
    if (max_targets == 0) {
        sim_log_info("targets: max_targets <= 0, nothing to do");
        close(fd_tgt_out);
        sim_log_info("targets: exiting (no capacity)");
        return EXIT_SUCCESS;
    }

    Target targets[SIM_MAX_TARGETS];

    // Seed RNG with time and PID to avoid identical maps across runs
    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    const double radius = 1.0;
    int next_id = 1;

    // Initialize the active targets
    for (int i = 0; i < active_count; ++i) {
        generate_random_target(&targets[i], params, radius, next_id++);
    }

    // Mark unused slots as inactive (only up to max_targets; rest ignored)
    for (int i = active_count; i < max_targets; ++i) {
        targets[i].x      = 0.0;
        targets[i].y      = 0.0;
        targets[i].radius = 0.0;
        targets[i].id     = 0;
        targets[i].active = 0;
        targets[i].time_created.tv_sec  = 0;
        targets[i].time_created.tv_nsec = 0;
    }

    ssize_t expected = (ssize_t)(max_targets * (int)sizeof(Target));

    // Send initial snapshot to bb_server
    ssize_t w = write_full(fd_tgt_out, targets,
                           (size_t)(max_targets * (int)sizeof(Target)));
    if (w != expected) {
        sim_log_info("targets: write_full(fd_tgt_out) failed, returned %zd (expected %zd)",
                     w, expected);
        close(fd_tgt_out);
        sim_log_info("targets: exiting (initial write failed)");
        return EXIT_FAILURE;
    }
    sim_log_info("targets: sent initial %d/%d targets to bb_server",
                 active_count, max_targets);

    // Sleep interval
    double interval = params->target_spawn_interval;
    if (interval <= 0.0) {
        interval = SIM_DEFAULT_TARGET_SPAWN_INTERVAL;
    }

    struct timespec sleep_ts;
    sleep_ts.tv_sec  = (time_t)interval;
    sleep_ts.tv_nsec = (long)((interval - (double)sleep_ts.tv_sec) * 1e9);
    if (sleep_ts.tv_nsec < 0) sleep_ts.tv_nsec = 0;

    int oldest_index = 0;

    while (running) {
        g_current_code_area = CODE_AREA_MAIN_LOOP;
        
        g_current_code_area = CODE_AREA_READ_PIPE;
        nanosleep(&sleep_ts, NULL);
        if (!running) break;

        g_current_code_area = CODE_AREA_PHYSICS_UPDATE;

        int idx;
        if (active_count < max_targets) {
            idx = active_count;
            active_count++;
        } else {
            idx = oldest_index;
            oldest_index = (oldest_index + 1) % max_targets;
        }

        generate_random_target(&targets[idx], params, radius, next_id++);

        g_current_code_area = CODE_AREA_WRITE_PIPE;

        w = write_full(fd_tgt_out, targets,
                       (size_t)(max_targets * (int)sizeof(Target)));
        if (w != expected) {
            sim_log_info("targets: write_full(fd_tgt_out) failed in loop, returned %zd (expected %zd)",
                         w, expected);
            break;
        }

        sim_log_info("targets: updated target at idx=%d (active=%d/%d, id=%d)",
                     idx, active_count, max_targets, targets[idx].id);
    }
    g_current_code_area = CODE_AREA_SHUTDOWN;

    close(fd_tgt_out);
    sim_log_info("targets: exiting (signal or pipe error)");
    return EXIT_SUCCESS;
}
