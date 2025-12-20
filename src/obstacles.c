#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  
#include <time.h>
#include <unistd.h>

#include "sim_const.h"
#include "sim_ipc.h"
#include "sim_log.h"
#include "sim_params.h"
#include "sim_types.h"

static volatile sig_atomic_t running = 1;

static pid_t g_wd_pid = -1;

static void wd_client_ping_handler(int sig)
{
    (void)sig;
    if (g_wd_pid > 1) {
        (void)kill(g_wd_pid, SIGUSR2);
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
    sa.sa_handler = wd_client_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_RESTART;

    (void)sigaction(SIGUSR1, &sa, NULL);
}

static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

static void generate_random_obstacle(Obstacle *o, const SimParams *params, double radius)
{
    double margin = radius;

    double x_range = (double)params->world_width  - 2.0 * margin;
    double y_range = (double)params->world_height - 2.0 * margin;
    if (x_range < 0.0) x_range = 0.0;
    if (y_range < 0.0) y_range = 0.0;

    double rx = (double)rand() / (double)RAND_MAX;
    double ry = (double)rand() / (double)RAND_MAX;

    o->x      = margin + rx * x_range;
    o->y      = margin + ry * y_range;
    o->radius = radius;
    o->active = 1;
}

int main(int argc, char *argv[])
{
    sim_log_init("obstacles");
    signal(SIGINT, handle_sigint);
    wd_client_init();

    // NEW usage:
    //   ./obstacles <fd_obs_out_bb> <fd_obs_out_drone>
    if (argc < 3) {
        sim_log_info("obstacles: usage: %s <fd_obs_out_bb> <fd_obs_out_drone>", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_obs_out_bb    = atoi(argv[1]);
    int fd_obs_out_drone = atoi(argv[2]);

    if (sim_params_load(NULL) != 0) {
        sim_log_info("obstacles: warning: could not load '%s', using built-in defaults",
                     SIM_PARAMS_DEFAULT_PATH);
    }
    const SimParams *params = sim_params_get();

    int max_obstacles = params->num_obstacles;
    if (max_obstacles < 0) max_obstacles = 0;
    if (max_obstacles > SIM_MAX_OBSTACLES) max_obstacles = SIM_MAX_OBSTACLES;

    int active_count = params->initial_obstacles;
    if (active_count < 0) active_count = 0;
    if (active_count > max_obstacles) active_count = max_obstacles;

    sim_log_info("obstacles: started (world=%dx%d, initial=%d, max=%d, spawn_interval=%.2f)",
                 params->world_width, params->world_height,
                 active_count, max_obstacles, params->obstacle_spawn_interval);

    if (max_obstacles == 0) {
        close(fd_obs_out_bb);
        close(fd_obs_out_drone);
        return EXIT_SUCCESS;
    }

    Obstacle obstacles[SIM_MAX_OBSTACLES];
    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    const double radius = 1.0;

    for (int i = 0; i < active_count; ++i) {
        generate_random_obstacle(&obstacles[i], params, radius);
    }
    for (int i = active_count; i < max_obstacles; ++i) {
        obstacles[i].x = obstacles[i].y = obstacles[i].radius = 0.0;
        obstacles[i].active = 0;
    }
    for (int i = max_obstacles; i < SIM_MAX_OBSTACLES; ++i) {
        obstacles[i].x = obstacles[i].y = obstacles[i].radius = 0.0;
        obstacles[i].active = 0;
    }

    ssize_t expected = (ssize_t)(max_obstacles * (int)sizeof(Obstacle));

    // Send initial snapshot to BOTH
    ssize_t w1 = write_full(fd_obs_out_bb, obstacles, (size_t)expected);
    ssize_t w2 = write_full(fd_obs_out_drone, obstacles, (size_t)expected);
    if (w1 != expected || w2 != expected) {
        sim_log_info("obstacles: initial write failed (bb=%zd, drone=%zd, expected=%zd)",
                     w1, w2, expected);
        close(fd_obs_out_bb);
        close(fd_obs_out_drone);
        return EXIT_FAILURE;
    }

    double interval = params->obstacle_spawn_interval;
    if (interval <= 0.0) interval = SIM_DEFAULT_OBSTACLE_SPAWN_INTERVAL;

    struct timespec sleep_ts;
    sleep_ts.tv_sec  = (time_t)interval;
    sleep_ts.tv_nsec = (long)((interval - (double)sleep_ts.tv_sec) * 1e9);
    if (sleep_ts.tv_nsec < 0) sleep_ts.tv_nsec = 0;

    int oldest_index = 0;

    while (running) {
        nanosleep(&sleep_ts, NULL);
        if (!running) break;

        int idx;
        if (active_count < max_obstacles) {
            idx = active_count++;
        } else {
            idx = oldest_index;
            oldest_index = (oldest_index + 1) % max_obstacles;
        }

        generate_random_obstacle(&obstacles[idx], params, radius);

        w1 = write_full(fd_obs_out_bb, obstacles, (size_t)expected);
        w2 = write_full(fd_obs_out_drone, obstacles, (size_t)expected);
        if (w1 != expected || w2 != expected) {
            sim_log_info("obstacles: write failed (bb=%zd, drone=%zd, expected=%zd)",
                         w1, w2, expected);
            break;
        }
    }

    close(fd_obs_out_bb);
    close(fd_obs_out_drone);
    return EXIT_SUCCESS;
}
