// Obstacles process.
// Generates obstacles over time and sends them periodically to bb_server
// via an anonymous pipe. bb_server stores them in WorldState and uses them
// for drawing / repulsion.

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_params.h"
#include "sim_log.h"
#include "sim_const.h"

static volatile sig_atomic_t running = 1;

static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// keep obstacle generation in one place
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

    if (argc < 2) {
        sim_log_info("obstacles: usage error: expected fd_obstacles_out argument");
        return EXIT_FAILURE;
    }

    int fd_obs_out = atoi(argv[SIM_ARG_OBS_OUT]);

    // Load runtime parameters for obstacles
    if (sim_params_load(NULL) != 0) {
        sim_log_info("obstacles: warning: could not load '%s', using built-in defaults",
                     SIM_PARAMS_DEFAULT_PATH);
    }

    const SimParams *params = sim_params_get();

    // IMPORTANT:
    // sim_params.c parses "max_obstacles" into params->num_obstacles
    // so num_obstacles is the hard cap.
    int max_obstacles = params->num_obstacles;
    if (max_obstacles < 0) {
        max_obstacles = 0;
    }
    if (max_obstacles > SIM_MAX_OBSTACLES) {
        max_obstacles = SIM_MAX_OBSTACLES;
    }

    int active_count = params->initial_obstacles;
    if (active_count < 0) active_count = 0;
    if (active_count > max_obstacles) active_count = max_obstacles;

    sim_log_info("obstacles: started (world=%dx%d, initial=%d, max=%d, spawn_interval=%.2f)",
                 params->world_width,
                 params->world_height,
                 active_count,
                 max_obstacles,
                 params->obstacle_spawn_interval);

    // If we have no capacity at all, just exit quietly
    if (max_obstacles == 0) {
        sim_log_info("obstacles: max_obstacles <= 0, nothing to do");
        close(fd_obs_out);
        sim_log_info("obstacles: exiting (no capacity)");
        return EXIT_SUCCESS;
    }

    Obstacle obstacles[SIM_MAX_OBSTACLES];

    // Seed RNG with time and PID to avoid identical maps across runs
    srand((unsigned)time(NULL) ^ (unsigned)getpid());

    // Fixed radius for obstacles (matches your UI + repulsion assumptions)
    const double radius = 1.0;

    // Initialize active obstacles
    for (int i = 0; i < active_count; ++i) {
        generate_random_obstacle(&obstacles[i], params, radius);
    }

    // Mark remaining slots (up to max_obstacles) as inactive
    for (int i = active_count; i < max_obstacles; ++i) {
        obstacles[i].x      = 0.0;
        obstacles[i].y      = 0.0;
        obstacles[i].radius = 0.0;
        obstacles[i].active = 0;
    }

    // Optional: clear the rest of the array too (debug-friendly)
    for (int i = max_obstacles; i < SIM_MAX_OBSTACLES; ++i) {
        obstacles[i].x      = 0.0;
        obstacles[i].y      = 0.0;
        obstacles[i].radius = 0.0;
        obstacles[i].active = 0;
    }

    ssize_t expected = (ssize_t)(max_obstacles * (int)sizeof(Obstacle));

    // Send initial snapshot to bb_server
    ssize_t w = write_full(fd_obs_out, obstacles,
                           (size_t)(max_obstacles * (int)sizeof(Obstacle)));
    if (w != expected) {
        sim_log_info("obstacles: write_full(fd_obs_out) failed, returned %zd (expected %zd)",
                     w, expected);
        close(fd_obs_out);
        sim_log_info("obstacles: exiting (initial write failed)");
        return EXIT_FAILURE;
    }

    sim_log_info("obstacles: sent initial %d/%d obstacles to bb_server",
                 active_count, max_obstacles);

    // Spawn interval
    double interval = params->obstacle_spawn_interval;
    if (interval <= 0.0) {
        interval = SIM_DEFAULT_OBSTACLE_SPAWN_INTERVAL;
    }

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
            idx = active_count;
            active_count++;
        } else {
            idx = oldest_index;
            oldest_index = (oldest_index + 1) % max_obstacles;
        }

        generate_random_obstacle(&obstacles[idx], params, radius);

        // Send full snapshot (fixed size)
        w = write_full(fd_obs_out, obstacles,
                       (size_t)(max_obstacles * (int)sizeof(Obstacle)));
        if (w != expected) {
            sim_log_info("obstacles: write_full(fd_obs_out) failed in loop, returned %zd (expected %zd)",
                         w, expected);
            break;
        }

        sim_log_info("obstacles: updated obstacle at idx=%d (active=%d/%d)",
                     idx, active_count, max_obstacles);
    }

    close(fd_obs_out);
    sim_log_info("obstacles: exiting (signal or pipe error)");
    return EXIT_SUCCESS;
}
