#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>  
#include <sys/select.h>
#include <sys/types.h>
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

static void apply_world_bounds(DroneState *d, double world_width, double world_height)
{
    if (d->x < 0.0) {
        d->x = 0.0;
    } else if (d->x > world_width) {
        d->x = world_width;
    }

    if (d->y < 0.0) {
        d->y = 0.0;
    } else if (d->y > world_height) {
        d->y = world_height;
    }
}

static void apply_motion_deadzone(DroneState *d)
{
    const double V_EPS = 1e-4;
    if (fabs(d->vx) < V_EPS) d->vx = 0.0;
    if (fabs(d->vy) < V_EPS) d->vy = 0.0;
}

static int drain_latest_command(int fd_cmd_in, CommandState *c)
{
    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_cmd_in, &rfds);

        struct timeval tv = {0, 0};

        int ready = select(fd_cmd_in + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("drone: select (drain cmd)");
            return -1;
        }

        if (ready == 0 || !FD_ISSET(fd_cmd_in, &rfds)) break;

        CommandState tmp;
        ssize_t r = read_full(fd_cmd_in, &tmp, sizeof(tmp));
        if (r == (ssize_t)sizeof(tmp)) {
            *c = tmp;
            continue; // keep draining, keep newest
        }
        if (r == 0) return -1;
        perror("drone: read_full(cmd)");
        return -1;
    }
    return 0;
}

static int drain_latest_obstacles(int fd_obs_in, Obstacle *obs, size_t nbytes)
{
    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_obs_in, &rfds);

        struct timeval tv = {0, 0};

        int ready = select(fd_obs_in + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("drone: select (drain obs)");
            return -1;
        }

        if (ready == 0 || !FD_ISSET(fd_obs_in, &rfds)) break;

        ssize_t r = read_full(fd_obs_in, obs, nbytes);
        if (r == (ssize_t)nbytes) {
            continue; // keep draining, keep newest snapshot
        }
        if (r == 0) return -1;
        perror("drone: read_full(obstacles)");
        return -1;
    }
    return 0;
}

static void sleep_dt(double dt)
{
    if (dt <= 0.0) return;

    struct timespec ts;
    ts.tv_sec  = (time_t)dt;
    ts.tv_nsec = (long)((dt - (double)ts.tv_sec) * 1e9);
    if (ts.tv_nsec < 0) ts.tv_nsec = 0;
    if (ts.tv_nsec > 999999999L) ts.tv_nsec = 999999999L;

    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {}
}

static void resolve_obstacle_collisions(DroneState *d, const Obstacle *obs, int obs_slots, int *collision_flag)
{
    const double DRONE_RADIUS = 0.5;

    for (int i = 0; i < obs_slots; ++i) {
        if (!obs[i].active) continue;

        double dx = d->x - obs[i].x;
        double dy = d->y - obs[i].y;

        double dist2 = dx * dx + dy * dy;
        double min_dist = obs[i].radius + DRONE_RADIUS;

        if (dist2 <= 1e-12) {
            d->x += min_dist;
            d->vx = 0.0;
            d->vy = 0.0;
            continue;
        }

        double dist = sqrt(dist2);
        if (dist < min_dist) {
            *collision_flag = 1; 

            double nx = dx / dist;
            double ny = dy / dist;

            double overlap = (min_dist - dist);
            d->x += nx * overlap;
            d->y += ny * overlap;

            double vdot = d->vx * nx + d->vy * ny;
            if (vdot < 0.0) {
                d->vx -= vdot * nx;
                d->vy -= vdot * ny;
            }
        }
    }
}

int main(int argc, char *argv[])
{
    sim_log_init("drone");
    sim_process_register("drone", getpid());
    
    signal(SIGINT, handle_sigint);
    wd_client_init();

    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "drone: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }
    const SimParams *params = sim_params_get();

    // ./drone <fd_cmd_in> <fd_state_out> [fd_obstacles_in]
    if (argc < 3) {
        fprintf(stderr, "drone: usage: %s <fd_cmd_in> <fd_state_out> [fd_obstacles_in]\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_cmd_in    = atoi(argv[1]);
    int fd_state_out = atoi(argv[2]);

    int fd_obs_in = -1;
    if (argc >= 4) fd_obs_in = atoi(argv[3]);

    const double dt           = params->dt;
    const double mass         = params->mass;
    const double damping      = params->damping;
    const double world_width  = (double)params->world_width;
    const double world_height = (double)params->world_height;

    int obs_slots = params->num_obstacles;
    if (obs_slots < 0) obs_slots = 0;
    if (obs_slots > SIM_MAX_OBSTACLES) obs_slots = SIM_MAX_OBSTACLES;

    Obstacle obstacles[SIM_MAX_OBSTACLES];
    for (int i = 0; i < SIM_MAX_OBSTACLES; ++i) {
        obstacles[i].x = obstacles[i].y = obstacles[i].radius = 0.0;
        obstacles[i].active = 0;
    }
    int have_obstacles = 0;
    size_t obs_nbytes = (size_t)(obs_slots * (int)sizeof(Obstacle));

    sim_log_info("drone: started (dt=%.3f, M=%.3f, K=%.3f, obs_slots=%d)\n",
                 dt, mass, damping, obs_slots);

    DroneState d = {0};
    CommandState c = {0};

    d.x  = world_width  / 2.0;
    d.y  = world_height / 2.0;

    while (running) {
        if (drain_latest_command(fd_cmd_in, &c) < 0) break;

        if (c.quit) break;

        if (c.reset) {
            d.x  = world_width  / 2.0;
            d.y  = world_height / 2.0;
            d.vx = 0.0;
            d.vy = 0.0;
            c.reset = 0;
        }

        if (fd_obs_in >= 0 && obs_slots > 0) {
            if (drain_latest_obstacles(fd_obs_in, obstacles, obs_nbytes) < 0) {
                fd_obs_in = -1; // disable quietly if pipe died
            } else {
                have_obstacles = 1;
            }
        }

        double ax = (c.fx - damping * d.vx) / mass;
        double ay = (c.fy - damping * d.vy) / mass;

        d.vx += ax * dt;
        d.vy += ay * dt;

        apply_motion_deadzone(&d);

        d.x += d.vx * dt;
        d.y += d.vy * dt;

        apply_world_bounds(&d, world_width, world_height);

        if (have_obstacles && obs_slots > 0) {
            int collision_occurred = 0;
            resolve_obstacle_collisions(&d, obstacles, obs_slots, &collision_occurred);
        }

        if (write_full(fd_state_out, &d, sizeof(d)) != (ssize_t)sizeof(d)) {
            perror("drone: write_full(fd_state_out)");
            break;
        }

        sleep_dt(dt);
    }

    close(fd_cmd_in);
    close(fd_state_out);
    if (fd_obs_in >= 0) close(fd_obs_in);
    return EXIT_SUCCESS;
}
