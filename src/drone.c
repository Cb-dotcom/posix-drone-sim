#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/select.h>
#include <math.h>
#include <time.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_params.h"

static volatile sig_atomic_t running = 1;

static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

static void apply_world_bounds(DroneState *d, double world_width, double world_height)
{
    if (d->x < 0.0) {
        d->x = 0.0;
        if (d->vx < 0.0) d->vx = 0.0;
    } else if (d->x > world_width) {
        d->x = world_width;
        if (d->vx > 0.0) d->vx = 0.0;
    }

    if (d->y < 0.0) {
        d->y = 0.0;
        if (d->vy < 0.0) d->vy = 0.0;
    } else if (d->y > world_height) {
        d->y = world_height;
        if (d->vy > 0.0) d->vy = 0.0;
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
    int updated = 0;

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_cmd_in, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

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
            updated = 1;
            continue;
        }
        if (r == 0) return -1;
        perror("drone: read_full(cmd)");
        return -1;
    }

    return updated;
}

static int drain_latest_obstacles(int fd_obs_in, Obstacle *obs, size_t nbytes)
{
    int updated = 0;

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_obs_in, &rfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        int ready = select(fd_obs_in + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("drone: select (drain obs)");
            return -1;
        }

        if (ready == 0 || !FD_ISSET(fd_obs_in, &rfds)) break;

        ssize_t r = read_full(fd_obs_in, obs, nbytes);
        if (r == (ssize_t)nbytes) {
            updated = 1;
            continue; // drain more, keep newest snapshot
        }
        if (r == 0) return -1;
        perror("drone: read_full(obstacles)");
        return -1;
    }

    return updated;
}

static void sleep_dt(double dt)
{
    if (dt <= 0.0) return;

    struct timespec ts;
    ts.tv_sec  = (time_t)dt;
    ts.tv_nsec = (long)((dt - (double)ts.tv_sec) * 1e9);
    if (ts.tv_nsec < 0) ts.tv_nsec = 0;
    if (ts.tv_nsec > 999999999L) ts.tv_nsec = 999999999L;

    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        // retry
    }
}

// HARD collision resolution: push out + kill inward velocity component
static void resolve_obstacle_collisions(DroneState *d,
                                        const Obstacle *obs,
                                        int obs_slots)
{
    const double DRONE_RADIUS = 0.5; 

    for (int i = 0; i < obs_slots; ++i) {
        if (!obs[i].active) continue;

        double dx = d->x - obs[i].x;
        double dy = d->y - obs[i].y;

        double dist2 = dx*dx + dy*dy;
        double min_dist = obs[i].radius + DRONE_RADIUS;

        if (dist2 <= 1e-12) {
            // sitting exactly on center; shove out safely
            d->x += min_dist;
            d->vx = 0.0;
            d->vy = 0.0;
            continue;
        }

        double dist = sqrt(dist2);
        if (dist < min_dist) {
            double nx = dx / dist;
            double ny = dy / dist;

            double overlap = (min_dist - dist);
            d->x += nx * overlap;
            d->y += ny * overlap;

            // remove velocity that points into obstacle normal
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
    signal(SIGINT, handle_sigint);

    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "drone: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }
    const SimParams *params = sim_params_get();

    // NEW usage:
    //   ./drone <fd_cmd_in> <fd_state_out> [fd_obstacles_in]
    if (argc < 3) {
        fprintf(stderr, "drone: usage: %s <fd_cmd_in> <fd_state_out> [fd_obstacles_in]\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_cmd_in    = atoi(argv[1]);
    int fd_state_out = atoi(argv[2]);

    int fd_obs_in = -1;
    if (argc >= 4) {
        fd_obs_in = atoi(argv[3]);
    }

    const double dt           = params->dt;
    const double mass         = params->mass;
    const double damping      = params->damping;
    const double world_width  = (double)params->world_width;
    const double world_height = (double)params->world_height;

    // how many obstacles are sent per snapshot
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

    DroneState d;
    CommandState c;

    d.x  = world_width  / 2.0;
    d.y  = world_height / 2.0;
    d.vx = 0.0;
    d.vy = 0.0;

    c.fx = c.fy = 0.0;
    c.brake = c.reset = c.quit = 0;
    c.last_key = 0;

    while (running) {
        // Commands
        if (drain_latest_command(fd_cmd_in, &c) < 0) break;

        if (c.quit) {
            sim_log_info("drone: quit flag set, exiting\n");
            break;
        }

        if (c.reset) {
            d.x  = world_width  / 2.0;
            d.y  = world_height / 2.0;
            d.vx = 0.0;
            d.vy = 0.0;
            c.reset = 0; // consume
        }

        // Obstacles snapshot (optional)
        if (fd_obs_in >= 0 && obs_slots > 0) {
            int upd = drain_latest_obstacles(fd_obs_in, obstacles, obs_nbytes);
            if (upd < 0) {
                // if obstacle pipe dies, just disable collisions (don’t crash sim)
                fd_obs_in = -1;
            } else if (upd > 0) {
                have_obstacles = 1;
            }
        }

        // Integrate
        double fx = c.fx;
        double fy = c.fy;

        double ax = (fx - damping * d.vx) / mass;
        double ay = (fy - damping * d.vy) / mass;

        d.vx += ax * dt;
        d.vy += ay * dt;

        apply_motion_deadzone(&d);

        d.x += d.vx * dt;
        d.y += d.vy * dt;

        apply_world_bounds(&d, world_width, world_height);

        // HARD obstacle collisions (Step 6)
        if (have_obstacles && obs_slots > 0) {
            resolve_obstacle_collisions(&d, obstacles, obs_slots);
        }

        if (write_full(fd_state_out, &d, sizeof(d)) != (ssize_t)sizeof(d)) {
            perror("drone: write_full(fd_state_out)");
            break;
        }

        sleep_dt(dt);
    }

    sim_log_info("drone: exiting\n");
    close(fd_cmd_in);
    close(fd_state_out);
    if (fd_obs_in >= 0) close(fd_obs_in);
    return EXIT_SUCCESS;
}
