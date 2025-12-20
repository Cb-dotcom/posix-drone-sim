#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/select.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>        // fabs for motion deadzone
#include <time.h>        // nanosleep

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_params.h"  // runtime parameters (mass, damping, dt, world size)

// Flag set by the SIGINT handler to request a clean shutdown
static volatile sig_atomic_t running = 1;

// Async-signal-safe SIGINT handler: just flip the running flag
static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

// Keep drone inside [0, world_width] × [0, world_height].
// Also zero velocity components that point into the wall so we don't keep
// bouncing or sliding along the boundary forever.
static void apply_world_bounds(DroneState *d, double world_width, double world_height)
{
    if (d->x < 0.0) {
        d->x = 0.0;
        if (d->vx < 0.0) d->vx = 0.0;      // kill velocity into left wall
    } else if (d->x > world_width) {
        d->x = world_width;
        if (d->vx > 0.0) d->vx = 0.0;      // kill velocity into right wall
    }

    if (d->y < 0.0) {
        d->y = 0.0;
        if (d->vy < 0.0) d->vy = 0.0;      // kill velocity into bottom wall
    } else if (d->y > world_height) {
        d->y = world_height;
        if (d->vy > 0.0) d->vy = 0.0;      // kill velocity into top wall
    }
}

// Zero out very small velocities so we don't get visual jitter from tiny
// residual motion near equilibrium (especially near walls).
static void apply_motion_deadzone(DroneState *d)
{
    const double V_EPS = 1e-4;

    if (fabs(d->vx) < V_EPS) d->vx = 0.0;
    if (fabs(d->vy) < V_EPS) d->vy = 0.0;
}

/*
 * Drain ALL pending CommandState messages and keep only the most recent one.
 *
 * Why: if bb_server writes faster than we integrate, commands can queue up.
 * Reading only one per tick causes "old" forces to be applied late and
 * can look like jumping/teleporting.
 *
 * Return values:
 *  1 = command updated
 *  0 = no pending commands
 * -1 = EOF or fatal read error
 */
static int drain_latest_command(int fd_cmd_in, CommandState *c)
{
    int updated = 0;

    while (1) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_cmd_in, &rfds);

        // Non-blocking poll: only drain what's already available
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;

        int ready = select(fd_cmd_in + 1, &rfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            perror("drone: select (drain)");
            return -1;
        }

        if (ready == 0 || !FD_ISSET(fd_cmd_in, &rfds)) {
            break; // nothing more to read right now
        }

        CommandState tmp;
        ssize_t r = read_full(fd_cmd_in, &tmp, sizeof(tmp));
        if (r == (ssize_t)sizeof(tmp)) {
            *c = tmp;
            updated = 1;
            continue; // try to drain more (keep newest)
        }

        if (r == 0) {
            sim_log_info("drone: cmd pipe EOF, exiting\n");
            return -1;
        }

        if (r < 0) {
            perror("drone: read_full(fd_cmd_in)");
            return -1;
        }

        // Partial read shouldn't happen with read_full, but keep safe.
        sim_log_info("drone: unexpected partial read (%zd bytes)\n", r);
        return -1;
    }

    return updated;
}

static void sleep_dt(double dt)
{
    if (dt <= 0.0) {
        return;
    }

    struct timespec ts;
    ts.tv_sec  = (time_t)dt;
    ts.tv_nsec = (long)((dt - (double)ts.tv_sec) * 1e9);

    if (ts.tv_nsec < 0) ts.tv_nsec = 0;
    if (ts.tv_nsec > 999999999L) ts.tv_nsec = 999999999L;

    // nanosleep can be interrupted; restart if needed
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        // continue
    }
}

int main(int argc, char *argv[])
{
    sim_log_init("drone");
    signal(SIGINT, handle_sigint);

    // Load runtime parameters in this process
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "drone: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }
    const SimParams *params = sim_params_get();

    // FDs for anonymous pipes are passed via argv by master:
    //   ./drone <fd_cmd_in> <fd_state_out>
    if (argc < 3) {
        fprintf(stderr, "drone: usage: %s <fd_cmd_in> <fd_state_out>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_cmd_in    = atoi(argv[SIM_ARG_DRONE_CMD_IN]);
    int fd_state_out = atoi(argv[SIM_ARG_DRONE_STATE_OUT]);

    // Use dt, mass, damping and world size from parameter file (or defaults)
    const double dt           = params->dt;
    const double mass         = params->mass;
    const double damping      = params->damping;
    const double world_width  = (double)params->world_width;
    const double world_height = (double)params->world_height;

    sim_log_info("drone: started (dt=%.3f, M=%.3f, K=%.3f)\n", dt, mass, damping);

    DroneState   d;
    CommandState c;

    // Start the drone at the center of the world
    d.x  = world_width  / 2.0;
    d.y  = world_height / 2.0;
    d.vx = 0.0;
    d.vy = 0.0;

    c.fx       = 0.0;
    c.fy       = 0.0;
    c.brake    = 0;
    c.reset    = 0;
    c.quit     = 0;
    c.last_key = 0;

    while (running) {
        // Drain all pending commands; keep only the newest one
        int drained = drain_latest_command(fd_cmd_in, &c);
        if (drained < 0) {
            break; // EOF or fatal error
        }

        if (c.quit) {
            sim_log_info("drone: quit flag set, exiting\n");
            break;
        }

        if (c.reset) {
            // Reset back to center of the world
            d.x  = world_width  / 2.0;
            d.y  = world_height / 2.0;
            d.vx = 0.0;
            d.vy = 0.0;

            // IMPORTANT: clear reset locally so we don't keep resetting
            c.reset = 0;
        }

        // Optional: strong brake behavior (keeps it stable if brake used)
        // If you want "brake = stop now", uncomment this block.
        /*
        if (c.brake) {
            d.vx = 0.0;
            d.vy = 0.0;
        }
        */

        // Integrate exactly one physics step per loop (fixed dt)
        double fx = c.fx;
        double fy = c.fy;

        double ax = (fx - damping * d.vx) / mass;
        double ay = (fy - damping * d.vy) / mass;

        d.vx += ax * dt;
        d.vy += ay * dt;

        apply_motion_deadzone(&d);

        d.x  += d.vx * dt;
        d.y  += d.vy * dt;

        apply_world_bounds(&d, world_width, world_height);

        if (write_full(fd_state_out, &d, sizeof(d)) != (ssize_t)sizeof(d)) {
            perror("drone: write_full(fd_state_out)");
            break;
        }

        // Sleep to maintain the dt cadence (instead of "dt is select timeout")
        sleep_dt(dt);
    }

    sim_log_info("drone: exiting\n");
    close(fd_cmd_in);
    close(fd_state_out);
    return EXIT_SUCCESS;
}
