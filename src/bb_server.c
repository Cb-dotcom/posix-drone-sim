#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/select.h>
#include <fcntl.h>
#include <curses.h>
#include <math.h>
#include <time.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_ui.h"
#include "sim_params.h"

// Crucial integer type used for providing variables that can be
// read and written by both the main prog and sign handler
// without introducing race conditions.
static volatile sig_atomic_t running = 1;

void play(const char *filename)
{
    pid_t pid = fork();
    if (pid == 0) {
        int fd = open("/dev/null", O_RDWR);
        if (fd >= 0) {
            dup2(fd, STDIN_FILENO);
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            if (fd > 2) close(fd);
        }

        execlp("mpg123", "mpg123", "-q", filename, (char *)NULL);
        _exit(1);
    }
}

// Async-signal-safe SIGINT handler: just flip the running flag
static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

/*
 * Latombe / Khatib-style repulsive force magnitude.
 *
 * distance       = distance to wall/obstacle (>= 0)
 * function_scale = eta (params->eta)
 * area_of_effect = rho0 (params->rho)
 * vel_x, vel_y   = drone velocity components
 *
 * F_rep(d) = eta * (1/d - 1/rho0) * (1/d^2) * |v|
 * only if 0 < d <= area_of_effect.
 * Direction (sign) is handled by the caller.
 */
static double repulsive_force(double distance,
                              double function_scale,
                              double area_of_effect,
                              double vel_x,
                              double vel_y)
{
    if (function_scale <= 0.0 || area_of_effect <= 0.0) {
        return 0.0;
    }

    // Avoid insane spikes and ignore outside radius
    const double min_dist = 0.1;
    if (distance <= min_dist || distance > area_of_effect) {
        return 0.0;
    }

    double vel_mag = sqrt(vel_x * vel_x + vel_y * vel_y);
    if (vel_mag <= 0.0) {
        return 0.0;  // if we're not moving, no repulsion
    }

    double inv_d   = 1.0 / distance;
    double inv_rho = 1.0 / area_of_effect;

    double base = (inv_d - inv_rho) * inv_d * inv_d;  // (1/d - 1/rho)/d^2
    if (base <= 0.0) {
        return 0.0;
    }

    return function_scale * base * vel_mag;
}

/*
 * Wall repulsion:
 * - radius:   params->rho
 * - strength: params->eta
 *
 * Same sign logic as the old project:
 *  - LEFT wall:  +Fx
 *  - RIGHT wall: -Fx
 *  - BOTTOM:     +Fy
 *  - TOP:        -Fy
 * Magnitude from repulsive_force(), using full |v|.
 */
static void compute_wall_repulsion(const WorldState *world,
                                   const SimParams  *params,
                                   double           *out_fx,
                                   double           *out_fy)
{
    double fx = 0.0;
    double fy = 0.0;

    double rho = params->rho;  // radius of influence
    double eta = params->eta;  // strength

    if (rho <= 0.0 || eta <= 0.0) {
        *out_fx = 0.0;
        *out_fy = 0.0;
        return;
    }

    double x  = world->drone.x;
    double y  = world->drone.y;
    double vx = world->drone.vx;
    double vy = world->drone.vy;

    double w = (double)params->world_width;
    double h = (double)params->world_height;

    // LEFT wall (x = 0): distance = x, push +x
    if (x < rho) {
        double d = x;
        double f = repulsive_force(d, eta, rho, vx, vy);
        fx += f;
    }

    // RIGHT wall (x = w): distance = w - x, push -x
    if (x > w - rho) {
        double d = w - x;
        double f = repulsive_force(d, eta, rho, vx, vy);
        fx -= f;
    }

    // BOTTOM wall (y = 0): distance = y, push +y
    if (y < rho) {
        double d = y;
        double f = repulsive_force(d, eta, rho, vx, vy);
        fy += f;
    }

    // TOP wall (y = h): distance = h - y, push -y
    if (y > h - rho) {
        double d = h - y;
        double f = repulsive_force(d, eta, rho, vx, vy);
        fy -= f;
    }

    *out_fx = fx;
    *out_fy = fy;
}

/*
 * Obstacle repulsion:
 * - same Latombe law, but vector points away from obstacle.
 * - uses a slightly BIGGER radius than walls: rho_obs = 1.5 * rho
 *
 * FIX: use distance to obstacle SURFACE (center_dist - obs->radius),
 *      and clamp close distances so repulsion stays strong.
 */
static void compute_obstacle_repulsion(const WorldState *world,
                                       const SimParams  *params,
                                       double           *out_fx,
                                       double           *out_fy)
{
    double fx = 0.0;
    double fy = 0.0;

    double rho     = params->rho;
    double eta     = params->eta;
    double rho_obs = rho * 1.5;   // obstacle radius-of-influence (bigger than walls)

    if (rho_obs <= 0.0 || eta <= 0.0) {
        *out_fx = 0.0;
        *out_fy = 0.0;
        return;
    }

    double x  = world->drone.x;
    double y  = world->drone.y;
    double vx = world->drone.vx;
    double vy = world->drone.vy;

    // IMPORTANT: repulsive_force() returns 0 when distance <= 0.1,
    // so clamp to a tiny bit ABOVE that threshold.
    const double REP_MIN_DIST = 0.100001;

    // NOTE: loop over slots received, not active count.
    for (int i = 0; i < world->obstacles_slots; ++i) {
        const Obstacle *obs = &world->obstacles[i];
        if (!obs->active) {
            continue;
        }

        double dx = obs->x - x;
        double dy = obs->y - y;

        double center_dist = sqrt(dx * dx + dy * dy);
        if (center_dist <= 1e-9) {
            continue;
        }

        // Distance to obstacle SURFACE (not center)
        double surface_dist = center_dist - obs->radius;

        // If we're inside / extremely close, clamp so repulsion doesn't turn off
        if (surface_dist < REP_MIN_DIST) {
            surface_dist = REP_MIN_DIST;
        }

        // Apply only within radius-of-influence from the surface
        if (surface_dist > rho_obs) {
            continue;
        }

        double f_mag = repulsive_force(surface_dist, eta, rho_obs, vx, vy);
        if (f_mag <= 0.0) {
            continue;
        }

        // Direction: away from obstacle (from obstacle to drone)
        double nx = (x - obs->x) / center_dist;
        double ny = (y - obs->y) / center_dist;

        fx += f_mag * nx;
        fy += f_mag * ny;
    }

    *out_fx = fx;
    *out_fy = fy;
}

/*
 * Helper: does the segment [x0,y0] -> [x1,y1] intersect
 * the circle centered at (cx,cy) with radius r ?
 *
 * Returns 1 if yes, 0 otherwise.
 */
static int segment_hits_circle(double x0, double y0,
                               double x1, double y1,
                               double cx, double cy,
                               double r)
{
    double r2 = r * r;

    // Endpoint inside circle?
    double dx0 = x0 - cx;
    double dy0 = y0 - cy;
    double dx1 = x1 - cx;
    double dy1 = y1 - cy;

    double dist0_sq = dx0 * dx0 + dy0 * dy0;
    double dist1_sq = dx1 * dx1 + dy1 * dy1;

    if (dist0_sq <= r2 || dist1_sq <= r2) {
        return 1;
    }

    // Degenerate segment
    double sx = x1 - x0;
    double sy = y1 - y0;
    double len2 = sx * sx + sy * sy;
    if (len2 <= 1e-9) {
        return 0;
    }

    // Projection of circle center onto segment
    double t = ((cx - x0) * sx + (cy - y0) * sy) / len2;
    if (t < 0.0) t = 0.0;
    else if (t > 1.0) t = 1.0;

    double closest_x = x0 + t * sx;
    double closest_y = y0 + t * sy;

    double dcx = closest_x - cx;
    double dcy = closest_y - cy;
    double dist_closest_sq = dcx * dcx + dcy * dcy;

    return dist_closest_sq <= r2;
}

/*
 * Target handling:
 * - use segment [prev_pos -> current_pos] vs circle intersection
 * - when hit: increase world->score, respawn target at random position
 * - ignore frames where the drone basically didn't move (to avoid weird
 *   initial hits / score jumps).
 */
static void handle_targets(WorldState *world,
                           const SimParams *params,
                           double prev_x,
                           double prev_y)
{
    if (world->targets_slots <= 0) {
        return;
    }

    double x1 = world->drone.x;
    double y1 = world->drone.y;

    // If we didn't move, skip hit detection this frame
    double move_dx = x1 - prev_x;
    double move_dy = y1 - prev_y;
    double move_sq = move_dx * move_dx + move_dy * move_dy;
    if (move_sq < 1e-6) {
        return;
    }

    // Hit radius in world coordinates (tweakable).
    const double HIT_RADIUS = 1.0;

    // NOTE: loop over slots received, not active count.
    for (int i = 0; i < world->targets_slots; ++i) {
        Target *tgt = &world->targets[i];
        if (!tgt->active) {
            continue;
        }

        double cx = tgt->x;
        double cy = tgt->y;

        int hit = segment_hits_circle(prev_x, prev_y, x1, y1, cx, cy, HIT_RADIUS);

        if (hit) {
            play("../../bin/conf/target.mp3");
            world->score += 1.0;

            sim_log_info("bb_server: TARGET HIT idx=%d pos=(%.2f,%.2f) score=%.1f",
                         i, tgt->x, tgt->y, world->score);

            // Respawn this target at a random location in the world
            double w = (double)params->world_width;
            double h = (double)params->world_height;

            tgt->x = ((double)rand() / (double)RAND_MAX) * w;
            tgt->y = ((double)rand() / (double)RAND_MAX) * h;
            tgt->active = 1;

            sim_log_info("bb_server: TARGET RESPAWN idx=%d new_pos=(%.2f,%.2f)",
                         i, tgt->x, tgt->y);
        }
    }
}

int main(int argc, char *argv[])
{
    sim_log_init("bb_server");
    signal(SIGINT, handle_sigint);

    // we add the music
    pid_t music = fork();
    if (music == 0) {
        // Detach completely
        int fd = open("/dev/null", O_RDWR);
        if (fd >= 0) {
            dup2(fd, STDIN_FILENO);
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            if (fd > 2) close(fd);
        }

        // Try MP3 looping with mpg123
        execlp("mpg123", "mpg123", "-f", "4098", "--loop", "-1",
               "../../bin/conf/music.mp3", (char *)NULL);

        perror("Music!");
        _exit(1);
    }

    // Load parameters in this process (master's load does not carry across exec)
    if (sim_params_load(NULL) != 0) {
        sim_log_info("bb_server: could not load '%s', using built-in defaults",
                     SIM_PARAMS_DEFAULT_PATH);
    }

    // Get current runtime parameters
    const SimParams *params = sim_params_get();
    sim_log_info("bb_server: params world=%dx%d obstacles=%d targets=%d "
                 "mass=%.2f damping=%.2f dt=%.3f",
                 params->world_width,
                 params->world_height,
                 params->num_obstacles,
                 params->num_targets,
                 params->mass,
                 params->damping,
                 params->dt);

    // Log repulsion parameters as well
    sim_log_info("bb_server: repulsion params rho=%.2f eta=%.2f",
                 params->rho, params->eta);

    int env_enabled = (params->rho > 0.0 && params->eta > 0.0);
    sim_log_info("bb_server: repulsion %s (Latombe-style |v|)",
                 env_enabled ? "ENABLED" : "DISABLED");

    // Seed RNG for target respawn
    srand((unsigned)time(NULL));

    // FDs for anonymous pipes are now passed via argv by master:
    //   ./bb_server <fd_drone_state_in> <fd_drone_cmd_out> <fd_input_cmd_in>
    //               <fd_obstacles_in> <fd_targets_in>
    if (argc < 6) {
        fprintf(stderr,
                "bb_server: usage: %s <fd_drone_state_in> <fd_drone_cmd_out> "
                "<fd_input_cmd_in> <fd_obstacles_in> <fd_targets_in>\n",
                argv[0]);
        return EXIT_FAILURE;
    }

    int fd_drone_in  = atoi(argv[SIM_ARG_BB_DRONE_STATE_IN]);
    int fd_drone_out = atoi(argv[SIM_ARG_BB_DRONE_CMD_OUT]);
    int fd_input_in  = atoi(argv[SIM_ARG_BB_INPUT_CMD_IN]);
    int fd_obs_in    = atoi(argv[SIM_ARG_BB_OBS_IN]);
    int fd_tgt_in    = atoi(argv[SIM_ARG_BB_TGT_IN]);

    sim_log_info("bb_server: pipe FDs: drone_in=%d drone_out=%d "
                 "input_in=%d obs_in=%d tgt_in=%d",
                 fd_drone_in, fd_drone_out, fd_input_in, fd_obs_in, fd_tgt_in);

    WorldState   world;
    CommandState user_cmd;   // pure user command (raw input)

    // Initialize world state
    world.drone.x  = 0.0;
    world.drone.y  = 0.0;
    world.drone.vx = 0.0;
    world.drone.vy = 0.0;
    world.world_width  = params->world_width;
    world.world_height = params->world_height;

    world.cmd.fx       = 0.0;
    world.cmd.fy       = 0.0;
    world.cmd.brake    = 0;
    world.cmd.reset    = 0;
    world.cmd.quit     = 0;
    world.cmd.last_key = 0;

    user_cmd = world.cmd;

    world.num_obstacles   = 0;
    world.obstacles_slots = 0;
    for (int i = 0; i < SIM_MAX_OBSTACLES; ++i) {
        world.obstacles[i].x      = 0.0;
        world.obstacles[i].y      = 0.0;
        world.obstacles[i].radius = 0.0;
        world.obstacles[i].active = 0;
    }

    world.num_targets   = 0;
    world.targets_slots = 0;
    for (int i = 0; i < SIM_MAX_TARGETS; ++i) {
        world.targets[i].x      = 0.0;
        world.targets[i].y      = 0.0;
        world.targets[i].radius = 0.0;
        world.targets[i].id     = 0;
        world.targets[i].active = 0;
        world.targets[i].time_created.tv_sec  = 0;
        world.targets[i].time_created.tv_nsec = 0;
    }

    world.score = 0.0;

    // Track previous drone position for target hit tests
    double prev_x           = 0.0;
    double prev_y           = 0.0;
    int    have_prev_pos    = 0;
    int    have_drone_state = 0;
    int    have_targets     = 0;

    // For repulsion logic
    int input_received   = 0;
    int wall_active_prev = 0;
    int rep_active_prev  = 0;  // IMPORTANT: prevents command flooding/teleporting

    // Init UI and show menu
    ui_init();

    int start_sim = 0;
    while (!start_sim && running) { // Handling choices of menu
        int choice = ui_show_start_menu();
        sim_log_info("bb_server: menu choice=%d (0=Start,1=Instr,2=Quit)", choice);

        if (choice == UI_MENU_QUIT) {
            running = 0;
        } else if (choice == UI_MENU_INSTRUCTIONS) {
            ui_show_instructions();
        } else if (choice == UI_MENU_START) {
            start_sim = 1;
        }
    }

    sim_log_info("bb_server: after menu loop: start_sim=%d running=%d",
                 start_sim, running);

    if (!running || !start_sim) {
        ui_shutdown();
        close(fd_drone_in);
        close(fd_drone_out);
        close(fd_input_in);
        close(fd_obs_in);
        close(fd_tgt_in);
        sim_log_info("bb_server: exiting from menu");
        return 0;
    }

    // Clear screen after menu so main UI has a clean canvas
    erase();
    refresh();

    sim_log_info("bb_server: entering main loop");

    // Precompute how many obstacles/targets we expect from generators
    int obs_to_read = params->num_obstacles;
    if (obs_to_read > SIM_MAX_OBSTACLES) {
        obs_to_read = SIM_MAX_OBSTACLES;
    } else if (obs_to_read < 0) {
        obs_to_read = 0;
    }

    int tgt_to_read = params->num_targets;
    if (tgt_to_read > SIM_MAX_TARGETS) {
        tgt_to_read = SIM_MAX_TARGETS;
    } else if (tgt_to_read < 0) {
        tgt_to_read = 0;
    }

    // Record slot counts received from generators (loop bounds)
    world.obstacles_slots = obs_to_read;
    world.targets_slots   = tgt_to_read;

    // Main display + IPC loop (pipe-based, no shared memory)
    while (running) {
        fd_set readfds;
        FD_ZERO(&readfds);

        FD_SET(fd_drone_in, &readfds);
        FD_SET(fd_input_in, &readfds);
        FD_SET(fd_obs_in,   &readfds);
        FD_SET(fd_tgt_in,   &readfds);

        int maxfd = fd_drone_in;
        if (fd_input_in > maxfd)  maxfd = fd_input_in;
        if (fd_obs_in   > maxfd)  maxfd = fd_obs_in;
        if (fd_tgt_in   > maxfd)  maxfd = fd_tgt_in;

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 33333; // Approx 30 Hz

        int ready = select(maxfd + 1, &readfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) {
                continue;
            }
            endwin();
            perror("bb_server: select");
            break;
        }

        input_received = 0;

        if (ready > 0) {
            // Data from drone (updated DroneState)
            if (FD_ISSET(fd_drone_in, &readfds)) {
                DroneState ds;
                ssize_t r = read_full(fd_drone_in, &ds, sizeof(ds));

                if (r == (ssize_t)sizeof(ds)) {
                    if (!have_prev_pos) {
                        // First real state: no motion yet
                        prev_x = ds.x;
                        prev_y = ds.y;
                        have_prev_pos = 1;
                    } else {
                        // Normal: prev = old position
                        prev_x = world.drone.x;
                        prev_y = world.drone.y;
                    }

                    world.drone      = ds;
                    have_drone_state = 1;
                } else if (r == 0) {
                    // EOF: drone closed its pipe
                    sim_log_info("bb_server: drone pipe EOF");
                    running = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(drone)");
                    running = 0;
                }
            }

            // Data from input (updated CommandState)
            if (FD_ISSET(fd_input_in, &readfds)) {
                CommandState cs;
                ssize_t r = read_full(fd_input_in, &cs, sizeof(cs));

                if (r == (ssize_t)sizeof(cs)) {
                    user_cmd       = cs;
                    input_received = 1;

                    if (!env_enabled) {
                        // Legacy mode: just forward user command
                        world.cmd = cs;

                        // Forward latest command to drone so it can update physics
                        ssize_t w = write_full(fd_drone_out, &cs, sizeof(cs));
                        if (w != (ssize_t)sizeof(cs)) {
                            endwin();
                            perror("bb_server: write_full(drone)");
                            running = 0;
                        }
                    }
                } else if (r == 0) {
                    sim_log_info("bb_server: input pipe EOF");
                    running = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(input)");
                    running = 0;
                }
            }

            // Data from obstacles (Obstacle array)
            if (FD_ISSET(fd_obs_in, &readfds) && obs_to_read > 0) {
                ssize_t expected = (ssize_t)(obs_to_read * (int)sizeof(Obstacle));
                ssize_t r = read_full(fd_obs_in, world.obstacles,
                                      (size_t)(obs_to_read * (int)sizeof(Obstacle)));

                if (r == expected) {
                    int count = 0;
                    for (int i = 0; i < obs_to_read; ++i) {
                        if (world.obstacles[i].active) {
                            ++count;
                        }
                    }
                    world.num_obstacles = count;
                } else if (r == 0) {
                    sim_log_info("bb_server: obstacles pipe EOF");
                    obs_to_read = 0;
                    world.obstacles_slots = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(obstacles)");
                    running = 0;
                }
            }

            // Data from targets (Target array)
            if (FD_ISSET(fd_tgt_in, &readfds) && tgt_to_read > 0) {
                ssize_t expected = (ssize_t)(tgt_to_read * (int)sizeof(Target));
                ssize_t r = read_full(fd_tgt_in, world.targets,
                                      (size_t)(tgt_to_read * (int)sizeof(Target)));

                if (r == expected) {
                    int count = 0;
                    for (int i = 0; i < tgt_to_read; ++i) {
                        if (world.targets[i].active) {
                            ++count;
                        }
                    }
                    world.num_targets = count;
                    have_targets      = 1;
                } else if (r == 0) {
                    sim_log_info("bb_server: targets pipe EOF");
                    tgt_to_read = 0;
                    world.targets_slots = 0;
                } else if (r < 0) {
                    endwin();
                    perror("bb_server: read_full(targets)");
                    running = 0;
                }
            }
        }

        // Handle targets: collision detection, scoring, respawn
        if (have_prev_pos && have_drone_state && have_targets) {
            handle_targets(&world, params, prev_x, prev_y);
        }

        // Apply wall + obstacle repulsion if environment enabled
        if (running && env_enabled) {
            double fx_wall = 0.0, fy_wall = 0.0;
            double fx_obs  = 0.0, fy_obs  = 0.0;

            compute_wall_repulsion(&world, params, &fx_wall, &fy_wall);
            compute_obstacle_repulsion(&world, params, &fx_obs, &fy_obs);

            double fx_rep = fx_wall + fx_obs;
            double fy_rep = fy_wall + fy_obs;

            int rep_active = (fx_rep != 0.0 || fy_rep != 0.0);

            // IMPORTANT: avoid flooding the drone pipe.
            // Send only when:
            //  - new input, OR
            //  - repulsion currently active, OR
            //  - repulsion was active last frame (send "off" once)
            if (input_received || rep_active || rep_active_prev) {
                CommandState out_cmd = user_cmd;

                if (rep_active) {
                    // Superposition: user force + wall + obstacle repulsion
                    out_cmd.fx = user_cmd.fx + fx_rep;
                    out_cmd.fy = user_cmd.fy + fy_rep;
                }

                ssize_t w = write_full(fd_drone_out, &out_cmd, sizeof(out_cmd));
                if (w != (ssize_t)sizeof(out_cmd)) {
                    endwin();
                    perror("bb_server: write_full(drone with repulsion)");
                    running = 0;
                }

                world.cmd = out_cmd;

                // Wall logging: only ON/OFF transitions (based on walls only)
                int wall_active = (fx_wall != 0.0 || fy_wall != 0.0);
                if (wall_active && !wall_active_prev) {
                    sim_log_info("bb_server: WALL ON  pos=(%.1f,%.1f) "
                                 "user=(%.2f,%.2f) wall=(%.2f,%.2f) "
                                 "obs=(%.2f,%.2f) total=(%.2f,%.2f)",
                                 world.drone.x, world.drone.y,
                                 user_cmd.fx, user_cmd.fy,
                                 fx_wall, fy_wall,
                                 fx_obs, fy_obs,
                                 out_cmd.fx, out_cmd.fy);
                } else if (!wall_active && wall_active_prev) {
                    sim_log_info("bb_server: WALL OFF pos=(%.1f,%.1f)",
                                 world.drone.x, world.drone.y);
                }
                wall_active_prev = wall_active;

                // Update repulsion tracker
                rep_active_prev = rep_active;
            }
        }

        ui_draw(&world);

        if (world.cmd.quit) {
            sim_log_info("bb_server: quit flag set, exiting");
            break;
        }
    }

    ui_shutdown();

    close(fd_drone_in);
    close(fd_drone_out);
    close(fd_input_in);
    close(fd_obs_in);
    close(fd_tgt_in);

    sim_log_info("bb_server: exited");
    sim_log_close();
    return EXIT_SUCCESS;
}
