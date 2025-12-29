#define _POSIX_C_SOURCE 200809L
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
#include <string.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_ui.h"
#include "sim_params.h"

#define DRONE_RADIUS 0.5

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

static void report_pid_if_requested(int argc, char **argv)
{
    if (argc < 2) return;

    char *end = NULL;
    long fd = strtol(argv[argc - 1], &end, 10);
    if (!end || *end != '\0') return;
    if (fd < 0) return;

    pid_t me = getpid();
    (void)write_full((int)fd, &me, sizeof(me));
    close((int)fd);
}

static void play(const char *filename)
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

static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

static double repulsive_force(double distance,
                              double function_scale,
                              double area_of_effect,
                              double vel_x,
                              double vel_y)
{
    if (function_scale <= 0.0 || area_of_effect <= 0.0) {
        return 0.0;
    }

    const double min_dist = 0.1;
    if (distance <= min_dist || distance > area_of_effect) {
        return 0.0;
    }

    double vel_mag = sqrt(vel_x * vel_x + vel_y * vel_y);
    if (vel_mag <= 0.0) {
        return 0.0;
    }

    double inv_d   = 1.0 / distance;
    double inv_rho = 1.0 / area_of_effect;

    double base = (inv_d - inv_rho) * inv_d * inv_d;
    if (base <= 0.0) {
        return 0.0;
    }

    return function_scale * base * vel_mag;
}

static void compute_obstacle_repulsion(const WorldState *world,
                                       const SimParams  *params,
                                       double           *out_fx,
                                       double           *out_fy)
{
    double fx = 0.0;
    double fy = 0.0;

    const double rho     = params->rho;
    const double eta     = params->eta;
    const double rho_obs = rho * 1.5;

    if (rho_obs <= 0.0 || eta <= 0.0) {
        *out_fx = 0.0;
        *out_fy = 0.0;
        return;
    }

    const double x  = world->drone.x;
    const double y  = world->drone.y;
    const double vx = world->drone.vx;
    const double vy = world->drone.vy;

    const double REP_MIN_DIST = 0.100001;

    for (int i = 0; i < world->obstacles_slots; ++i) {
        const Obstacle *obs = &world->obstacles[i];
        if (!obs->active) continue;

        const double dx = obs->x - x;
        const double dy = obs->y - y;

        const double center_dist = sqrt(dx * dx + dy * dy);
        if (center_dist <= 1e-9) continue;

        const double combined_r = obs->radius + DRONE_RADIUS;
        double surface_dist = center_dist - combined_r;

        if (surface_dist < REP_MIN_DIST) surface_dist = REP_MIN_DIST;
        if (surface_dist > rho_obs) continue;

        const double f_mag = repulsive_force(surface_dist, eta, rho_obs, vx, vy);
        if (f_mag <= 0.0) continue;

        const double nx = (x - obs->x) / center_dist;
        const double ny = (y - obs->y) / center_dist;

        fx += f_mag * nx;
        fy += f_mag * ny;
    }

    *out_fx = fx;
    *out_fy = fy;
}

typedef struct {
    double ux;
    double uy;
    const char *name;
} Dir8;

static int map_repulsion_to_virtual_key(double px, double py,
                                        double *out_fx, double *out_fy,
                                        const char **out_dir_name)
{
    if (fabs(px) < 1e-12 && fabs(py) < 1e-12) {
        *out_fx = 0.0;
        *out_fy = 0.0;
        if (out_dir_name) *out_dir_name = "NONE";
        return 0;
    }

    const double inv_sqrt2 = 0.70710678118654752440;

    const Dir8 dirs[8] = {
        {  1.0,        0.0,        "E"  },
        {  inv_sqrt2, -inv_sqrt2,  "SE" },
        {  0.0,       -1.0,        "S"  },
        { -inv_sqrt2, -inv_sqrt2,  "SW" },
        { -1.0,        0.0,        "W"  },
        { -inv_sqrt2,  inv_sqrt2,  "NW" },
        {  0.0,        1.0,        "N"  },
        {  inv_sqrt2,  inv_sqrt2,  "NE" },
    };

    double best_proj = 0.0;
    int best_i = -1;

    for (int i = 0; i < 8; ++i) {
        double proj = px * dirs[i].ux + py * dirs[i].uy;
        if (proj > best_proj) {
            best_proj = proj;
            best_i = i;
        }
    }

    if (best_i < 0 || best_proj <= 0.0) {
        *out_fx = 0.0;
        *out_fy = 0.0;
        if (out_dir_name) *out_dir_name = "NONE";
        return 0;
    }

    *out_fx = best_proj * dirs[best_i].ux;
    *out_fy = best_proj * dirs[best_i].uy;
    if (out_dir_name) *out_dir_name = dirs[best_i].name;
    return 1;
}

static int segment_hits_circle(double x0, double y0,
                               double x1, double y1,
                               double cx, double cy,
                               double r)
{
    double r2 = r * r;

    double dx0 = x0 - cx;
    double dy0 = y0 - cy;
    double dx1 = x1 - cx;
    double dy1 = y1 - cy;

    double dist0_sq = dx0 * dx0 + dy0 * dy0;
    double dist1_sq = dx1 * dx1 + dy1 * dy1;

    if (dist0_sq <= r2 || dist1_sq <= r2) {
        return 1;
    }

    double sx = x1 - x0;
    double sy = y1 - y0;
    double len2 = sx * sx + sy * sy;
    if (len2 <= 1e-9) {
        return 0;
    }

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

    double move_dx = x1 - prev_x;
    double move_dy = y1 - prev_y;
    double move_sq = move_dx * move_dx + move_dy * move_dy;
    if (move_sq < 1e-6) {
        return;
    }

    const double HIT_RADIUS = 1.0;

    for (int i = 0; i < world->targets_slots; ++i) {
        Target *tgt = &world->targets[i];
        if (!tgt->active) continue;

        int hit = segment_hits_circle(prev_x, prev_y, x1, y1, tgt->x, tgt->y, HIT_RADIUS);
        if (!hit) continue;

        // === NEW SCORING FORMULA ===
    
        // 1. Base points for hitting target
        double base_points = 100.0;
        
        // 2. Time bonus (faster = better)
        struct timespec now;
        clock_gettime(CLOCK_REALTIME, &now);
        
        double time_since_last = (now.tv_sec - world->last_target_time.tv_sec) +
                                (now.tv_nsec - world->last_target_time.tv_nsec) / 1e9;
        
        // Award bonus if collected quickly (< 5 seconds)
        double time_bonus = 0.0;
        if (time_since_last < 5.0) {
            time_bonus = 50.0 * (5.0 - time_since_last) / 5.0;  // 0-50 points
        }
        
        // 3. Efficiency penalty (distance traveled)
        // Penalize if traveled far to reach target
        double distance_to_target = sqrt((tgt->x - prev_x) * (tgt->x - prev_x) +
                                        (tgt->y - prev_y) * (tgt->y - prev_y));
        double distance_penalty = distance_to_target * 0.5;  // 0.5 points per unit
        
        // 4. Obstacle collision penalty
        double collision_penalty = world->obstacle_collisions * 10.0;
        
        // 5. Calculate final score for this target
        double target_score = base_points + time_bonus - distance_penalty - collision_penalty;
        if (target_score < 0.0) target_score = 0.0;
        
        world->score += target_score;
        world->targets_collected++;
        world->last_target_time = now;
        
        sim_log_info("bb_server: TARGET HIT idx=%d score=+%.1f (time_bonus=%.1f dist_penalty=%.1f) total=%.1f",
                    i, target_score, time_bonus, distance_penalty, world->score);


        play("../../bin/conf/target.mp3");
        //world->score += 1.0;

        sim_log_info("bb_server: TARGET HIT idx=%d pos=(%.2f,%.2f) score=%.1f",
                     i, tgt->x, tgt->y, world->score);

        double w = (double)params->world_width;
        double h = (double)params->world_height;

        tgt->x = ((double)rand() / (double)RAND_MAX) * w;
        tgt->y = ((double)rand() / (double)RAND_MAX) * h;
        tgt->active = 1;

        sim_log_info("bb_server: TARGET RESPAWN idx=%d new_pos=(%.2f,%.2f)",
                     i, tgt->x, tgt->y);
    }
}

int main(int argc, char *argv[])
{
    sim_log_init("bb_server");
    sim_process_register("bb_server", getpid());

    signal(SIGINT, handle_sigint);
    report_pid_if_requested(argc, argv);
    wd_client_init();

    pid_t music = fork();
    if (music == 0) {
        int fd = open("/dev/null", O_RDWR);
        if (fd >= 0) {
            dup2(fd, STDIN_FILENO);
            dup2(fd, STDOUT_FILENO);
            dup2(fd, STDERR_FILENO);
            if (fd > 2) close(fd);
        }

        execlp("mpg123", "mpg123", "-f", "4098", "--loop", "-1",
               "../../bin/conf/music.mp3", (char *)NULL);
        perror("Music!");
        _exit(1);
    }

    if (sim_params_load(NULL) != 0) {
        sim_log_info("bb_server: could not load '%s', using built-in defaults",
                     SIM_PARAMS_DEFAULT_PATH);
    }

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

    sim_log_info("bb_server: repulsion params rho=%.2f eta=%.2f (DRONE_RADIUS=%.2f)",
                 params->rho, params->eta, (double)DRONE_RADIUS);

    int env_enabled = (params->rho > 0.0 && params->eta > 0.0);
    sim_log_info("bb_server: obstacle repulsion %s (Latombe-style |v|, virtual-key mapping)",
                 env_enabled ? "ENABLED" : "DISABLED");

    srand((unsigned)time(NULL));

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

    sim_log_info("bb_server: pipe FDs: drone_in=%d drone_out=%d input_in=%d obs_in=%d tgt_in=%d",
                 fd_drone_in, fd_drone_out, fd_input_in, fd_obs_in, fd_tgt_in);

    WorldState   world;
    CommandState user_cmd;

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

    clock_gettime(CLOCK_REALTIME, &world.sim_start_time);
    world.last_target_time = world.sim_start_time;
    world.total_distance = 0.0;
    world.targets_collected = 0;
    world.obstacle_collisions = 0;

    double prev_x           = 0.0;
    double prev_y           = 0.0;
    int    have_prev_pos    = 0;
    int    have_drone_state = 0;
    int    have_targets     = 0;

    int input_received  = 0;
    int rep_active_prev = 0;

    ui_init();

    int start_sim = 0;
    while (!start_sim && running) {
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

    sim_log_info("bb_server: after menu loop: start_sim=%d running=%d", start_sim, running);

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

    erase();
    refresh();

    sim_log_info("bb_server: entering main loop");

    int obs_to_read = params->num_obstacles;
    if (obs_to_read > SIM_MAX_OBSTACLES) obs_to_read = SIM_MAX_OBSTACLES;
    if (obs_to_read < 0) obs_to_read = 0;

    int tgt_to_read = params->num_targets;
    if (tgt_to_read > SIM_MAX_TARGETS) tgt_to_read = SIM_MAX_TARGETS;
    if (tgt_to_read < 0) tgt_to_read = 0;

    world.obstacles_slots = obs_to_read;
    world.targets_slots   = tgt_to_read;

    while (running) {
        fd_set readfds;
        FD_ZERO(&readfds);

        FD_SET(fd_drone_in, &readfds);
        FD_SET(fd_input_in, &readfds);
        FD_SET(fd_obs_in,   &readfds);
        FD_SET(fd_tgt_in,   &readfds);

        int maxfd = fd_drone_in;
        if (fd_input_in > maxfd) maxfd = fd_input_in;
        if (fd_obs_in   > maxfd) maxfd = fd_obs_in;
        if (fd_tgt_in   > maxfd) maxfd = fd_tgt_in;

        struct timeval tv;
        tv.tv_sec  = 0;
        tv.tv_usec = 33333;

        int ready = select(maxfd + 1, &readfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            endwin();
            perror("bb_server: select");
            break;
        }

        input_received = 0;

        if (ready > 0) {
            if (FD_ISSET(fd_drone_in, &readfds)) {
                DroneState ds;
                ssize_t r = read_full(fd_drone_in, &ds, sizeof(ds));
                if (r == (ssize_t)sizeof(ds)) {
                    if (!have_prev_pos) {
                        prev_x = ds.x;
                        prev_y = ds.y;
                        have_prev_pos = 1;
                    } else {
                        prev_x = world.drone.x;
                        prev_y = world.drone.y;
                    }
                    world.drone      = ds;
                    have_drone_state = 1;

                    if (have_drone_state && have_prev_pos) {
                        double dx = world.drone.x - prev_x;
                        double dy = world.drone.y - prev_y;
                        double dist = sqrt(dx * dx + dy * dy);
                        world.total_distance += dist;
                    }

                } else if (r == 0) {
                    sim_log_info("bb_server: drone pipe EOF");
                    running = 0;
                } else {
                    endwin();
                    perror("bb_server: read_full(drone)");
                    running = 0;
                }
            }

            if (FD_ISSET(fd_input_in, &readfds)) {
                CommandState cs;
                ssize_t r = read_full(fd_input_in, &cs, sizeof(cs));
                if (r == (ssize_t)sizeof(cs)) {
                    user_cmd       = cs;
                    input_received = 1;

                    if (!env_enabled) {
                        world.cmd = cs;
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
                } else {
                    endwin();
                    perror("bb_server: read_full(input)");
                    running = 0;
                }
            }

            if (FD_ISSET(fd_obs_in, &readfds) && obs_to_read > 0) {
                ssize_t expected = (ssize_t)(obs_to_read * (int)sizeof(Obstacle));
                ssize_t r = read_full(fd_obs_in, world.obstacles,
                                      (size_t)(obs_to_read * (int)sizeof(Obstacle)));
                if (r == expected) {
                    int count = 0;
                    for (int i = 0; i < obs_to_read; ++i) {
                        if (world.obstacles[i].active) ++count;
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

            if (FD_ISSET(fd_tgt_in, &readfds) && tgt_to_read > 0) {
                ssize_t expected = (ssize_t)(tgt_to_read * (int)sizeof(Target));
                ssize_t r = read_full(fd_tgt_in, world.targets,
                                      (size_t)(tgt_to_read * (int)sizeof(Target)));
                if (r == expected) {
                    int count = 0;
                    for (int i = 0; i < tgt_to_read; ++i) {
                        if (world.targets[i].active) ++count;
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

        if (have_prev_pos && have_drone_state && have_targets) {
            handle_targets(&world, params, prev_x, prev_y);
        }

        if (running && env_enabled) {
            double fx_obs  = 0.0, fy_obs  = 0.0;

            compute_obstacle_repulsion(&world, params, &fx_obs, &fy_obs);

            double fx_vk_obs = 0.0, fy_vk_obs = 0.0;
            const char *vk_dir = "NONE";
            int obs_rep_active = map_repulsion_to_virtual_key(fx_obs, fy_obs, &fx_vk_obs, &fy_vk_obs, &vk_dir);

            int rep_active = obs_rep_active;

            if (input_received || rep_active || rep_active_prev) {
                CommandState out_cmd = user_cmd;

                if (rep_active) {
                    out_cmd.fx = user_cmd.fx + fx_vk_obs;
                    out_cmd.fy = user_cmd.fy + fy_vk_obs;
                }

                ssize_t w = write_full(fd_drone_out, &out_cmd, sizeof(out_cmd));
                if (w != (ssize_t)sizeof(out_cmd)) {
                    endwin();
                    perror("bb_server: write_full(drone with repulsion)");
                    running = 0;
                }

                world.cmd = out_cmd;

                if (rep_active && !rep_active_prev) {
                    sim_log_info("bb_server: REP ON obs_vk=%s(%.2f,%.2f)",
                                 vk_dir, fx_vk_obs, fy_vk_obs);
                } else if (!rep_active && rep_active_prev) {
                    sim_log_info("bb_server: REP OFF");
                }

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
