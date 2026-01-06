// Problem: When spawning processes via konsole, the direct child is the
//          konsole process itself, not our actual simulation process.
//          The real process (bb_server/input) is a grandchild with an
//          unknown PID.
//
// Solution: Each konsole-launched process writes its real PID back to
//           master via a dedicated pipe.
//
// Flow:
//   1. Master creates a pipe (pidpipe_bb for bb_server, pidpipe_in for input)
//   2. Master forks and execs konsole, passing pipe write-end as last arg
//   3. Konsole spawns shell, shell spawns actual process
//   4. Actual process writes getpid() to pipe in report_pid_if_requested()
//   5. Master reads real PID from pipe read-end
//   6. Master sends REAL PIDs (not konsole PIDs) to watchdog
//
// This ensures watchdog monitors the actual simulation processes.

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>
#include <libgen.h> 

#include "sim_ipc.h"
#include "sim_params.h"

static void safe_close(int fd)
{
    if (fd >= 0) (void)close(fd);
}

static int read_pid_from_pipe(int rfd, pid_t *out_pid, const char *who)
{
    pid_t p = -1;
    ssize_t r = read_full(rfd, &p, sizeof(p));
    safe_close(rfd);

    if (r != (ssize_t)sizeof(p) || p <= 1) {
        fprintf(stderr, "master: failed to read real pid from %s (r=%zd pid=%d)\n",
                who, r, (int)p);
        return -1;
    }

    *out_pid = p;
    return 0;
}

int main(int argc, char *argv[])
{
    // Dynamic Path Resolution: Ensures relative paths work from any launch directory
    char self_path[1024];
    ssize_t len = readlink("/proc/self/exe", self_path, sizeof(self_path) - 1);
    if (len != -1) {
        self_path[len] = '\0';
        char *dir = dirname(self_path);
        if (chdir(dir) != 0) {
            perror("master: chdir to executable directory failed");
        }
    }

    // we unlink them so the past executions are not saved
    unlink("../../bin/log/processes.log"); 
    unlink("../../bin/log/processes.pid"); 

    sim_log_init("master");
    sim_process_register("master", getpid());
    
    // Load runtime parameters from config file (or fall back to defaults)
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "master: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }

    // 2. [RELAY LOGIC] Check if exe.sh passed a mode number
    int mode_to_use = SIM_MODE_NORMAL;  // default
    if (argc > 1) {
        mode_to_use = atoi(argv[1]);
        sim_params_set_mode(mode_to_use);
        sim_log_info("master: mode set via argument to %d", mode_to_use);
    }

    // CRITICAL FIX: Export mode as environment variable so all children inherit it
    {
        char mode_str[32];
        snprintf(mode_str, sizeof(mode_str), "%d", mode_to_use);
        setenv("SIM_MODE", mode_str, 1);
        sim_log_info("master: exported SIM_MODE=%s to environment", mode_str);
    }

    const SimParams *params = sim_params_get();     // get the parameters

    // Debug output
    fprintf(stderr, "\n=== MASTER DEBUG INFO ===\n");
    fprintf(stderr, "Mode: %d (0=NORMAL, 1=SERVER, 2=CLIENT)\n", params->mode);
    fprintf(stderr, "Server address: %s\n", params->server_address);
    fprintf(stderr, "Server port: %d\n", params->server_port);
    fprintf(stderr, "World: %dx%d\n", params->world_width, params->world_height);
    fprintf(stderr, "=========================\n\n");


    int pipe_drone_cmd[2];          // bb_server -> drone (CommandState)
    int pipe_drone_state[2];        // drone -> bb_server (DroneState)
    int pipe_input_cmd[2];          // input -> bb_server (CommandState)
    int pipe_obstacles[2];          // obstacles -> bb_server (Obstacle[])
    int pipe_targets[2];            // targets   -> bb_server (Target[])
    int pipe_obstacles_drone[2];    // obstacles -> drone (Obstacle[])
    int pipe_watchdog[2];           // master -> watchdog (pid list)

    // new pipe declarations for third assignment
    int pipe_network_drone_in[2];      // bb_server -> network process (drone position)
    int pipe_network_obstacle_in[2];   // network -> bb_server (client drone as obstacle - SERVER mode)
    int pipe_network_server_drone[2];  // network -> bb_server (server drone position - CLIENT mode)
    int pipe_network_window_size[2];   // network -> bb_server (window dimensions - CLIENT mode)


    // Create PID report pipes for konsole-launched processes
    int pidpipe_bb[2]; // bb_server -> master PID reporting
    int pidpipe_in[2]; // input -> master PID reporting

    if (pipe(pipe_drone_cmd) == -1) { perror("master: pipe_drone_cmd"); return EXIT_FAILURE; }
    if (pipe(pipe_drone_state) == -1) { perror("master: pipe_drone_state"); return EXIT_FAILURE; }
    if (pipe(pipe_input_cmd) == -1) { perror("master: pipe_input_cmd"); return EXIT_FAILURE; }
    if (pipe(pipe_obstacles) == -1) { perror("master: pipe_obstacles"); return EXIT_FAILURE; }
    if (pipe(pipe_targets) == -1) { perror("master: pipe_targets"); return EXIT_FAILURE; }
    if (pipe(pipe_obstacles_drone) == -1) { perror("master: pipe_obstacles_drone"); return EXIT_FAILURE; }
    if (pipe(pipe_watchdog) == -1) { perror("master: pipe_watchdog"); return EXIT_FAILURE; }

    if (pipe(pidpipe_bb) == -1) { perror("master: pidpipe_bb"); return EXIT_FAILURE; }
    if (pipe(pidpipe_in) == -1) { perror("master: pidpipe_in"); return EXIT_FAILURE; }

    // now we create the pipes that we added
    if (pipe(pipe_network_drone_in) == -1) { perror("master: pipe_network_drone_in"); return EXIT_FAILURE; }
    if (pipe(pipe_network_obstacle_in) == -1) { perror("master: pipe_network_obstacle_in"); return EXIT_FAILURE; }
    if (pipe(pipe_network_server_drone) == -1) { perror("master: pipe_network_server_drone"); return EXIT_FAILURE; }
    if (pipe(pipe_network_window_size) == -1) { perror("master: pipe_network_window_size"); return EXIT_FAILURE; }

    // THE WATCHDOG SHOULD BE SPAWNED IF THE MODE IS NORMAL!!!

    pid_t wd_pid = -1; // lets initialize it to -1

    if (params->mode == SIM_MODE_NORMAL) {
        // Start WATCHDOG first (blocks reading until master writes pid list)
        wd_pid = fork();        // here we give it its actual value
        if (wd_pid < 0) {
            perror("master: fork watchdog");
            return EXIT_FAILURE;
        }
        if (wd_pid == 0) {
            // Child: watchdog keeps pipe_watchdog[0]
            safe_close(pipe_watchdog[1]);

            // Close everything else
            safe_close(pipe_drone_cmd[0]); safe_close(pipe_drone_cmd[1]);
            safe_close(pipe_drone_state[0]); safe_close(pipe_drone_state[1]);
            safe_close(pipe_input_cmd[0]); safe_close(pipe_input_cmd[1]);
            safe_close(pipe_obstacles[0]); safe_close(pipe_obstacles[1]);
            safe_close(pipe_targets[0]); safe_close(pipe_targets[1]);
            safe_close(pipe_obstacles_drone[0]); safe_close(pipe_obstacles_drone[1]);
            safe_close(pidpipe_bb[0]); safe_close(pidpipe_bb[1]);
            safe_close(pidpipe_in[0]); safe_close(pidpipe_in[1]);

            char fd_read[16];
            snprintf(fd_read, sizeof(fd_read), "%d", pipe_watchdog[0]);
            execl("./watchdog", "./watchdog", fd_read, (char *)NULL);
            perror("master: exec watchdog");
            _exit(EXIT_FAILURE);
        }
        else{
            // Parent: will write pid list later
            safe_close(pipe_watchdog[0]);
            //safe_close(pipe_watchdog[1]);
        }
        

        // Export watchdog PID to all children (they inherit env)
        {
            char wd_pid_str[32];
            snprintf(wd_pid_str, sizeof(wd_pid_str), "%d", (int)wd_pid);
            setenv("SIM_WD_PID", wd_pid_str, 1);
        }
    }

    // bb_server (konsole) + PID handshake
    pid_t bb_konsole_pid = fork();
    if (bb_konsole_pid < 0) {
        perror("master: fork bb_server");
        return EXIT_FAILURE;
    }
    if (bb_konsole_pid == 0) {
        // Child that will exec konsole (or bb_server fallback)

        // bb_server needs:
        //   pipe_drone_state[0], pipe_drone_cmd[1], pipe_input_cmd[0], pipe_obstacles[0], pipe_targets[0]
        close(pipe_drone_state[1]);
        close(pipe_drone_cmd[0]);
        close(pipe_input_cmd[1]);

        // mode check
        if (params->mode == SIM_MODE_NORMAL) {
            // Normal mode: bb_server needs obstacle/target pipes
            close(pipe_obstacles[1]);
            close(pipe_targets[1]);
            
            // Close unused network pipes
            close(pipe_network_drone_in[0]); close(pipe_network_drone_in[1]);
            close(pipe_network_obstacle_in[0]); close(pipe_network_obstacle_in[1]);
            close(pipe_network_server_drone[0]); close(pipe_network_server_drone[1]);
            close(pipe_network_window_size[0]); close(pipe_network_window_size[1]);
        }
        else if (params->mode == SIM_MODE_SERVER) {
            // Server mode: need network pipes, no obstacle/target pipes
            close(pipe_obstacles[0]); close(pipe_obstacles[1]);
            close(pipe_targets[0]); close(pipe_targets[1]);
            
            close(pipe_network_drone_in[0]);      // bb_server writes here
            close(pipe_network_obstacle_in[1]);   // bb_server reads here
            
            // Close unused client-specific pipes
            close(pipe_network_server_drone[0]); close(pipe_network_server_drone[1]);
            close(pipe_network_window_size[0]); close(pipe_network_window_size[1]);
        }
        else if (params->mode == SIM_MODE_CLIENT) {
            // Client mode: need network pipes, no obstacle/target pipes
            close(pipe_obstacles[0]); close(pipe_obstacles[1]);
            close(pipe_targets[0]); close(pipe_targets[1]);
            
            close(pipe_network_drone_in[0]);        // bb_server writes here
            close(pipe_network_server_drone[1]);    // bb_server reads here
            close(pipe_network_window_size[1]);     // bb_server reads here
            
            // Close unused server-specific pipes
            close(pipe_network_obstacle_in[0]); close(pipe_network_obstacle_in[1]);
        }

        // not used here
        close(pipe_obstacles_drone[0]);
        close(pipe_obstacles_drone[1]);
        // watchdog pipe not used by bb_server
        close(pipe_watchdog[1]);
        // PID pipe: keep write-end only
        close(pidpipe_bb[0]);
        // close other pid pipe fully
        close(pidpipe_in[0]); close(pidpipe_in[1]);

        // we modify the exec calls to make them aware about the mode
        char fd_drone_state_in[16], fd_drone_cmd_out[16], fd_input_cmd_in[16];
        char fd_obs_in[16], fd_tgt_in[16], fd_pid_report[16];
        char fd_net_drone_out[16], fd_net_obs_in[16];
        char fd_net_server_drone_in[16], fd_net_window_size_in[16];

        snprintf(fd_drone_state_in, sizeof(fd_drone_state_in), "%d", pipe_drone_state[0]);
        snprintf(fd_drone_cmd_out,  sizeof(fd_drone_cmd_out),  "%d", pipe_drone_cmd[1]);
        snprintf(fd_input_cmd_in,   sizeof(fd_input_cmd_in),   "%d", pipe_input_cmd[0]);
        snprintf(fd_pid_report,     sizeof(fd_pid_report),     "%d", pidpipe_bb[1]);

        if (params->mode == SIM_MODE_NORMAL) {
            snprintf(fd_obs_in, sizeof(fd_obs_in), "%d", pipe_obstacles[0]);
            snprintf(fd_tgt_in, sizeof(fd_tgt_in), "%d", pipe_targets[0]);

            execlp("konsole", "konsole",
                   "-T", "BB_SERVER",
                   "-e", "./bb_server",
                   fd_drone_state_in, fd_drone_cmd_out, fd_input_cmd_in,
                   fd_obs_in, fd_tgt_in, fd_pid_report,
                   (char *)NULL);

            execl("./bb_server", "./bb_server",
                  fd_drone_state_in, fd_drone_cmd_out, fd_input_cmd_in,
                  fd_obs_in, fd_tgt_in, fd_pid_report,
                  (char *)NULL);
        }
        else if (params->mode == SIM_MODE_SERVER) {
            snprintf(fd_net_drone_out, sizeof(fd_net_drone_out), "%d", pipe_network_drone_in[1]);
            snprintf(fd_net_obs_in, sizeof(fd_net_obs_in), "%d", pipe_network_obstacle_in[0]);

            execlp("konsole", "konsole",
                   "-T", "BB_SERVER (SERVER MODE)",
                   "-e", "./bb_server",
                   fd_drone_state_in, fd_drone_cmd_out, fd_input_cmd_in,
                   fd_net_drone_out, fd_net_obs_in, fd_pid_report,
                   (char *)NULL);

            execl("./bb_server", "./bb_server",
                  fd_drone_state_in, fd_drone_cmd_out, fd_input_cmd_in,
                  fd_net_drone_out, fd_net_obs_in, fd_pid_report,
                  (char *)NULL);
        }
        else if (params->mode == SIM_MODE_CLIENT) {
            snprintf(fd_net_drone_out, sizeof(fd_net_drone_out), "%d", pipe_network_drone_in[1]);
            snprintf(fd_net_server_drone_in, sizeof(fd_net_server_drone_in), "%d", pipe_network_server_drone[0]);
            snprintf(fd_net_window_size_in, sizeof(fd_net_window_size_in), "%d", pipe_network_window_size[0]);

            execlp("konsole", "konsole",
                   "-T", "BB_SERVER (CLIENT MODE)",
                   "-e", "./bb_server",
                   fd_drone_state_in, fd_drone_cmd_out, fd_input_cmd_in,
                   fd_net_drone_out, fd_net_server_drone_in, fd_net_window_size_in,
                   fd_pid_report,
                   (char *)NULL);

            execl("./bb_server", "./bb_server",
                  fd_drone_state_in, fd_drone_cmd_out, fd_input_cmd_in,
                  fd_net_drone_out, fd_net_server_drone_in, fd_net_window_size_in,
                  fd_pid_report,
                  (char *)NULL);
        }

        perror("master: exec bb_server");
        _exit(EXIT_FAILURE);
    }

    // Parent: read real bb_server pid from pidpipe_bb
    safe_close(pidpipe_bb[1]); // close write end in parent
    pid_t bb_real_pid = -1;
    if (read_pid_from_pipe(pidpipe_bb[0], &bb_real_pid, "bb_server") != 0) {
        // If we can't get the real pid, watchdog will be wrong -> bail hard.
        (void)kill(wd_pid, SIGINT);
        return EXIT_FAILURE;
    }


    // input (konsole) + PID handshake
    pid_t input_konsole_pid = fork();
    if (input_konsole_pid < 0) {
        perror("master: fork input");
        return EXIT_FAILURE;
    }
    if (input_konsole_pid == 0) {
        close(pipe_input_cmd[0]);

        // not used by input
        close(pipe_drone_cmd[0]); close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]); close(pipe_drone_state[1]);
        close(pipe_obstacles[0]); close(pipe_obstacles[1]);
        close(pipe_targets[0]); close(pipe_targets[1]);
        close(pipe_obstacles_drone[0]); close(pipe_obstacles_drone[1]);
        close(pipe_watchdog[1]);

        // PID pipe: keep write-end only
        close(pidpipe_in[0]);
        // close other pid pipe fully
        close(pidpipe_bb[0]); close(pidpipe_bb[1]);

        char fd_cmd_out[16];
        char fd_pid_report[16];
        snprintf(fd_cmd_out,     sizeof(fd_cmd_out),     "%d", pipe_input_cmd[1]);
        snprintf(fd_pid_report,  sizeof(fd_pid_report),  "%d", pidpipe_in[1]);

        execlp("konsole", "konsole",
               "-T", "INPUT",
               "-e", "./input",
               fd_cmd_out,
               fd_pid_report,          // <-- last arg = pid report fd
               (char *)NULL);

        execl("./input", "./input",
              fd_cmd_out,
              fd_pid_report,
              (char *)NULL);

        perror("master: exec input");
        _exit(EXIT_FAILURE);
    }

    // Parent: read real input pid from pidpipe_in
    safe_close(pidpipe_in[1]);
    pid_t input_real_pid = -1;
    if (read_pid_from_pipe(pidpipe_in[0], &input_real_pid, "input") != 0) {
        (void)kill(wd_pid, SIGINT);
        return EXIT_FAILURE;
    }


    // Drone (direct child, real pid)
    pid_t drone_pid = fork();
    if (drone_pid < 0) {
        perror("master: fork drone");
        return EXIT_FAILURE;
    }
    if (drone_pid == 0) {
        // Keep these pipes open
        close(pipe_drone_cmd[1]);       // keep READ end
        close(pipe_drone_state[0]);     // keep WRITE end
        
        // Close all unused pipes
        close(pipe_input_cmd[0]); 
        close(pipe_input_cmd[1]);
        close(pipe_obstacles[0]); 
        close(pipe_obstacles[1]);
        close(pipe_targets[0]); 
        close(pipe_targets[1]);
        close(pipe_obstacles_drone[1]);  // keep READ end for obstacles
        close(pipe_watchdog[1]);
        
        // Close PID report pipes
        close(pidpipe_bb[0]); 
        close(pidpipe_bb[1]);
        close(pidpipe_in[0]); 
        close(pidpipe_in[1]);
        
        // Close network pipes (all modes)
        close(pipe_network_drone_in[0]); 
        close(pipe_network_drone_in[1]);
        close(pipe_network_obstacle_in[0]); 
        close(pipe_network_obstacle_in[1]);
        close(pipe_network_server_drone[0]); 
        close(pipe_network_server_drone[1]);
        close(pipe_network_window_size[0]); 
        close(pipe_network_window_size[1]);

        // Prepare arguments for drone
        char fd_cmd_in[16], fd_state_out[16], fd_obs_in[16];
        snprintf(fd_cmd_in,    sizeof(fd_cmd_in),    "%d", pipe_drone_cmd[0]);
        snprintf(fd_state_out, sizeof(fd_state_out), "%d", pipe_drone_state[1]);
        snprintf(fd_obs_in,    sizeof(fd_obs_in),    "%d", pipe_obstacles_drone[0]);

        // Execute drone
        execl("./drone", "./drone", fd_cmd_in, fd_state_out, fd_obs_in, (char *)NULL);
        
        // If execl returns, it failed
        perror("master: exec drone");
        _exit(EXIT_FAILURE);
    }

    // Obstacles (direct child). We check if the mode
    pid_t obstacles_pid = -1;
    if (params->mode == SIM_MODE_NORMAL) {
        
        obstacles_pid = fork();

        if (obstacles_pid < 0) {
            perror("master: fork obstacles");
            return EXIT_FAILURE;
        }
        if (obstacles_pid == 0) {
            // keep: pipe_obstacles[1], pipe_obstacles_drone[1]
            close(pipe_obstacles[0]);
            close(pipe_obstacles_drone[0]);

            close(pipe_drone_cmd[0]); close(pipe_drone_cmd[1]);
            close(pipe_drone_state[0]); close(pipe_drone_state[1]);
            close(pipe_input_cmd[0]); close(pipe_input_cmd[1]);
            close(pipe_targets[0]); close(pipe_targets[1]);
            close(pipe_watchdog[1]);

            // close pidpipes
            close(pidpipe_bb[0]); close(pidpipe_bb[1]);
            close(pidpipe_in[0]); close(pidpipe_in[1]);

            char fd_obs_out_bb[16], fd_obs_out_drone[16];
            snprintf(fd_obs_out_bb,    sizeof(fd_obs_out_bb),    "%d", pipe_obstacles[1]);
            snprintf(fd_obs_out_drone, sizeof(fd_obs_out_drone), "%d", pipe_obstacles_drone[1]);

            execl("./obstacles", "./obstacles", fd_obs_out_bb, fd_obs_out_drone, (char *)NULL);
            perror("master: exec obstacles");
            _exit(EXIT_FAILURE);
        }
    } else{
        // Network modes: no obstacles process
        close(pipe_obstacles[0]); close(pipe_obstacles[1]);
        close(pipe_obstacles_drone[0]); close(pipe_obstacles_drone[1]);
    }



    // Targets (direct child)
    pid_t targets_pid = -1;
    if (params->mode == SIM_MODE_NORMAL) {

        targets_pid = fork();

        if (targets_pid < 0) {
            perror("master: fork targets");
            return EXIT_FAILURE;
        }
        if (targets_pid == 0) {
            close(pipe_targets[0]);

            close(pipe_drone_cmd[0]); close(pipe_drone_cmd[1]);
            close(pipe_drone_state[0]); close(pipe_drone_state[1]);
            close(pipe_input_cmd[0]); close(pipe_input_cmd[1]);
            close(pipe_obstacles[0]); close(pipe_obstacles[1]);
            close(pipe_obstacles_drone[0]); close(pipe_obstacles_drone[1]);

            close(pipe_watchdog[1]);

            // close pidpipes
            close(pidpipe_bb[0]); close(pidpipe_bb[1]);
            close(pidpipe_in[0]); close(pidpipe_in[1]);

            char fd_tgt_out[16];
            snprintf(fd_tgt_out, sizeof(fd_tgt_out), "%d", pipe_targets[1]);

            execl("./targets", "./targets", fd_tgt_out, (char *)NULL);
            perror("master: exec targets");
            _exit(EXIT_FAILURE);
        }
    }
    else {
        // Network modes: no targets process
        close(pipe_targets[0]); close(pipe_targets[1]);
    }

    // we add network proces spawning
    pid_t network_pid = -1;
    if (params->mode == SIM_MODE_SERVER) {
        network_pid = fork();
        if (network_pid < 0) {
            perror("master: fork network_server");
            return EXIT_FAILURE;
        }
        if (network_pid == 0) {
            // Child: network_server
            close(pipe_network_drone_in[1]);      // keep read end
            close(pipe_network_obstacle_in[0]);   // keep write end

            // Close all other pipes
            close(pipe_drone_cmd[0]); close(pipe_drone_cmd[1]);
            close(pipe_drone_state[0]); close(pipe_drone_state[1]);
            close(pipe_input_cmd[0]); close(pipe_input_cmd[1]);
            close(pipe_obstacles[0]); close(pipe_obstacles[1]);
            close(pipe_targets[0]); close(pipe_targets[1]);
            close(pipe_obstacles_drone[0]); close(pipe_obstacles_drone[1]);
            close(pipe_watchdog[1]);
            close(pidpipe_bb[0]); close(pidpipe_bb[1]);
            close(pidpipe_in[0]); close(pidpipe_in[1]);
            close(pipe_network_server_drone[0]); close(pipe_network_server_drone[1]);
            close(pipe_network_window_size[0]); close(pipe_network_window_size[1]);

            char fd_drone_in[16], fd_obstacle_out[16];
            snprintf(fd_drone_in, sizeof(fd_drone_in), "%d", pipe_network_drone_in[0]);
            snprintf(fd_obstacle_out, sizeof(fd_obstacle_out), "%d", pipe_network_obstacle_in[1]);

            execl("./network_server", "./network_server", fd_drone_in, fd_obstacle_out, (char *)NULL);
            perror("master: exec network_server");
            _exit(EXIT_FAILURE);
        }
        
        // Parent: close unused ends
        close(pipe_network_drone_in[0]);
        close(pipe_network_obstacle_in[1]);
    }
    else if (params->mode == SIM_MODE_CLIENT) {
        network_pid = fork();
        if (network_pid < 0) {
            perror("master: fork network_client");
            return EXIT_FAILURE;
        }
        if (network_pid == 0) {
            // Child: network_client
            close(pipe_network_drone_in[1]);        // keep read end
            close(pipe_network_server_drone[0]);    // keep write end
            close(pipe_network_window_size[0]);     // keep write end

            // Close all other pipes
            close(pipe_drone_cmd[0]); close(pipe_drone_cmd[1]);
            close(pipe_drone_state[0]); close(pipe_drone_state[1]);
            close(pipe_input_cmd[0]); close(pipe_input_cmd[1]);
            close(pipe_obstacles[0]); close(pipe_obstacles[1]);
            close(pipe_targets[0]); close(pipe_targets[1]);
            close(pipe_obstacles_drone[0]); close(pipe_obstacles_drone[1]);
            close(pipe_watchdog[1]);
            close(pidpipe_bb[0]); close(pidpipe_bb[1]);
            close(pidpipe_in[0]); close(pidpipe_in[1]);
            close(pipe_network_obstacle_in[0]); close(pipe_network_obstacle_in[1]);

            char fd_drone_in[16], fd_server_drone_out[16], fd_window_size_out[16];
            snprintf(fd_drone_in, sizeof(fd_drone_in), "%d", pipe_network_drone_in[0]);
            snprintf(fd_server_drone_out, sizeof(fd_server_drone_out), "%d", pipe_network_server_drone[1]);
            snprintf(fd_window_size_out, sizeof(fd_window_size_out), "%d", pipe_network_window_size[1]);

            execl("./network_client", "./network_client",
                  fd_drone_in, fd_server_drone_out, fd_window_size_out,
                  (char *)NULL);
            perror("master: exec network_client");
            _exit(EXIT_FAILURE);
        }
        
        // Parent: close unused ends
        close(pipe_network_drone_in[0]);
        close(pipe_network_server_drone[1]);
        close(pipe_network_window_size[1]);
    }
    else {
        // Normal mode: close all network pipes
        close(pipe_network_drone_in[0]); close(pipe_network_drone_in[1]);
        close(pipe_network_obstacle_in[0]); close(pipe_network_obstacle_in[1]);
        close(pipe_network_server_drone[0]); close(pipe_network_server_drone[1]);
        close(pipe_network_window_size[0]); close(pipe_network_window_size[1]);
    }



    if (params->mode == SIM_MODE_NORMAL){
        // Send PID list to watchdog (REAL PIDs for konsole-launched procs)
        pid_t pids[16];
        int n = 0;

        pids[n++] = bb_real_pid;
        pids[n++] = input_real_pid;
        pids[n++] = drone_pid;
        pids[n++] = obstacles_pid;
        pids[n++] = targets_pid;

        if (write_full(pipe_watchdog[1], &n, sizeof(n)) != (ssize_t)sizeof(n)) {
            perror("master: write pid count to watchdog");
        } else {
            size_t bytes = (size_t)(n * (int)sizeof(pid_t));
            if (write_full(pipe_watchdog[1], pids, bytes) != (ssize_t)bytes) {
                perror("master: write pid list to watchdog");
            }
        }
        safe_close(pipe_watchdog[1]);
    }


    // Close unused pipes in master
    safe_close(pipe_drone_cmd[0]);   safe_close(pipe_drone_cmd[1]);
    safe_close(pipe_drone_state[0]); safe_close(pipe_drone_state[1]);
    safe_close(pipe_input_cmd[0]);   safe_close(pipe_input_cmd[1]);
    safe_close(pipe_obstacles[0]);   safe_close(pipe_obstacles[1]);
    safe_close(pipe_targets[0]);     safe_close(pipe_targets[1]);
    safe_close(pipe_obstacles_drone[0]); safe_close(pipe_obstacles_drone[1]);

    // The correct processes for each mode will be waited

    // Wait for direct children
    // NOTE: bb_server/input real pids are NOT our children when using konsole.
    // We must wait for the konsole processes we forked.
    int status;
    
    // Always wait for drone
    sim_log_info("master: waiting for drone to exit");
    (void)waitpid(drone_pid, &status, 0);
    
    if (params->mode == SIM_MODE_NORMAL) {
        sim_log_info("master: waiting for obstacles and targets to exit");
        (void)waitpid(obstacles_pid, &status, 0);
        (void)waitpid(targets_pid, &status, 0);
    } else {
        // Network modes: wait for network process
        if (network_pid > 0) {
            sim_log_info("master: waiting for network process to exit");
            (void)waitpid(network_pid, &status, 0);
        }
    }

    (void)waitpid(input_konsole_pid, &status, 0);
    (void)waitpid(bb_konsole_pid, &status, 0);

    // Stop watchdog if running
    if (wd_pid > 1) {
        (void)kill(wd_pid, SIGINT);
        (void)waitpid(wd_pid, &status, 0);
    }

    return EXIT_SUCCESS;
}