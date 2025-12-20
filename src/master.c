#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <string.h>

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

int main(void)
{
    // Load runtime parameters from config file (or fall back to defaults)
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "master: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }

    int pipe_drone_cmd[2];          // bb_server -> drone (CommandState)
    int pipe_drone_state[2];        // drone -> bb_server (DroneState)
    int pipe_input_cmd[2];          // input -> bb_server (CommandState)
    int pipe_obstacles[2];          // obstacles -> bb_server (Obstacle[])
    int pipe_targets[2];            // targets   -> bb_server (Target[])
    int pipe_obstacles_drone[2];    // obstacles -> drone (Obstacle[])
    int pipe_watchdog[2];           // master -> watchdog (pid list)

    // PID-report pipes (child writes pid to parent)
    int pidpipe_bb[2];
    int pidpipe_in[2];

    if (pipe(pipe_drone_cmd) == -1) { perror("master: pipe_drone_cmd"); return EXIT_FAILURE; }
    if (pipe(pipe_drone_state) == -1) { perror("master: pipe_drone_state"); return EXIT_FAILURE; }
    if (pipe(pipe_input_cmd) == -1) { perror("master: pipe_input_cmd"); return EXIT_FAILURE; }
    if (pipe(pipe_obstacles) == -1) { perror("master: pipe_obstacles"); return EXIT_FAILURE; }
    if (pipe(pipe_targets) == -1) { perror("master: pipe_targets"); return EXIT_FAILURE; }
    if (pipe(pipe_obstacles_drone) == -1) { perror("master: pipe_obstacles_drone"); return EXIT_FAILURE; }
    if (pipe(pipe_watchdog) == -1) { perror("master: pipe_watchdog"); return EXIT_FAILURE; }

    if (pipe(pidpipe_bb) == -1) { perror("master: pidpipe_bb"); return EXIT_FAILURE; }
    if (pipe(pidpipe_in) == -1) { perror("master: pidpipe_in"); return EXIT_FAILURE; }

    // ----------------------------
    // Start WATCHDOG first (blocks reading until master writes pid list)
    // ----------------------------
    pid_t wd_pid = fork();
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

    // Parent: will write pid list later
    safe_close(pipe_watchdog[0]);

    // Export watchdog PID to all children (they inherit env)
    {
        char wd_pid_str[32];
        snprintf(wd_pid_str, sizeof(wd_pid_str), "%d", (int)wd_pid);
        setenv("SIM_WD_PID", wd_pid_str, 1);
    }

    // ----------------------------
    // bb_server (konsole) + PID handshake
    // ----------------------------
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
        close(pipe_obstacles[1]);
        close(pipe_targets[1]);

        // not used here
        close(pipe_obstacles_drone[0]);
        close(pipe_obstacles_drone[1]);

        // watchdog pipe not used by bb_server
        close(pipe_watchdog[1]);

        // PID pipe: keep write-end only
        close(pidpipe_bb[0]);
        // close other pid pipe fully
        close(pidpipe_in[0]); close(pidpipe_in[1]);

        char fd_drone_state_in[16], fd_drone_cmd_out[16], fd_input_cmd_in[16], fd_obs_in[16], fd_tgt_in[16];
        char fd_pid_report[16];

        snprintf(fd_drone_state_in, sizeof(fd_drone_state_in), "%d", pipe_drone_state[0]);
        snprintf(fd_drone_cmd_out,  sizeof(fd_drone_cmd_out),  "%d", pipe_drone_cmd[1]);
        snprintf(fd_input_cmd_in,   sizeof(fd_input_cmd_in),   "%d", pipe_input_cmd[0]);
        snprintf(fd_obs_in,         sizeof(fd_obs_in),         "%d", pipe_obstacles[0]);
        snprintf(fd_tgt_in,         sizeof(fd_tgt_in),         "%d", pipe_targets[0]);
        snprintf(fd_pid_report,     sizeof(fd_pid_report),     "%d", pidpipe_bb[1]);

        execlp("konsole", "konsole",
               "-T", "BB_SERVER",
               "-e", "./bb_server",
               fd_drone_state_in,
               fd_drone_cmd_out,
               fd_input_cmd_in,
               fd_obs_in,
               fd_tgt_in,
               fd_pid_report,            // <-- last arg = pid report fd
               (char *)NULL);

        // Fallback if konsole not available
        execl("./bb_server", "./bb_server",
              fd_drone_state_in,
              fd_drone_cmd_out,
              fd_input_cmd_in,
              fd_obs_in,
              fd_tgt_in,
              fd_pid_report,             // <-- keep same last arg
              (char *)NULL);

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

    // ----------------------------
    // input (konsole) + PID handshake
    // ----------------------------
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

    // ----------------------------
    // Drone (direct child, real pid)
    // ----------------------------
    pid_t drone_pid = fork();
    if (drone_pid < 0) {
        perror("master: fork drone");
        return EXIT_FAILURE;
    }
    if (drone_pid == 0) {
        // keep: pipe_drone_cmd[0], pipe_drone_state[1], pipe_obstacles_drone[0]
        close(pipe_drone_cmd[1]);
        close(pipe_drone_state[0]);

        close(pipe_input_cmd[0]); close(pipe_input_cmd[1]);
        close(pipe_obstacles[0]); close(pipe_obstacles[1]);
        close(pipe_targets[0]); close(pipe_targets[1]);

        close(pipe_obstacles_drone[1]); // keep read end
        close(pipe_watchdog[1]);

        // close pidpipes
        close(pidpipe_bb[0]); close(pidpipe_bb[1]);
        close(pidpipe_in[0]); close(pidpipe_in[1]);

        char fd_cmd_in[16], fd_state_out[16], fd_obs_in[16];
        snprintf(fd_cmd_in,    sizeof(fd_cmd_in),    "%d", pipe_drone_cmd[0]);
        snprintf(fd_state_out, sizeof(fd_state_out), "%d", pipe_drone_state[1]);
        snprintf(fd_obs_in,    sizeof(fd_obs_in),    "%d", pipe_obstacles_drone[0]);

        execl("./drone", "./drone", fd_cmd_in, fd_state_out, fd_obs_in, (char *)NULL);
        perror("master: exec drone");
        _exit(EXIT_FAILURE);
    }

    // ----------------------------
    // Obstacles (direct child)
    // ----------------------------
    pid_t obstacles_pid = fork();
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

    // ----------------------------
    // Targets (direct child)
    // ----------------------------
    pid_t targets_pid = fork();
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

    // ----------------------------
    // Send PID list to watchdog (REAL PIDs for konsole-launched procs)
    // ----------------------------
    {
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

    // ----------------------------
    // Close unused pipes in master
    // ----------------------------
    safe_close(pipe_drone_cmd[0]);   safe_close(pipe_drone_cmd[1]);
    safe_close(pipe_drone_state[0]); safe_close(pipe_drone_state[1]);
    safe_close(pipe_input_cmd[0]);   safe_close(pipe_input_cmd[1]);
    safe_close(pipe_obstacles[0]);   safe_close(pipe_obstacles[1]);
    safe_close(pipe_targets[0]);     safe_close(pipe_targets[1]);
    safe_close(pipe_obstacles_drone[0]); safe_close(pipe_obstacles_drone[1]);

    // pidpipes already handled; watchdog pipe closed after write

    // ----------------------------
    // Wait for direct children
    // NOTE: bb_server/input real pids are NOT our children when using konsole.
    // We must wait for the konsole processes we forked.
    // ----------------------------
    int status;
    (void)waitpid(drone_pid, &status, 0);
    (void)waitpid(obstacles_pid, &status, 0);
    (void)waitpid(targets_pid, &status, 0);

    (void)waitpid(input_konsole_pid, &status, 0);
    (void)waitpid(bb_konsole_pid, &status, 0);

    // stop watchdog if still alive
    (void)kill(wd_pid, SIGINT);
    (void)waitpid(wd_pid, &status, 0);

    return EXIT_SUCCESS;
}
