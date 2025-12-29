// The watchdog monitors all simulation processes for failures and hangs.
// It uses a signal-based ping-ack protocol to detect unresponsive processes.
//
// Architecture:
//   1. Master spawns watchdog FIRST (before any monitored processes)
//   2. Watchdog blocks reading PID list from pipe (ensures ready state)
//   3. Watchdog enters periodic health check loop:
//      - Send SIGUSR1 ping to each process
//      - Wait up to WD_ACK_TIMEOUT_MS for SIGUSR2 ack
//      - If no ack or process dead -> kill all and exit
//
// Failure Detection:
//   - Process death: kill(pid, 0) returns ESRCH
//   - Hung process: No SIGUSR2 ack within timeout
//   - Signal delivery failure: kill() returns error
//
// Recovery Strategy:
//   - Fail-fast: Any failure triggers immediate shutdown
//   - Send SIGKILL to all monitored processes
//   - No automatic restart (manual intervention required)


#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>

#include "sim_ipc.h"   // read_full / write_full
#include "sim_log.h"

#ifndef WD_PING_SIGNAL
#define WD_PING_SIGNAL SIGUSR1
#endif

#ifndef WD_ACK_SIGNAL
#define WD_ACK_SIGNAL  SIGUSR2
#endif

#ifndef WD_POLL_PERIOD_MS
#define WD_POLL_PERIOD_MS 300   // how often to check each process
#endif

#ifndef WD_ACK_TIMEOUT_MS
#define WD_ACK_TIMEOUT_MS 200   // max wait for ack per process
#endif

#ifndef WD_MAX_PROCS
#define WD_MAX_PROCS 16
#endif

// global state tracking for signal handler (must be volatile sig_atomic_t)
static volatile sig_atomic_t g_running = 1;                 // the control flag for the main loop
static volatile sig_atomic_t g_acked[WD_MAX_PROCS];         // ack-flags for each process
static volatile sig_atomic_t g_acked_code_area[WD_MAX_PROCS];       // the location of the code where the ack was sent

// the pids that are being monitored
static pid_t  g_pids[WD_MAX_PROCS];
static int    g_npids = 0;

// signal handler for ctrl+c
static void handle_sigint(int sig)
{
    (void)sig;
    g_running = 0;  // allows the main loop to exit gracefully
}

// maps the pid to its index
static int find_pid_index(pid_t pid)
{
    for (int i = 0; i < g_npids; ++i) {
        if (g_pids[i] == pid) return i;
    }
    return -1;  // if non is found it returns -1
}

// siguser2 handler -> receives acknowledgments from monitored processes
// called asynchronously when a process responds to our siguser1 ping
static void ack_handler(int sig, siginfo_t *info, void *ctx)
{
    (void)sig;
    (void)ctx;

    if (!info) return;

    pid_t sender = info->si_pid; // extract sender pid

    // find which of the processes that are monitored sent the data
    int idx = find_pid_index(sender);
    if (idx >= 0) {
        g_acked[idx] = 1;   // we say that this process has ack-ed the ping 
        g_acked_code_area[idx] = info->si_value.sival_int;  // we store the code area 
    }
}

// very precise sleep
static void sleep_ms(long ms)
{
    if (ms <= 0) return;
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;

    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        // loop ends only it is interrupted by a signal, or if the time runs out
    }
}

//kills all the processes that are being monitored. used when we detecct a failure in one of the processes
static void kill_all_monitored(void)
{
    for (int i = 0; i < g_npids; ++i) {
        if (g_pids[i] > 1) {
            (void)kill(g_pids[i], SIGKILL);
        }
    }
}

static int pid_is_alive(pid_t pid)
{
    if (pid <= 1) return 0;

    // kill(pid,0) checks existence/permission without sending a signal
    if (kill(pid, 0) == 0) return 1;
    return (errno != ESRCH) ? 1 : 0;
}





int main(int argc, char *argv[])
{
    sim_log_init("watchdog");
    sim_process_register("watchdog", getpid());

    // we validate the command line arguments
    if (argc < 2) {
        sim_log_info("watchdog: usage: %s <read_fd>", argv[0]);
        return EXIT_FAILURE;
    }

    // setup the signal handler that stops the watchdog with Ctrl+C 
    signal(SIGINT, handle_sigint);

    // install SIGUSR2 handler. used for ack-s
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = ack_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_RESTART; // sa_siginfo gives us the pid of the sender
    if (sigaction(WD_ACK_SIGNAL, &sa, NULL) != 0) {
        perror("watchdog: sigaction");
        return EXIT_FAILURE;
    }

    // we parse the pipe fd from the master process
    int rfd = atoi(argv[1]);
    if (rfd < 0) {
        sim_log_info("watchdog: invalid read fd");
        return EXIT_FAILURE;
    }

    // blocking intialization: read PID list from master
    // this ensures watchdog is fully initialized before any monitored
    // processes start (master writes pid-s after spawning all children)

    // we read the number of processes to monitor
    int n = 0;
    ssize_t rr = read_full(rfd, &n, sizeof(n));
    if (rr != (ssize_t)sizeof(n)) {
        sim_log_info("watchdog: failed to read pid count (rr=%zd)", rr);
        close(rfd);
        return EXIT_FAILURE;
    }

    // clamp
    if (n < 0) n = 0;
    if (n > WD_MAX_PROCS) n = WD_MAX_PROCS;

    // array of pids what we are reading from
    rr = read_full(rfd, g_pids, (size_t)(n * (int)sizeof(pid_t)));
    if (rr != (ssize_t)(n * (int)sizeof(pid_t))) {
        sim_log_info("watchdog: failed to read pid list (rr=%zd)", rr);
        close(rfd);
        return EXIT_FAILURE;
    }
    close(rfd);

    g_npids = n;

    // now we log the pids that are being monitored
    sim_log_info("watchdog: monitoring %d pids", g_npids);
    for (int i = 0; i < g_npids; ++i) {
        sim_log_info("watchdog: pid[%d]=%d", i, (int)g_pids[i]);
    }

    // main monitoring loop which pings each process and waits for their ack
    while (g_running) {
        for (int i = 0; i < g_npids && g_running; ++i) {
            pid_t pid = g_pids[i];

            // if dead: kill all and exit
            if (!pid_is_alive(pid)) {
                sim_log_info("watchdog: pid %d is dead -> killing all", (int)pid);
                kill_all_monitored();
                sim_log_close();
                return EXIT_SUCCESS;
            }

            // reset ack flag then ping
            g_acked[i] = 0;
            
            // send the siguser1, which will trigger the client_ping_handler
            if (kill(pid, WD_PING_SIGNAL) != 0) {
                // second failure check
                sim_log_info("watchdog: failed to ping pid %d -> killing all", (int)pid);
                kill_all_monitored();
                sim_log_close();
                return EXIT_SUCCESS;
            }

            // wait for ack up to timeout
            long waited = 0;
            const long step = 10; // we check every 10 ms
            while (g_running && !g_acked[i] && waited < WD_ACK_TIMEOUT_MS) {
                sleep_ms(step);
                waited += step;
            }

            // we check if we were interrupted by the sigint signal
            if (!g_running) break;

            // third failure check
            if (!g_acked[i]) {
                sim_log_info("watchdog: pid %d did not ack in %dms -> killing all",
                             (int)pid, WD_ACK_TIMEOUT_MS);
                kill_all_monitored();
                sim_log_close();
                return EXIT_SUCCESS;
            }

            // it was a success. the process responded in time, so we log
            if (g_acked[i]) {
                sim_log_info("watchdog: pid %d ack (code_area=%d)", 
                             (int)pid, (int)g_acked_code_area[i]);
            }

            // small spacing between checks
            sleep_ms(WD_POLL_PERIOD_MS);
        }
    }

    // shutdown via sigint
    sim_log_info("watchdog: exiting (SIGINT)");
    sim_log_close();
    return EXIT_SUCCESS;
}
