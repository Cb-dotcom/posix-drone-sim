// watchdog.c
// A simple watchdog process:
// - master starts watchdog first and passes a read-fd via argv[1]
// - watchdog reads: int n, then n pid_t values from that fd
// - watchdog periodically SIGUSR1-pings each pid, expects SIGUSR2 ack
// - if a pid is dead OR doesn't ack within timeout => kill all and exit

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

static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_acked[WD_MAX_PROCS];
static volatile sig_atomic_t g_acked_code_area[WD_MAX_PROCS];

static pid_t  g_pids[WD_MAX_PROCS];
static int    g_npids = 0;

static void handle_sigint(int sig)
{
    (void)sig;
    g_running = 0;
}

static int find_pid_index(pid_t pid)
{
    for (int i = 0; i < g_npids; ++i) {
        if (g_pids[i] == pid) return i;
    }
    return -1;
}

static void ack_handler(int sig, siginfo_t *info, void *ctx)
{
    (void)sig;
    (void)ctx;

    if (!info) return;
    pid_t sender = info->si_pid;
    int idx = find_pid_index(sender);
    if (idx >= 0) {
        g_acked[idx] = 1;
        g_acked_code_area[idx] = info->si_value.sival_int;
    }
}

static void sleep_ms(long ms)
{
    if (ms <= 0) return;
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;

    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
        // restart
    }
}

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

    if (argc < 2) {
        sim_log_info("watchdog: usage: %s <read_fd>", argv[0]);
        return EXIT_FAILURE;
    }

    // Stop watchdog with Ctrl+C (or forwarded SIGINT)
    signal(SIGINT, handle_sigint);

    // Install SIGUSR2 handler (ack)
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_sigaction = ack_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_RESTART;
    if (sigaction(WD_ACK_SIGNAL, &sa, NULL) != 0) {
        perror("watchdog: sigaction");
        return EXIT_FAILURE;
    }

    int rfd = atoi(argv[1]);
    if (rfd < 0) {
        sim_log_info("watchdog: invalid read fd");
        return EXIT_FAILURE;
    }

    // Read number of processes then pids from master
    int n = 0;
    ssize_t rr = read_full(rfd, &n, sizeof(n));
    if (rr != (ssize_t)sizeof(n)) {
        sim_log_info("watchdog: failed to read pid count (rr=%zd)", rr);
        close(rfd);
        return EXIT_FAILURE;
    }

    if (n < 0) n = 0;
    if (n > WD_MAX_PROCS) n = WD_MAX_PROCS;

    rr = read_full(rfd, g_pids, (size_t)(n * (int)sizeof(pid_t)));
    if (rr != (ssize_t)(n * (int)sizeof(pid_t))) {
        sim_log_info("watchdog: failed to read pid list (rr=%zd)", rr);
        close(rfd);
        return EXIT_FAILURE;
    }
    close(rfd);

    g_npids = n;

    sim_log_info("watchdog: monitoring %d pids", g_npids);
    for (int i = 0; i < g_npids; ++i) {
        sim_log_info("watchdog: pid[%d]=%d", i, (int)g_pids[i]);
    }

    while (g_running) {
        for (int i = 0; i < g_npids && g_running; ++i) {
            pid_t pid = g_pids[i];

            // If dead: kill all and exit
            if (!pid_is_alive(pid)) {
                sim_log_info("watchdog: pid %d is dead -> killing all", (int)pid);
                kill_all_monitored();
                sim_log_close();
                return EXIT_SUCCESS;
            }

            // Reset ack flag then ping
            g_acked[i] = 0;

            if (kill(pid, WD_PING_SIGNAL) != 0) {
                sim_log_info("watchdog: failed to ping pid %d -> killing all", (int)pid);
                kill_all_monitored();
                sim_log_close();
                return EXIT_SUCCESS;
            }

            // Wait for ack up to timeout
            long waited = 0;
            const long step = 10; // ms
            while (g_running && !g_acked[i] && waited < WD_ACK_TIMEOUT_MS) {
                sleep_ms(step);
                waited += step;
            }

            if (!g_running) break;

            if (!g_acked[i]) {
                sim_log_info("watchdog: pid %d did not ack in %dms -> killing all",
                             (int)pid, WD_ACK_TIMEOUT_MS);
                kill_all_monitored();
                sim_log_close();
                return EXIT_SUCCESS;
            }

            if (g_acked[i]) {
                sim_log_info("watchdog: pid %d ack (code_area=%d)", 
                             (int)pid, (int)g_acked_code_area[i]);
            }

            // small spacing between checks
            sleep_ms(WD_POLL_PERIOD_MS);
        }
    }

    sim_log_info("watchdog: exiting (SIGINT)");
    sim_log_close();
    return EXIT_SUCCESS;
}
