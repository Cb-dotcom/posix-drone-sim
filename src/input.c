#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <curses.h>
#include <math.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include "sim_types.h"
#include "sim_ipc.h"
#include "sim_const.h"
#include "sim_log.h"
#include "sim_params.h"   // runtime parameters (force_step, max_force)
#include <unistd.h>
#include <stdlib.h>
#include "sim_ipc.h"

static void report_pid_if_requested(int argc, char **argv)
{
    // convention: last arg is pid_report_fd
    // example: ./bb_server ... <pid_report_fd>
    if (argc < 2) return;

    char *end = NULL;
    long fd = strtol(argv[argc - 1], &end, 10);
    if (!end || *end != '\0') return;
    if (fd < 0) return;

    pid_t me = getpid();
    (void)write_full((int)fd, &me, sizeof(me));
    close((int)fd);
}

// Flag set by the SIGINT handler to request a clean shutdown
volatile sig_atomic_t g_current_code_area = 0;

static volatile sig_atomic_t running = 1;
static pid_t g_wd_pid = -1;

// [REQUIRED] Update handler signature to 3 arguments
static void wd_client_ping_handler(int sig, siginfo_t *info, void *ctx)
{
    (void)sig; (void)ctx;
    if (g_wd_pid > 1) {
        union sigval sv;
        sv.sival_int = (int)g_current_code_area;
        sigqueue(g_wd_pid, SIGUSR2, sv);
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
    sa.sa_sigaction = wd_client_ping_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO | SA_RESTART;

    // Watchdog pings us with SIGUSR1
    (void)sigaction(SIGUSR1, &sa, NULL);
}


// to play the music
void play_sfx(const char *filename) {
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

// Helper to clamp a value to [-max, +max]
static double clamp(double v, double max)
{
    if (v >  max) return  max;
    if (v < -max) return -max;
    return v;
}

// Render the input UI (direction pad + flags + current command state)
static void draw_ui(const CommandState *cmd)
{
    int H, W;
    getmaxyx(stdscr, H, W);

    erase();

    mvprintw(0, 0, "INPUT PROCESS (ncurses) - controls:");
    mvprintw(2, 0, "Direction pad:");

    // PAD: 3x3 box just under the label
    int pad_y = 3;
    int pad_x = 2;

    mvprintw(pad_y + 0, pad_x, "+---+---+---+");
    mvprintw(pad_y + 1, pad_x, "| q | w | e |");
    mvprintw(pad_y + 2, pad_x, "+---+---+---+");
    mvprintw(pad_y + 3, pad_x, "| a | s | d |");
    mvprintw(pad_y + 4, pad_x, "+---+---+---+");
    mvprintw(pad_y + 5, pad_x, "| z | x | c |");
    mvprintw(pad_y + 6, pad_x, "+---+---+---+");

    struct KeyCell { int ch; int y; int x; } cells[] = {
        { 'q', pad_y + 1, pad_x + 2 },
        { 'w', pad_y + 1, pad_x + 6 },
        { 'e', pad_y + 1, pad_x +10 },
        { 'a', pad_y + 3, pad_x + 2 },
        { 's', pad_y + 3, pad_x + 6 },
        { 'd', pad_y + 3, pad_x +10 },
        { 'z', pad_y + 5, pad_x + 2 },
        { 'x', pad_y + 5, pad_x + 6 },
        { 'c', pad_y + 5, pad_x +10 },
    };

    int lk = cmd->last_key;
    if (lk == ' ') lk = 's';

    for (size_t i = 0; i < sizeof(cells)/sizeof(cells[0]); ++i) {
        if (cells[i].ch == lk) {
            attron(A_REVERSE);
            mvaddch(cells[i].y, cells[i].x, cells[i].ch);
            attroff(A_REVERSE);
        }
    }

    int special_y = pad_y + 8;
    mvprintw(special_y + 0, 0, "Special:");
    mvprintw(special_y + 1, 2, "s or SPACE = brake (zero force)");
    mvprintw(special_y + 2, 2, "r          = reset drone");
    mvprintw(special_y + 3, 2, "Q          = quit simulation & exit input");

    int cmd_y = special_y + 5;
    mvprintw(cmd_y + 0, 0, "Current command:");
    mvprintw(cmd_y + 1, 2, "fx = %6.2f  fy = %6.2f", cmd->fx, cmd->fy);
    mvprintw(cmd_y + 2, 2, "brake = %d  reset = %d  quit = %d",
             cmd->brake, cmd->reset, cmd->quit);
    mvprintw(cmd_y + 3, 2, "last_key = %d (%c)",
             cmd->last_key,
             (cmd->last_key >= 32 && cmd->last_key <= 126)
                 ? cmd->last_key : '.');

    refresh();
}

int main(int argc, char *argv[])
{
    sim_log_init("input");
    sim_process_register("input", getpid());
    
    signal(SIGINT, handle_sigint);
    report_pid_if_requested(argc, argv);
    wd_client_init();


    // Optional: don't die on broken pipe; log instead
    signal(SIGPIPE, SIG_IGN);

    setbuf(stderr, NULL);

    // Load runtime parameters in this process
    if (sim_params_load(NULL) != 0) {
        fprintf(stderr,
                "input: warning: could not load '%s', using built-in defaults\n",
                SIM_PARAMS_DEFAULT_PATH);
    }
    const SimParams *params = sim_params_get();

    // FDs for anonymous pipes are passed via argv by master:
    //   ./input <fd_cmd_out>
    if (argc < 2) {
        fprintf(stderr, "input: usage: %s <fd_cmd_out>\n", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_to_srv = atoi(argv[SIM_ARG_INPUT_CMD_OUT]);

    const double FORCE_STEP = params->force_step;
    const double MAX_FORCE  = params->max_force;
    const double INV_SQRT2  = 0.70710678118;

    CommandState cmd;
    cmd.fx       = 0.0;
    cmd.fy       = 0.0;
    cmd.brake    = 0;
    cmd.reset    = 0;
    cmd.quit     = 0;
    cmd.last_key = 0;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(100);

    sim_log_info("input: started (ncurses)\n");
    fprintf(stderr, "input: started (ncurses)\n");

    while (running) {
        g_current_code_area = CODE_AREA_MAIN_LOOP;

        draw_ui(&cmd);

        g_current_code_area = CODE_AREA_READ_PIPE;

        int ch = getch();
        if (ch == ERR) {
            continue;
        }

        g_current_code_area = CODE_AREA_PHYSICS_UPDATE;

        cmd.last_key = ch;
        cmd.brake    = 0;

        double fx = cmd.fx;
        double fy = cmd.fy;

        switch (ch) {
            case 'q': 
                play_sfx("../../bin/conf/press.mp3");
                fx -= FORCE_STEP * INV_SQRT2; fy -= FORCE_STEP * INV_SQRT2; 
                break;
            case 'e': 
                play_sfx("../../bin/conf/press.mp3");
                fx += FORCE_STEP * INV_SQRT2; fy -= FORCE_STEP * INV_SQRT2; 
                break;
            case 'z': 
                play_sfx("../../bin/conf/press.mp3");
                fx -= FORCE_STEP * INV_SQRT2; fy += FORCE_STEP * INV_SQRT2; 
                break;
            case 'c': 
                play_sfx("../../bin/conf/press.mp3");
                fx += FORCE_STEP * INV_SQRT2; fy += FORCE_STEP * INV_SQRT2; 
                break;
            case 'w': 
                play_sfx("../../bin/conf/press.mp3");
                fy -= FORCE_STEP; 
                break;
            case 'x': 
                play_sfx("../../bin/conf/press.mp3");
                fy += FORCE_STEP; 
                break;
            case 'a': 
                play_sfx("../../bin/conf/press.mp3");
                fx -= FORCE_STEP; 
                break;
            case 'd': 
                play_sfx("../../bin/conf/press.mp3");
                fx += FORCE_STEP; 
                break;
            case 's':
            case ' ':
                play_sfx("../../bin/conf/stop.mp3");
                fx = 0.0; fy = 0.0; cmd.brake = 1; break;
            case 'r':
                play_sfx("../../bin/conf/reset.mp3");
                cmd.reset = 1; fx = 0.0; fy = 0.0; break;
            case 'Q':
                cmd.quit  = 1;
                running   = 0;
                break;
            default:
                break;
        }

        fx = clamp(fx, MAX_FORCE);
        fy = clamp(fy, MAX_FORCE);

        cmd.fx = fx;
        cmd.fy = fy;

        g_current_code_area = CODE_AREA_WRITE_PIPE;

        ssize_t w = write_full(fd_to_srv, &cmd, sizeof(cmd));
        if (w != (ssize_t)sizeof(cmd)) {
            endwin();
            perror("input: write_full(fd_to_srv)");
            fprintf(stderr, "input: write_full returned %zd (expected %zu)\n",
                    w, sizeof(cmd));
            break;
        }

        if (cmd.reset == 1) {
            cmd.reset = 0;
        }

        if (cmd.quit) {
            sim_log_info("input: quit flag set, exiting\n");
            fprintf(stderr, "input: quit flag set, exiting\n");
            break;
        }
    }
    g_current_code_area = CODE_AREA_SHUTDOWN;

    sim_log_info("input: exiting\n");
    fprintf(stderr, "input: exiting\n");

    endwin();
    close(fd_to_srv);
    return EXIT_SUCCESS;
}
