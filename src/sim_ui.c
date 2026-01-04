#define _XOPEN_SOURCE_EXTENDED
#include <locale.h>
#include <ncurses.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdlib.h>

#include "sim_ui.h"
#include "sim_const.h"
#include "sim_types.h"

// Single map window, below the header, centered horizontally
static WINDOW *map_win = NULL;

// Track last screen size to avoid unnecessary resizes
static int last_screen_h = 0;
static int last_screen_w = 0;

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

// Compute map window size/position and create/resize it if needed
static void layout_map_window(void)
{
    int H, W;
    getmaxyx(stdscr, H, W);

    if (H == last_screen_h && W == last_screen_w && map_win != NULL) {
        return;
    }

    last_screen_h = H;
    last_screen_w = W;

    // Reserve the first 5 rows for header / status + legend
    int header_rows = 5;
    int bottom_margin = 1;

    int wh = H - header_rows - bottom_margin;
    if (wh < 3) wh = 3;

    int side_margin = 2;
    int ww = W - 2 * side_margin;
    if (ww < 3) ww = 3;

    int start_y = header_rows;       // map starts just below header
    int start_x = (W - ww) / 2;
    if (start_x < 0) start_x = 0;

    if (map_win == NULL) {
        map_win = newwin(wh, ww, start_y, start_x);
    } else {
        wresize(map_win, wh, ww);
        mvwin(map_win, start_y, start_x);
    }
}

// Initialize color pairs used by menus and titles
static void ui_setup_colors(void)
{
    if (!has_colors())
        return;

    start_color();
    use_default_colors();

    init_pair(1, COLOR_BLACK, COLOR_CYAN);   // menu highlight
    init_pair(2, COLOR_YELLOW, -1);          // normal menu text / obstacles
    init_pair(3, COLOR_MAGENTA, -1);         // title / targets
    init_pair(4, COLOR_CYAN, -1);            // server drone in client mode
}

// Render the full-screen main menu and return selected option index
static int show_menu_internal(void)
{
    const char *options[] = { "Start Simulation", "Instructions", "Quit" };
    int n_options = 3;
    int choice = 0;
    int ch;

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    WINDOW *menu_win = newwin(rows, cols, 0, 0);
    keypad(menu_win, TRUE);

    while (1) {
        werase(menu_win);
        wborder(menu_win, '|', '|', '-', '-', '+', '+', '+', '+');

        const char *title = "=== DRONE SIMULATOR ===";
        wattron(menu_win, COLOR_PAIR(3) | A_BOLD);
        mvwprintw(menu_win, 2, (cols - (int)strlen(title)) / 2, "%s", title);
        wattroff(menu_win, COLOR_PAIR(3) | A_BOLD);

        int start_y = rows / 2 - n_options;
        for (int i = 0; i < n_options; i++) {
            int x = (cols - (int)strlen(options[i])) / 2;
            int y = start_y + i * 2;
            if (i == choice) {
                wattron(menu_win, COLOR_PAIR(1) | A_BOLD);
                mvwprintw(menu_win, y, x, "%s", options[i]);
                wattroff(menu_win, COLOR_PAIR(1) | A_BOLD);
            } else {
                wattron(menu_win, COLOR_PAIR(2));
                mvwprintw(menu_win, y, x, "%s", options[i]);
                wattroff(menu_win, COLOR_PAIR(2));
            }
        }

        wrefresh(menu_win);

        ch = wgetch(menu_win);
        switch (ch) {
            case KEY_UP:
                play_sfx("../../bin/conf/scroll.mp3");
                if (choice > 0) choice--;
                break;

            case KEY_DOWN:
                play_sfx("../../bin/conf/scroll.mp3");
                if (choice < n_options - 1) choice++;
                break;

            case '\n':
                play_sfx("../../bin/conf/select.mp3");
                delwin(menu_win);
                return choice;
        }
    }
}

// Show a modal instructions/help window centered in the screen
static void show_instructions_internal(void)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    int win_rows = rows - 4;
    int win_cols = cols - 10;
    int start_y = 2;
    int start_x = 5;

    WINDOW *instr_win = newwin(win_rows, win_cols, start_y, start_x);
    keypad(instr_win, TRUE);

    werase(instr_win);
    box(instr_win, 0, 0);

    const char *title = "=== INSTRUCTIONS ===";
    int title_x = (win_cols - (int)strlen(title)) / 2;
    wattron(instr_win, A_BOLD | COLOR_PAIR(3));
    mvwprintw(instr_win, 1, title_x, "%s", title);
    wattroff(instr_win, A_BOLD | COLOR_PAIR(3));

    const char *lines[] = {
        "Controls are in the INPUT window:",
        "q w e / a s d / z x c = direction of force",
        "s or SPACE = brake (zero force)",
        "r = reset drone position",
        "Q = quit simulation",
        "",
        "This window shows the map, obstacles, and targets.",
        "Resize the terminal to see the window adjust.",
        "",
        "Legend: '@' = drone, '#' = obstacle, '+' = target",
        "",
        "Press any key to return to menu..."
    };
    int n_lines = (int)(sizeof(lines) / sizeof(lines[0]));
    int start_y_text = 3;
    for (int i = 0; i < n_lines; i++) {
        int x = (win_cols - (int)strlen(lines[i])) / 2;
        mvwprintw(instr_win, start_y_text + i, x, "%s", lines[i]);
    }

    wrefresh(instr_win);
    wgetch(instr_win);
    play_sfx("../../bin/conf/select.mp3");
    delwin(instr_win);
}

// Initialize ncurses and the main map window
void ui_init(void)
{
    setlocale(LC_ALL, "");
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    curs_set(0);

    ui_setup_colors();
    layout_map_window();
    werase(stdscr);
    refresh();
}

// For defs check sim_ui.h
void ui_shutdown(void)
{
    if (map_win) {
        delwin(map_win);
        map_win = NULL;
    }
    endwin();
}

int ui_show_start_menu(void)
{
    return show_menu_internal();
}

void ui_show_instructions(void)
{
    show_instructions_internal();
}

void ui_draw(const WorldState *world)
{
    if (!world) return;

    layout_map_window();

    int H, W;
    getmaxyx(map_win, H, W);

    // Only clear the map window, not the whole screen, to avoid flicker
    werase(map_win);
    box(map_win, 0, 0);

    // Use runtime world dimensions provided by bb_server
    double ww = (double)world->world_width;
    double wh = (double)world->world_height;
    if (ww <= 1.0) ww = 1.0;
    if (wh <= 1.0) wh = 1.0;

    // Map drone position into world bounds
    double wx = world->drone.x;
    double wy = world->drone.y;

    if (wx < 0.0) wx = 0.0;
    if (wx > ww)  wx = ww;
    if (wy < 0.0) wy = 0.0;
    if (wy > wh)  wy = wh;

    // Map world coords to interior of map window (1..W-2, 1..H-2)
    int px = 1 + (int)((wx / ww) * (W - 2));
    int py = 1 + (int)((wy / wh) * (H - 2));

    if (px < 1) px = 1;
    if (px > W - 2) px = W - 2;
    if (py < 1) py = 1;
    if (py > H - 2) py = H - 2;

    // Status lines at the top of the main screen (rows 0..3)
    mvprintw(0, 0,
             "x=%6.2f y=%6.2f  vx=%6.2f vy=%6.2f",
             world->drone.x, world->drone.y,
             world->drone.vx, world->drone.vy);

    mvprintw(1, 0,
             "fx=%6.2f fy=%6.2f  brake=%d reset=%d quit=%d last_key=%d",
             world->cmd.fx, world->cmd.fy,
             world->cmd.brake, world->cmd.reset,
             world->cmd.quit, world->cmd.last_key);

    mvprintw(2, 0,
             "obstacles=%d targets=%d score=%6.2f",
             world->num_obstacles, world->num_targets, world->score);

             
    if (world->mode == SIM_MODE_CLIENT) {
        mvprintw(3, 0,
                 "Legend: '@'=your_drone  '*'=server_drone  '#'=obstacle  '+'=target   |   Press 'Q' in INPUT to quit");
    } else {
        mvprintw(3, 0,
                 "Legend: '@'=drone  '#'=obstacle  '+'=target   |   Press 'Q' in INPUT window to quit");
    }
             
    // row 4 is left empty as a visual spacer; map border starts at row 5

    // Draw obstacles as '#'
    for (int i = 0; i < world->obstacles_slots; ++i) {
        if (!world->obstacles[i].active)
            continue;

        double ox = world->obstacles[i].x;
        double oy = world->obstacles[i].y;

        if (ox < 0.0) ox = 0.0;
        if (ox > ww)  ox = ww;
        if (oy < 0.0) oy = 0.0;
        if (oy > wh)  oy = wh;

        int opx = 1 + (int)((ox / ww) * (W - 2));
        int opy = 1 + (int)((oy / wh) * (H - 2));

        if (opx < 1 || opx > W - 2 || opy < 1 || opy > H - 2)
            continue;

        wattron(map_win, COLOR_PAIR(2));
        mvwaddch(map_win, opy, opx, '#');
        wattroff(map_win, COLOR_PAIR(2));
    }

    // Draw targets as '+'
    for (int i = 0; i < world->targets_slots; ++i) {
        if (!world->targets[i].active)
            continue;

        double tx = world->targets[i].x;
        double ty = world->targets[i].y;

        if (tx < 0.0) tx = 0.0;
        if (tx > ww)  tx = ww;
        if (ty < 0.0) ty = 0.0;
        if (ty > wh)  ty = wh;

        int tpx = 1 + (int)((tx / ww) * (W - 2));
        int tpy = 1 + (int)((ty / wh) * (H - 2));

        if (tpx < 1 || tpx > W - 2 || tpy < 1 || tpy > H - 2)
            continue;

        wattron(map_win, COLOR_PAIR(3) | A_BOLD);
        mvwaddch(map_win, tpy, tpx, '+');
        wattroff(map_win, COLOR_PAIR(3) | A_BOLD);
    }

    // Draw drone symbol at mapped position inside the map window
    mvwaddch(map_win, py, px, '@');

    // If in client mode, also draw server's drone with different symbol
    if (world->mode == SIM_MODE_CLIENT && world->has_server_drone) {
        double sx = world->server_drone_x;
        double sy = world->server_drone_y;

        // Clamp server drone to world bounds
        if (sx < 0.0) sx = 0.0;
        if (sx > ww)  sx = ww;
        if (sy < 0.0) sy = 0.0;
        if (sy > wh)  sy = wh;

        // Map server drone to screen coordinates
        int spx = 1 + (int)((sx / ww) * (W - 2));
        int spy = 1 + (int)((sy / wh) * (H - 2));

        if (spx < 1) spx = 1;
        if (spx > W - 2) spx = W - 2;
        if (spy < 1) spy = 1;
        if (spy > H - 2) spy = H - 2;

        // Draw server drone with different color and symbol
        wattron(map_win, COLOR_PAIR(4) | A_BOLD);
        mvwaddch(map_win, spy, spx, '*');  // '*' for server drone
        wattroff(map_win, COLOR_PAIR(4) | A_BOLD);
    }

    refresh();
    wrefresh(map_win);
}
