/*
    Network Server Process - FIXED for Stef504 compatibility
    
    Key changes:
    1. Size format: "size W,H" (single line with comma)
    2. Coordinate format: "X.X, Y.Y" (space after comma)
    3. Precision reduced to %.1f to match Stef504
*/

#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include "sim_ipc.h"
#include "sim_log.h"
#include "sim_network.h"
#include "sim_params.h"
#include "sim_types.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static volatile sig_atomic_t running = 1;
static void handle_sigint(int sig) { (void)sig; running = 0; }

/* ---------- Env-driven coordinate config ---------- */

static int get_env_i(const char *name, int defv)
{
    const char *s = getenv(name);
    if (!s || !*s) return defv;
    return (int)strtol(s, NULL, 10);
}

static int norm_alpha_deg(int a)
{
    if (a == 0 || a == 90 || a == -90 || a == 180) return a;
    if (a == 270) return -90;
    if (a == -270) return 90;
    return 0;
}

static void coord_local_to_virtual(double xl, double yl,
                                   int W, int H,
                                   int flip_y, int alpha_deg,
                                   double *xv, double *yv)
{
    double x = xl;
    double y = flip_y ? ((double)H - yl) : yl;

    alpha_deg = norm_alpha_deg(alpha_deg);
    double xr = x, yr = y;

    if (alpha_deg == 0) {
        xr = x;  yr = y;
    } else if (alpha_deg == 180) {
        xr = -x; yr = -y;
    } else if (alpha_deg == 90) {
        xr = -y; yr = x;
    } else if (alpha_deg == -90) {
        xr = y;  yr = -x;
    }

    if (alpha_deg == 0) {
        *xv = xr;
        *yv = yr;
    } else if (alpha_deg == 180) {
        *xv = xr + W;
        *yv = yr + H;
    } else if (alpha_deg == 90) {
        *xv = xr + W;
        *yv = yr;
    } else {
        *xv = xr;
        *yv = yr + H;
    }
}

static void coord_virtual_to_local(double xv, double yv,
                                   int W, int H,
                                   int flip_y, int alpha_deg,
                                   double *xl, double *yl)
{
    alpha_deg = norm_alpha_deg(alpha_deg);

    double xt = xv, yt = yv;

    if (alpha_deg == 0) {
        xt = xv;      yt = yv;
    } else if (alpha_deg == 180) {
        xt = xv - W;  yt = yv - H;
    } else if (alpha_deg == 90) {
        xt = xv - W;  yt = yv;
    } else {
        xt = xv;      yt = yv - H;
    }

    double x = xt, y = yt;

    if (alpha_deg == 0) {
        x = xt;  y = yt;
    } else if (alpha_deg == 180) {
        x = -xt; y = -yt;
    } else if (alpha_deg == 90) {
        x = yt;  y = -xt;
    } else {
        x = -yt; y = xt;
    }

    double yl_local = flip_y ? ((double)H - y) : y;

    *xl = x;
    *yl = yl_local;
}

/* ---------- Nonblocking helpers ---------- */

static int set_nonblocking(int fd)
{
    int fl = fcntl(fd, F_GETFL, 0);
    if (fl < 0) return -1;
    if (fcntl(fd, F_SETFL, fl | O_NONBLOCK) < 0) return -1;
    return 0;
}

static int drain_latest_drone(int fd, DroneState *latest, int *have_latest)
{
    for (;;) {
        DroneState ds;
        ssize_t r = read(fd, &ds, sizeof(ds));
        if (r == (ssize_t)sizeof(ds)) {
            *latest = ds;
            *have_latest = 1;
            continue;
        }
        if (r == 0) return 1;
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        return -1;
    }
}

static int try_write_struct(int fd, const void *p, size_t n)
{
    ssize_t w = write(fd, p, n);
    if (w == (ssize_t)n) return 1;
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
    return -1;
}

typedef struct {
    char   buf[2048];
    size_t len;
} LineAcc;

static int acc_recv_lines(int sockfd, LineAcc *acc)
{
    for (;;) {
        ssize_t r = recv(sockfd, acc->buf + acc->len, sizeof(acc->buf) - acc->len, 0);
        if (r > 0) {
            acc->len += (size_t)r;
            if (acc->len == sizeof(acc->buf)) return 0;
            continue;
        }
        if (r == 0) return 1;
        if (errno == EINTR) continue;
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
}

static int acc_pop_line(LineAcc *acc, char *out, size_t cap)
{
    for (size_t i = 0; i < acc->len; ++i) {
        if (acc->buf[i] == '\n') {
            size_t L = i;
            if (L > 0 && acc->buf[L - 1] == '\r') L--;

            size_t copy = (L < cap - 1) ? L : (cap - 1);
            memcpy(out, acc->buf, copy);
            out[copy] = '\0';

            size_t remain = acc->len - (i + 1);
            memmove(acc->buf, acc->buf + i + 1, remain);
            acc->len = remain;
            return 1;
        }
    }
    return 0;
}

typedef struct {
    char   buf[4096];
    size_t off;
    size_t len;
} SendQ;

static void sq_clear(SendQ *q) { q->off = 0; q->len = 0; }

static int sq_append_line(SendQ *q, const char *s)
{
    size_t sl = strlen(s);
    if (sl + 1 > sizeof(q->buf) - q->len) return -1;
    memcpy(q->buf + q->len, s, sl);
    q->len += sl;
    q->buf[q->len++] = '\n';
    return 0;
}

static int sq_flush(int sockfd, SendQ *q)
{
    while (q->off < q->len) {
        ssize_t w = send(sockfd, q->buf + q->off, q->len - q->off, MSG_NOSIGNAL);
        if (w > 0) {
            q->off += (size_t)w;
            continue;
        }
        if (w < 0 && errno == EINTR) continue;
        if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return 0;
        return -1;
    }
    sq_clear(q);
    return 1;
}

static int parse_double_strict(const char *s, double *out)
{
    char *end = NULL;
    errno = 0;
    double v = strtod(s, &end);
    if (errno != 0) return -1;
    if (!end || *end != '\0') return -1;
    *out = v;
    return 0;
}

/* ---------- Server protocol state machine ---------- */

typedef enum {
    S_WAIT_OOK = 0,
    S_SEND_SIZE,
    S_WAIT_SOK,

    S_SEND_DRONE,
    S_WAIT_DOK,

    S_SEND_OBST,
    S_WAIT_OBST_X,
    S_WAIT_OBST_Y,
    S_SEND_POK,

    S_WAIT_QOK,
    S_DONE
} SState;

int main(int argc, char *argv[])
{
    sim_log_init("network_server");
    sim_process_register("network_server", getpid());

    signal(SIGINT, handle_sigint);
    signal(SIGPIPE, SIG_IGN);

    if (argc < 3) {
        sim_log_info("network_server: usage: %s <fd_drone_pos_in> <fd_obstacle_out>", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_drone_in = atoi(argv[1]);
    int fd_obstacle_out = atoi(argv[2]);

    if (sim_params_load(NULL) != 0) {
        sim_log_info("network_server: warning: could not load config, using defaults");
    }
    const SimParams *params = sim_params_get();

    const int W = params->world_width;
    const int H = params->world_height;

    const int flip_y   = get_env_i("SIM_NET_FLIP_Y", 0);
    const int alpha_deg = norm_alpha_deg(get_env_i("SIM_NET_ALPHA", 0));

    sim_log_info("network_server: coord cfg flip_y=%d alpha=%d", flip_y, alpha_deg);

    const char *bind_addr = "0.0.0.0";
    int port = params->server_port;
    if (port <= 0 || port > 65535) port = NET_DEFAULT_PORT;

    sim_log_info("network_server: starting on %s:%d", bind_addr, port);

    int listen_fd = net_create_server_socket(bind_addr, port);
    if (listen_fd < 0) {
        perror("network_server: net_create_server_socket");
        return EXIT_FAILURE;
    }

    sim_log_info("network_server: listening...");

    int sock = net_accept_client(listen_fd);
    if (sock < 0) {
        perror("network_server: net_accept_client");
        close(listen_fd);
        return EXIT_FAILURE;
    }
    close(listen_fd);

    sim_log_info("network_server: client connected");

    (void)net_set_nonblocking(sock);
    (void)set_nonblocking(fd_drone_in);
    (void)set_nonblocking(fd_obstacle_out);

    LineAcc in = { .len = 0 };
    SendQ   out; sq_clear(&out);

    DroneState latest_server = {0};
    int have_server = 0;

    double obst_x_v = 0.0, obst_y_v = 0.0;
    double obst_x_l = 0.0, obst_y_l = 0.0;

    // FIXED: Start with ok (compatible with Stef504)
    (void)sq_append_line(&out, NET_TOK_OK);
    SState st = S_WAIT_OOK;

    while (st != S_DONE) {
        int dr = drain_latest_drone(fd_drone_in, &latest_server, &have_server);
        if (dr == 1) {
            sim_log_info("network_server: bb_server closed drone pipe -> shutdown");
            running = 0;
        } else if (dr < 0) {
            sim_log_info("network_server: error reading drone pipe -> shutdown");
            running = 0;
        }

        if (!running && st != S_WAIT_QOK && st != S_DONE) {
            sq_clear(&out);
            (void)sq_append_line(&out, NET_TOK_Q);
            st = S_WAIT_QOK;
        }

        fd_set rfds, wfds;
        FD_ZERO(&rfds);
        FD_ZERO(&wfds);

        FD_SET(sock, &rfds);
        if (out.len > 0) FD_SET(sock, &wfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 20000;

        int ready = select(sock + 1, &rfds, &wfds, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("network_server: select");
            break;
        }

        if (FD_ISSET(sock, &wfds)) {
            if (sq_flush(sock, &out) < 0) {
                sim_log_info("network_server: socket send failed");
                break;
            }
        }

        if (FD_ISSET(sock, &rfds)) {
            int rr = acc_recv_lines(sock, &in);
            if (rr == 1) {
                sim_log_info("network_server: client disconnected");
                break;
            }
            if (rr < 0) {
                sim_log_info("network_server: socket recv error");
                break;
            }
        }

        if (out.len == 0) {
            if (st == S_SEND_SIZE) {
                // FIXED: Send as "size W,H" (single line, matches Stef504)
                char b[64];
                snprintf(b, sizeof(b), "size %d,%d", W, H);
                (void)sq_append_line(&out, b);
                st = S_WAIT_SOK;
            } else if (st == S_SEND_DRONE) {
                char b[64];

                double xl = have_server ? latest_server.x : 0.0;
                double yl = have_server ? latest_server.y : 0.0;

                double xv, yv;
                coord_local_to_virtual(xl, yl, W, H, flip_y, alpha_deg, &xv, &yv);

                (void)sq_append_line(&out, NET_TOK_DRONE);
                // FIXED: Use "%.1f, %.1f" format (space after comma, matches Stef504)
                snprintf(b, sizeof(b), "%.1f, %.1f", xv, yv);
                (void)sq_append_line(&out, b);

                st = S_WAIT_DOK;
            } else if (st == S_SEND_OBST) {
                (void)sq_append_line(&out, NET_TOK_OBST);
                st = S_WAIT_OBST_X;
            } else if (st == S_SEND_POK) {
                (void)sq_append_line(&out, NET_TOK_POK);
                st = S_SEND_DRONE;
            }
        }

        char line[256];
        while (acc_pop_line(&in, line, sizeof(line))) {
            if (st == S_WAIT_OOK) {
                if (strcmp(line, NET_TOK_OOK) != 0) { st = S_DONE; break; }
                st = S_SEND_SIZE;
            }
            else if (st == S_WAIT_SOK) {
                if (strcmp(line, NET_TOK_SOK) != 0) { st = S_DONE; break; }
                sim_log_info("network_server: handshake complete, size=%dx%d", W, H);
                st = S_SEND_DRONE;
            }
            else if (st == S_WAIT_DOK) {
                if (strcmp(line, NET_TOK_DOK) != 0) { st = S_DONE; break; }
                st = S_SEND_OBST;
            }
            else if (st == S_WAIT_OBST_X) {
                if (strcmp(line, NET_TOK_Q) == 0) {
                    (void)sq_append_line(&out, NET_TOK_QOK);
                    st = S_DONE;
                    break;
                }
                // FIXED: Try both formats (with and without space after comma)
                if (sscanf(line, "%lf, %lf", &obst_x_v, &obst_y_v) == 2 ||
                    sscanf(line, "%lf,%lf", &obst_x_v, &obst_y_v) == 2) {
                    // Successfully parsed both coordinates from single line
                    coord_virtual_to_local(obst_x_v, obst_y_v, W, H, flip_y, alpha_deg, &obst_x_l, &obst_y_l);

                    Obstacle obs;
                    obs.x = obst_x_l;
                    obs.y = obst_y_l;
                    obs.radius = 1.0;
                    obs.active = 1;

                    int wr = try_write_struct(fd_obstacle_out, &obs, sizeof(obs));
                    if (wr < 0) { st = S_DONE; break; }

                    st = S_SEND_POK;
                } else {
                    // Try parsing as single X value (original behavior)
                    if (parse_double_strict(line, &obst_x_v) != 0) { st = S_DONE; break; }
                    st = S_WAIT_OBST_Y;
                }
            }
            else if (st == S_WAIT_OBST_Y) {
                if (parse_double_strict(line, &obst_y_v) != 0) { st = S_DONE; break; }

                coord_virtual_to_local(obst_x_v, obst_y_v, W, H, flip_y, alpha_deg, &obst_x_l, &obst_y_l);

                Obstacle obs;
                obs.x = obst_x_l;
                obs.y = obst_y_l;
                obs.radius = 1.0;
                obs.active = 1;

                int wr = try_write_struct(fd_obstacle_out, &obs, sizeof(obs));
                if (wr < 0) { st = S_DONE; break; }

                st = S_SEND_POK;
            }
            else if (st == S_WAIT_QOK) {
                if (strcmp(line, NET_TOK_QOK) == 0) { st = S_DONE; break; }
            }
        }
    }

    sim_log_info("network_server: shutting down");
    close(sock);
    close(fd_drone_in);
    close(fd_obstacle_out);
    sim_log_info("network_server: exited");
    sim_log_close();
    return EXIT_SUCCESS;
}