/*
    Network Client Process - Stats tracking for logging only (not sent to bb_server)
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
#include <time.h>

#include "sim_ipc.h"
#include "sim_log.h"
#include "sim_network.h"
#include "sim_params.h"
#include "sim_types.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#define MAX_RECONNECT_ATTEMPTS 5
#define RECONNECT_DELAY_SEC 3

static volatile sig_atomic_t running = 1;
static void handle_sigint(int sig) { (void)sig; running = 0; }

typedef struct { int width; int height; } WindowDimensions;

/* ---------- Network Statistics Helpers (for logging only) ---------- */

static void stats_init(NetworkStats *stats)
{
    memset(stats, 0, sizeof(NetworkStats));
    stats->connected = 0;
    clock_gettime(CLOCK_MONOTONIC, &stats->connection_start);
}

static void stats_on_connect(NetworkStats *stats)
{
    stats->connected = 1;
    stats->reconnect_attempts = 0;
    clock_gettime(CLOCK_MONOTONIC, &stats->connection_start);
    stats->packets_sent = 0;
    stats->packets_received = 0;
    stats->bytes_sent = 0;
    stats->bytes_received = 0;
}

static void stats_on_disconnect(NetworkStats *stats)
{
    stats->connected = 0;
    stats->connection_drops++;
}

static void stats_on_send(NetworkStats *stats, size_t bytes)
{
    stats->packets_sent++;
    stats->bytes_sent += bytes;
    clock_gettime(CLOCK_MONOTONIC, &stats->last_packet_time);
}

static void stats_on_receive(NetworkStats *stats, size_t bytes)
{
    stats->packets_received++;
    stats->bytes_received += bytes;
    
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    
    double dt = (now.tv_sec - stats->last_packet_time.tv_sec) +
                (now.tv_nsec - stats->last_packet_time.tv_nsec) / 1e9;
    stats->latency_ms = dt * 1000.0;
    
    if (stats->avg_latency_ms == 0.0) {
        stats->avg_latency_ms = stats->latency_ms;
    } else {
        stats->avg_latency_ms = 0.7 * stats->avg_latency_ms + 0.3 * stats->latency_ms;
    }
    
    double total_time = (now.tv_sec - stats->connection_start.tv_sec) +
                        (now.tv_nsec - stats->connection_start.tv_nsec) / 1e9;
    if (total_time > 0.0) {
        stats->bandwidth_kbps = (stats->bytes_sent + stats->bytes_received) / total_time / 1024.0;
    }
}

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

    if (alpha_deg == 0)      { xr = x;  yr = y; }
    else if (alpha_deg == 180){ xr = -x; yr = -y; }
    else if (alpha_deg == 90) { xr = -y; yr = x; }
    else if (alpha_deg == -90){ xr = y;  yr = -x; }

    if (alpha_deg == 0) {
        *xv = xr; *yv = yr;
    } else if (alpha_deg == 180) {
        *xv = xr + W; *yv = yr + H;
    } else if (alpha_deg == 90) {
        *xv = xr + W; *yv = yr;
    } else {
        *xv = xr;     *yv = yr + H;
    }
}

static void coord_virtual_to_local(double xv, double yv,
                                   int W, int H,
                                   int flip_y, int alpha_deg,
                                   double *xl, double *yl)
{
    alpha_deg = norm_alpha_deg(alpha_deg);

    double xt = xv, yt = yv;

    if (alpha_deg == 0)       { xt = xv;      yt = yv; }
    else if (alpha_deg == 180){ xt = xv - W;  yt = yv - H; }
    else if (alpha_deg == 90) { xt = xv - W;  yt = yv; }
    else                      { xt = xv;      yt = yv - H; }

    double x = xt, y = yt;

    if (alpha_deg == 0)       { x = xt;  y = yt; }
    else if (alpha_deg == 180){ x = -xt; y = -yt; }
    else if (alpha_deg == 90) { x = yt;  y = -xt; }
    else                      { x = -yt; y = xt; }

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

static int acc_recv_lines(int sockfd, LineAcc *acc, NetworkStats *stats)
{
    for (;;) {
        ssize_t r = recv(sockfd, acc->buf + acc->len, sizeof(acc->buf) - acc->len, 0);
        if (r > 0) {
            acc->len += (size_t)r;
            stats_on_receive(stats, (size_t)r);
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
    char   buf[2048];
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

static int sq_flush(int sockfd, SendQ *q, NetworkStats *stats)
{
    while (q->off < q->len) {
        ssize_t w = send(sockfd, q->buf + q->off, q->len - q->off, MSG_NOSIGNAL);
        if (w > 0) {
            q->off += (size_t)w;
            stats_on_send(stats, (size_t)w);
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

/* ---------- Client state machine ---------- */

typedef enum {
    C_WAIT_OK = 0,
    C_WAIT_SIZE,
    C_RUN_WAIT_TOKEN,
    C_RUN_WAIT_DRONE_COORDS,
    C_RUN_WAIT_POK,
    C_DONE
} CState;

/* ---------- Connection with retry ---------- */

static int connect_with_retry(const char *address, int port, NetworkStats *stats)
{
    int attempts = 0;
    
    while (attempts < MAX_RECONNECT_ATTEMPTS && running) {
        sim_log_info("network_client: connecting to %s:%d (attempt %d/%d)", 
                     address, port, attempts + 1, MAX_RECONNECT_ATTEMPTS);
        
        int sock = net_connect_to_server(address, port);
        if (sock >= 0) {
            sim_log_info("network_client: connected successfully");
            stats_on_connect(stats);
            return sock;
        }
        
        attempts++;
        stats->reconnect_attempts++;
        
        if (attempts < MAX_RECONNECT_ATTEMPTS && running) {
            sim_log_info("network_client: connection failed, retrying in %d seconds...", 
                         RECONNECT_DELAY_SEC);
            sleep(RECONNECT_DELAY_SEC);
        }
    }
    
    sim_log_info("network_client: failed to connect after %d attempts", MAX_RECONNECT_ATTEMPTS);
    return -1;
}

int main(int argc, char *argv[])
{
    sim_log_init("network_client");
    sim_process_register("network_client", getpid());

    signal(SIGINT, handle_sigint);
    signal(SIGPIPE, SIG_IGN);

    if (argc < 4) {
        sim_log_info("network_client: usage: %s <fd_drone_pos_in> <fd_server_drone_out> <fd_window_size_out>", argv[0]);
        return EXIT_FAILURE;
    }

    int fd_drone_in = atoi(argv[1]);
    int fd_server_drone_out = atoi(argv[2]);
    int fd_window_size_out = atoi(argv[3]);

    if (sim_params_load(NULL) != 0) {
        sim_log_info("network_client: warning: could not load config, using defaults");
    }
    const SimParams *params = sim_params_get();

    const char *address = params->server_address;
    if (!address || address[0] == '\0') address = NET_DEFAULT_ADDRESS;

    int port = params->server_port;
    if (port <= 0 || port > 65535) port = NET_DEFAULT_PORT;

    (void)set_nonblocking(fd_drone_in);
    (void)set_nonblocking(fd_server_drone_out);

    NetworkStats stats;
    stats_init(&stats);

    int world_w = 0, world_h = 0;
    int window_size_sent = 0;

    const int flip_y    = get_env_i("SIM_NET_FLIP_Y", 0);
    const int alpha_deg = norm_alpha_deg(get_env_i("SIM_NET_ALPHA", 0));
    sim_log_info("network_client: coord cfg flip_y=%d alpha=%d", flip_y, alpha_deg);

    while (running) {
        int sock = connect_with_retry(address, port, &stats);
        if (sock < 0) {
            sim_log_info("network_client: unable to establish connection, exiting");
            break;
        }

        (void)net_set_nonblocking(sock);

        LineAcc in = { .len = 0 };
        SendQ   out; sq_clear(&out);

        CState st = C_WAIT_OK;

        DroneState latest_client = {0};
        int have_client = 0;

        double tmp_x_v = 0.0, tmp_y_v = 0.0;
        
        int connection_alive = 1;

        while (running && st != C_DONE && connection_alive) {
            int dr = drain_latest_drone(fd_drone_in, &latest_client, &have_client);
            if (dr == 1) { 
                sim_log_info("network_client: bb_server closed drone pipe"); 
                connection_alive = 0;
                running = 0;
                break; 
            }
            if (dr < 0)  { 
                sim_log_info("network_client: error reading drone pipe"); 
                connection_alive = 0;
                running = 0;
                break; 
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
                perror("network_client: select");
                connection_alive = 0;
                break;
            }

            if (FD_ISSET(sock, &wfds)) {
                if (sq_flush(sock, &out, &stats) < 0) {
                    sim_log_info("network_client: socket send failed");
                    connection_alive = 0;
                    break;
                }
            }

            if (FD_ISSET(sock, &rfds)) {
                int rr = acc_recv_lines(sock, &in, &stats);
                if (rr == 1) { 
                    sim_log_info("network_client: server closed connection"); 
                    connection_alive = 0;
                    break; 
                }
                if (rr < 0)  { 
                    sim_log_info("network_client: socket recv error"); 
                    connection_alive = 0;
                    break; 
                }
            }

            // Log stats periodically (not sent to bb_server)
            static int stats_log_counter = 0;
            if (++stats_log_counter % 100 == 0) {
                sim_log_info("network_client: stats - sent:%llu recv:%llu lat:%.1fms bw:%.2fKB/s",
                             (unsigned long long)stats.packets_sent,
                             (unsigned long long)stats.packets_received,
                             stats.avg_latency_ms,
                             stats.bandwidth_kbps);
            }

            char line[256];
            while (acc_pop_line(&in, line, sizeof(line))) {
                if (st == C_WAIT_OK) {
                    if (strcmp(line, NET_TOK_OK) != 0) { 
                        stats.protocol_errors++;
                        st = C_DONE; 
                        break; 
                    }
                    (void)sq_append_line(&out, NET_TOK_OOK);
                    st = C_WAIT_SIZE;
                }
                else if (st == C_WAIT_SIZE) {
                    if (sscanf(line, "size %d,%d", &world_w, &world_h) == 2) {
                        (void)sq_append_line(&out, NET_TOK_SOK);

                        if (!window_size_sent) {
                            WindowDimensions dims = { world_w, world_h };
                            if (write_full(fd_window_size_out, &dims, sizeof(dims)) != (ssize_t)sizeof(dims)) {
                                sim_log_info("network_client: failed to forward window size to bb_server");
                                st = C_DONE;
                                break;
                            }
                            window_size_sent = 1;
                        }

                        sim_log_info("network_client: handshake complete, size=%dx%d", world_w, world_h);
                        st = C_RUN_WAIT_TOKEN;
                    } else {
                        sim_log_info("network_client: invalid size format: '%s'", line);
                        stats.protocol_errors++;
                        st = C_DONE;
                        break;
                    }
                }
                else if (st == C_RUN_WAIT_TOKEN) {
                    if (strcmp(line, NET_TOK_Q) == 0) {
                        (void)sq_append_line(&out, NET_TOK_QOK);
                        st = C_DONE;
                        break;
                    }
                    if (strcmp(line, NET_TOK_DRONE) == 0) {
                        st = C_RUN_WAIT_DRONE_COORDS;
                        continue;
                    }
                    if (strcmp(line, NET_TOK_OBST) == 0) {
                        double xl = have_client ? latest_client.x : 0.0;
                        double yl = have_client ? latest_client.y : 0.0;

                        double xv, yv;
                        coord_local_to_virtual(xl, yl, world_w, world_h, flip_y, alpha_deg, &xv, &yv);

                        char b[64];
                        snprintf(b, sizeof(b), "%.1f, %.1f", xv, yv);
                        (void)sq_append_line(&out, b);

                        st = C_RUN_WAIT_POK;
                        continue;
                    }

                    sim_log_info("network_client: unexpected token '%s'", line);
                    stats.protocol_errors++;
                    st = C_DONE;
                    break;
                }
                else if (st == C_RUN_WAIT_DRONE_COORDS) {
                    if (sscanf(line, "%lf, %lf", &tmp_x_v, &tmp_y_v) == 2 ||
                        sscanf(line, "%lf,%lf", &tmp_x_v, &tmp_y_v) == 2) {
                        (void)sq_append_line(&out, NET_TOK_DOK);

                        double xl, yl;
                        coord_virtual_to_local(tmp_x_v, tmp_y_v, world_w, world_h, flip_y, alpha_deg, &xl, &yl);

                        DroneState sd = {0};
                        sd.x = xl;
                        sd.y = yl;

                        int wr = try_write_struct(fd_server_drone_out, &sd, sizeof(sd));
                        if (wr < 0) { st = C_DONE; break; }

                        st = C_RUN_WAIT_TOKEN;
                    } else {
                        sim_log_info("network_client: invalid drone coordinate format: '%s'", line);
                        stats.protocol_errors++;
                        st = C_DONE;
                        break;
                    }
                }
                else if (st == C_RUN_WAIT_POK) {
                    if (strcmp(line, NET_TOK_POK) != 0) { 
                        stats.protocol_errors++;
                        st = C_DONE; 
                        break; 
                    }
                    st = C_RUN_WAIT_TOKEN;
                }
            }
        }

        sim_log_info("network_client: connection closed");
        stats_on_disconnect(&stats);
        close(sock);

        if (running && connection_alive == 0) {
            sim_log_info("network_client: attempting to reconnect...");
            sleep(1);
        } else {
            break;
        }
    }

    sim_log_info("network_client: shutting down");
    close(fd_drone_in);
    close(fd_server_drone_out);
    if (!window_size_sent) close(fd_window_size_out);
    sim_log_info("network_client: exited (total connections: %llu, drops: %llu)", 
                 (unsigned long long)stats.reconnect_attempts + 1,
                 (unsigned long long)stats.connection_drops);
    sim_log_close();
    return EXIT_SUCCESS;
}