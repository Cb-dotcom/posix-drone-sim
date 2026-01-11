#define _POSIX_C_SOURCE 200809L
#include "sim_network.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

// ------------------------- raw full read/write -------------------------

ssize_t socket_read_full(int sockfd, void *buf, size_t count)
{
    size_t total = 0;
    char *ptr = (char *)buf;

    while (total < count) {
        ssize_t n = recv(sockfd, ptr + total, count - total, 0);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (n == 0) {
            return (total > 0) ? (ssize_t)total : 0;
        }
        total += (size_t)n;
    }
    return (ssize_t)total;
}

ssize_t socket_write_full(int sockfd, const void *buf, size_t count)
{
    size_t total = 0;
    const char *ptr = (const char *)buf;

    while (total < count) {
        ssize_t n = send(sockfd, ptr + total, count - total, MSG_NOSIGNAL);
        if (n < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        total += (size_t)n;
    }
    return (ssize_t)total;
}

// ------------------------- line based helpers -------------------------

int net_send_line(int sockfd, const char *s)
{
    if (!s) {
        errno = EINVAL;
        return -1;
    }

    size_t len = strlen(s);
    // send s
    if (len > 0) {
        if (socket_write_full(sockfd, s, len) != (ssize_t)len) return -1;
    }
    // send '\n'
    char nl = '\n';
    if (socket_write_full(sockfd, &nl, 1) != 1) return -1;

    return 0;
}

int net_recv_line(int sockfd, char *buf, size_t cap)
{
    if (!buf || cap == 0) {
        errno = EINVAL;
        return -1;
    }

    size_t i = 0;

    for (;;) {
        char c = 0;
        ssize_t r = recv(sockfd, &c, 1, 0);
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        if (r == 0) {
            // EOF before any byte
            if (i == 0) return 0;
            // EOF after partial line (treat as line end)
            break;
        }

        if (c == '\n') {
            break;
        }

        if (i + 1 >= cap) {
            errno = EMSGSIZE;
            return -1;
        }

        buf[i++] = c;
    }

    // trim optional '\r' (CRLF)
    if (i > 0 && buf[i - 1] == '\r') {
        i--;
    }

    buf[i] = '\0';
    return (int)i;
}

int net_receive_token(int sockfd, char *buf, size_t cap)
{
    return net_recv_line(sockfd, buf, cap);
}

// ------------------------- small parsing/format helpers -------------------------

static int net_send_double(int sockfd, double v)
{
    char tmp[64];
    // plenty for double; keep it consistent and parseable
    // (%.10g is a good compromise; you can change to %.6f if you prefer)
    snprintf(tmp, sizeof(tmp), "%.10g", v);
    return net_send_line(sockfd, tmp);
}

static int net_recv_double(int sockfd, double *out)
{
    char tmp[128];
    int r = net_recv_line(sockfd, tmp, sizeof(tmp));
    if (r <= 0) return r; // 0 EOF, -1 error

    char *end = NULL;
    errno = 0;
    double v = strtod(tmp, &end);
    if (errno != 0 || !end || *end != '\0') {
        errno = EPROTO;
        return -1;
    }

    *out = v;
    return 1;
}

static int net_send_int(int sockfd, int v)
{
    char tmp[64];
    snprintf(tmp, sizeof(tmp), "%d", v);
    return net_send_line(sockfd, tmp);
}

static int net_recv_int(int sockfd, int *out)
{
    char tmp[128];
    int r = net_recv_line(sockfd, tmp, sizeof(tmp));
    if (r <= 0) return r;

    char *end = NULL;
    errno = 0;
    long vv = strtol(tmp, &end, 10);
    if (errno != 0 || !end || *end != '\0') {
        errno = EPROTO;
        return -1;
    }

    *out = (int)vv;
    return 1;
}

// ------------------------- protocol: startup handshake -------------------------

// server side: snd ok; rcv ook; snd size; snd l; snd h; rcv sok
int net_send_window_size(int sockfd, int width, int height)
{
    char tok[64];

    if (net_send_line(sockfd, NET_TOK_OK) != 0) return -1;

    if (net_receive_token(sockfd, tok, sizeof(tok)) <= 0) return -1;
    if (strcmp(tok, NET_TOK_OOK) != 0) { errno = EPROTO; return -1; }

    if (net_send_line(sockfd, NET_TOK_SIZE) != 0) return -1;
    if (net_send_int(sockfd, width) != 0) return -1;
    if (net_send_int(sockfd, height) != 0) return -1;

    if (net_receive_token(sockfd, tok, sizeof(tok)) <= 0) return -1;
    if (strcmp(tok, NET_TOK_SOK) != 0) { errno = EPROTO; return -1; }

    return 0;
}

// client side: rcv ok; snd ook; rcv size; rcv l; rcv h; snd sok
int net_receive_window_size(int sockfd, int *width, int *height)
{
    char tok[64];

    int r = net_receive_token(sockfd, tok, sizeof(tok));
    if (r <= 0) return r;
    if (strcmp(tok, NET_TOK_OK) != 0) { errno = EPROTO; return -1; }

    if (net_send_line(sockfd, NET_TOK_OOK) != 0) return -1;

    r = net_receive_token(sockfd, tok, sizeof(tok));
    if (r <= 0) return r;
    if (strcmp(tok, NET_TOK_SIZE) != 0) { errno = EPROTO; return -1; }

    if (net_recv_int(sockfd, width) <= 0) return -1;
    if (net_recv_int(sockfd, height) <= 0) return -1;

    if (net_send_line(sockfd, NET_TOK_SOK) != 0) return -1;
    return 1;
}

// ------------------------- protocol: drone exchange -------------------------

// server side: snd drone; snd x; snd y; rcv dok
int net_send_drone_pos(int sockfd, double x, double y, double vx, double vy)
{
    (void)vx; (void)vy;

    char tok[64];

    if (net_send_line(sockfd, NET_TOK_DRONE) != 0) return -1;
    if (net_send_double(sockfd, x) != 0) return -1;
    if (net_send_double(sockfd, y) != 0) return -1;

    int r = net_receive_token(sockfd, tok, sizeof(tok));
    if (r <= 0) return r;
    if (strcmp(tok, NET_TOK_DOK) != 0) { errno = EPROTO; return -1; }

    return 1;
}

// client side: rcv token; if q -> snd qok return 0; if drone -> rcv x,y; snd dok
int net_receive_drone_pos(int sockfd, double *x, double *y, double *vx, double *vy)
{
    (void)vx; (void)vy;

    char tok[64];

    int r = net_receive_token(sockfd, tok, sizeof(tok));
    if (r <= 0) return r;

    if (strcmp(tok, NET_TOK_Q) == 0) {
        // server quitting
        (void)net_send_line(sockfd, NET_TOK_QOK);
        return 0;
    }

    if (strcmp(tok, NET_TOK_DRONE) != 0) {
        errno = EPROTO;
        return -1;
    }

    if (net_recv_double(sockfd, x) <= 0) return -1;
    if (net_recv_double(sockfd, y) <= 0) return -1;

    if (net_send_line(sockfd, NET_TOK_DOK) != 0) return -1;

    return 1;
}

// ------------------------- protocol: obstacle exchange -------------------------

// server side: snd obst; rcv x; rcv y; snd pok
int net_server_request_obstacle(int sockfd, double *x, double *y)
{
    if (net_send_line(sockfd, NET_TOK_OBST) != 0) return -1;

    int r = net_recv_double(sockfd, x);
    if (r <= 0) return r;

    r = net_recv_double(sockfd, y);
    if (r <= 0) return r;

    if (net_send_line(sockfd, NET_TOK_POK) != 0) return -1;
    return 1;
}

// client side: (after receiving 'obst' token in your loop) snd x; snd y; rcv pok
int net_client_send_obstacle(int sockfd, double x, double y)
{
    char tok[64];

    if (net_send_double(sockfd, x) != 0) return -1;
    if (net_send_double(sockfd, y) != 0) return -1;

    int r = net_receive_token(sockfd, tok, sizeof(tok));
    if (r <= 0) return r;
    if (strcmp(tok, NET_TOK_POK) != 0) { errno = EPROTO; return -1; }

    return 1;
}

// ------------------------- protocol: disconnect -------------------------

// server side: snd q; rcv qok
int net_send_disconnect(int sockfd)
{
    char tok[64];

    if (net_send_line(sockfd, NET_TOK_Q) != 0) return -1;

    int r = net_receive_token(sockfd, tok, sizeof(tok));
    if (r <= 0) return -1;
    if (strcmp(tok, NET_TOK_QOK) != 0) { errno = EPROTO; return -1; }

    return 0;
}

// ------------------------- socket setup helpers -------------------------

int net_create_server_socket(const char *address, int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) return -1;

    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        close(sockfd);
        return -1;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((uint16_t)port);

    if (inet_pton(AF_INET, address, &server_addr.sin_addr) <= 0) {
        close(sockfd);
        return -1;
    }

    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(sockfd);
        return -1;
    }

    if (listen(sockfd, 1) < 0) {
        close(sockfd);
        return -1;
    }

    return sockfd;
}

int net_accept_client(int listen_fd)
{
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &addr_len);
    if (client_fd < 0) return -1;
    return client_fd;
}

int net_connect_to_server(const char *address, int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) return -1;

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((uint16_t)port);

    if (inet_pton(AF_INET, address, &server_addr.sin_addr) <= 0) {
        close(sockfd);
        return -1;
    }

    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(sockfd);
        return -1;
    }

    return sockfd;
}

int net_set_nonblocking(int sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) return -1;
    return fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

int net_set_timeouts(int sockfd, int recv_timeout, int send_timeout)
{
    struct timeval tv;

    tv.tv_sec = recv_timeout;
    tv.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) return -1;

    tv.tv_sec = send_timeout;
    tv.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) return -1;

    return 0;
}
