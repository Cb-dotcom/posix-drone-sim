#ifndef SIM_NETWORK_H
#define SIM_NETWORK_H

#include <stddef.h>
#include <sys/types.h>

// Default network parameters (overridden by config)
#define NET_DEFAULT_PORT       8888
#define NET_DEFAULT_ADDRESS    "127.0.0.1"
#define NET_MAX_ADDR_LEN       64

// Assignment 3 protocol tokens (newline-delimited)
#define NET_TOK_OK     "ok"
#define NET_TOK_OOK    "ook"
#define NET_TOK_SIZE   "size"
#define NET_TOK_SOK    "sok"
#define NET_TOK_DRONE  "drone"
#define NET_TOK_DOK    "dok"
#define NET_TOK_OBST   "obst"
#define NET_TOK_POK    "pok"
#define NET_TOK_Q      "q"
#define NET_TOK_QOK    "qok"

/*
    Socket I/O helpers (handle partial reads/writes and EINTR).
    These operate on raw bytes (not line/framing).
*/
ssize_t socket_read_full(int sockfd, void *buf, size_t count);
ssize_t socket_write_full(int sockfd, const void *buf, size_t count);

/*
    Line-based helpers (newline-delimited).
    - net_send_line(): sends "s\n"
    - net_recv_line(): reads one line into buf (without trailing '\n', trims optional '\r')
    Returns:
      >0: length of line
       0: EOF (peer closed cleanly before any byte)
      -1: error (errno set)
*/
int net_send_line(int sockfd, const char *s);
int net_recv_line(int sockfd, char *buf, size_t cap);

/*
    Token helper: reads a line and stores it in buf.
    Same return convention as net_recv_line().
*/
int net_receive_token(int sockfd, char *buf, size_t cap);

/*
    Assignment 3 protocol helpers.

    Startup handshake:
      server: net_send_window_size()
      client: net_receive_window_size()

    Drone exchange (server->client):
      server: net_send_drone_pos()  sends "drone", "x", "y" then waits "dok"
      client: net_receive_drone_pos() expects "drone", reads x,y then sends "dok"
        - also handles "q" by replying "qok" and returning 0

    Obstacle exchange (client->server):
      server: net_server_request_obstacle() sends "obst", reads x,y then sends "pok"
      client: net_client_send_obstacle() sends x,y then reads "pok"

    Return convention:
      1  success
      0  connection closed OR received 'q' (client sends qok internally)
     -1  error / protocol violation
*/
int net_send_window_size(int sockfd, int width, int height);
int net_receive_window_size(int sockfd, int *width, int *height);

int net_send_drone_pos(int sockfd, double x, double y, double vx, double vy);
int net_receive_drone_pos(int sockfd, double *x, double *y, double *vx, double *vy);

int net_server_request_obstacle(int sockfd, double *x, double *y);
int net_client_send_obstacle(int sockfd, double x, double y);

int net_send_disconnect(int sockfd); // sends 'q' and waits for 'qok'

// Socket setup helpers
int net_create_server_socket(const char *address, int port);
int net_accept_client(int listen_fd);
int net_connect_to_server(const char *address, int port);
int net_set_nonblocking(int sockfd);
int net_set_timeouts(int sockfd, int recv_timeout, int send_timeout);

#endif
