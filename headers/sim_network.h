/*
    Network protocol definitions and socket helper functions.
    
    Protocol Overview:
    - TCP-based client-server communication
    - Server sends window dimensions at connection start
    - Bidirectional drone position updates
    - Graceful disconnect handling
    
    Message Format:
    All messages follow: [NetHeader][Payload]
    Header contains message type and payload length
*/

#ifndef SIM_NETWORK_H
#define SIM_NETWORK_H

#include <stdint.h>
#include <stddef.h>
#include <sys/types.h>

// Message type identifiers
#define NET_MSG_WINDOW_SIZE    1
#define NET_MSG_DRONE_POS      2
#define NET_MSG_DISCONNECT     3

// Default network parameters (overridden by config)
#define NET_DEFAULT_PORT       8888
#define NET_DEFAULT_ADDRESS    "127.0.0.1"
#define NET_MAX_ADDR_LEN       64

// Network message header (fixed size, always sent first)
typedef struct {
    uint8_t  type;      // Message type (NET_MSG_*)
    uint16_t length;    // Payload length in bytes (network byte order)
} __attribute__((packed)) NetHeader;

// Payload: Window dimensions (sent once at connection start)
typedef struct {
    int32_t width;      // World width (network byte order)
    int32_t height;     // World height (network byte order)
} __attribute__((packed)) NetWindowSize;

// Payload: Drone position update (sent continuously)
typedef struct {
    double x;           // X position
    double y;           // Y position
    double vx;          // X velocity
    double vy;          // Y velocity
} __attribute__((packed)) NetDronePos;

/*
    Socket I/O helpers (handle partial reads/writes and EINTR)
*/

// Read exactly 'count' bytes from socket, handling interrupts
// Returns: count on success, 0 on EOF, -1 on error
ssize_t socket_read_full(int sockfd, void *buf, size_t count);

// Write exactly 'count' bytes to socket, handling interrupts
// Returns: count on success, -1 on error
ssize_t socket_write_full(int sockfd, const void *buf, size_t count);

/*
    High-level message functions
*/

// Send a complete message (header + payload)
// Returns: 0 on success, -1 on error
int net_send_message(int sockfd, uint8_t type, const void *payload, size_t payload_len);

// Receive a complete message (header + payload)
// Reads header first, then payload into provided buffer
// Returns: payload length on success, 0 on EOF, -1 on error
// Sets *type to the received message type
int net_receive_message(int sockfd, uint8_t *type, void *payload, size_t max_payload_len);

/*
    Specialized message senders (convenience wrappers)
*/

int net_send_window_size(int sockfd, int width, int height);
int net_send_drone_pos(int sockfd, double x, double y, double vx, double vy);
int net_send_disconnect(int sockfd);

/*
    Specialized message receivers (type-safe wrappers)
*/

// Returns: 1 on success, 0 on EOF, -1 on error
int net_receive_window_size(int sockfd, int *width, int *height);
int net_receive_drone_pos(int sockfd, double *x, double *y, double *vx, double *vy);

/*
    Socket setup helpers
*/

// Create server socket, bind, and listen
// Returns: listening socket fd on success, -1 on error
int net_create_server_socket(const char *address, int port);

// Accept a client connection (blocking)
// Returns: client socket fd on success, -1 on error
int net_accept_client(int listen_fd);

// Connect to server (blocking with timeout)
// Returns: connected socket fd on success, -1 on error
int net_connect_to_server(const char *address, int port);

// Set socket to non-blocking mode
int net_set_nonblocking(int sockfd);

// Set socket timeouts (in seconds)
int net_set_timeouts(int sockfd, int recv_timeout, int send_timeout);

#endif