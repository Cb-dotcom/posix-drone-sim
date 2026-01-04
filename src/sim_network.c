#define _POSIX_C_SOURCE 200809L
#include "sim_network.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/time.h>

// Socket I/O: Read exactly 'count' bytes, handling EINTR and partial reads
ssize_t socket_read_full(int sockfd, void *buf, size_t count)
{
    size_t total = 0;
    char *ptr = (char *)buf;

    while (total < count) {
        ssize_t n = recv(sockfd, ptr + total, count - total, 0);
        
        if (n < 0) {
            if (errno == EINTR) {
                continue;  // interrupted by signal, retry
            }
            return -1;  // real error
        }
        
        if (n == 0) {
            // Connection closed by peer
            return (total > 0) ? (ssize_t)total : 0;
        }
        
        total += (size_t)n;
    }

    return (ssize_t)total;
}

// Socket I/O: Write exactly 'count' bytes, handling EINTR and partial writes
ssize_t socket_write_full(int sockfd, const void *buf, size_t count)
{
    size_t total = 0;
    const char *ptr = (const char *)buf;

    while (total < count) {
        // MSG_NOSIGNAL prevents SIGPIPE on broken connections
        ssize_t n = send(sockfd, ptr + total, count - total, MSG_NOSIGNAL);
        
        if (n < 0) {
            if (errno == EINTR) {
                continue;  // interrupted by signal, retry
            }
            return -1;  // real error (including EPIPE)
        }
        
        total += (size_t)n;
    }

    return (ssize_t)total;
}

// Send a complete message: [header][payload]
int net_send_message(int sockfd, uint8_t type, const void *payload, size_t payload_len)
{
    NetHeader hdr;
    hdr.type = type;
    hdr.length = htons((uint16_t)payload_len);  // convert to network byte order

    // Send header
    if (socket_write_full(sockfd, &hdr, sizeof(hdr)) != sizeof(hdr)) {
        return -1;
    }

    // Send payload (if any)
    if (payload_len > 0 && payload != NULL) {
        if (socket_write_full(sockfd, payload, payload_len) != (ssize_t)payload_len) {
            return -1;
        }
    }

    return 0;
}

// Receive a complete message: [header][payload]
int net_receive_message(int sockfd, uint8_t *type, void *payload, size_t max_payload_len)
{
    NetHeader hdr;

    // Read header
    ssize_t r = socket_read_full(sockfd, &hdr, sizeof(hdr));
    if (r == 0) {
        return 0;  // EOF (connection closed)
    }
    if (r != sizeof(hdr)) {
        return -1;  // error or partial header
    }

    *type = hdr.type;
    uint16_t payload_len = ntohs(hdr.length);  // convert from network byte order

    // Validate payload length
    if (payload_len > max_payload_len) {
        return -1;  // payload too large for buffer
    }

    // Read payload (if any)
    if (payload_len > 0) {
        if (payload == NULL) {
            return -1;  // null buffer but payload expected
        }
        
        r = socket_read_full(sockfd, payload, payload_len);
        if (r != payload_len) {
            return -1;  // incomplete payload
        }
    }

    return (int)payload_len;
}

// Convenience: Send window size
int net_send_window_size(int sockfd, int width, int height)
{
    NetWindowSize msg;
    msg.width = htonl((uint32_t)width);
    msg.height = htonl((uint32_t)height);
    
    return net_send_message(sockfd, NET_MSG_WINDOW_SIZE, &msg, sizeof(msg));
}

// Convenience: Send drone position
int net_send_drone_pos(int sockfd, double x, double y, double vx, double vy)
{
    NetDronePos msg;
    msg.x = x;
    msg.y = y;
    msg.vx = vx;
    msg.vy = vy;
    
    return net_send_message(sockfd, NET_MSG_DRONE_POS, &msg, sizeof(msg));
}

// Convenience: Send disconnect notification
int net_send_disconnect(int sockfd)
{
    return net_send_message(sockfd, NET_MSG_DISCONNECT, NULL, 0);
}

// Convenience: Receive window size
int net_receive_window_size(int sockfd, int *width, int *height)
{
    uint8_t type;
    NetWindowSize msg;
    
    int r = net_receive_message(sockfd, &type, &msg, sizeof(msg));
    if (r <= 0) {
        return r;  // error or EOF
    }
    
    if (type != NET_MSG_WINDOW_SIZE) {
        return -1;  // unexpected message type
    }
    
    *width = (int)ntohl((uint32_t)msg.width);
    *height = (int)ntohl((uint32_t)msg.height);
    
    return 1;  // success
}

// Convenience: Receive drone position
int net_receive_drone_pos(int sockfd, double *x, double *y, double *vx, double *vy)
{
    uint8_t type;
    NetDronePos msg;
    
    int r = net_receive_message(sockfd, &type, &msg, sizeof(msg));
    if (r <= 0) {
        return r;  // error or EOF
    }
    
    if (type != NET_MSG_DRONE_POS) {
        return -1;  // unexpected message type
    }
    
    *x = msg.x;
    *y = msg.y;
    *vx = msg.vx;
    *vy = msg.vy;
    
    return 1;  // success
}

// Create server socket, bind, and listen
int net_create_server_socket(const char *address, int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        return -1;
    }

    // Allow address reuse (helpful for quick restarts)
    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        close(sockfd);
        return -1;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((uint16_t)port);

    // Parse address (supports "0.0.0.0" or specific IP)
    if (inet_pton(AF_INET, address, &server_addr.sin_addr) <= 0) {
        close(sockfd);
        return -1;
    }

    // Bind to address
    if (bind(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(sockfd);
        return -1;
    }

    // Listen for connections (backlog of 1, we only accept one client)
    if (listen(sockfd, 1) < 0) {
        close(sockfd);
        return -1;
    }

    return sockfd;
}

// Accept a client connection (blocking)
int net_accept_client(int listen_fd)
{
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);

    int client_fd = accept(listen_fd, (struct sockaddr *)&client_addr, &addr_len);
    if (client_fd < 0) {
        return -1;
    }

    return client_fd;
}

// Connect to server (blocking)
int net_connect_to_server(const char *address, int port)
{
    int sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        return -1;
    }

    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons((uint16_t)port);

    // Parse server address
    if (inet_pton(AF_INET, address, &server_addr.sin_addr) <= 0) {
        close(sockfd);
        return -1;
    }

    // Connect to server
    if (connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr)) < 0) {
        close(sockfd);
        return -1;
    }

    return sockfd;
}

// Set socket to non-blocking mode
int net_set_nonblocking(int sockfd)
{
    int flags = fcntl(sockfd, F_GETFL, 0);
    if (flags < 0) {
        return -1;
    }
    
    return fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);
}

// Set socket timeouts (in seconds)
int net_set_timeouts(int sockfd, int recv_timeout, int send_timeout)
{
    struct timeval tv;

    // Set receive timeout
    tv.tv_sec = recv_timeout;
    tv.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
        return -1;
    }

    // Set send timeout
    tv.tv_sec = send_timeout;
    tv.tv_usec = 0;
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) < 0) {
        return -1;
    }

    return 0;
}