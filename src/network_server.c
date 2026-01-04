/*
    Network Server Process
    
    Responsibilities:
    1. Create TCP server socket and wait for ONE client connection
    2. Send window dimensions to client at connection start
    3. Continuously:
       - Read server's drone position from bb_server via pipe
       - Send drone position to client via socket
       - Receive client's drone position from socket
       - Write client drone as obstacle to bb_server via pipe
    4. Handle disconnection and shutdown gracefully
    
    Command-line arguments (passed by master):
    argv[1] = fd_drone_pos_in   (read server's drone from bb_server)
    argv[2] = fd_obstacle_out   (write client drone as obstacle to bb_server)
*/

#define _POSIX_C_SOURCE 200809L
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <unistd.h>

#include "sim_const.h"
#include "sim_ipc.h"
#include "sim_log.h"
#include "sim_network.h"
#include "sim_params.h"
#include "sim_types.h"

// Global flag for signal-based shutdown
static volatile sig_atomic_t running = 1;

static void handle_sigint(int sig)
{
    (void)sig;
    running = 0;
}

int main(int argc, char *argv[])
{
    sim_log_init("network_server");
    sim_process_register("network_server", getpid());

    signal(SIGINT, handle_sigint);
    signal(SIGPIPE, SIG_IGN);  // ignore broken pipe errors

    // Parse command-line arguments
    if (argc < 3) {
        sim_log_info("network_server: usage: %s <fd_drone_pos_in> <fd_obstacle_out>",
                     argv[0]);
        return EXIT_FAILURE;
    }

    int fd_drone_in = atoi(argv[1]);    // read server drone position
    int fd_obstacle_out = atoi(argv[2]); // write client drone as obstacle

    // Load configuration for network settings
    if (sim_params_load(NULL) != 0) {
        sim_log_info("network_server: warning: could not load config, using defaults");
    }
    const SimParams *params = sim_params_get();

    const char *address = "0.0.0.0";  // listen on all interfaces
    int port = params->server_port;
    if (port <= 0 || port > 65535) {
        port = NET_DEFAULT_PORT;
    }

    sim_log_info("network_server: starting on %s:%d", address, port);

    // Create server socket
    int listen_fd = net_create_server_socket(address, port);
    if (listen_fd < 0) {
        perror("network_server: net_create_server_socket");
        sim_log_info("network_server: failed to create server socket");
        return EXIT_FAILURE;
    }

    sim_log_info("network_server: listening for client connection...");

    // Accept ONE client connection (blocking)
    int client_fd = net_accept_client(listen_fd);
    if (client_fd < 0) {
        perror("network_server: net_accept_client");
        sim_log_info("network_server: failed to accept client");
        close(listen_fd);
        return EXIT_FAILURE;
    }

    sim_log_info("network_server: client connected!");

    // No longer need listening socket
    close(listen_fd);

    // Set socket timeouts to detect dead connections
    if (net_set_timeouts(client_fd, 5, 5) != 0) {
        sim_log_info("network_server: warning: could not set socket timeouts");
    }

    // Send window dimensions to client
    int world_width = params->world_width;
    int world_height = params->world_height;
    
    if (net_send_window_size(client_fd, world_width, world_height) != 0) {
        sim_log_info("network_server: failed to send window size to client");
        close(client_fd);
        return EXIT_FAILURE;
    }

    sim_log_info("network_server: sent window size (%dx%d) to client",
                 world_width, world_height);

    // Main communication loop
    DroneState server_drone;
    int have_drone = 0;

    while (running) {
        // Use select() to check for available data on pipe (non-blocking check)
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(fd_drone_in, &readfds);

        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 50000;  // 50ms timeout

        int ready = select(fd_drone_in + 1, &readfds, NULL, NULL, &tv);
        if (ready < 0) {
            if (errno == EINTR) continue;
            perror("network_server: select");
            break;
        }

        // Read server's drone position from pipe if available
        if (ready > 0 && FD_ISSET(fd_drone_in, &readfds)) {
            ssize_t r = read_full(fd_drone_in, &server_drone, sizeof(server_drone));
            if (r == (ssize_t)sizeof(server_drone)) {
                have_drone = 1;
            } else if (r == 0) {
                sim_log_info("network_server: drone pipe closed (EOF)");
                break;
            } else {
                sim_log_info("network_server: error reading from drone pipe");
                break;
            }
        }

        // Send server drone position to client (if we have data)
        if (have_drone) {
            if (net_send_drone_pos(client_fd,
                                   server_drone.x,
                                   server_drone.y,
                                   server_drone.vx,
                                   server_drone.vy) != 0) {
                sim_log_info("network_server: failed to send drone pos to client");
                break;
            }
        }

        // Receive client drone position from socket
        double client_x, client_y, client_vx, client_vy;
        int recv_result = net_receive_drone_pos(client_fd,
                                                &client_x,
                                                &client_y,
                                                &client_vx,
                                                &client_vy);
        
        if (recv_result == 0) {
            sim_log_info("network_server: client disconnected");
            break;
        } else if (recv_result < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout, no data yet - continue
                continue;
            }
            sim_log_info("network_server: error receiving from client");
            break;
        }

        // Convert client drone to obstacle and send to bb_server
        Obstacle client_obstacle;
        client_obstacle.x = client_x;
        client_obstacle.y = client_y;
        client_obstacle.radius = 0.5;  // same as drone radius
        client_obstacle.active = 1;

        ssize_t w = write_full(fd_obstacle_out, &client_obstacle, sizeof(client_obstacle));
        if (w != (ssize_t)sizeof(client_obstacle)) {
            sim_log_info("network_server: error writing obstacle to bb_server");
            break;
        }
    }

    // Clean shutdown
    sim_log_info("network_server: shutting down");
    
    // Notify client of disconnect
    net_send_disconnect(client_fd);
    
    close(client_fd);
    close(fd_drone_in);
    close(fd_obstacle_out);

    sim_log_info("network_server: exited");
    sim_log_close();
    return EXIT_SUCCESS;
}