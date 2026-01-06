/*
    Network Client Process
    
    Responsibilities:
    1. Connect to server via TCP
    2. Receive window dimensions from server
    3. Send window dimensions to bb_server via pipe
    4. Continuously:
       - Read client's own drone position from bb_server via pipe
       - Send drone position to server via socket
       - Receive server's drone position from socket
       - Write server drone position to bb_server via pipe
    5. Handle disconnection gracefully and exit
    
    Command-line arguments (passed by master):
    argv[1] = fd_drone_pos_in       (read client's drone from bb_server)
    argv[2] = fd_server_drone_out   (write server's drone to bb_server)
    argv[3] = fd_window_size_out    (write window dimensions to bb_server)
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
    sim_log_init("network_client");
    sim_process_register("network_client", getpid());

    signal(SIGINT, handle_sigint);
    signal(SIGPIPE, SIG_IGN);

    // Parse command-line arguments
    if (argc < 4) {
        sim_log_info("network_client: usage: %s <fd_drone_pos_in> <fd_server_drone_out> <fd_window_size_out>",
                     argv[0]);
        return EXIT_FAILURE;
    }

    int fd_drone_in = atoi(argv[1]);        // read client's drone position
    int fd_server_drone_out = atoi(argv[2]); // write server's drone position
    int fd_window_size_out = atoi(argv[3]);  // write window dimensions

    // Load configuration for network settings
    if (sim_params_load(NULL) != 0) {
        sim_log_info("network_client: warning: could not load config, using defaults");
    }
    const SimParams *params = sim_params_get();

    // Get server address and port from config
    const char *address = params->server_address;
    if (address == NULL || address[0] == '\0') {
        address = NET_DEFAULT_ADDRESS;
    }

    int port = params->server_port;
    if (port <= 0 || port > 65535) {
        port = NET_DEFAULT_PORT;
    }

    sim_log_info("network_client: connecting to %s:%d", address, port);

    // Connect to server
    int server_fd = net_connect_to_server(address, port);
    if (server_fd < 0) {
        perror("network_client: net_connect_to_server");
        sim_log_info("network_client: failed to connect to server");
        return EXIT_FAILURE;
    }

    sim_log_info("network_client: connected to server!");

    // Set socket timeouts
    if (net_set_timeouts(server_fd, 5, 5) != 0) {
        sim_log_info("network_client: warning: could not set socket timeouts");
    }

    // Receive window dimensions from server
    int world_width, world_height;
    int recv_result = net_receive_window_size(server_fd, &world_width, &world_height);
    
    if (recv_result <= 0) {
        sim_log_info("network_client: failed to receive window size from server");
        close(server_fd);
        return EXIT_FAILURE;
    }

    sim_log_info("network_client: received window size (%dx%d) from server",
                 world_width, world_height);

    // Send window dimensions to bb_server via pipe
    // bb_server expects a simple struct with width/height
    typedef struct {
        int width;
        int height;
    } WindowDimensions;

    WindowDimensions win_dims;
    win_dims.width = world_width;
    win_dims.height = world_height;

    ssize_t w = write_full(fd_window_size_out, &win_dims, sizeof(win_dims));
    if (w != (ssize_t)sizeof(win_dims)) {
        sim_log_info("network_client: failed to send window size to bb_server");
        close(server_fd);
        return EXIT_FAILURE;
    }

    sim_log_info("network_client: sent window dimensions to bb_server");

    // Main communication loop
    DroneState client_drone;
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
            perror("network_client: select");
            break;
        }

        // Read client's drone position from pipe if available
        if (ready > 0 && FD_ISSET(fd_drone_in, &readfds)) {
            ssize_t r = read_full(fd_drone_in, &client_drone, sizeof(client_drone));
            if (r == (ssize_t)sizeof(client_drone)) {
                have_drone = 1;
            } else if (r == 0) {
                sim_log_info("network_client: drone pipe closed (bb_server quit)");
                running = 0;
                break;
            } else {
                sim_log_info("network_client: error reading from drone pipe");
                running = 0;
                break;
            }
        }

        // IMPORTANT: Check if running flag was set to 0
        if (!running) {
            break;
        }

        // Send client drone position to server (if we have data)
        if (have_drone) {
            if (net_send_drone_pos(server_fd,
                                   client_drone.x,
                                   client_drone.y,
                                   client_drone.vx,
                                   client_drone.vy) != 0) {
                sim_log_info("network_client: failed to send drone pos to server");
                break;
            }
        }

        // Receive server drone position from socket
        double server_x, server_y, server_vx, server_vy;
        recv_result = net_receive_drone_pos(server_fd,
                                           &server_x,
                                           &server_y,
                                           &server_vx,
                                           &server_vy);
        
        if (recv_result == 0) {
            sim_log_info("network_client: server disconnected");
            break;
        } else if (recv_result < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                // Timeout, no data yet - continue
                continue;
            }
            sim_log_info("network_client: error receiving from server");
            break;
        }

        // Send server drone position to bb_server
        DroneState server_drone;
        server_drone.x = server_x;
        server_drone.y = server_y;
        server_drone.vx = server_vx;
        server_drone.vy = server_vy;

        // Debug log
        sim_log_info("network_client: received server drone at (%.2f, %.2f)",
                     server_x, server_y);

        w = write_full(fd_server_drone_out, &server_drone, sizeof(server_drone));
        if (w != (ssize_t)sizeof(server_drone)) {
            sim_log_info("network_client: error writing server drone to bb_server");
            break;
        }
    }

    // Clean shutdown
    sim_log_info("network_client: shutting down");
    
    close(server_fd);
    close(fd_drone_in);
    close(fd_server_drone_out);
    close(fd_window_size_out);

    sim_log_info("network_client: exited");
    sim_log_close();
    return EXIT_SUCCESS;
}