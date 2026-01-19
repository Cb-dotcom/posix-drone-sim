/*
    Shared data model for the simulator.

    DroneState  = physical state of the drone (position and velocity).
    CommandState = user command state (forces and control flags), written by the
                input process and consumed by the drone + UI.
    WorldState  = the full "blackboard" snapshot stored in shared memory and
                protected by a single global semaphore. All processes attach
                to the same WorldState instance and only access it under the
                SIM_SEM_WORLD mutex (check sim_ipc.h for more details).
*/

#ifndef SIM_TYPES_H
#define SIM_TYPES_H

#include <time.h> 
#include <signal.h>
#include <stdint.h>

typedef enum {
    SIM_MODE_NORMAL = 0,
    SIM_MODE_SERVER = 1,
    SIM_MODE_CLIENT = 2
} SimMode;


// Maximum sizes for obstacle/target arrays in WorldState.
#define SIM_MAX_OBSTACLES 64
#define SIM_MAX_TARGETS   32

typedef struct {
    double x;
    double y;
    double vx;
    double vy;
} DroneState;

typedef struct {
    double fx;       
    double fy;      
    int    brake;    
    int    reset;    
    int    quit;     
    int    last_key;
} CommandState;

typedef enum {
    CODE_AREA_INIT = 0,
    CODE_AREA_MAIN_LOOP,
    CODE_AREA_READ_PIPE,
    CODE_AREA_WRITE_PIPE,
    CODE_AREA_PHYSICS_UPDATE,
    CODE_AREA_UI_RENDER,
    CODE_AREA_SHUTDOWN
} CodeArea;

extern volatile sig_atomic_t g_current_code_area;

typedef struct {
    double x;       
    double y;
    double radius; 
    int    active;  
} Obstacle;

typedef struct {
    double x;       
    double y;
    double radius;  
    int    id;     
    int    active;  
    struct timespec time_created;
} Target;

// network state
typedef struct {
    // Connection state
    int connected;              // 0 = disconnected, 1 = connected
    int reconnect_attempts;     // Number of reconnection attempts
    
    // Packet statistics
    uint64_t packets_sent;      // Total packets sent
    uint64_t packets_received;  // Total packets received
    uint64_t bytes_sent;        // Total bytes sent
    uint64_t bytes_received;    // Total bytes received
    
    // Timing statistics
    double latency_ms;          // Current round-trip latency in milliseconds
    double avg_latency_ms;      // Moving average latency
    double bandwidth_kbps;      // Current bandwidth in KB/s
    
    // Error statistics
    uint64_t protocol_errors;   // Protocol violation count
    uint64_t connection_drops;  // Number of disconnects
    
    // Timestamps
    struct timespec last_packet_time;    // Last packet sent/received
    struct timespec connection_start;    // When connection was established
} NetworkStats;


typedef struct {
    DroneState   drone; 
    CommandState cmd; 
    
    int world_width;
    int world_height;

    int          num_obstacles;        
    int          obstacles_slots;      
    Obstacle     obstacles[SIM_MAX_OBSTACLES];

    int          num_targets;          
    int          targets_slots;        
    Target       targets[SIM_MAX_TARGETS];

    double       score;

    struct timespec sim_start_time;     // Simulation start
    struct timespec last_target_time;   // Last target collection
    double total_distance;              // Cumulative distance traveled
    int targets_collected;              // Number of targets hit
    int obstacle_collisions;            // Penalty counter

    // Network mode data
    SimMode mode;
    int     has_server_drone;  // client only
    double  server_drone_x;
    double  server_drone_y;

    NetworkStats net_stats;

} WorldState;

#endif