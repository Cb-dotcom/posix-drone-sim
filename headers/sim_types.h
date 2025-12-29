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


} WorldState;

#endif