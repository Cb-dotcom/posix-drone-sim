/* 
    Global Simulation const headers. 
    It defines physical model defaults &
    world geometry in case we have an error in param file. 
*/

#ifndef SIM_CONST_H
#define SIM_CONST_H
#define NET_DEFAULT_ADDRESS "127.0.0.1"
#define NET_DEFAULT_PORT 8888

// Drone dynamics
static const double SIM_DEFAULT_MASS    = 1.0;
static const double SIM_DEFAULT_DAMPING = 1.0;
static const double SIM_DEFAULT_DT      = 0.05;

// Window size
static const double SIM_WORLD_WIDTH     = 50.0;
static const double SIM_WORLD_HEIGHT    = 50.0;

// User command forces
static const double SIM_DEFAULT_FORCE_STEP = 1.5; 
static const double SIM_DEFAULT_MAX_FORCE  = 15.0;

// Repulsion params
static const double SIM_DEFAULT_RHO    = 5.0;  
static const double SIM_DEFAULT_ETA    = 1.0;

// Env population (caps)
static const int    SIM_DEFAULT_NUM_OBSTACLES = 20;   
static const int    SIM_DEFAULT_NUM_TARGETS   = 20;   

// Env population (spawn control)
static const int    SIM_DEFAULT_INITIAL_OBSTACLES       = 10;  
static const int    SIM_DEFAULT_INITIAL_TARGETS         = 10;  
static const double SIM_DEFAULT_OBSTACLE_SPAWN_INTERVAL = 2.0;  
static const double SIM_DEFAULT_TARGET_SPAWN_INTERVAL   = 2.0;  

#endif
