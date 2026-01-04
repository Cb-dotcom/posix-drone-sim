#ifndef SIM_PARAMS_H
#define SIM_PARAMS_H
#include "sim_types.h" 

// Default config file path used if sim_params_load(NULL) is called.
#define SIM_PARAMS_DEFAULT_PATH "../../bin/conf/drone_parameters.conf"

/*
    Global simulation parameters.

    NOTE about naming:
    - The struct fields are still named num_targets / num_obstacles (historical).
    - Your conf keys "max_targets" and "max_obstacles" are supported and are parsed
      into num_targets / num_obstacles in sim_params.c.
*/

typedef struct {
    // World geometry (simulation coordinates)
    int    world_width;
    int    world_height;

    // Drone dynamics
    double mass;
    double damping;
    double dt;

    // User command forces
    double force_step;
    double max_force;

    // Potential-field repulsion parameters
    double rho;
    double eta;

    // Environment population (caps)
    int    num_obstacles;
    int    num_targets;

    // Environment spawn control
    int    initial_obstacles;
    int    initial_targets;
    double obstacle_spawn_interval;
    double target_spawn_interval;

    // network settings
    SimMode mode;
    char    server_address[64];
    int     server_port;
} SimParams;

int sim_params_load(const char *path);
const SimParams *sim_params_get(void);
void sim_params_get_copy(SimParams *out);

#endif
