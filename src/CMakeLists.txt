add_library(sim_core STATIC
    sim_log.c
    sim_params.c
    sim_ipc.c
)

target_link_libraries(sim_core
    PUBLIC
        sim_headers
)

# New: UI library
add_library(sim_ui STATIC
    sim_ui.c
)

target_link_libraries(sim_ui
    PRIVATE
        sim_headers
        ncurses
)

# Executables (processes)
add_executable(master    master.c)
add_executable(bb_server bb_server.c)
add_executable(drone     drone.c)
add_executable(input     input.c)
add_executable(obstacles obstacles.c)
add_executable(targets   targets.c)
add_executable(watchdog  watchdog.c)

set(SIM_EXECUTABLES master bb_server drone input obstacles targets watchdog)

foreach(target ${SIM_EXECUTABLES})
    target_link_libraries(${target}
        PRIVATE
            sim_core
            sim_ui       
            sim_headers
            m
            ncurses
    )
endforeach()
