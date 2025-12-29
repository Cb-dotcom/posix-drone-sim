/*
    Minimal logging interface shared by all simulator processes.
    sim_log_init() sets up per-process logging.
    sim_log_info() writes formatted INFO-level messages.
    sim_log_close() allows cleanup on exit.

    Not used yet in our implementation. We might opt for a 
    more complex bin/log/<process>.log redirection later.
*/

#ifndef SIM_LOG_H
#define SIM_LOG_H
#include <sys/types.h>

void sim_log_init(const char *process_name);
void sim_log_info(const char *fmt, ...);
void sim_log_close(void);
void sim_process_register(const char *process_name, pid_t pid);

#endif
