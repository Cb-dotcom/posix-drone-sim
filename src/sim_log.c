#include "sim_log.h"

#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <string.h>
#include <sys/file.h> // Required for flock()
#include <unistd.h>   // Required for fileno()

static FILE *log_fp = NULL;
static int   log_owns_fp = 0; 
static int   is_shared_log = 0; // Flag to track if we need locking

// Internal helper: get ISO-like timestamp "YYYY-MM-DD HH:MM:SS"
static void log_get_timestamp(char *buf, size_t buf_size)
{
    if (!buf || buf_size == 0) return;

    time_t now = time(NULL);
    struct tm tm_now;

#if defined(_POSIX_THREAD_SAFE_FUNCTIONS)
    localtime_r(&now, &tm_now);
#else
    struct tm *tmp = localtime(&now);
    if (!tmp) {
        buf[0] = '\0';
        return;
    }
    tm_now = *tmp;
#endif

    if (strftime(buf, buf_size, "%Y-%m-%d %H:%M:%S", &tm_now) == 0) {
        buf[0] = '\0';
    }
}

void sim_log_init(const char *process_name)
{
    if (log_fp) return; 

    if (!process_name || process_name[0] == '\0') {
        log_fp = stderr;
        log_owns_fp = 0;
        return;
    }

    char path[256];
    const char *mode;

    // --- Logic Split: Watchdog vs. Others ---
    if (strcmp(process_name, "watchdog") == 0) {
        // Watchdog gets its own private log
        // "w" mode deletes previous content (Fresh Start)
        snprintf(path, sizeof(path), "../../bin/log/watchdog.log");
        mode = "w"; 
        is_shared_log = 0;
    } else {
        // All others share one file
        // "a" mode allows appending without wiping other running processes
        snprintf(path, sizeof(path), "../../bin/log/processes.log");
        mode = "a"; 
        is_shared_log = 1;
    }

    FILE *fp = fopen(path, mode);
    if (!fp) {
        log_fp = stderr;
        log_owns_fp = 0;
        fprintf(stderr, 
                "sim_log: could not open '%s' (mode %s), falling back to stderr\n", 
                path, mode);
        return;
    }

    // Line buffering
    setvbuf(fp, NULL, _IOLBF, 0);

    log_fp = fp;
    log_owns_fp = 1;

    // Header
    char ts[32];
    log_get_timestamp(ts, sizeof(ts));
    
    // Acquire Lock for the Header write if shared
    if (is_shared_log) flock(fileno(log_fp), LOCK_EX);
    
    fprintf(log_fp, "[%s] [INFO] --- %s started ---\n",
            (ts[0] ? ts : "??????????"), process_name);
            
    if (is_shared_log) flock(fileno(log_fp), LOCK_UN);
}

void sim_log_info(const char *fmt, ...)
{
    if (!log_fp) {
        log_fp = stderr;
        log_owns_fp = 0;
    }

    char ts[32];
    log_get_timestamp(ts, sizeof(ts));

    // --- CRITICAL SECTION START ---
    // If shared, lock the file descriptor to prevent interleaved writes
    if (is_shared_log) {
        flock(fileno(log_fp), LOCK_EX);
    }

    fprintf(log_fp, "[%s] [INFO] ", (ts[0] ? ts : "??????????"));

    va_list ap;
    va_start(ap, fmt);
    vfprintf(log_fp, fmt, ap);
    va_end(ap);

    fputc('\n', log_fp);
    fflush(log_fp); // Ensure data hits disk before unlocking

    // --- CRITICAL SECTION END ---
    if (is_shared_log) {
        flock(fileno(log_fp), LOCK_UN);
    }
}

void sim_log_close(void)
{
    if (log_fp && log_owns_fp) {
        fclose(log_fp); // fclose releases any flock locks automatically
    }
    log_fp = NULL;
    log_owns_fp = 0;
}