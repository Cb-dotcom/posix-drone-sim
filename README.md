<a id="readme-top"></a>

# Table of Contents
<details>
  <summary>View Dropdown</summary>
  <ol>
    <li><a href="#posix-drone-simulator---assignment-2">POSIX Drone Simulator – Assignment 2</a></li>
    <li><a href="#beloved-contributors">Beloved Contributors</a></li>
    <li><a href="#detailed-description">Detailed Description</a>
      <ul>
        <li><a href="#assignment-2-overview">Assignment 2 Overview</a></li>
        <li><a href="#how-to-build-and-run">How to build and run</a>
          <ul>
            <li><a href="#prerequisites">Prerequisites</a></li>
            <li><a href="#build">Build</a></li>
          </ul>
        </li>
        <li><a href="#watchdog-implementation">Watchdog Implementation</a>
          <ul>
            <li><a href="#watchdog-architecture">Watchdog Architecture</a></li>
            <li><a href="#watchdog-process">Watchdog Process</a></li>
            <li><a href="#ping-ack-protocol">Ping-Ack Protocol</a></li>
            <li><a href="#fault-detection-and-recovery">Fault Detection and Recovery</a></li>
            <li><a href="#pid-handshake-mechanism">PID Handshake Mechanism</a></li>
          </ul>
        </li>
        <li><a href="#logging-system">Logging System</a>
          <ul>
            <li><a href="#log-file-organization">Log File Organization</a></li>
            <li><a href="#file-locking-mechanism">File Locking Mechanism</a></li>
            <li><a href="#logging-api">Logging API</a></li>
          </ul>
        </li>
        <li><a href="#integration-with-existing-system">Integration with Existing System</a>
          <ul>
            <li><a href="#master-process-changes">Master Process Changes</a></li>
            <li><a href="#client-process-integration">Client Process Integration</a></li>
          </ul>
        </li>
        <li><a href="#configuration">Configuration</a></li>
      </ul>
    </li>
    <li><a href="#project-architecture">Project Architecture</a></li>
  </ol>
</details>
<br>

# POSIX Drone Simulator - Assignment 2

This repository contains the implementation for the **second ARP course assignment**, which extends the multi-process drone simulator from Assignment 1 with two key reliability features:

1. **Watchdog Process**: A monitoring system that continuously checks the health of all simulation processes and takes corrective action when failures are detected.

2. **Enhanced Logging System**: A file-based logging infrastructure with proper synchronization for concurrent writes, featuring separate logs for the watchdog and shared logs for simulation processes.

The foundation of this project is the multi-process drone simulator built in Assignment 1, which uses **POSIX IPC** (anonymous pipes), **ncurses** for UI, and simulates a 2D drone with physics-based dynamics. Assignment 2 focuses specifically on adding fault tolerance and observability to this existing system.

# Beloved Contributors

<a href="https://github.com/Cb-dotcom/posix-drone-sim/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Cb-dotcom/posix-drone-sim&branch=main&v=3" alt="Contributors" />
</a>
<br><br>

# Detailed Description

## Assignment 2 Overview

Assignment 2 introduces reliability and monitoring capabilities to the drone simulator. The key additions are:

**Watchdog System:**
- A dedicated watchdog process that monitors all simulation processes
- Periodic health checks using signal-based ping-ack protocol
- Automatic fault detection and system shutdown on failures
- Protection against hung or crashed processes

**Logging Infrastructure:**
- File-based logging with timestamps and process identification
- Separate log files for watchdog and simulation processes
- File locking (`flock`) to prevent concurrent write corruption
- Persistent logs that survive across multiple simulation runs

These features ensure the simulator can detect and respond to process failures while maintaining detailed logs for debugging and analysis.

## How to build and run

### Prerequisites

- POSIX system (Linux recommended)
- C toolchain (`gcc` or compatible)
- **CMake** ≥ 3.x
- **ncurses** development libraries
- **Konsole** terminal emulator (or adjust to your terminal of choice)
- **mpg123** (optional) for sound effects

### Build

From the project root:

```bash
./run.sh 
```

The build script will:
1. Check and install `mpg123` if needed (for audio)
2. Configure the project with CMake
3. Build all executables including the new `watchdog`
4. Launch the master process which starts the entire system

---

## Watchdog Implementation

### Watchdog Architecture

The watchdog is implemented as a separate process that runs independently from the simulation logic. It communicates with the master process via an anonymous pipe and with monitored processes via POSIX signals.

**Key Design Decisions:**

1. **Early Startup**: The watchdog is the first process spawned by master, before any simulation processes
2. **Blocking Initialization**: The watchdog blocks on reading the PID list, ensuring it's ready before simulation starts
3. **Signal-Based Communication**: Uses `SIGUSR1` for pings and `SIGUSR2` for acknowledgments
4. **Centralized Fault Handling**: All fault detection logic is in the watchdog; monitored processes simply respond to pings

### Watchdog Process

The watchdog process implements health monitoring through a continuous cycle:

**Initialization:**
- Receives the number of processes to monitor from master via pipe
- Receives the array of process IDs (PIDs) to monitor
- Sets up signal handlers for receiving acknowledgments

**Main Loop:**
1. For each monitored process:
   - Check if process is alive using `kill(pid, 0)`
   - Send `SIGUSR1` ping signal
   - Wait up to `WD_ACK_TIMEOUT_MS` for `SIGUSR2` acknowledgment
   - If no ack or process is dead → kill all and exit
2. Sleep for `WD_POLL_PERIOD_MS` before next round

**Configuration Parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `WD_PING_SIGNAL` | `SIGUSR1` | Signal sent to processes for health check |
| `WD_ACK_SIGNAL` | `SIGUSR2` | Signal expected back from processes |
| `WD_POLL_PERIOD_MS` | 300 ms | Time between health check rounds |
| `WD_ACK_TIMEOUT_MS` | 200 ms | Maximum wait time for acknowledgment |
| `WD_MAX_PROCS` | 16 | Maximum number of processes to monitor |

**Termination:**
- On `SIGINT`: Clean exit
- On fault detection: Sends `SIGKILL` to all monitored processes, then exits

### Ping-Ack Protocol

The health check protocol is based on POSIX signals:

**Ping Phase (Watchdog → Process):**
The watchdog sends `SIGUSR1` to each monitored process in sequence.

**Ack Phase (Process → Watchdog):**
Each process has a signal handler that immediately responds by sending `SIGUSR2` back to the watchdog.

**Protocol Properties:**
- **Asynchronous**: Processes respond immediately via signal handler
- **Lightweight**: Minimal overhead on simulation processes
- **Timeout-Based**: Watchdog detects hung processes via timeout
- **SA_RESTART**: Signal handlers use `SA_RESTART` to minimize system call interruption

### Fault Detection and Recovery

The watchdog detects three types of failures:

**1. Process Death:**
Before sending a ping, the watchdog checks if the process is still alive. If not, the watchdog immediately kills all monitored processes.

**2. No Acknowledgment (Hung Process):**
After sending a ping, if a process doesn't respond within the timeout period, the watchdog considers it hung and kills all monitored processes.

**3. Signal Delivery Failure:**
If the watchdog cannot deliver the ping signal (e.g., due to permission issues), it kills all monitored processes.

**Recovery Strategy:**
- **Fail-Fast**: On any failure, immediately kill all processes
- **Clean Shutdown**: Use `SIGKILL` to ensure termination
- **No Restart**: System requires manual restart (as per assignment requirements)

### PID Handshake Mechanism

A critical challenge is monitoring processes launched via `konsole`, which creates an intermediate shell:

**Problem:**
When master spawns a process using konsole, the direct child is the konsole process, not the actual simulation process. The real process (like `bb_server`) is a grandchild, and master doesn't know its PID.

**Solution - PID Reporting:**

The solution uses a dedicated pipe for each konsole-launched process to report its real PID back to master:

1. Master creates PID report pipes (one for `bb_server`, one for `input`)
2. Master passes the write-end file descriptor as the last command-line argument
3. The real process (after konsole/shell layers) writes its PID to this pipe
4. Master reads the real PID from the pipe's read-end
5. Master sends these real PIDs (not konsole PIDs) to the watchdog

This ensures the watchdog monitors the actual simulation processes, not the konsole wrapper processes.

**Process Categories:**
- **Konsole-launched**: `bb_server`, `input` → need PID handshake
- **Direct children**: `drone`, `obstacles`, `targets` → PID known immediately

---

## Logging System

### Log File Organization

Assignment 2 introduces a dual-logging system with different strategies for different process types:

**Watchdog Log (`bin/log/watchdog.log`):**
- **Exclusive to watchdog process**
- **Write mode**: Fresh log on each startup (previous content erased)
- **No locking needed**: Single writer, no concurrency issues
- **Purpose**: Monitor system health and record fault events

**Shared Process Log (`bin/log/processes.log`):**
- **Shared by all simulation processes**: `bb_server`, `drone`, `input`, `obstacles`, `targets`
- **Append mode**: Preserves logs from concurrent processes
- **File locking required**: Multiple writers need synchronization
- **Purpose**: Consolidated view of simulation activity
- **Cleanup**: Master deletes this file at startup for a fresh shared log

**Directory Structure:**
```
bin/log/
├── .gitkeep            # Keeps directory in version control
├── watchdog.log        # Watchdog-only log (fresh each run)
└── processes.log       # Shared simulation log (fresh each run)
```

### File Locking Mechanism

The shared log uses `flock()` from `<sys/file.h>` to prevent write corruption when multiple processes write simultaneously.

**Implementation Strategy:**

Each process that writes to `processes.log`:
1. Acquires an exclusive lock (`LOCK_EX`) before writing
2. Writes its timestamp and message
3. Flushes the buffer to ensure data reaches disk
4. Releases the lock (`LOCK_UN`)

This creates a critical section around each log write operation, ensuring that log entries are atomic and not interleaved.

**Why `flock()` and not other mechanisms?**

- **Advisory locking**: Cooperating processes respect locks
- **Automatic release**: Kernel releases lock if process dies
- **File descriptor scope**: Lock tied to FD, released on `fclose()`
- **Blocking behavior**: Writer waits if another process holds lock

**Alternative Approaches Considered:**
1. `fcntl()` with `F_SETLKW`: More portable but more complex API
2. Named semaphores: Would work but requires additional IPC resource
3. Record locking: Overkill for appending to log file

We chose `flock()` for its simplicity and automatic cleanup properties.

### Logging API

The logging system provides a simple API with three functions:

**Initialization:**
`sim_log_init(const char *process_name)` determines which log file to use based on the process name. If the name is "watchdog", it opens the dedicated watchdog log in write mode. Otherwise, it opens the shared processes log in append mode with locking enabled.

**Logging:**
`sim_log_info(const char *fmt, ...)` writes formatted messages with automatic timestamps in ISO format (YYYY-MM-DD HH:MM:SS). For the shared log, it acquires a lock before writing and releases it after flushing.

**Cleanup:**
`sim_log_close(void)` flushes buffers, closes the file descriptor, and automatically releases any `flock()` locks.

**Log Entry Format:**
```
[2025-01-15 14:23:45] [INFO] --- drone started ---
[2025-01-15 14:23:45] [INFO] drone: started (dt=0.050, M=1.000, K=1.000)
```

---

## Integration with Existing System

### Master Process Changes

The master process has been significantly enhanced to support watchdog monitoring:

**1. New Pipes Created:**
- `pipe_watchdog`: Master to watchdog for sending PID list
- `pidpipe_bb`: bb_server to master for real PID reporting
- `pidpipe_in`: input to master for real PID reporting

**2. Spawn Order Modified:**

Assignment 1 order:
```
bb_server → input → drone → obstacles → targets
```

Assignment 2 order:
```
watchdog → bb_server → input → drone → obstacles → targets
(blocks)   (PID rpt)  (PID rpt)
```

The watchdog is spawned first and blocks waiting for the PID list, ensuring it's ready before any monitored process starts.

**3. Environment Variable Export:**
Master sets the `SIM_WD_PID` environment variable containing the watchdog's PID. All child processes inherit this variable and can read it to know where to send acknowledgment signals.

**4. PID Collection and Transmission:**
After all processes are spawned and PID handshakes are complete, master collects the PIDs into an array and sends them to the watchdog via the pipe. This includes real PIDs from konsole-launched processes and direct child PIDs.

**5. Cleanup Order:**
When shutting down, master waits for all simulation processes first, then stops the watchdog last with `SIGINT`.

### Client Process Integration

Each monitored process integrates watchdog support through a simple client interface with minimal code:

**1. Initialization:**
At startup, each process calls `wd_client_init()` which reads the watchdog PID from the `SIM_WD_PID` environment variable and installs a signal handler for `SIGUSR1`.

**2. Signal Handler:**
The handler `wd_client_ping_handler()` is installed for `SIGUSR1`. When invoked, it simply sends `SIGUSR2` back to the watchdog PID.

**3. Main Function Integration:**
Each process's main function adds one line after initialization to enable watchdog support: `wd_client_init()`.

**Key Properties:**
- **Minimal code**: Only ~20 lines per process
- **Non-intrusive**: Main loop unchanged
- **Asynchronous**: Handler runs independently
- **Fail-safe**: Missing watchdog PID is silently ignored

---

## Configuration

Watchdog behavior can be tuned by modifying constants in `watchdog.c`:

**Available Parameters:**
- `WD_PING_SIGNAL`: Signal used for pings (default: `SIGUSR1`)
- `WD_ACK_SIGNAL`: Signal expected for acks (default: `SIGUSR2`)
- `WD_POLL_PERIOD_MS`: Time between health check rounds (default: 300ms)
- `WD_ACK_TIMEOUT_MS`: Maximum wait for acknowledgment (default: 200ms)
- `WD_MAX_PROCS`: Maximum processes to monitor (default: 16)

**Tuning Recommendations:**

- **Fast systems / low load**: Keep defaults (300ms poll, 200ms timeout)
- **Slow systems / high load**: Increase timeout to 500-1000ms
- **Stress testing**: Decrease poll period to 100ms
- **Production use**: Increase poll period to 1000ms to reduce overhead

The simulation parameters remain in `bin/conf/drone_parameters.conf` as in Assignment 1.

---

## Project Architecture 

```
├── bin
│   ├── conf
│   │   ├── drone_parameters.conf
│   │   └── [audio files...]
│   └── log
│       ├── .gitkeep
│       ├── processes.log       ← NEW: Shared log with locking
│       └── watchdog.log        ← NEW: Dedicated watchdog log
├── build
├── files
│   └── assignmentsv4.1.pdf
├── headers
│   ├── CMakeLists.txt
│   ├── sim_const.h
│   ├── sim_ipc.h
│   ├── sim_log.h
│   ├── sim_params.h
│   ├── sim_types.h
│   └── sim_ui.h
├── src
│   ├── bb_server.c             ← UPDATED: PID handshake, watchdog client
│   ├── CMakeLists.txt          ← UPDATED: Added watchdog target
│   ├── drone.c                 ← UPDATED: Watchdog client integration
│   ├── input.c                 ← UPDATED: PID handshake, watchdog client
│   ├── master.c                ← UPDATED: Watchdog spawn, PID collection
│   ├── obstacles.c             ← UPDATED: Watchdog client integration
│   ├── sim_ipc.c
│   ├── sim_log.c               ← UPDATED: File locking support
│   ├── sim_params.c
│   ├── sim_ui.c
│   ├── targets.c               ← UPDATED: Watchdog client integration
│   └── watchdog.c              ← NEW: Watchdog implementation
├── .gitignore
├── .gitkeep
├── CMakeLists.txt
├── LICENSE
├── README.md
└── run.sh
```

**Key File Changes from Assignment 1:**

| File | Status | Changes |
|------|--------|---------|
| `watchdog.c` | **NEW** | Complete watchdog implementation |
| `sim_log.c` | **UPDATED** | Added `flock()` based locking |
| `master.c` | **UPDATED** | Watchdog spawn, PID handshake |
| `bb_server.c` | **UPDATED** | PID reporting, watchdog client |
| `input.c` | **UPDATED** | PID reporting, watchdog client |
| `drone.c` | **UPDATED** | Watchdog client integration |
| `obstacles.c` | **UPDATED** | Watchdog client integration |
| `targets.c` | **UPDATED** | Watchdog client integration |
| `processes.log` | **NEW** | Shared log file |
| `watchdog.log` | **NEW** | Watchdog-specific log file |

<p align="right">(<a href="#readme-top">back to top</a>)</p>