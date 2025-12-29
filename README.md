<a id="readme-top"></a>

# Table of Contents
<details>
  <summary>View Dropdown</summary>
  <ol>
    <li><a href="#posix-drone-simulator---assignment-2">POSIX Drone Simulator - Assignment 2</a></li>
    <li><a href="#beloved-contributors">Beloved Contributors</a></li>
    <li><a href="#assignment-2-overview">Assignment 2 Overview</a></li>
    <li><a href="#new-components-in-assignment-2">New Components in Assignment 2</a></li>
    <li><a href="#watchdog-system">Watchdog System</a>
      <ul>
        <li><a href="#overview">Overview</a></li>
        <li><a href="#architecture">Architecture</a></li>
        <li><a href="#ping-ack-protocol">Ping-Ack Protocol</a></li>
        <li><a href="#failure-detection">Failure Detection</a></li>
        <li><a href="#recovery-strategy">Recovery Strategy</a></li>
        <li><a href="#pid-handshake-mechanism">PID Handshake Mechanism</a></li>
        <li><a href="#watchdog-integration-in-processes">Watchdog Integration in Processes</a></li>
        <li><a href="#configuration-parameters">Configuration Parameters</a></li>
      </ul>
    </li>
    <li><a href="#logging-system">Logging System</a>
      <ul>
        <li><a href="#overview-1">Overview</a></li>
        <li><a href="#log-file-organization">Log File Organization</a></li>
        <li><a href="#file-locking-mechanism">File Locking Mechanism</a></li>
        <li><a href="#log-entry-format">Log Entry Format</a></li>
        <li><a href="#logging-api">Logging API</a></li>
        <li><a href="#log-inspection">Log Inspection</a></li>
      </ul>
    </li>
    <li><a href="#process-registration-file">Process Registration File</a>
      <ul>
        <li><a href="#overview-2">Overview</a></li>
        <li><a href="#format">Format</a></li>
        <li><a href="#purpose">Purpose</a></li>
        <li><a href="#locking-mechanism">Locking Mechanism</a></li>
        <li><a href="#lifecycle">Lifecycle</a></li>
      </ul>
    </li>
    <li><a href="#integration-with-assignment-1">Integration with Assignment 1</a></li>
    <li><a href="#architecture-summary">Architecture Summary</a></li>
    <li><a href="#updated-project-architecture">Updated Project Architecture</a></li>
    <li><a href="#summary-of-assignment-2-additions">Summary of Assignment 2 Additions</a></li>
  </ol>
</details>
<br>

# POSIX Drone Simulator - Assignment 2

This repository contains the implementation for the **second ARP course assignment**, which extends the multi-process drone simulator from Assignment 1 with reliability and observability features.

# Beloved Contributors

<a href="https://github.com/Cb-dotcom/posix-drone-sim/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Cb-dotcom/posix-drone-sim&branch=main&v=3" alt="Contributors" />
</a>
<br><br>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Assignment 2 Overview

Assignment 2 extends the multi-process drone simulator from Assignment 1 with **reliability and observability features**. While the core simulation remains unchanged, the system now includes fault detection and comprehensive logging capabilities.

The two major additions are:

1. **Watchdog Process**: A dedicated monitoring system that continuously checks the health of all simulation processes and terminates the system when failures are detected.

2. **Enhanced Logging Infrastructure**: A file-based logging system with proper synchronization for concurrent writes, featuring separate logs for different process groups.

These features ensure the simulator can detect and respond to process failures while maintaining detailed logs for debugging and analysis.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## New Components in Assignment 2

### Active Components Update

The system now includes **seven processes** (one more than Assignment 1):

- `master.c` - orchestrator
- `bb_server.c` - blackboard and UI
- `drone.c` - physics simulation
- `input.c` - user controls
- `obstacles.c` - environment generator
- `targets.c` - target generator
- **`watchdog.c`** - **NEW:** health monitor

All processes now integrate with the watchdog monitoring system and use the enhanced logging infrastructure.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Watchdog System

### Overview

The watchdog is a dedicated monitoring process that ensures all simulation processes remain responsive. It uses a **signal-based ping-acknowledgment protocol** to detect hung or crashed processes.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Architecture

**Startup Order:**
The watchdog is the **first process** spawned by master, before any simulation processes. This ensures the monitoring system is ready before monitored processes begin execution.

**Initialization Flow:**
1. Master creates a pipe for PID list communication
2. Master spawns watchdog (watchdog blocks reading from pipe)
3. Master spawns all simulation processes
4. Master collects real PIDs (including handshake for konsole-launched processes)
5. Master writes PID list to watchdog pipe
6. Watchdog unblocks and begins monitoring

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Ping-Ack Protocol

The watchdog uses POSIX signals for health checks:

**Ping Phase:**
- Watchdog sends `SIGUSR1` to each monitored process sequentially
- Each process has a signal handler installed for `SIGUSR1`

**Ack Phase:**
- Process signal handler immediately responds with `SIGUSR2` back to watchdog
- Watchdog waits up to `WD_ACK_TIMEOUT_MS` (default: 200ms) for response
- Acknowledgment includes the current code area the process was executing

**Monitoring Cycle:**
- Watchdog checks each process in round-robin fashion
- Between full rounds, sleeps for `WD_POLL_PERIOD_MS` (default: 300ms)
- Continues indefinitely until failure detected or `SIGINT` received

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Failure Detection

The watchdog detects three types of failures:

**1. Process Death:**
Before sending a ping, the watchdog verifies the process exists using `kill(pid, 0)`. If the process is dead, immediate shutdown is triggered.

**2. No Acknowledgment (Hung Process):**
If a process doesn't respond with `SIGUSR2` within the timeout period, it's considered hung. This catches processes stuck in infinite loops or blocking system calls.

**3. Signal Delivery Failure:**
If the watchdog cannot deliver the ping signal (permission denied, invalid PID, etc.), it triggers shutdown to prevent monitoring a compromised system.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Recovery Strategy

The watchdog follows a **fail-fast approach**:

**On Any Failure:**
- Immediately send `SIGKILL` to all monitored processes
- `SIGKILL` cannot be caught or ignored, ensuring termination
- Watchdog logs the failure reason and exits
- Master process detects watchdog exit and performs cleanup

**No Automatic Restart:**
The system requires manual restart after failure. This design choice ensures that transient issues don't cause restart loops, and operators can investigate root causes.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### PID Handshake Mechanism

A critical challenge is monitoring processes launched via `konsole`, which creates an intermediate shell layer.

**The Problem:**
When master spawns a process using konsole:
- The direct child is the konsole process itself
- The actual simulation process is a grandchild with an unknown PID
- Master only knows the konsole PID, not the real process PID

**The Solution:**
A dedicated pipe-based PID reporting mechanism:

1. Master creates a PID report pipe for each konsole-launched process
2. Master passes the pipe's write-end file descriptor as the **last command-line argument**
3. After the real process starts (post-konsole, post-shell), it calls `report_pid_if_requested()`
4. Real process writes its PID using `write_full()` and closes the pipe
5. Master reads the real PID from the pipe's read-end (blocking until available)
6. Master sends the real PIDs (not konsole PIDs) to watchdog

**Process Categories:**
- **Konsole-launched**: `bb_server`, `input` → require PID handshake
- **Direct children**: `drone`, `obstacles`, `targets` → PID known immediately after fork

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Watchdog Integration in Processes

Each monitored process integrates with the watchdog through minimal code:

**Environment Variable:**
Master exports `SIM_WD_PID` containing the watchdog's PID. All child processes inherit this variable.

**Signal Handler:**
Each process installs a handler for `SIGUSR1` that:
- Reads the watchdog PID from the environment variable
- Sends `SIGUSR2` back to the watchdog using `sigqueue()`
- Includes the current `g_current_code_area` value for debugging

**Code Area Tracking:**
Processes maintain a `g_current_code_area` variable that indicates which section of code is executing:
- `CODE_AREA_INIT` - initialization phase
- `CODE_AREA_MAIN_LOOP` - main processing loop
- `CODE_AREA_READ_PIPE` - reading from pipes
- `CODE_AREA_WRITE_PIPE` - writing to pipes
- `CODE_AREA_PHYSICS_UPDATE` - computation phase
- `CODE_AREA_SHUTDOWN` - cleanup phase

This information is included in watchdog logs to help diagnose where processes hang.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Configuration Parameters

Watchdog behavior can be tuned in `watchdog.c`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `WD_PING_SIGNAL` | `SIGUSR1` | Signal sent for health checks |
| `WD_ACK_SIGNAL` | `SIGUSR2` | Signal expected for acknowledgments |
| `WD_POLL_PERIOD_MS` | 300 ms | Time between monitoring rounds |
| `WD_ACK_TIMEOUT_MS` | 200 ms | Maximum wait for acknowledgment |
| `WD_MAX_PROCS` | 16 | Maximum processes to monitor |

**Tuning Recommendations:**

**Fast systems with low load:** Keep defaults (300ms poll, 200ms timeout)

**Slow systems or high load:** Increase timeout to 500-1000ms to avoid false positives

**Stress testing:** Decrease poll period to 100ms for aggressive monitoring

**Production deployment:** Increase poll period to 1000ms to reduce CPU overhead

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Logging System

### Overview

Assignment 2 introduces a **dual-logging architecture** with different strategies for different process types. This ensures comprehensive observability while avoiding log corruption from concurrent writes.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Log File Organization

**Watchdog Log (`bin/log/watchdog.log`):**
- **Exclusive to watchdog process**
- **Write mode**: Fresh log on each startup (previous content erased)
- **No locking needed**: Single writer, no concurrency issues
- **Purpose**: Monitor system health, record ping-ack cycles, log fault events

**Shared Process Log (`bin/log/processes.log`):**
- **Shared by all simulation processes**: `bb_server`, `drone`, `input`, `obstacles`, `targets`
- **Append mode**: Preserves logs from all concurrent processes
- **File locking required**: Multiple writers need synchronization
- **Purpose**: Consolidated view of all simulation activity
- **Cleanup**: Master deletes this file at startup for a fresh simulation run

**Directory Structure:**
```
bin/log/
├── .gitkeep              # Keeps directory in version control
├── watchdog.log          # Watchdog-only log (fresh each run)
├── processes.log         # Shared simulation log (fresh each run)
└── processes.pid         # Process registration file (fresh each run)
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### File Locking Mechanism

The shared log uses **advisory file locking** via `flock()` to prevent corruption when multiple processes write simultaneously.

**The Concurrency Problem:**
Without locking, when multiple processes write to the same file:
- Process A writes timestamp, gets preempted
- Process B writes full log line
- Process A resumes and writes its message
- Result: Corrupted output with interleaved partial lines

**The Solution - Advisory Locking:**

Each log write operation follows this protocol:

1. **Acquire Lock**: Call `flock(fd, LOCK_EX)` for exclusive access
2. **Write**: Output timestamp, level, and message
3. **Flush**: Call `fflush()` to ensure data reaches disk
4. **Release Lock**: Call `flock(fd, LOCK_UN)` to allow others to write

The kernel **automatically serializes** writes by blocking processes that try to acquire a held lock.

**Why flock() over alternatives:**

**Automatic cleanup:** Kernel releases lock if process crashes, preventing deadlock

**Per-file-descriptor scope:** Lock tied to FD, released on `fclose()` or process exit

**Blocking behavior:** Calling process sleeps (not spin-waits) until lock available

**Simplicity:** Single function call, no complex API

**Considered alternatives:**
- `fcntl(F_SETLKW)` - More portable but more complex API
- Named semaphores - Requires additional IPC resource management
- Record locking - Overkill for append-only log files

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Log Entry Format

All log entries follow a consistent timestamp format:

```
[YYYY-MM-DD HH:MM:SS] [INFO] process_name: message
```

**Example entries:**
```
[2025-01-15 14:23:45] [INFO] --- drone started ---
[2025-01-15 14:23:45] [INFO] drone: started (dt=0.050, M=1.000, K=1.000)
[2025-01-15 14:23:46] [INFO] bb_server: TARGET HIT idx=3 score=+100.0 total=100.0
[2025-01-15 14:23:50] [INFO] watchdog: pid 12345 ack (code_area=1)
```

The timestamp uses 24-hour format with second precision, sufficient for debugging multi-process interactions.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Logging API

Processes interact with the logging system through three functions:

**`sim_log_init(const char *process_name)`**
Initializes logging for the calling process. Determines which log file to use based on process name:
- Watchdog → `watchdog.log` (write mode, exclusive)
- All others → `processes.log` (append mode, with locking)

**`sim_log_info(const char *fmt, ...)`**
Writes formatted messages with automatic timestamps. For shared log, acquires lock before writing and releases after flushing. Supports printf-style format strings.

**`sim_log_close(void)`**
Flushes buffers, closes the file descriptor, and releases any locks. Called during process cleanup. Locks are also released automatically by kernel on process exit.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Log Inspection

**During execution:**
You can tail logs in real-time to observe system behavior:

```bash
tail -f bin/log/processes.log    # Watch simulation activity
tail -f bin/log/watchdog.log      # Watch health checks
```

**After execution:**
Logs persist across runs (watchdog log is overwritten, shared log is deleted by master at startup). You can search for specific events:

```bash
grep "TARGET HIT" bin/log/processes.log    # Find all target collections
grep "did not ack" bin/log/watchdog.log    # Find hung process events
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Process Registration File

### Overview

The **process registration file** (`bin/log/processes.pid`) maintains a record of all process names and PIDs at startup. This provides a snapshot of the system state for debugging.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Format

Each line records one process in the format:

```
[YYYY-MM-DD HH:MM:SS] process_name PID=12345
```

**Example file:**
```
[2025-01-15 14:23:45] master PID=12340
[2025-01-15 14:23:45] watchdog PID=12341
[2025-01-15 14:23:45] bb_server PID=12342
[2025-01-15 14:23:45] input PID=12343
[2025-01-15 14:23:45] drone PID=12344
[2025-01-15 14:23:45] obstacles PID=12345
[2025-01-15 14:23:45] targets PID=12346
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Purpose

**Debugging aid:** Quickly identify which PID corresponds to which process when inspecting system state with `ps`, `top`, or `/proc` filesystem.

**Watchdog verification:** Confirm the watchdog is monitoring the correct PIDs by comparing registration file against watchdog log entries.

**Crash investigation:** If the system crashes, the PID file shows which processes were running, enabling correlation with system logs or core dumps.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Locking Mechanism

Like the shared log, the PID file uses `flock()` for atomic writes. Each process:
1. Opens file in append mode
2. Acquires exclusive lock
3. Writes single line with timestamp, name, and PID
4. Flushes buffer
5. Releases lock and closes file

This prevents interleaved writes if multiple processes register simultaneously during startup.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Lifecycle

**Created by:** Each process registers itself by calling `sim_process_register()` early in initialization

**Cleanup:** Master deletes the file at startup to ensure a fresh registration for each simulation run

**Persistence:** File persists after simulation ends, allowing post-mortem analysis

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Integration with Assignment 1

### Unchanged Components

The following Assignment 1 features remain **exactly the same** in Assignment 2:

- Core simulation logic (drone physics, repulsion forces, scoring)
- IPC architecture (anonymous pipes between processes)
- User interface (ncurses-based visualization and controls)
- Configuration file system (`drone_parameters.conf`)
- Environment generation (obstacles and targets)

### Changed Components

**Master Process:**
- Now spawns watchdog first (before all other processes)
- Implements PID handshake mechanism for konsole-launched processes
- Collects and transmits PID list to watchdog
- Exports `SIM_WD_PID` environment variable
- Deletes shared log and PID file at startup

**All Simulation Processes:**
- Integrate watchdog client (signal handler for `SIGUSR1`)
- Track current code area in `g_current_code_area` variable
- Use enhanced logging with file locking
- Register themselves in `processes.pid` at startup
- For konsole-launched processes: report real PID via pipe

**Logging:**
- Switched from per-process log files to dual-log architecture
- Added file locking for concurrent writes
- Timestamps now in ISO-like format for consistency

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Architecture Summary

### Process Hierarchy

```
master (orchestrator)
├── watchdog (health monitor)         ← spawned FIRST
├── bb_server (konsole wrapper)       ← PID handshake
│   └── bb_server (real process)      ← monitored by watchdog
├── input (konsole wrapper)           ← PID handshake
│   └── input (real process)          ← monitored by watchdog
├── drone                             ← monitored by watchdog
├── obstacles                         ← monitored by watchdog
└── targets                           ← monitored by watchdog
```

### Communication Topology

**Simulation Pipes (unchanged from Assignment 1):**
- `pipe_drone_cmd`: bb_server → drone (CommandState)
- `pipe_drone_state`: drone → bb_server (DroneState)
- `pipe_input_cmd`: input → bb_server (CommandState)
- `pipe_obstacles`: obstacles → bb_server (Obstacle[])
- `pipe_targets`: targets → bb_server (Target[])
- `pipe_obstacles_drone`: obstacles → drone (Obstacle[])

**Watchdog Communication (new in Assignment 2):**
- `pipe_watchdog`: master → watchdog (PID list, one-time)
- `SIM_WD_PID` environment variable: master → all children
- Signals: watchdog ↔ all processes (`SIGUSR1` ping, `SIGUSR2` ack)

**PID Handshake (new in Assignment 2):**
- `pidpipe_bb`: bb_server → master (real PID reporting)
- `pidpipe_in`: input → master (real PID reporting)

### File Outputs

**Logs:**
- `bin/log/watchdog.log` - watchdog monitoring events (exclusive, overwritten each run)
- `bin/log/processes.log` - simulation process logs (shared with locking, deleted at startup)

**Process Registry:**
- `bin/log/processes.pid` - process name and PID mapping (deleted at startup)

**Configuration (read-only):**
- `bin/conf/drone_parameters.conf` - runtime parameters (unchanged from Assignment 1)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Updated Project Architecture

```
├── bin
│   ├── conf
│   │   ├── drone_parameters.conf
│   │   └── [audio files: music.mp3, press.mp3, etc.]
│   └── log
│       ├── .gitkeep
│       ├── processes.log          ← NEW: Shared log with locking
│       ├── processes.pid          ← NEW: Process registration
│       └── watchdog.log           ← NEW: Watchdog-specific log
├── build
├── files
│   └── assignmentsv4.7.pdf        ← Updated assignment specification
├── headers
│   ├── CMakeLists.txt
│   ├── sim_const.h
│   ├── sim_ipc.h
│   ├── sim_log.h                  ← UPDATED: Added locking support
│   ├── sim_params.h
│   ├── sim_types.h                ← UPDATED: Added code area enum
│   └── sim_ui.h
├── src
│   ├── bb_server.c                ← UPDATED: Watchdog client + PID handshake
│   ├── CMakeLists.txt             ← UPDATED: Added watchdog target
│   ├── drone.c                    ← UPDATED: Watchdog client
│   ├── input.c                    ← UPDATED: Watchdog client + PID handshake
│   ├── master.c                   ← UPDATED: Watchdog spawn, PID management
│   ├── obstacles.c                ← UPDATED: Watchdog client
│   ├── sim_ipc.c
│   ├── sim_log.c                  ← UPDATED: File locking implementation
│   ├── sim_params.c
│   ├── sim_ui.c
│   ├── targets.c                  ← UPDATED: Watchdog client
│   └── watchdog.c                 ← NEW: Complete watchdog implementation
├── .gitignore
├── CMakeLists.txt
├── LICENSE
├── README.md                      ← UPDATED: Assignment 2 documentation
└── run.sh
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Summary of Assignment 2 Additions

| Feature | Purpose | Implementation |
|---------|---------|----------------|
| **Watchdog Process** | Detect hung or crashed processes | Signal-based ping-ack protocol with configurable timeouts |
| **PID Handshake** | Monitor konsole-launched processes | Dedicated pipes for real PID reporting to master |
| **Dual Logging** | Separate watchdog from simulation logs | `watchdog.log` (exclusive) + `processes.log` (shared with locking) |
| **File Locking** | Prevent log corruption | `flock()` for atomic writes to shared log |
| **Process Registration** | Track system state at startup | `processes.pid` file with timestamps and PIDs |
| **Code Area Tracking** | Debug process hangs | `g_current_code_area` variable reported in acks |

These additions transform the Assignment 1 simulator into a **production-ready system** with fault detection, detailed observability, and crash recovery capabilities.

<p align="right">(<a href="#readme-top">back to top</a>)</p>