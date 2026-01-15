<a id="readme-top"></a>

# Table of Contents
<details>
  <summary>View Dropdown</summary>
  <ol>
    <li><a href="#posix-drone-simulator---assignment-3">POSIX Drone Simulator - Assignment 3</a></li>
    <li><a href="#beloved-contributors">Beloved Contributors</a></li>
    <li><a href="#assignment-3-overview">Assignment 3 Overview</a></li>
    <li><a href="#new-components-in-assignment-3">New Components in Assignment 3</a></li>
    <li><a href="#network-architecture">Network Architecture</a>
      <ul>
        <li><a href="#operating-modes">Operating Modes</a></li>
        <li><a href="#process-topology">Process Topology</a></li>
        <li><a href="#coordinate-systems">Coordinate Systems</a></li>
      </ul>
    </li>
    <li><a href="#network-protocol">Network Protocol</a>
      <ul>
        <li><a href="#protocol-overview">Protocol Overview</a></li>
        <li><a href="#handshake-sequence">Handshake Sequence</a></li>
        <li><a href="#data-exchange">Data Exchange</a></li>
        <li><a href="#disconnect-sequence">Disconnect Sequence</a></li>
        <li><a href="#message-format">Message Format</a></li>
      </ul>
    </li>
    <li><a href="#coordinate-transformation">Coordinate Transformation</a>
      <ul>
        <li><a href="#virtual-coordinate-system">Virtual Coordinate System</a></li>
        <li><a href="#transformation-parameters">Transformation Parameters</a></li>
        <li><a href="#transformation-algorithm">Transformation Algorithm</a></li>
      </ul>
    </li>
    <li><a href="#server-mode">Server Mode</a>
      <ul>
        <li><a href="#server-architecture">Server Architecture</a></li>
        <li><a href="#server-data-flow">Server Data Flow</a></li>
        <li><a href="#server-state-machine">Server State Machine</a></li>
      </ul>
    </li>
    <li><a href="#client-mode">Client Mode</a>
      <ul>
        <li><a href="#client-architecture">Client Architecture</a></li>
        <li><a href="#client-data-flow">Client Data Flow</a></li>
        <li><a href="#client-state-machine">Client State Machine</a></li>
      </ul>
    </li>
    <li><a href="#implementation-details">Implementation Details</a>
      <ul>
        <li><a href="#nonblocking-io">Nonblocking I/O</a></li>
        <li><a href="#buffering-strategy">Buffering Strategy</a></li>
        <li><a href="#error-handling">Error Handling</a></li>
      </ul>
    </li>
    <li><a href="#configuration">Configuration</a>
      <ul>
        <li><a href="#config-file-parameters">Config File Parameters</a></li>
        <li><a href="#environment-variables">Environment Variables</a></li>
        <li><a href="#mode-selection">Mode Selection</a></li>
      </ul>
    </li>
    <li><a href="#usage">Usage</a>
      <ul>
        <li><a href="#running-server-mode">Running Server Mode</a></li>
        <li><a href="#running-client-mode">Running Client Mode</a></li>
        <li><a href="#running-normal-mode">Running Normal Mode</a></li>
      </ul>
    </li>
    <li><a href="#integration-with-assignment-2">Integration with Assignment 2</a></li>
    <li><a href="#updated-project-architecture">Updated Project Architecture</a></li>
    <li><a href="#summary-of-assignment-3-additions">Summary of Assignment 3 Additions</a></li>
  </ol>
</details>
<br>

# POSIX Drone Simulator - Assignment 3

This repository contains the implementation for the **third ARP course assignment**, which extends the multi-process drone simulator with **network communication capabilities**, enabling two instances to interact over TCP/IP.

# Beloved Contributors

<a href="https://github.com/Cb-dotcom/posix-drone-sim/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=Cb-dotcom/posix-drone-sim&branch=main&v=3" alt="Contributors" />
</a>
<br><br>

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Assignment 3 Overview

Assignment 3 extends the multi-process drone simulator from Assignments 1 and 2 with **network communication capabilities**. The system can now operate in three modes: standalone (normal), server, or client. In networked modes, two simulator instances communicate over TCP/IP to exchange drone positions and create an interactive multi-drone environment.

The major additions are:

1. **Network Processes**: Dedicated `network_server` and `network_client` processes handle TCP/IP communication with proper nonblocking I/O and state machine protocols.

2. **Multi-Mode Operation**: The simulator can run as a standalone system, as a network server hosting a simulation, or as a client connecting to a remote server.

3. **Coordinate Transformation**: A configurable coordinate system conversion allows different local UI orientations (top-left vs bottom-left) to be reconciled through a shared virtual coordinate space.

4. **Interactive Launch System**: The `exe.sh` script provides an interactive menu for mode selection with automatic IP detection and configuration.

These features enable distributed multi-agent simulations while maintaining the reliability and observability features from Assignment 2.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## New Components in Assignment 3

### Active Components Update

The system now includes **nine processes** (two more than Assignment 2):

- `master.c` - orchestrator (mode-aware)
- `bb_server.c` - blackboard and UI (mode-aware)
- `drone.c` - physics simulation (always present)
- `input.c` - user controls (always present)
- `obstacles.c` - environment generator (**normal mode only**)
- `targets.c` - target generator (**normal mode only**)
- `watchdog.c` - health monitor (**normal mode only**)
- **`network_server.c`** - **NEW:** TCP/IP server for hosting simulations
- **`network_client.c`** - **NEW:** TCP/IP client for connecting to servers

### Mode-Dependent Process Spawning

**Normal Mode (Standalone):**
- All processes from Assignment 2 (watchdog, obstacles, targets)
- No network processes

**Server Mode:**
- Watchdog, obstacles, and targets are **NOT** spawned
- Server drone is controlled locally via input
- Client drone appears as a dynamic obstacle
- `network_server` process handles client communication

**Client Mode:**
- Watchdog, obstacles, and targets are **NOT** spawned
- Client drone is controlled locally via input
- Server drone position is visualized but not interactive
- `network_client` process handles server communication

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Network Architecture

### Operating Modes

The simulator supports three mutually exclusive operating modes:

**Mode 0: Normal (Standalone)**
- Single-instance simulation
- All features from Assignments 1 and 2
- Watchdog monitoring, obstacles, and targets active
- No network communication

**Mode 1: Server**
- Hosts a networked simulation
- One client can connect
- Server drone controlled locally (keyboard input)
- Client drone appears as obstacle with repulsion forces
- No static obstacles or targets (client is the only obstacle)
- No watchdog (network process manages lifecycle)

**Mode 2: Client**
- Connects to a remote server
- Client drone controlled locally (keyboard input)
- Server drone visualized as read-only position marker
- No static obstacles or targets
- Window dimensions received from server
- No watchdog (network process manages lifecycle)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Process Topology

**Normal Mode:**
```
master
├── watchdog (monitors all)
├── bb_server (konsole)
├── input (konsole)
├── drone
├── obstacles
└── targets
```

**Server Mode:**
```
master
├── bb_server (konsole)
├── input (konsole)
├── drone
└── network_server
    └── [TCP socket] → network_client
```

**Client Mode:**
```
master
├── bb_server (konsole)
├── input (konsole)
├── drone
└── network_client
    └── [TCP socket] → network_server
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Coordinate Systems

Network communication requires coordinate transformation because different instances may use different UI coordinate conventions:

**Local Coordinate System:**
- Each instance's internal representation
- May be top-left origin with Y-down (typical for graphics)
- Or bottom-left origin with Y-up (typical for math/physics)

**Virtual Coordinate System:**
- Shared reference frame for network exchange
- Always bottom-left origin with Y-up
- Rotation parameter allows instances at different orientations

**Transformation Flow:**
```
Server Local → Virtual → Network → Virtual → Client Local
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Network Protocol

### Protocol Overview

The network protocol is a **line-based, text-oriented, synchronous request-response protocol**. All messages are newline-delimited ASCII strings. The protocol uses tokens to identify message types and maintains strict state machine sequencing.

**Design Principles:**
- **Human-readable:** Text format for easy debugging with tools like `nc` or telnet
- **Synchronous:** Each request expects immediate response before next message
- **Stateful:** Both sides maintain protocol state machines
- **Error-intolerant:** Protocol violations trigger immediate disconnection

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Handshake Sequence

The connection begins with a handshake to establish window dimensions:

**Step 1: Server sends greeting**
```
Server → Client: ok
```

**Step 2: Client acknowledges**
```
Client → Server: ook
```

**Step 3: Server sends window size**
```
Server → Client: size
Server → Client: W,H
```
Example: `size 50,50` (single line with comma separator)

**Step 4: Client acknowledges**
```
Client → Server: sok
```

After successful handshake, the protocol enters the data exchange phase.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Data Exchange

Once handshake completes, the server and client exchange data in a repeating cycle:

**Drone Position Exchange (Server → Client):**
```
Server → Client: drone
Server → Client: X.X, Y.Y
Client → Server: dok
```
- Coordinates in virtual system
- Format: `%.1f, %.1f` (one decimal place, space after comma)
- Example: `25.5, 30.0`

**Obstacle Position Exchange (Client → Server):**
```
Server → Client: obst
Client → Server: X.X, Y.Y
Server → Client: pok
```
- Client sends its drone position as obstacle
- Same coordinate format as drone exchange
- Example: `15.0, 20.5`

**Exchange Cycle:**
1. Server sends drone position, waits for `dok`
2. Server requests obstacle (`obst`), waits for coordinates
3. Server acknowledges with `pok`
4. Repeat from step 1

This cycle continues until disconnect.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Disconnect Sequence

Either side can initiate graceful disconnect:

**Server-Initiated Disconnect:**
```
Server → Client: q
Client → Server: qok
```

**Client-Initiated Disconnect:**
The client can send `q` at any point where it's expecting a server message (e.g., instead of `drone` or `obst`):
```
Client → Server: q
Server → Client: qok
```

After exchange of `q` and `qok`, both sides close the socket.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Message Format

**Token Messages:**
- Single-word identifiers
- No parameters
- Examples: `ok`, `ook`, `drone`, `dok`, `obst`, `pok`, `q`, `qok`

**Coordinate Messages:**
- Format: `X.X, Y.Y` (space after comma)
- Precision: `%.1f` (one decimal place)
- Range: 0.0 to window dimension
- Example: `25.5, 30.0`

**Size Message:**
- Format: `size W,H` (single line, comma separator)
- Integer values
- Example: `size 50,50`

**Line Termination:**
- All messages end with `\n` (LF)
- CRLF (`\r\n`) is accepted and automatically trimmed
- Empty lines are ignored

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Coordinate Transformation

### Virtual Coordinate System

The virtual coordinate system serves as the common reference frame for all networked instances:

**Properties:**
- **Origin:** Bottom-left corner (0, 0)
- **X-axis:** Increases to the right
- **Y-axis:** Increases upward
- **Dimensions:** Same as server's local world (e.g., 50x50)

**Purpose:**
- Decouples local UI conventions from network exchange
- Allows instances with different orientations to communicate
- Enables future support for rotated or mirrored displays

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Transformation Parameters

Coordinate transformation is controlled by two environment variables:

**`SIM_NET_FLIP_Y`:**
- **Purpose:** Converts between top-left and bottom-left origins
- **Values:**
  - `0` - Local Y-axis increases upward (bottom-left origin)
  - `1` - Local Y-axis increases downward (top-left origin)
- **Default:** `1` (most UI toolkits use top-left)

**`SIM_NET_ALPHA`:**
- **Purpose:** Rotates local axes relative to virtual system
- **Values:** `0`, `90`, `-90`, `180` (degrees)
- **Default:** `0` (no rotation)
- **Use Case:** Accommodate instances running on rotated displays

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Transformation Algorithm

**Local to Virtual Conversion:**

1. **Y-axis flip** (if `SIM_NET_FLIP_Y=1`):
   ```
   y_flipped = world_height - y_local
   ```

2. **Rotation** (by `SIM_NET_ALPHA` degrees):
   - **0°:** `(x_virtual, y_virtual) = (x, y_flipped)`
   - **90°:** `(x_virtual, y_virtual) = (-y_flipped + W, x)`
   - **-90°:** `(x_virtual, y_virtual) = (y_flipped, -x + H)`
   - **180°:** `(x_virtual, y_virtual) = (-x + W, -y_flipped + H)`

**Virtual to Local Conversion:**

1. **Inverse rotation** (reverse of step 2 above)

2. **Y-axis unflip** (if `SIM_NET_FLIP_Y=1`):
   ```
   y_local = world_height - y_unrotated
   ```

**Example (typical case: `FLIP_Y=1, ALPHA=0`):**
```
Server local (25, 10) with 50x50 world:
  → y_flip: y = 50 - 10 = 40
  → rotation 0°: (25, 40) virtual
  
Client receives (25, 40) virtual:
  → rotation 0°: (25, 40)
  → y_unflip: y = 50 - 40 = 10
  → (25, 10) client local
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Server Mode

### Server Architecture

The server process (`network_server`) acts as the bridge between the local simulation and remote client:

**Responsibilities:**
- Listen for incoming TCP connections (port configured in `drone_parameters.conf`)
- Accept exactly one client connection
- Execute protocol handshake (send window dimensions)
- Read local drone position from bb_server via pipe
- Send local drone position to client via socket
- Receive client drone position from socket
- Write client position as obstacle to bb_server via pipe
- Perform coordinate transformations (local ↔ virtual)

**Process Lifecycle:**
1. Create listening socket on configured port
2. Block waiting for client connection (`accept`)
3. Handshake with client (send window size)
4. Enter nonblocking data exchange loop
5. On disconnect or error, close sockets and exit

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Server Data Flow

**Pipes:**
- `pipe_network_drone_in[0]` - **Read** from bb_server (DroneState)
- `pipe_network_obstacle_in[1]` - **Write** to bb_server (Obstacle)

**Flow Diagram:**
```
bb_server → pipe_network_drone_in[1] → network_server (read end)
                                           ↓ transform to virtual
                                           ↓ send via socket
                                           ↓
network_client ← socket ← "drone X.X, Y.Y"
                                           ↓
network_client → socket → "X.X, Y.Y" (client drone)
                                           ↓ recv via socket
                                           ↓ transform to local
network_server → pipe_network_obstacle_in[1] → bb_server (as obstacle[0])
```

**Key Points:**
- Server's own drone is controlled via local `input` process
- Client drone appears in `obstacles[0]` with radius 1.0
- Repulsion forces apply to client drone (it's a real obstacle)
- No static obstacles or targets exist in server mode

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Server State Machine

The server uses a state machine to manage protocol flow:

**States:**

1. **`S_WAIT_OOK`**: Sent `ok`, waiting for client `ook`
2. **`S_SEND_SIZE`**: Ready to send `size W,H`
3. **`S_WAIT_SOK`**: Sent size, waiting for `sok`
4. **`S_SEND_DRONE`**: Ready to send `drone` + coordinates
5. **`S_WAIT_DOK`**: Sent drone coords, waiting for `dok`
6. **`S_SEND_OBST`**: Ready to send `obst` request
7. **`S_WAIT_OBST_X`**: Sent `obst`, waiting for X coordinate (or X,Y pair)
8. **`S_WAIT_OBST_Y`**: Received X, waiting for Y coordinate
9. **`S_SEND_POK`**: Ready to send `pok` acknowledgment
10. **`S_WAIT_QOK`**: Sent `q`, waiting for `qok`
11. **`S_DONE`**: Connection complete, exit loop

**State Transitions:**
```
S_WAIT_OOK → S_SEND_SIZE → S_WAIT_SOK → S_SEND_DRONE → S_WAIT_DOK
                                              ↑               ↓
                                              ↑          S_SEND_OBST
                                              ↑               ↓
                                         S_SEND_POK ← S_WAIT_OBST_X/Y
                                              ↑               ↓
                                              └───────────────┘
                                            (repeating cycle)
```

**Error Handling:**
- Invalid token → transition to `S_DONE`
- Parse failure → transition to `S_DONE`
- Socket error → break main loop
- Ctrl+C (`SIGINT`) → send `q`, transition to `S_WAIT_QOK`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Client Mode

### Client Architecture

The client process (`network_client`) connects to a remote server and exchanges drone positions:

**Responsibilities:**
- Connect to server TCP socket (address and port from config)
- Execute protocol handshake (receive window dimensions)
- Forward window dimensions to bb_server (so UI matches server)
- Read local drone position from bb_server via pipe
- Send local drone position to server as obstacle
- Receive server drone position from socket
- Write server drone position to bb_server via pipe (for visualization)
- Perform coordinate transformations (local ↔ virtual)

**Process Lifecycle:**
1. Connect to server socket
2. Handshake with server (receive and forward window size)
3. Enter nonblocking data exchange loop
4. On disconnect or error, close socket and pipes, exit

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Client Data Flow

**Pipes:**
- `pipe_network_drone_in[0]` - **Read** from bb_server (DroneState)
- `pipe_network_server_drone[1]` - **Write** to bb_server (DroneState - server's drone)
- `pipe_network_window_size[1]` - **Write** to bb_server (WindowDimensions - one-time)

**Flow Diagram:**
```
network_server → socket → "size W,H"
                              ↓
network_client → pipe_network_window_size[1] → bb_server (sets world dimensions)

network_server → socket → "drone X.X, Y.Y"
                              ↓ transform to local
network_client → pipe_network_server_drone[1] → bb_server (visualize server drone)

bb_server → pipe_network_drone_in[1] → network_client (read end)
                                           ↓ transform to virtual
                                           ↓
network_server ← socket ← "X.X, Y.Y" (client drone as obstacle)
```

**Key Points:**
- Client's own drone is controlled via local `input` process
- Server drone is **read-only** (displayed as `*` symbol in cyan)
- Client drone is sent to server, which treats it as obstacle
- No repulsion from server drone (it's display-only)
- Window dimensions must match server for consistent physics

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Client State Machine

The client uses a simpler state machine than the server:

**States:**

1. **`C_WAIT_OK`**: Waiting for initial `ok` from server
2. **`C_WAIT_SIZE`**: Sent `ook`, waiting for `size W,H`
3. **`C_RUN_WAIT_TOKEN`**: Main loop, waiting for `drone`, `obst`, or `q`
4. **`C_RUN_WAIT_DRONE_COORDS`**: Received `drone`, waiting for coordinates
5. **`C_RUN_WAIT_POK`**: Sent obstacle coords, waiting for `pok`
6. **`C_DONE`**: Connection complete, exit loop

**State Transitions:**
```
C_WAIT_OK → C_WAIT_SIZE → C_RUN_WAIT_TOKEN → C_RUN_WAIT_DRONE_COORDS
                                 ↑                        ↓
                                 ↑                   send dok
                                 └────────────────────────┘
                                 ↓
                           (receive obst)
                                 ↓
                           send X.X, Y.Y
                                 ↓
                          C_RUN_WAIT_POK
                                 ↓
                           (receive pok)
                                 ↓
                        back to C_RUN_WAIT_TOKEN
```

**Special Cases:**
- Receiving `q` at `C_RUN_WAIT_TOKEN` → send `qok`, transition to `C_DONE`
- Any protocol violation → transition to `C_DONE`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Implementation Details

### Nonblocking I/O

Both network processes use nonblocking sockets and pipes to avoid deadlocks:

**Why Nonblocking?**
- Prevents blocking on socket read while pipe has data to send
- Allows simultaneous monitoring of multiple file descriptors
- Enables responsive shutdown on `SIGINT` or pipe closure

**Implementation:**
```c
// Set socket nonblocking
int flags = fcntl(sockfd, F_GETFL, 0);
fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

// Set pipes nonblocking
fcntl(pipe_fd, F_SETFL, fcntl(pipe_fd, F_GETFL) | O_NONBLOCK);
```

**Select Loop:**
```c
fd_set rfds, wfds;
FD_ZERO(&rfds);
FD_ZERO(&wfds);

FD_SET(sockfd, &rfds);          // always monitor socket for reads
if (send_queue.len > 0)
    FD_SET(sockfd, &wfds);      // only monitor for writes if data pending

select(maxfd + 1, &rfds, &wfds, NULL, &timeout);
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Buffering Strategy

Network processes use separate input and output buffers to handle partial reads/writes:

**Input Buffering (LineAcc):**
```c
typedef struct {
    char   buf[2048];
    size_t len;
} LineAcc;
```
- Accumulates received bytes until complete line (ending with `\n`)
- Supports CRLF (`\r\n`) by trimming `\r` before line delivery
- Handles fragmented receives (partial lines across multiple `recv` calls)

**Output Buffering (SendQ):**
```c
typedef struct {
    char   buf[4096];
    size_t off;    // bytes already sent
    size_t len;    // total bytes to send
} SendQ;
```
- Queues complete lines for transmission
- Handles partial `send` (some bytes sent, some buffered)
- Automatically advances `off` as data is sent

**Drain Pattern (Pipes):**
```c
int drain_latest_drone(int fd, DroneState *latest, int *have_latest) {
    for (;;) {
        DroneState ds;
        ssize_t r = read(fd, &ds, sizeof(ds));
        if (r == sizeof(ds)) {
            *latest = ds;      // keep newest
            continue;          // drain more
        }
        if (r < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            return 0;          // no more data
        return (r == 0) ? 1 : -1;  // EOF or error
    }
}
```
- Continuously reads until `EAGAIN`, keeping only the latest value
- Prevents old drone positions from accumulating in pipe

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Error Handling

Network processes handle errors at multiple levels:

**Socket Errors:**
- Connection refused → log and exit (client cannot proceed)
- Connection reset → log disconnect, clean shutdown
- Broken pipe → log disconnect, clean shutdown

**Protocol Errors:**
- Unexpected token → transition to `DONE` state, log violation
- Parse failure → transition to `DONE` state, log invalid data
- Timeout (future enhancement) → close connection

**Pipe Errors:**
- bb_server closed pipe → assume UI quit, initiate graceful disconnect
- Write failure → assume downstream dead, exit
- Read EOF → normal shutdown condition

**Signal Handling:**
- `SIGINT` (Ctrl+C) → set running flag to 0, send `q` to peer
- `SIGPIPE` → ignored (handled via `errno == EPIPE`)

**Logging:**
All errors are logged via `sim_log_info()` for post-mortem analysis:
```
network_server: client disconnected
network_client: socket recv error
network_server: invalid token 'foo'
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Configuration

### Config File Parameters

The `bin/conf/drone_parameters.conf` file contains network-specific settings:

```conf
# Network mode (controls which processes are spawned)
network_mode normal         # normal | server | client

# Server settings (used in server and client modes)
server_address 127.0.0.1   # IP address for server/client
server_port 8888            # TCP port number
```

**Parameter Details:**

**`network_mode`:**
- **Values:** `normal`, `server`, `client`
- **Default:** `normal`
- **Effect:** Determines which processes master spawns
- **Note:** Overridden by `exe.sh` interactive mode selection

**`server_address`:**
- **Server mode:** Bind address (use `0.0.0.0` for all interfaces)
- **Client mode:** Server IP to connect to
- **Default:** `127.0.0.1` (localhost)

**`server_port`:**
- **Range:** 1-65535 (avoid <1024 without root)
- **Default:** `8888`
- **Recommendation:** Use ports >1024 to avoid permission issues

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Environment Variables

Runtime behavior is controlled by environment variables:

**`SIM_MODE`:**
- **Purpose:** Override mode from config file
- **Values:** `0` (normal), `1` (server), `2` (client)
- **Set by:** `exe.sh` or `master` command-line argument
- **Precedence:** Highest (overrides config and defaults)

**`SIM_NET_FLIP_Y`:**
- **Purpose:** Control Y-axis orientation for coordinate transformation
- **Values:** `0` (bottom-left origin), `1` (top-left origin)
- **Default:** `1` (matches most UI toolkits)
- **Recommendation:** Keep default unless custom UI

**`SIM_NET_ALPHA`:**
- **Purpose:** Rotate local coordinates before network exchange
- **Values:** `0`, `90`, `-90`, `180` (degrees)
- **Default:** `0` (no rotation)
- **Use Case:** Displays at different physical orientations

**`SIM_WD_PID`:**
- **Purpose:** Watchdog PID for monitored processes (Assignment 2)
- **Set by:** `master` in normal mode only
- **Effect:** Enables watchdog ping-ack protocol

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Mode Selection

There are three ways to select the operating mode:

**1. Interactive Script (Recommended):**
```bash
./exe.sh
# Presents menu:
#  1) Normal (Standalone)
#  2) Server (Host)
#  3) Client (Connect)
```
- Auto-detects local IP for server mode
- Prompts for server IP/port in client mode
- Updates config file automatically
- Sets `SIM_MODE` environment variable

**2. Direct Execution:**
```bash
cd build/src
./master 0    # Normal mode
./master 1    # Server mode
./master 2    # Client mode
```
- Bypasses interactive setup
- Requires manually editing config file
- Useful for scripting

**3. Environment Variable:**
```bash
export SIM_MODE=1
./run.sh
```
- Overrides config file setting
- Useful for testing without modifying config

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Usage

### Running Server Mode

**Step 1: Configure and Build**
```bash
./exe.sh
# Select option 2 (Server)
# Enter port (default 8888)
# Note the displayed IP address for clients
```

The script will display:
```
Server will LISTEN on 0.0.0.0:8888
Clients should CONNECT to: 192.168.1.100:8888
```

**Step 2: Server Starts**
- BB_SERVER window opens with simulation view
- INPUT window opens for keyboard controls
- Console shows "network_server: listening..."

**Step 3: Wait for Client**
- Server blocks until client connects
- Console shows "network_server: client connected" when client joins

**Step 4: Control Server Drone**
- Use INPUT window controls (q/w/e/a/s/d/z/x/c)
- Server drone is the `@` symbol
- Client drone appears as `#` (obstacle with repulsion)

**Step 5: Graceful Shutdown**
- Press `Q` in INPUT window to quit
- Server sends `q` to client
- Both sides close cleanly

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Running Client Mode

**Step 1: Configure and Build**
```bash
./exe.sh
# Select option 3 (Client)
# Enter server IP (e.g., 192.168.1.100)
# Enter server port (default 8888)
```

**Step 2: Client Starts**
- BB_SERVER window opens
- INPUT window opens
- Console shows "network_client: connecting to <IP>:<port>..."

**Step 3: Connection Established**
- Client receives window dimensions from server
- BB_SERVER resizes to match server world
- Console shows "network_client: handshake complete, size=WxH"

**Step 4: Control Client Drone**
- Use INPUT window controls
- Client drone is the `@` symbol
- Server drone appears as `*` (cyan, read-only)

**Step 5: Visual Synchronization**
- Client drone movements are sent to server
- Server drone position updates in real-time
- No local obstacles or targets (server controls environment)

**Step 6: Graceful Shutdown**
- Press `Q` in INPUT window, OR
- Server initiates disconnect (`q` received)
- Client sends `qok` and exits

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Running Normal Mode

**Step 1: Configure and Build**
```bash
./exe.sh
# Select option 1 (Normal)
```

**Step 2: Normal Execution**
- All features from Assignments 1 and 2
- Watchdog monitors all processes
- Obstacles and targets spawn dynamically
- No network communication

**Differences from Network Modes:**
- Only one drone (`@` symbol)
- Static obstacles (`#` symbols, yellow)
- Targets (`+` symbols, magenta)
- Watchdog ping-ack logging
- Score tracking for target collection

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Integration with Assignment 2

### Unchanged Components

The following Assignment 2 features remain **exactly the same** in normal mode:

- Watchdog process and ping-ack protocol
- File-based logging with locking (`processes.log`, `watchdog.log`)
- Process registration file (`processes.pid`)
- Core simulation logic (drone physics, repulsion forces)
- Ncurses-based UI with dynamic resizing
- Configuration file system

### Changed Components

**Master Process:**
- Now accepts mode argument from command line
- Exports `SIM_MODE` environment variable
- Spawns network processes in server/client modes
- Skips watchdog/obstacles/targets in network modes
- Creates additional pipes for network data flow

**BB_Server Process:**
- Mode-aware pipe configuration (different FD arguments per mode)
- Server mode: reads obstacles from network (client drone)
- Client mode: reads server drone position for visualization
- Client mode: receives window dimensions from network
- Updated UI to distinguish server drone (`*`) from local (`@`)

**Drone Process:**
- Unchanged! Always receives commands, outputs state
- Network-agnostic (doesn't know if forces come from network or local obstacles)

**Input Process:**
- Unchanged! Always reads keyboard, outputs commands
- Works identically in all three modes

**Obstacles/Targets Processes:**
- Skipped entirely in server/client modes
- Only spawned in normal mode

**Watchdog Process:**
- Skipped entirely in server/client modes
- Only spawned in normal mode
- Network processes self-monitor via socket state

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### New Components

**Network Server (`network_server.c`):**
- 400+ lines of protocol state machine
- Nonblocking I/O with select loop
- Coordinate transformation (local → virtual → client)
- Buffered send/receive for partial operations
- Graceful disconnect handling

**Network Client (`network_client.c`):**
- 350+ lines of protocol state machine
- Nonblocking I/O with select loop
- Coordinate transformation (virtual → local)
- Window dimension forwarding to bb_server
- Server drone visualization support

**Network Library (`sim_network.c` / `sim_network.h`):**
- Protocol token constants
- Line-based send/receive helpers
- Socket setup utilities (server, client, nonblocking, timeouts)
- Coordinate parsing and formatting

**Interactive Launcher (`exe.sh`):**
- Mode selection menu with visual prompts
- Automatic IP detection (Linux: `ip route`, macOS: `ipconfig`)
- Config file updating (sed-based)
- Server IP/port prompting for client mode
- Build system integration (CMake invocation)

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Updated Project Architecture

```
├── bin
│   ├── conf
│   │   ├── drone_parameters.conf  ← UPDATED: network_mode, server_address, server_port
│   │   └── [audio files: music.mp3, press.mp3, etc.]
│   └── log
│       ├── .gitkeep
│       ├── processes.log
│       ├── processes.pid
│       └── watchdog.log
├── build
├── files
│   └── assignmentsv4.7.pdf
├── headers
│   ├── CMakeLists.txt
│   ├── sim_const.h                ← UPDATED: NET_DEFAULT_ADDRESS, NET_DEFAULT_PORT
│   ├── sim_ipc.h
│   ├── sim_log.h
│   ├── sim_network.h              ← NEW: Network protocol definitions
│   ├── sim_params.h               ← UPDATED: Added network fields to SimParams
│   ├── sim_types.h                ← UPDATED: Added SimMode enum, network fields to WorldState
│   └── sim_ui.h
├── src
│   ├── bb_server.c                ← UPDATED: Mode-aware pipe handling, server drone visualization
│   ├── CMakeLists.txt             ← UPDATED: Added network_server, network_client targets
│   ├── drone.c
│   ├── input.c
│   ├── master.c                   ← UPDATED: Mode-aware process spawning, network pipe creation
│   ├── network_client.c           ← NEW: TCP client with protocol state machine
│   ├── network_server.c           ← NEW: TCP server with protocol state machine
│   ├── obstacles.c
│   ├── sim_ipc.c
│   ├── sim_log.c
│   ├── sim_network.c              ← NEW: Network protocol implementation
│   ├── sim_params.c               ← UPDATED: Parse network_mode, server_address, server_port
│   ├── sim_ui.c                   ← UPDATED: Display server drone, mode-aware legend
│   ├── targets.c
│   └── watchdog.c
├── .gitignore
├── CMakeLists.txt
├── diag.sh                        ← NEW: Diagnostic script for troubleshooting
├── exe.sh                         ← NEW: Interactive mode selection launcher
├── LICENSE
├── README.md                      ← UPDATED: Assignment 3 documentation (this file)
└── run.sh                         ← UPDATED: Network binary building
```

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Summary of Assignment 3 Additions

| Feature | Purpose | Implementation |
|---------|---------|----------------|
| **Network Server** | Host simulation for remote clients | TCP server with state machine protocol |
| **Network Client** | Connect to remote server | TCP client with state machine protocol |
| **Multi-Mode Operation** | Support normal/server/client modes | Mode-aware process spawning in master |
| **Coordinate Transformation** | Reconcile different UI orientations | Virtual coordinate system with flip/rotation |
| **Line-Based Protocol** | Human-readable network exchange | Newline-delimited tokens and coordinates |
| **Nonblocking I/O** | Simultaneous socket and pipe handling | fcntl O_NONBLOCK + select loop |
| **Interactive Launcher** | User-friendly mode selection | exe.sh with menu and IP auto-detection |
| **Window Synchronization** | Client matches server world size | Handshake transmits dimensions |
| **Server Drone Visualization** | Client sees server's drone | Additional pipe from network_client to bb_server |
| **Dynamic Obstacle** | Server sees client as obstacle | Client position sent as obstacle with repulsion |

### Key Architectural Changes

**Process Spawning (Mode-Dependent):**
- Normal: watchdog + obstacles + targets + no network
- Server: network_server + no watchdog/obstacles/targets
- Client: network_client + no watchdog/obstacles/targets

**Communication Topology:**

**Normal Mode:**
```
obstacles → bb_server → UI
targets   → bb_server → UI
input     → bb_server → drone → bb_server (closed loop)
```

**Server Mode:**
```
input → bb_server → drone → bb_server → network_server → [socket]
                                                             ↓
                        network_server ← [socket] (client drone as obstacle)
                                ↓
                           bb_server (repulsion forces)
```

**Client Mode:**
```
input → bb_server → drone → bb_server → network_client → [socket] (obstacle)
                                                             ↓
network_client ← [socket] (server drone) → bb_server (visualization)
```

### Protocol Guarantees

**Reliability:**
- TCP ensures ordered, reliable delivery
- No message loss or reordering
- Broken connections detected immediately

**Synchronization:**
- Server and client exchange positions every cycle (20ms typical)
- Coordinate transformations ensure consistent physics
- Window dimensions synchronized at handshake

**Error Recovery:**
- Protocol violations → graceful disconnect
- Pipe closure → send `q` to peer, clean shutdown
- SIGINT → send `q` to peer, wait for `qok`

These additions transform the Assignment 2 simulator into a **distributed multi-agent system** with networked drones, configurable coordinate systems, and interactive deployment options, while maintaining the reliability and observability features from previous assignments.

<p align="right">(<a href="#readme-top">back to top</a>)</p>