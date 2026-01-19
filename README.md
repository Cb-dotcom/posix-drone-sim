<a id="readme-top"></a>

# Table of Contents
<details>
  <summary>View Dropdown</summary>
  <ol>
    <li><a href="#posix-drone-simulator---assignment-3">POSIX Drone Simulator - Assignment 3</a></li>
    <li><a href="#beloved-contributors">Beloved Contributors</a></li>
    <li><a href="#compatibility">Compatibility</a></li>
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
    <li><a href="#network-statistics-panel">Network Statistics Panel</a>
      <ul>
        <li><a href="#statistics-overview">Statistics Overview</a></li>
        <li><a href="#metrics-tracked">Metrics Tracked</a></li>
        <li><a href="#visual-indicators">Visual Indicators</a></li>
      </ul>
    </li>
    <li><a href="#connection-resilience">Connection Resilience</a>
      <ul>
        <li><a href="#automatic-reconnection">Automatic Reconnection</a></li>
        <li><a href="#reconnection-strategy">Reconnection Strategy</a></li>
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

## Compatibility

This implementation has been **tested and verified to work** with the following compatible implementation:

**Compatible Repository:** [https://github.com/Stef504/ARP-Assignment3](https://github.com/Stef504/ARP-Assignment3)

Our network protocol is fully interoperable with this implementation, allowing:
- Cross-implementation server-client connections
- Bidirectional communication (either implementation as server or client)
- Consistent coordinate transformation and data exchange

**Testing Scenarios Verified:**
- Our server ↔ Their client
- Their server ↔ Our client
- Extended gameplay sessions with stable communication
- Reconnection handling after network interruptions

This compatibility demonstrates the robustness of the protocol specification and our implementation's adherence to the assignment requirements.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Assignment 3 Overview

Assignment 3 extends the multi-process drone simulator from Assignments 1 and 2 with **network communication capabilities**. The system can now operate in three modes: standalone (normal), server, or client. In networked modes, two simulator instances communicate over TCP/IP to exchange drone positions and create an interactive multi-drone environment.

The major additions are:

1. **Network Processes**: Dedicated `network_server` and `network_client` processes handle TCP/IP communication with proper nonblocking I/O and state machine protocols.

2. **Multi-Mode Operation**: The simulator can run as a standalone system, as a network server hosting a simulation, or as a client connecting to a remote server.

3. **Coordinate Transformation**: A configurable coordinate system conversion allows different local UI orientations (top-left vs bottom-left) to be reconciled through a shared virtual coordinate space.

4. **Interactive Launch System**: The `exe.sh` script provides an interactive menu for mode selection with automatic IP detection and configuration.

5. **Real-Time Network Statistics**: A dedicated statistics panel provides live monitoring of connection health, throughput, latency, and error rates.

6. **Automatic Reconnection**: Client mode implements intelligent reconnection with exponential backoff to handle temporary network disruptions.

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
- **Network statistics panel** displays server-side metrics

**Client Mode:**
- Watchdog, obstacles, and targets are **NOT** spawned
- Client drone is controlled locally via input
- Server drone position is visualized but not interactive
- `network_client` process handles server communication
- **Network statistics panel** displays client-side metrics
- **Automatic reconnection** attempts on connection loss

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
- **Real-time statistics panel** shows connection health

**Mode 2: Client**
- Connects to a remote server
- Client drone controlled locally (keyboard input)
- Server drone visualized as read-only position marker
- No static obstacles or targets
- Window dimensions received from server
- No watchdog (network process manages lifecycle)
- **Automatic reconnection** with up to 5 retry attempts
- **Real-time statistics panel** shows connection metrics

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
├── bb_server (konsole + stats panel)
├── input (konsole)
├── drone
└── network_server
    └── [TCP socket] → network_client
```

**Client Mode:**
```
master
├── bb_server (konsole + stats panel)
├── input (konsole)
├── drone
└── network_client (with auto-reconnect)
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
Server → Server: pok
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

## Network Statistics Panel

### Statistics Overview

Both server and client modes feature a **real-time network statistics panel** displayed on the right side of the screen. This panel provides instant visibility into connection health, data throughput, and communication quality.

**Panel Location:**
- Right side of BB_SERVER window
- Updates automatically every frame (30Hz typical)
- Color-coded indicators for quick status assessment

**Purpose:**
- Monitor connection stability during gameplay
- Diagnose network issues in real-time
- Track performance metrics for optimization
- Provide feedback during reconnection attempts

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Metrics Tracked

The statistics panel displays the following metrics:

**Connection Status:**
- **CONNECTED** (green) - Active network connection
- **DISCONNECTED** (red) - No active connection
- Updated immediately on connection state changes

**Packet Statistics:**
- **Packets Sent** - Total outbound protocol messages
- **Packets Received** - Total inbound protocol messages
- Includes all protocol tokens and data exchanges

**Data Transfer:**
- **Bytes Sent** - Total outbound data in KB
- **Bytes Received** - Total inbound data in KB
- Converted from raw byte counts for readability

**Latency Metrics:**
- **Current Latency** - Most recent round-trip time (ms)
- **Average Latency** - Moving average with 0.7 smoothing factor
- Measured from packet send to acknowledgment receipt

**Bandwidth:**
- **Current Bandwidth** - Data rate in KB/s
- Calculated over connection lifetime
- Includes both sent and received data

**Error Statistics:**
- **Protocol Errors** - Invalid message format or sequence
- **Connection Drops** - Number of disconnections
- Only displayed when non-zero (keeps panel clean)

**Reconnection Tracking:**
- **Reconnect Attempts** - Counter for client retry attempts
- Only shown in client mode after reconnection
- Helps diagnose persistent connection issues

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Visual Indicators

The panel uses color coding for quick status assessment:

**Color Scheme:**
- **Green (COLOR_PAIR 5)** - Good/normal status
  - CONNECTED state
  - Latency < 50ms
  - General statistics display

- **Yellow (COLOR_PAIR 2)** - Warning state
  - Latency 50-100ms
  - Indicates potential network congestion

- **Red (COLOR_PAIR 6)** - Critical state
  - DISCONNECTED state
  - Latency >= 100ms
  - Protocol errors or connection drops

**Example Display:**
```
┌─ Network Stats ──┐
│ Status: CONNECTED│  (green)
│                  │
│ Packets:         │
│   Sent: 1523     │
│   Recv: 1521     │
│                  │
│ Data Transfer:   │
│   Sent: 76.15 KB │
│   Recv: 76.05 KB │
│                  │
│ Latency:         │
│   Avg: 15.3 ms   │  (green)
│   Cur: 14.8 ms   │
│                  │
│ Bandwidth:       │
│   2.45 KB/s      │
└──────────────────┘
```

**Panel Updates:**
- Statistics refresh every 30 frames (~1 second)
- Latency calculated per exchange cycle
- Bandwidth averaged over connection lifetime
- Panel only visible in server/client modes

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Connection Resilience

### Automatic Reconnection

The client implementation features **automatic reconnection** to handle temporary network disruptions gracefully. This ensures uninterrupted gameplay despite transient connection issues.

**Key Features:**
- Up to 5 reconnection attempts before giving up
- 3-second delay between attempts (prevents server flooding)
- Preserves local state during reconnection
- Detailed logging of each attempt
- Transparent to user (no manual intervention needed)

**Reconnection Triggers:**
- Server closes connection unexpectedly
- Network timeout during data exchange
- Socket read/write errors
- Protocol state machine errors

**User Experience:**
- Local drone continues operating during reconnection
- Statistics panel shows reconnection attempts counter
- Console displays "attempting to reconnect..." messages
- Simulation pauses network exchanges but maintains physics

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Reconnection Strategy

The client uses a **simple retry with fixed delay** strategy:

**Reconnection Flow:**
```
1. Detect connection failure
2. Close existing socket
3. Wait 3 seconds (prevents server overload)
4. Attempt TCP connection to server
5. If successful:
   - Reset statistics
   - Restart protocol handshake
   - Resume normal operation
6. If failed:
   - Increment attempt counter
   - Check if < MAX_RECONNECT_ATTEMPTS (5)
   - If yes: goto step 3
   - If no: exit client process
```

**Implementation Details:**
```c
#define MAX_RECONNECT_ATTEMPTS 5
#define RECONNECT_DELAY_SEC 3

static int connect_with_retry(const char *address, int port, NetworkStats *stats)
{
    int attempts = 0;
    
    while (attempts < MAX_RECONNECT_ATTEMPTS && running) {
        sim_log_info("network_client: connecting to %s:%d (attempt %d/%d)", 
                     address, port, attempts + 1, MAX_RECONNECT_ATTEMPTS);
        
        int sock = net_connect_to_server(address, port);
        if (sock >= 0) {
            sim_log_info("network_client: connected successfully");
            stats_on_connect(stats);
            return sock;
        }
        
        attempts++;
        stats->reconnect_attempts++;
        
        if (attempts < MAX_RECONNECT_ATTEMPTS && running) {
            sim_log_info("network_client: connection failed, retrying in %d seconds...", 
                         RECONNECT_DELAY_SEC);
            sleep(RECONNECT_DELAY_SEC);
        }
    }
    
    sim_log_info("network_client: failed to connect after %d attempts", MAX_RECONNECT_ATTEMPTS);
    return -1;
}
```

**Reconnection Scenarios:**

**Scenario 1: Temporary Server Restart**
- Server shuts down for maintenance
- Client detects disconnection
- Client waits 3 seconds (server restarts)
- Client reconnects successfully on attempt 2
- Gameplay resumes with fresh statistics

**Scenario 2: Network Interruption**
- WiFi drops briefly
- Client detects socket error
- Client attempts reconnection while network recovers
- Reconnection succeeds when network restored
- No data loss (stateless protocol)

**Scenario 3: Permanent Server Failure**
- Server crashes and doesn't restart
- Client attempts 5 reconnections (15 seconds total)
- All attempts fail
- Client logs failure and exits gracefully
- User sees final statistics and reconnection count

**Logging Output:**
```
network_client: connection closed
network_client: attempting to reconnect...
network_client: connecting to 192.168.1.100:8888 (attempt 1/5)
network_client: connection failed, retrying in 3 seconds...
network_client: connecting to 192.168.1.100:8888 (attempt 2/5)
network_client: connected successfully
network_client: handshake complete, size=50x50
```

**Statistics Tracking:**
- Each reconnection increments `reconnect_attempts` counter
- Displayed in statistics panel
- Logged in `processes.log` for post-mortem analysis
- Connection drops tracked separately from attempts

**Why Not Exponential Backoff?**
Our fixed 3-second delay is intentional:
- Server restarts typically take 2-5 seconds
- Exponential backoff unnecessary for 5-attempt limit
- Simpler implementation, easier to reason about
- Prevents indefinite reconnection loops

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
- Track and log network statistics

**Process Lifecycle:**
1. Create listening socket on configured port
2. Block waiting for client connection (`accept`)
3. Handshake with client (send window size)
4. Enter nonblocking data exchange loop
5. On disconnect or error, close sockets
6. Return to listening state (server persistence)

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
                                           ↓ update stats
                                           ↓
network_client ← socket ← "drone X.X, Y.Y"
                                           ↓
network_client → socket → "X.X, Y.Y" (client drone)
                                           ↓ recv via socket
                                           ↓ update stats
                                           ↓ transform to local
network_server → pipe_network_obstacle_in[1] → bb_server (as obstacle[0])
```

**Key Points:**
- Server's own drone is controlled via local `input` process
- Client drone appears in `obstacles[0]` with radius 1.0
- Repulsion forces apply to client drone (it's a real obstacle)
- No static obstacles or targets exist in server mode
- Statistics logged but not sent to bb_server (local metrics only)

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
11. **`S_DONE`**: Connection complete, return to listening

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
- Socket error → break main loop, return to listening
- Ctrl+C (`SIGINT`) → send `q`, transition to `S_WAIT_QOK`

<p align="right">(<a href="#readme-top">back to top</a>)</p>

---

## Client Mode

### Client Architecture

The client process (`network_client`) connects to a remote server and exchanges drone positions:

**Responsibilities:**
- Connect to server TCP socket (address and port from config)
- **Retry connection** up to 5 times with 3-second delays
- Execute protocol handshake (receive window dimensions)
- Forward window dimensions to bb_server (so UI matches server)
- Read local drone position from bb_server via pipe
- Send local drone position to server as obstacle
- Receive server drone position from socket
- Write server drone position to bb_server via pipe (for visualization)
- Perform coordinate transformations (local ↔ virtual)
- Track and log network statistics
- **Automatically reconnect** on connection loss

**Process Lifecycle:**
1. Attempt connection to server (with retry logic)
2. Handshake with server (receive and forward window size)
3. Enter nonblocking data exchange loop
4. On disconnect or error:
   - Close socket and log disconnection
   - Attempt reconnection (up to 5 times)
   - If all attempts fail: exit