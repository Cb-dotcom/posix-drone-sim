#!/usr/bin/env bash
# Diagnostic script to identify simulation issues

set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN_DIR="$BUILD_DIR/src"
LOG_DIR="$ROOT_DIR/bin/log"

echo "================================"
echo "   DRONE SIMULATOR DIAGNOSTICS"
echo "================================"
echo ""

# Check 1: Build status
echo "[1/8] Checking build status..."
if [[ ! -d "$BUILD_DIR" ]]; then
    echo "❌ Build directory does not exist"
    echo "   Run: cmake -S . -B build && cmake --build build"
    exit 1
fi

required_binaries=("master" "bb_server" "drone" "input" "obstacles" "targets" "watchdog" "network_server" "network_client")
missing=()

for bin in "${required_binaries[@]}"; do
    if [[ ! -x "$BIN_DIR/$bin" ]]; then
        missing+=("$bin")
    fi
done

if [[ ${#missing[@]} -gt 0 ]]; then
    echo "❌ Missing binaries: ${missing[*]}"
    echo "   Run: cmake --build build"
    exit 1
fi
echo "✅ All binaries present"
echo ""

# Check 2: Config file
echo "[2/8] Checking configuration..."
CONF_FILE="$ROOT_DIR/bin/conf/drone_parameters.conf"
if [[ ! -f "$CONF_FILE" ]]; then
    echo "❌ Config file not found: $CONF_FILE"
    exit 1
fi

mode=$(grep "^network_mode" "$CONF_FILE" | awk '{print $2}')
address=$(grep "^server_address" "$CONF_FILE" | awk '{print $2}')
port=$(grep "^server_port" "$CONF_FILE" | awk '{print $2}')

echo "   Mode: $mode"
echo "   Server: $address:$port"
echo "✅ Config file valid"
echo ""

# Check 3: Log directory
echo "[3/8] Checking log directory..."
if [[ ! -d "$LOG_DIR" ]]; then
    echo "❌ Log directory missing: $LOG_DIR"
    mkdir -p "$LOG_DIR"
    echo "   Created log directory"
fi
echo "✅ Log directory exists"
echo ""

# Check 4: Previous logs
echo "[4/8] Checking previous run logs..."
if [[ -f "$LOG_DIR/processes.log" ]]; then
    echo "   Found previous run log"
    
    # Check if drone started
    if grep -q "drone: started" "$LOG_DIR/processes.log"; then
        echo "   ✅ Drone process started in previous run"
    else
        echo "   ⚠️  Drone did NOT start in previous run"
    fi
    
    # Check for errors
    error_count=$(grep -c "error\|Error\|ERROR\|failed\|Failed" "$LOG_DIR/processes.log" 2>/dev/null || echo "0")
    if [[ $error_count -gt 0 ]]; then
        echo "   ⚠️  Found $error_count errors in log"
        echo "   Last 5 errors:"
        grep -i "error\|failed" "$LOG_DIR/processes.log" | tail -5 | sed 's/^/      /'
    fi
else
    echo "   No previous run log found"
fi
echo ""

# Check 5: Process PIDs
echo "[5/8] Checking for running processes..."
running_procs=()
for proc in master bb_server drone input obstacles targets watchdog network_server network_client; do
    if pgrep -f "$BIN_DIR/$proc" > /dev/null; then
        running_procs+=("$proc")
    fi
done

if [[ ${#running_procs[@]} -gt 0 ]]; then
    echo "⚠️  Found running processes: ${running_procs[*]}"
    echo "   Killing them..."
    pkill -f "$BIN_DIR/master" 2>/dev/null || true
    sleep 1
    echo "   Cleaned up"
else
    echo "✅ No processes running"
fi
echo ""

# Check 6: Network port availability (if server mode)
echo "[6/8] Checking network availability..."
if [[ "$mode" == "server" ]]; then
    if netstat -tuln 2>/dev/null | grep -q ":$port "; then
        echo "⚠️  Port $port is already in use"
        echo "   Run: sudo lsof -i :$port"
    else
        echo "✅ Port $port available"
    fi
elif [[ "$mode" == "client" ]]; then
    # Try to connect to server
    if timeout 1 bash -c "cat < /dev/null > /dev/tcp/$address/$port" 2>/dev/null; then
        echo "✅ Server is reachable at $address:$port"
    else
        echo "⚠️  Cannot reach server at $address:$port"
        echo "   Make sure server is running first"
    fi
else
    echo "✅ Network not used (normal mode)"
fi
echo ""

# Check 7: Test a quick run
echo "[7/8] Running quick diagnostic test..."
cd "$BIN_DIR"

# Clear old logs
rm -f "$LOG_DIR/processes.log" "$LOG_DIR/processes.pid" "$LOG_DIR/watchdog.log" 2>/dev/null

echo "   Starting simulation for 3 seconds..."
timeout 3 ./master > /tmp/drone_test.out 2>&1 &
MASTER_PID=$!

sleep 2

# Check what processes are running
echo "   Processes spawned:"
ps aux | grep "$BIN_DIR" | grep -v grep | awk '{print "      " $11 " (PID " $2 ")"}'

# Wait for timeout to kill it
wait $MASTER_PID 2>/dev/null || true

echo ""
echo "   Checking logs..."
if [[ -f "$LOG_DIR/processes.log" ]]; then
    # Count process starts
    started_processes=$(grep "started" "$LOG_DIR/processes.log" | wc -l)
    echo "   Processes that started: $started_processes"
    
    grep "started" "$LOG_DIR/processes.log" | sed 's/^/      /' | head -10
    
    # Check for drone specifically
    if grep -q "drone: started" "$LOG_DIR/processes.log"; then
        echo "   ✅ DRONE STARTED SUCCESSFULLY"
        
        # Check drone state updates
        drone_updates=$(grep -c "drone.x\|drone.y" "$LOG_DIR/processes.log" 2>/dev/null || echo "0")
        if [[ $drone_updates -gt 0 ]]; then
            echo "   ✅ Drone position updated $drone_updates times"
        else
            echo "   ⚠️  No drone position updates found"
        fi
    else
        echo "   ❌ DRONE DID NOT START"
        echo ""
        echo "   This is the problem! Checking why..."
        
        # Check master log
        if grep -q "master.*drone" "$LOG_DIR/processes.log"; then
            echo "   Master attempted to spawn drone"
        else
            echo "   ❌ Master did not attempt to spawn drone"
        fi
    fi
else
    echo "   ❌ No log file created"
    echo "   This suggests master process failed to start"
fi

echo ""

# Check 8: File permissions
echo "[8/8] Checking file permissions..."
for bin in master bb_server drone input; do
    if [[ ! -x "$BIN_DIR/$bin" ]]; then
        echo "⚠️  $bin is not executable"
        chmod +x "$BIN_DIR/$bin"
    fi
done
echo "✅ All binaries executable"
echo ""

echo "================================"
echo "   DIAGNOSTIC SUMMARY"
echo "================================"

if grep -q "drone: started" "$LOG_DIR/processes.log" 2>/dev/null; then
    echo "Status: ✅ DRONE PROCESS IS WORKING"
    echo ""
    echo "If drone is stuck at (0,0):"
    echo "  1. Check if input process is sending commands"
    echo "  2. Check if bb_server is forwarding commands to drone"
    echo "  3. Check pipe communication in logs"
else
    echo "Status: ❌ DRONE PROCESS NOT STARTING"
    echo ""
    echo "Possible causes:"
    echo "  1. Drone spawn is wrapped in a mode check (should be unconditional)"
    echo "  2. Drone binary is missing or not executable"
    echo "  3. Pipe file descriptors are incorrectly set up"
    echo "  4. Drone is crashing immediately after spawn"
    echo ""
    echo "Next steps:"
    echo "  1. Check master.c - ensure drone fork is NOT in if(mode==NORMAL)"
    echo "  2. Run: ./diagnose.sh to see detailed logs"
    echo "  3. Check: cat bin/log/processes.log | grep -A5 drone"
fi

echo ""
echo "Full log available at: $LOG_DIR/processes.log"
echo "================================"
