#!/usr/bin/env bash
set -e

# --- Audio Dependencies Check (Keep as is) ---
if ! command -v mpg123 >/dev/null 2>&1 && \
   ! command -v paplay >/dev/null 2>&1 && \
   ! command -v aplay >/dev/null 2>&1; then
    echo "No audio player found. Install mpg123? (y/n)"
    read -r reply
    if [[ $reply == "y" ]]; then
        sudo apt update && sudo apt install -y mpg123
    fi
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BIN_DIR="$BUILD_DIR/src"
CONF_FILE="$ROOT_DIR/bin/conf/drone_parameters.conf"

# --- Mode Selection Menu ---
echo "================================"
echo " Select Simulation Mode:"
echo " 1) Normal (Standalone)"
echo " 2) Server (Host)"
echo " 3) Client (Connect)"
echo "================================"
read -p "Enter choice [1-3]: " mode_choice

# Convert choice to internal ID (0, 1, 2)
# We do NOT touch the config file for the mode.
RUN_MODE=0 

case $mode_choice in
    2)
        echo "=== SERVER MODE SETUP ==="
        RUN_MODE=1
        
        read -p "Enter port to listen on [8888]: " server_port
        server_port=${server_port:-8888}
        
        # Only update Port/IP in config
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/^server_address.*/server_address 0.0.0.0/" "$CONF_FILE"
            sed -i '' "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        else
            sed -i "s/^server_address.*/server_address 0.0.0.0/" "$CONF_FILE"
            sed -i "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        fi
        
        echo "Ready. Press Enter to start..."
        read
        ;;
        
    3)
        echo "=== CLIENT MODE SETUP ==="
        RUN_MODE=2
        
        read -p "Enter server IP [127.0.0.1]: " server_ip
        server_ip=${server_ip:-127.0.0.1}
        
        read -p "Enter server port [8888]: " server_port
        server_port=${server_port:-8888}
        
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/^server_address.*/server_address $server_ip/" "$CONF_FILE"
            sed -i '' "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        else
            sed -i "s/^server_address.*/server_address $server_ip/" "$CONF_FILE"
            sed -i "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        fi
        
        echo "Ready. Press Enter to start..."
        read
        ;;
        
    *)
        echo "Setting mode to NORMAL..."
        RUN_MODE=0
        ;;
esac

# --- Build ---
if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    rm -rf "$BUILD_DIR"
fi

cmake -S "$ROOT_DIR" -B "$BUILD_DIR"
cmake --build "$BUILD_DIR" --target master bb_server drone input obstacles targets watchdog network_server network_client

# --- Execute with Relay ---
echo ""
echo "Launching Master with Mode ID: $RUN_MODE"
sleep 1

cd "$BIN_DIR"
# The Magic Line: Pass the variable to the C program
exec ./master "$RUN_MODE"