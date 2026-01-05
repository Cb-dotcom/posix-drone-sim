#!/usr/bin/env bash
set -e

# --- Audio Dependencies Check ---
if ! command -v mpg123 >/dev/null 2>&1 && \
   ! command -v paplay >/dev/null 2>&1 && \
   ! command -v aplay >/dev/null 2>&1; then

    echo "No audio player found (mpg123/paplay/aplay)."
    echo "Install mpg123? (y/n)"
    read -r reply
    if [[ $reply == "y" ]]; then
        sudo apt update
        sudo apt install -y mpg123
    else
        echo "Warning: No audio playback will be available."
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

case $mode_choice in
    2)
        echo "=== SERVER MODE SETUP ==="
        
        # Get server port
        read -p "Enter port to listen on [8888]: " server_port
        server_port=${server_port:-8888}
        
        echo "Setting mode to SERVER..."
        echo "Server will listen on 0.0.0.0:$server_port"
        
        # Update config file
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/^network_mode.*/network_mode server/" "$CONF_FILE"
            sed -i '' "s/^server_address.*/server_address 0.0.0.0/" "$CONF_FILE"
            sed -i '' "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        else
            sed -i "s/^network_mode.*/network_mode server/" "$CONF_FILE"
            sed -i "s/^server_address.*/server_address 0.0.0.0/" "$CONF_FILE"
            sed -i "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        fi
        
        echo ""
        echo "Server configured. Waiting for client connection..."
        echo "Press Enter to start server..."
        read
        ;;
        
    3)
        echo "=== CLIENT MODE SETUP ==="
        
        # Get server address
        read -p "Enter server IP address [127.0.0.1]: " server_ip
        server_ip=${server_ip:-127.0.0.1}
        
        # Get server port
        read -p "Enter server port [8888]: " server_port
        server_port=${server_port:-8888}
        
        echo "Setting mode to CLIENT..."
        echo "Client will connect to $server_ip:$server_port"
        
        # Update config file
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/^network_mode.*/network_mode client/" "$CONF_FILE"
            sed -i '' "s/^server_address.*/server_address $server_ip/" "$CONF_FILE"
            sed -i '' "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        else
            sed -i "s/^network_mode.*/network_mode client/" "$CONF_FILE"
            sed -i "s/^server_address.*/server_address $server_ip/" "$CONF_FILE"
            sed -i "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        fi
        
        echo ""
        echo "Client configured."
        echo "Press Enter to connect to server..."
        read
        ;;
        
    *)
        echo "Setting mode to NORMAL..."
        
        # Update config file
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/^network_mode.*/network_mode normal/" "$CONF_FILE"
        else
            sed -i "s/^network_mode.*/network_mode normal/" "$CONF_FILE"
        fi
        ;;
esac

# --- Build and Run ---

# If cache is broken or missing, clean build folder
if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    echo "No valid CMake cache found. Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Configure
echo "Configuring build..."
cmake -S "$ROOT_DIR" -B "$BUILD_DIR"

# Build targets
echo "Building executables..."
cmake --build "$BUILD_DIR" --target master bb_server drone input obstacles targets watchdog network_server network_client

# Sanity check
if [[ ! -x "$BIN_DIR/master" ]]; then
    echo "Error: $BIN_DIR/master not found or not executable."
    exit 1
fi

# Show final config
echo ""
echo "=== Configuration ==="
grep "^network_mode" "$CONF_FILE"
grep "^server_address" "$CONF_FILE"
grep "^server_port" "$CONF_FILE"
echo "====================="
echo ""
echo "Starting simulation..."
sleep 1

cd "$BIN_DIR"
exec ./master
