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

# --- Auto IP helpers (minimal add) ---
get_local_ip_linux() {
  ip -4 route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -n1
}
get_local_ip_darwin() {
  # best effort; depends on active interface, but good enough as a hint
  ipconfig getifaddr en0 2>/dev/null || ipconfig getifaddr en1 2>/dev/null
}
get_default_gateway_linux() {
  ip route 2>/dev/null | awk '/^default/ {print $3; exit}'
}

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

        # Only update Port/IP in config (server binds to all interfaces)
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/^server_address.*/server_address 0.0.0.0/" "$CONF_FILE"
            sed -i '' "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        else
            sed -i "s/^server_address.*/server_address 0.0.0.0/" "$CONF_FILE"
            sed -i "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        fi

        # Print the actual IP clients should use (auto-detect)
        if [[ "$OSTYPE" == "darwin"* ]]; then
            server_ip_detected="$(get_local_ip_darwin)"
        else
            server_ip_detected="$(get_local_ip_linux)"
        fi
        echo ""
        echo "Server will LISTEN on 0.0.0.0:$server_port"
        if [[ -n "$server_ip_detected" ]]; then
            echo "Clients should CONNECT to: $server_ip_detected:$server_port"
        else
            echo "Could not auto-detect server IP. Run: ip -4 route get 1.1.1.1"
        fi

        echo "Ready. Press Enter to start..."
        read
        ;;

    3)
        echo "=== CLIENT MODE SETUP ==="
        RUN_MODE=2

        # Auto-suggest:
        # - If SERVER_IP env var exists, use it
        # - Else on Linux, try default gateway as a hint
        # - Else fallback to 127.0.0.1
        if [[ -n "${SERVER_IP:-}" ]]; then
            default_server_ip="$SERVER_IP"
        elif [[ "$OSTYPE" == "darwin"* ]]; then
            default_server_ip="127.0.0.1"
        else
            gw="$(get_default_gateway_linux)"
            default_server_ip="${gw:-127.0.0.1}"
        fi

        read -p "Enter server IP [$default_server_ip]: " server_ip
        server_ip=${server_ip:-$default_server_ip}

        read -p "Enter server port [8888]: " server_port
        server_port=${server_port:-8888}

        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' "s/^server_address.*/server_address $server_ip/" "$CONF_FILE"
            sed -i '' "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        else
            sed -i "s/^server_address.*/server_address $server_ip/" "$CONF_FILE"
            sed -i "s/^server_port.*/server_port $server_port/" "$CONF_FILE"
        fi

        echo ""
        echo "Client will CONNECT to: $server_ip:$server_port"
        echo "Tip: if server printed an IP in Server mode, use that exact IP here."
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
