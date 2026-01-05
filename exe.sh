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
echo "--------------------------------"
echo " Select Simulation Mode:"
echo " 1) Normal (Standalone)"
echo " 2) Server (Host)"
echo " 3) Client (Connect)"
echo "--------------------------------"
read -p "Enter choice [1-3]: " mode_choice

case $mode_choice in
    2)
        echo "Setting mode to SERVER..."
        # Update config file using sed
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' 's/^network_mode.*/network_mode server/' "$CONF_FILE"
        else
            sed -i 's/^network_mode.*/network_mode server/' "$CONF_FILE"
        fi
        ;;
    3)
        echo "Setting mode to CLIENT..."
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' 's/^network_mode.*/network_mode client/' "$CONF_FILE"
        else
            sed -i 's/^network_mode.*/network_mode client/' "$CONF_FILE"
        fi
        ;;
    *)
        echo "Setting mode to NORMAL..."
        if [[ "$OSTYPE" == "darwin"* ]]; then
            sed -i '' 's/^network_mode.*/network_mode normal/' "$CONF_FILE"
        else
            sed -i 's/^network_mode.*/network_mode normal/' "$CONF_FILE"
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
cmake -S "$ROOT_DIR" -B "$BUILD_DIR"

# Build targets
cmake --build "$BUILD_DIR" --target master bb_server drone input obstacles targets watchdog

# Sanity check
if [[ ! -x "$BIN_DIR/master" ]]; then
    echo "Error: $BIN_DIR/master not found or not executable."
    exit 1
fi

cd "$BIN_DIR"
exec ./master
