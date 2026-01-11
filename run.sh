#!/usr/bin/env bash
set -e

# install the required dependencies for the music
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

# If cache is broken or missing, clean build folder
if [[ ! -f "$BUILD_DIR/CMakeCache.txt" ]]; then
    echo "No valid CMake cache found. Cleaning build directory..."
    rm -rf "$BUILD_DIR"
fi

# Configure
cmake -S "$ROOT_DIR" -B "$BUILD_DIR"

# Build targets (include network binaries for assignment 3)
cmake --build "$BUILD_DIR" --target \
    master bb_server drone input obstacles targets watchdog \
    network_server network_client

# Sanity check
if [[ ! -x "$BIN_DIR/master" ]]; then
    echo "Error: $BIN_DIR/master not found or not executable."
    echo "Contents of $BIN_DIR:"
    ls -l "$BIN_DIR"
    exit 1
fi

cd "$BIN_DIR"

# Mode selection:
#   0 = normal (default)
#   1 = server
#   2 = client
MODE="${1:-0}"

# Coordinate exchange settings (virtual system is bottom-left)
# If your local UI coords are top-left with y going DOWN, set this to 1.
export SIM_NET_FLIP_Y=1

# Rotation in degrees: 0, 90, -90, 180
export SIM_NET_ALPHA=0

exec ./master "$MODE"
