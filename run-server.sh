#!/usr/bin/env bash
# Run the melosim-server with frontend and correct library paths.
# Usage: ./run-server.sh [PORT] [MESH_DIR]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# MuJoCo library path (auto-downloaded by mujoco-rs)
MUJOCO_LIB="$SCRIPT_DIR/.mujoco/mujoco-3.9.0/lib"
if [ -d "$MUJOCO_LIB" ]; then
  export LD_LIBRARY_PATH="$MUJOCO_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
else
  echo "Warning: MuJoCo library not found at $MUJOCO_LIB"
  echo "Run 'nix develop' first to download MuJoCo."
fi

# Check for myo_sim test fixtures
MYO_SIM_DIR="$SCRIPT_DIR/tests/fixtures/myo_sim"
if [ ! -d "$MYO_SIM_DIR" ]; then
  echo "Test fixtures not found. Cloning MyoSuite models..."
  cd "$SCRIPT_DIR/tests/fixtures"
  git clone https://github.com/MyoHub/myo_sim.git
  cd "$SCRIPT_DIR"
fi

# Build frontend if needed
FRONTEND_DIR="$SCRIPT_DIR/frontend"
FRONTEND_DIST="$FRONTEND_DIR/dist"
if [ -d "$FRONTEND_DIR" ]; then
  if [ ! -d "$FRONTEND_DIST" ] || [ "$FRONTEND_DIR/src" -nt "$FRONTEND_DIST" ]; then
    echo "Building frontend..."
    cd "$FRONTEND_DIR"
    if [ ! -d "node_modules" ]; then
      npm install
    fi
    npm run build
    cd "$SCRIPT_DIR"
  fi
fi

# Default port
PORT="${1:-3000}"
export PORT

# Default mesh directory
MESH_DIR="${2:-tests/fixtures/myo_sim/meshes}"
export MESH_DIR

# Serve frontend from dist
STATIC_DIR="${FRONTEND_DIST:-static}"
export STATIC_DIR

echo "Starting melosim-server on port $PORT"
echo "  Mesh directory: $MESH_DIR"
echo "  Static files: $STATIC_DIR"
echo ""
echo "  Open http://localhost:$PORT in your browser"

# Build and run
exec cargo run -p melosim-server
