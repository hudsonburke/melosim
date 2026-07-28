#!/usr/bin/env bash
# Run the melosim-server with correct library paths.
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

# Default port
PORT="${1:-3000}"
export PORT

# Default mesh directory
MESH_DIR="${2:-tests/fixtures/myo_sim/meshes}"
export MESH_DIR

echo "Starting melosim-server on port $PORT"
echo "Mesh directory: $MESH_DIR"

# Build and run
exec cargo run -p melosim-server
