#!/usr/bin/env bash
# Cargo runner (see .cargo/config.toml). A configured runner owns the whole
# runtime environment, so supply the dynamic library paths cargo normally
# would — bevy's dylib (dynamic_linking), the rustc sysroot libs, and the
# downloaded MuJoCo — plus Ubuntu's /usr/lib/x86_64-linux-gnu (the nix glibc
# loader skips it, and libpython3.12.so for the `opensim` feature lives there).
#
# All paths are resolved relative to the repo root so this works on any
# machine (not just a hardcoded dev home).

# Repo root = parent of this script's directory (scripts/).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# rustc sysroot libs (path varies per toolchain/host).
RUST_SYSROOT_LIB=""
if command -v rustc >/dev/null 2>&1; then
  RUST_SYSROOT_LIB="$(rustc --print sysroot 2>/dev/null)/lib/rustlib/x86_64-unknown-linux-gnu/lib"
fi

export PYTHONPATH="${REPO_ROOT}/.venv-sys/lib/python3.12/site-packages:/opt/opensim-gui/sdk/Python${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="${REPO_ROOT}/target/debug/deps:${RUST_SYSROOT_LIB}:${REPO_ROOT}/.mujoco/mujoco-3.9.0/lib:/opt/opensim-gui/sdk/lib:/opt/opensim-gui/sdk/Simbody/lib:/opt/opensim-gui/bin:/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

exec "$@"
