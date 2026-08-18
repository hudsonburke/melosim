#!/usr/bin/env bash
# Cargo runner (see .cargo/config.toml). A configured runner owns the whole
# runtime environment, so supply the dynamic library paths cargo normally
# would — bevy's dylib (dynamic_linking) and the rustc sysroot libs, the
# downloaded MuJoCo — plus Ubuntu's /usr/lib/x86_64-linux-gnu: the nix glibc
# loader skips it, and libpython3.12.so (opensim feature) lives there.
export LD_LIBRARY_PATH="target/debug/deps:/home/hudson/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/lib/rustlib/x86_64-unknown-linux-gnu/lib:/home/hudson/melosim/.mujoco/mujoco-3.9.0/lib:/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$@"
