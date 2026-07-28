{
  description = "melosim — biomechanics ECS in Rust";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, rust-overlay }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        lib = nixpkgs.lib;
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };
        pkgsX86 = import nixpkgs { system = "x86_64-linux"; };
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
        };

        # Native Python for core crate tests (no OpenSim needed)
        pythonEnv = (pkgs.python313.withPackages (ps: with ps; [
          pip setuptools wheel numpy
        ]));

        qemu = pkgs.qemu;
        isAarch64 = system == "aarch64-linux";

        # MuJoCo: auto-downloaded at build time via mujoco-rs crate.
        # Set MUJOCO_DOWNLOAD_DIR so the crate knows where to store it.
        # Also add the .so to LD_LIBRARY_PATH for runtime.
        mujocoDownloadDir = "$PWD/.mujoco";
      in {
        packages.default = rustToolchain;

        devShells.default = pkgs.mkShell {
          buildInputs = [
            rustToolchain
            pkgs.maturin
            pythonEnv
            pkgs.openssl
            pkgs.libGL pkgs.libGLU pkgs.libX11 pkgs.libXi
            pkgs.libXmu pkgs.libXt pkgs.freetype pkgs.fontconfig
            pkgs.libxcursor pkgs.libxrandr pkgs.libxinerama
            qemu
          ];

          nativeBuildInputs = [ pkgs.pkg-config pkgs.cmake ];

          # Cross-arch roundtrip setup:
          #   x86_64:    native Python + pip install opensim
          #   aarch64:   QEMU + x86_64 Python fetched from cache + pip install opensim
          #
          # The x86_64 Python is fetched from the binary cache (not built locally).
          # We avoid pkgsX86.python313.withPackages() because buildEnv derivations
          # can't be substituted cross-arch — we use bare python313 instead and
          # create the venv at runtime.
          shellHook = ''
            echo "melosim dev shell"
            echo "  system: ${system}"
            echo "  rust: $(rustc --version)"
            echo "  cargo: $(cargo --version)"

            # ── MuJoCo setup ──
            # mujoco-rs auto-downloads MuJoCo 3.9.0 into MUJOCO_DOWNLOAD_DIR
            # on first build. We also add the .so to LD_LIBRARY_PATH.
            export MUJOCO_DOWNLOAD_DIR="${mujocoDownloadDir}"
            if [ -d "${mujocoDownloadDir}/mujoco-3.9.0/lib" ]; then
              export LD_LIBRARY_PATH="${mujocoDownloadDir}/mujoco-3.9.0/lib''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
            fi

            if [ "${system}" = "aarch64-linux" ]; then
              echo "  arch: aarch64 (QEMU for x86_64 OpenSim)"

              X86_QEMU="${qemu}/bin/qemu-x86_64"
              X86_PY="${pkgsX86.python313}/bin/python3.13"
              X86_ZLIB="${pkgsX86.zlib}/lib"
              X86_GCC_LIB="${pkgsX86.stdenv.cc.cc.lib}/lib"
              X86_GLIBC="${pkgsX86.glibc}/lib"

              # Create x86_64 environment for OpenSim under QEMU.
              # pip is not available in Nix's bare python313, so we bootstrap it.
              if [ ! -f "$PWD/.x86-pip-ready" ]; then
                echo "  Setting up x86_64 Python with OpenSim (under QEMU)..."
                # Bootstrap pip
                curl -sS https://bootstrap.pypa.io/get-pip.py 2>/dev/null | \
                  $X86_QEMU $X86_PY - --break-system-packages 2>/dev/null
                # Install opensim
                LD_LIBRARY_PATH="$X86_ZLIB:$X86_GCC_LIB" \
                  $X86_QEMU $X86_PY -m pip install opensim --break-system-packages 2>/dev/null
                touch "$PWD/.x86-pip-ready"
                echo "  x86_64 OpenSim ready."
              fi

              LD_LIBRARY_PATH="$X86_ZLIB:$X86_GCC_LIB''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

              roundtrip() {
                local input="$1"
                local output="''${2:-roundtrip_output.osim}"

                if [ "''${input#*.}" = "json" ]; then
                  cargo run -p melosim-py --bin roundtrip -- --from-json "$input" "$output"
                else
                  local json_tmp=$(mktemp /tmp/melosim-XXXXXX.json)
                  echo "[1/2] Extracting via QEMU..."
                  LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
                    $X86_QEMU $X86_PY scripts/extract_opensim.py "$input" "$json_tmp"
                  echo "[2/2] Importing + exporting (native)..."
                  cargo run -p melosim-py --bin roundtrip -- --from-json "$json_tmp" "$output"
                  rm -f "$json_tmp"
                fi
              }

            else
              # x86_64 — native OpenSim
              echo "  arch: x86_64 (native OpenSim)"

              if [ ! -d .venv ]; then
                echo "  Setting up Python venv with OpenSim..."
                python3 -m venv .venv
                .venv/bin/pip install --quiet opensim
              fi
              source .venv/bin/activate

              roundtrip() {
                cargo run -p melosim-py --bin roundtrip -- "$@"
              }
            fi

            echo "  Roundtrip: roundtrip <input.osim> [output.osim]"
            echo "  Server:    ./run-server.sh [PORT] [MESH_DIR]"
            echo "  python: $(python --version 2>/dev/null || echo 'n/a')"
          '';
        };

        # OCI container for CI / portable testing.
        # Best built on x86_64 where OpenSim pip package works natively.
        packages.opensim-container = pkgs.dockerTools.buildImage {
          name = "melosim";
          tag = "latest";
          copyToRoot = pkgs.buildEnv {
            name = "image-root";
            paths = with pkgs; [
              bashInteractive coreutils-full gnutar gzip git gcc cmake
              pkg-config openssl
              libGL libGLU libX11 libXi libXmu libXt
              freetype fontconfig libxcursor libxrandr libxinerama
              pythonEnv
              qemu
              (rustToolchain.override { extensions = [ "rust-src" "clippy" ]; })
            ];
          };
          config = {
            Cmd = [ "${pkgs.bashInteractive}/bin/bash" ];
            WorkingDir = "/workspace";
            Env = [
              "LD_LIBRARY_PATH=${with pkgs; lib.makeLibraryPath [
                libGL libGLU libX11 libXi libXmu libXt freetype fontconfig
                libxcursor libxrandr libxinerama
              ]}"
              "MUJOCO_DOWNLOAD_DIR=/workspace/.mujoco"
            ];
          };
        };
      });
}
