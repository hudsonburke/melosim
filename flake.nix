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
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" ] (system:
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

        # Helper: write a shell script that becomes a flake app
        mkAppScript = name: text: pkgs.writeShellScriptBin name text;

        # Shared env setup prefix for anything that needs MuJoCo + frontend
        setupPrefix = ''
          set -euo pipefail
          cd "$PWD"

          # MuJoCo
          export MUJOCO_DOWNLOAD_DIR="${mujocoDownloadDir}"
          MUJOCO_LIB="${mujocoDownloadDir}/mujoco-3.9.0/lib"
          if [ -d "$MUJOCO_LIB" ]; then
            export LD_LIBRARY_PATH="$MUJOCO_LIB''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
          fi

          # Auto-clone myo_sim test fixtures
          MYO_SIM_DIR="$PWD/tests/fixtures/myo_sim"
          if [ ! -d "$MYO_SIM_DIR" ]; then
            echo "Cloning MyoSuite test fixtures..."
            mkdir -p "$PWD/tests/fixtures"
            git clone --depth 1 https://github.com/MyoHub/myo_sim.git "$MYO_SIM_DIR"
          fi
        '';

        # ── Flake apps ──

        serverScript = mkAppScript "melosim-server" ''
          ${setupPrefix}

          # Parse args
          PORT="''${MELSIM_PORT:-3000}"
          MESH_DIR="''${MELSIM_MESH_DIR:-tests/fixtures/myo_sim/meshes}"
          while [[ $# -gt 0 ]]; do
            case "$1" in
              --port|-p) PORT="$2"; shift 2 ;;
              --mesh-dir|-m) MESH_DIR="$2"; shift 2 ;;
              --help|-h)
                echo "Usage: melosim-server [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  -p, --port PORT        Server port (default: 3000 or \$MELSIM_PORT)"
                echo "  -m, --mesh-dir DIR     Mesh directory (default: tests/fixtures/myo_sim/meshes or \$MELSIM_MESH_DIR)"
                echo "  -h, --help             Show this help"
                exit 0
                ;;
              *) echo "Unknown option: $1 (try --help)"; exit 1 ;;
            esac
          done

          STATIC_DIR="frontend/dist"
          export PORT MESH_DIR STATIC_DIR

          # Build frontend if stale
          if [ -d "frontend" ]; then
            if [ ! -d "frontend/dist" ] || [ "frontend/src" -nt "frontend/dist" ]; then
              echo "Building frontend..."
              (cd frontend && npm install --silent && npm run build)
            fi
          fi

          echo "Starting melosim-server on port $PORT"
          echo "  Mesh directory: $MESH_DIR"
          echo "  Static files:   $STATIC_DIR"
          echo "  Open http://localhost:$PORT"
          echo ""

          exec cargo run -p melosim-server --release
        '';

        frontendDevScript = mkAppScript "melosim-frontend-dev" ''
          cd "$PWD/frontend"
          if [ ! -d "node_modules" ]; then
            npm install
          fi

          # Parse args
          VITE_PORT="5173"
          API_PORT="3000"
          while [[ $# -gt 0 ]]; do
            case "$1" in
              --port|-p) VITE_PORT="$2"; shift 2 ;;
              --api-port|-a) API_PORT="$2"; shift 2 ;;
              --help|-h)
                echo "Usage: melosim-frontend-dev [OPTIONS]"
                echo ""
                echo "Options:"
                echo "  -p, --port PORT       Vite dev server port (default: 5173)"
                echo "  -a, --api-port PORT   API server port (default: 3000)"
                echo "  -h, --help            Show this help"
                exit 0
                ;;
              *) echo "Unknown option: $1 (try --help)"; exit 1 ;;
            esac
          done

          # Vite proxy forwards /scene, /import, etc. to the API server
          export VITE_API_BASE=""
          echo "Vite dev server on :$VITE_PORT (API proxied to :$API_PORT)"
          exec npm run dev -- --port "$VITE_PORT"
        '';

        buildFrontendScript = mkAppScript "melosim-build-frontend" ''
          cd "$PWD/frontend"
          if [ ! -d "node_modules" ]; then
            npm install --silent
          fi
          exec npm run build
        '';

        cargoBuildScript = mkAppScript "melosim-build" ''
          ${setupPrefix}
          echo "Building melosim workspace..."
          exec cargo build "''${@}"
        '';

        cargoTestScript = mkAppScript "melosim-test" ''
          ${setupPrefix}
          echo "Running tests..."
          exec cargo test "''${@}"
        '';
      in {
        packages.default = rustToolchain;

        # ── Apps: `nix develop --command melosim-server` etc. ──
        # These are shell scripts that rely on the dev shell's PATH.
        # They're added to buildInputs below so they're available
        # inside `nix develop`.

        devShells.default = pkgs.mkShell {
          buildInputs = [
            rustToolchain
            pkgs.maturin
            pythonEnv
            pkgs.openssl
            pkgs.libGL pkgs.libGLU pkgs.libX11 pkgs.libXi
            pkgs.libXmu pkgs.libXt pkgs.freetype pkgs.fontconfig
            pkgs.libxcursor pkgs.libxrandr pkgs.libxinerama
            # Frontend
            pkgs.nodejs_22
            # Flake apps (so they're available inside nix develop too)
            serverScript frontendDevScript buildFrontendScript
            cargoBuildScript cargoTestScript
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

            # ── Frontend setup ──
            if [ -d "$PWD/frontend" ] && [ ! -d "$PWD/frontend/node_modules" ]; then
              echo "  Installing frontend dependencies..."
              cd "$PWD/frontend" && npm install && cd "$PWD"
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

            echo ""
            echo "  Commands (inside nix develop):"
            echo "    melosim-server [-p PORT] [-m MESH_DIR]  Start the server + frontend"
            echo "    melosim-frontend-dev [PORT]             Start Vite dev server"
            echo "    melosim-build-frontend                  Build frontend to dist/"
            echo "    melosim-build                           cargo build"
            echo "    melosim-test                            cargo test"
            echo "    roundtrip <input.osim>                  Roundtrip via OpenSim"
            echo ""
            echo "  Or via nix develop --command:"
            echo "    nix develop --command melosim-server -- -p 4000"
            echo ""
            echo "  python: $(python --version 2>/dev/null || echo 'n/a')"
            echo "  node: $(node --version 2>/dev/null || echo 'n/a')"
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
              nodejs_22
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
