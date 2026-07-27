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
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs { inherit system overlays; };
        rustToolchain = pkgs.rust-bin.stable.latest.default.override {
          extensions = [ "rust-src" "rust-analyzer" "clippy" "rustfmt" ];
        };

        # Python with OpenSim available via pip.
        # OpenSim PyPI wheel provides the native library on x86_64-linux.
        # Must pin to Python 3.13 — OpenSim wheels only support up to 3.13
        # and PyO3 0.23 doesn't support 3.14 either.
        pythonEnv = (pkgs.python313.withPackages (ps: with ps; [
          pip
          setuptools
          wheel
          numpy
        ]));
      in {
        packages.default = rustToolchain;

        devShells.default = pkgs.mkShell {
          buildInputs = [
            rustToolchain
            pkgs.maturin
            pythonEnv
            pkgs.openssl
            pkgs.libGL
            pkgs.libGLU
            pkgs.libX11
            pkgs.libXi
            pkgs.libXmu
            pkgs.libXt
            pkgs.freetype
            pkgs.fontconfig
            pkgs.libxcursor
            pkgs.libxrandr
            pkgs.libxinerama
          ];

          nativeBuildInputs = [
            pkgs.pkg-config
            pkgs.cmake
          ];

          shellHook = ''
            echo "melosim dev shell"
            echo "  rust: $(rustc --version)"
            echo "  cargo: $(cargo --version)"

            # Create venv if it doesn't exist, install opensim
            if [ ! -d .venv ]; then
              echo "  Setting up Python venv with OpenSim..."
              python3 -m venv .venv
              .venv/bin/pip install --quiet opensim
              echo "  OpenSim installed."
            fi
            source .venv/bin/activate
            echo "  python: $(python --version)"
            echo "  opensim: $(python -c 'import opensim; print(opensim.__version__)' 2>/dev/null || echo 'not loaded')"
          '';
        };

        # OCI container with Rust + Python + OpenSim pre-installed.
        # Can be used for CI or by users who want to run the importer
        # without setting up the full Nix environment.
        packages.opensim-container = pkgs.dockerTools.buildImage {
          name = "melosim";
          tag = "latest";
          copyToRoot = pkgs.buildEnv {
            name = "image-root";
            paths = with pkgs; [
              bashInteractive
              coreutils-full
              gnutar
              gzip
              git
              gcc
              cmake
              pkg-config
              openssl
              libGL
              libGLU
              libX11
              libXi
              libXmu
              libXt
              pkgs.freetype
              pkgs.fontconfig
              pkgs.libxcursor
              pkgs.libxrandr
              pkgs.libxinerama
              pythonEnv
              (rustToolchain.override {
                extensions = [ "rust-src" "clippy" ];
              })
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
            ];
          };
        };
      });
}
