{
  description = "melosim — Rust neutral data model for musculoskeletal simulation (Bevy editor app)";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      systems = [ "x86_64-linux" "aarch64-linux" ];
      forAllSystems = nixpkgs.lib.genAttrs systems;
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
          # Native libs Bevy needs for windowing (x11/wayland) and wgpu (GL/Vulkan).
          # Used as buildInputs so pkg-config can find their .pc files, and put on
          # LD_LIBRARY_PATH so link/runtime resolve them.
          nativeLibs = with pkgs; [
            alsa-lib
            libxkbcommon
            wayland
            wayland-protocols
            libxcb
            libGL
            libglvnd
            vulkan-loader
            udev
            libx11
            libxcursor
            libxrandr
            libxi
            # GTK3 — `rfd` (native file dialog) links against gtk+-3.0. Its
            # transitive deps (glib, gdk-pixbuf, cairo, pango, ...) come along.
            gtk3
            webkitgtk_4_1
            libsoup_3
            openssl
            librsvg
          ];
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              # Rust toolchain
              cargo
              rustc
              rust-analyzer
              # C toolchain — Bevy build scripts need a C compiler + pkg-config
              gcc
              binutils
              pkg-config
              cmake
              nodejs_22
              patchelf
            ];

            # Make native deps visible to pkg-config and the dynamic loader.
            buildInputs = nativeLibs;
            PKG_CONFIG_PATH =
              (pkgs.lib.makeSearchPath "lib/pkgconfig" nativeLibs)
              + ":" + (pkgs.lib.makeSearchPath "share/pkgconfig" nativeLibs);
            LD_LIBRARY_PATH = pkgs.lib.makeLibraryPath nativeLibs;

            shellHook = ''
              echo "melosim dev shell: cargo/rustc ready (Bevy native deps loaded)."
              echo "  - cargo check                    : compile check"
              echo "  - cargo run                      : run the editor app"
              echo "  - cargo run --example myoarm_sketch : run the myoarm example"
            '';
          };
        });
    };
}
