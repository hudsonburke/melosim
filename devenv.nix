{
  pkgs,
  lib,
  config,
  ...
}:

let
  # Native libs Tauri needs for WebKitGTK rendering on Linux.
  tauriNativeLibs = with pkgs; [
    webkitgtk_4_1
    gtk3
    glib
    gdk-pixbuf
    cairo
    pango
    harfbuzz
    atk
    libxkbcommon
    xdotool
    openssl
    librsvg
    # X11 libs (GTK backend)
    xorg.libX11
    xorg.libXcursor
    xorg.libXrandr
    xorg.libXi
    xorg.libXext
    xorg.libXfixes
    # Wayland libs (GTK backend)
    wayland
    wayland-protocols
    # GL/Vulkan (WebKitGTK rendering)
    libGL
    libglvnd
    mesa
    vulkan-loader
    # Misc
    udev
    libsoup_3
  ];

  # MuJoCo shared libs (auto-downloaded by mujoco-rs into .mujoco/).
  mujocoLibDir = "${config.env.DEVENV_ROOT}/.mujoco/mujoco-3.9.0/lib";
in
{
  # ── Rust ──────────────────────────────────────────────────────────
  languages.rust = {
    enable = true;
    channel = "nightly";
  };

  # ── Node (for the R3F UI) ────────────────────────────────────────
  languages.javascript = {
    enable = true;
    package = pkgs.nodejs_22;
  };

  # ── Build tools ──────────────────────────────────────────────────
  packages =
    with pkgs;
    [
      cargo-tauri
      pkg-config
      cmake
      patchelf
      curl
      wget
      file
    ]
    ++ tauriNativeLibs;

  env = {
    MUJOCO_DOWNLOAD_DIR = "${config.env.DEVENV_ROOT}/.mujoco";

    # Opensim feature: PyO3 needs a system-python venv (not a nix-python venv,
    # whose libpython can't embed into a nix-glibc binary).MUJOCO_DOWNLOAD_DIR
    PYO3_PYTHON = "${config.env.DEVENV_ROOT}/.venv-sys/bin/python";
    PYTHONPATH = "${config.env.DEVENV_ROOT}/.venv-sys/lib/python3.12/site-packages";

    # All runtime native libs in one place — cargo, tauri.mjs, and
    # direct cargo run all inherit this.
    LD_LIBRARY_PATH = lib.concatStringsSep ":" [
      (lib.makeLibraryPath tauriNativeLibs)
      mujocoLibDir
    ];

    PKG_CONFIG_PATH = lib.concatStringsSep ":" (
      lib.concatMap (pkg: [
        "${pkg}/lib/pkgconfig"
        "${pkg}/share/pkgconfig"
      ]) tauriNativeLibs
    );
  };

  # ── Scripts ──────────────────────────────────────────────────────
  scripts = {
    check.exec = "cargo check --features web-editor";
    test.exec = "cargo test --features web-editor";
    clippy.exec = "cargo clippy --features web-editor";

    dev.exec = ''exec node "$DEVENV_ROOT/ui/scripts/dev.mjs"'';
    dev-backend.exec = ''cd "$DEVENV_ROOT" && exec cargo run --features web-editor --bin web-editor'';
    dev-frontend.exec = ''cd "$DEVENV_ROOT/ui" && exec node node_modules/vite/bin/vite.js'';

    build-ui.exec = "cd ui && npm run build";
    typecheck-ui.exec = "cd ui && npm run typecheck";

    tauri.exec = ''exec node "$DEVENV_ROOT/ui/scripts/tauri.mjs" dev "$@"'';
  };

  enterShell = ''
    # Add rustc sysroot libs (for dynamic_linking / bevy_dylib builds).
    export LD_LIBRARY_PATH="$(rustc --print sysroot)/lib/rustlib/x86_64-unknown-linux-gnu/lib:$LD_LIBRARY_PATH"

    echo "melosim dev shell"
    echo "  check         cargo check --features web-editor"
    echo "  test          cargo test --features web-editor"
    echo "  clippy        cargo clippy --features web-editor"
    echo "  dev           run backend + frontend together"
    echo "  dev-backend   Rust HTTP backend on :7421"
    echo "  dev-frontend  Vite dev server on :5173"
    echo "  tauri         Tauri desktop app"
    echo "  build-ui      production UI build"
  '';
}
