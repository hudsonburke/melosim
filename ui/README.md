# React Three Fiber editor

React owns the panels; React Three Fiber and Drei own the viewport. Bevy ECS remains the model authority. Both transports use the same serialized Rust commands, with compiled MuJoCo mesh vertices (no glTF conversion).

From this directory run `npm ci`, then `npm run backend` and `npm run dev` in separate terminals. Open http://127.0.0.1:5173 and choose **Load MyoArm**. Browser file paths are paths on the machine running the Rust backend. The backend listens on loopback port 7421 only; Vite proxies `/api`.

For the desktop app use `npm run tauri dev`. Linux needs WebKitGTK 4.1 and GTK3 development libraries; the root `nix develop` environment provides these. Desktop mode embeds the Rust worker and provides native file dialogs; it does not need the HTTP backend.

Supported: MJCF import, hierarchy and mesh selection, coordinate posing, local transforms/gizmos, fixed box bodies, native STL/OBJ parts, new joints, cable creation, surface-picked or numeric sites, ordered cable paths, validation, and MJCF export with mesh assets. Undo/redo covers property and path edits; structural edits clear history. Joint insertion refuses to replace existing articulation. Export refuses to overwrite an existing destination.

This is still an editor prototype, not a simulation runner. Cable parameter editing is available in the ECS, but the current exporter does not serialize all cable material/actuator parameters or original muscle wrapping. Export is not a lossless round-trip of every MuJoCo feature. Save to a new path and validate in MuJoCo before simulation.

Checks: `npm run build`; from the repository root, `cargo test --features web-editor --lib web_editor::tests`.
