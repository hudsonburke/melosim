# React Three Fiber editor

React owns the panels; React Three Fiber and Drei own the viewport. Bevy ECS remains the model authority. Both transports use the same serialized Rust commands, with compiled MuJoCo mesh vertices (no glTF conversion).

Enter `devenv shell`, then run `dev` (or `npm run dev:all` from this directory) to start the backend and Vite together. Run `npm ci` here first when dependencies change. Open http://127.0.0.1:5173 and choose **Load MyoArm**. Browser file paths are paths on the machine running the Rust backend. The backend listens on loopback port 7421 only; Vite proxies `/api`.

For the desktop app use `tauri` in the devenv shell or `npm run tauri dev` here. The devenv environment provides WebKitGTK 4.1 and GTK3 development libraries. Desktop mode embeds the Rust worker and provides native file dialogs; it does not need the HTTP backend. Use either browser development or desktop development, not both: both own Vite port 5173.

These launchers supervise their own process groups. Ctrl+C, SIGTERM, terminal hangup, or a child exiting stops the remaining children and grandchildren, with a two-second grace period before forced cleanup. Closing the native app ends its development session and stops Vite. Closing a browser tab does **not** stop `dev`: stop its terminal process separately. Avoid `cargo tauri dev` directly when you want this supervisor's lifecycle guarantees. SIGKILL or a host crash cannot run cleanup handlers.

For an old leftover listener, inspect `ss -ltnp '( sport = :5173 or sport = :7421 )'`, verify the PID and command, then stop that specific process. Do not blindly kill everything using a port. Process cleanup regressions are covered by `npm run test:processes` (Linux/macOS).

Supported: MJCF import, hierarchy and mesh selection, coordinate posing, local transforms/gizmos, fixed box bodies, native STL/OBJ parts, new joints, cable creation, surface-picked or numeric sites, ordered cable paths, validation, and MJCF export with mesh assets. Undo/redo covers property and path edits; structural edits clear history. Joint insertion refuses to replace existing articulation. Export refuses to overwrite an existing destination.

**Save project as** creates a portable folder (for example `arm.melosim`) containing `project.json`, the source MJCF, captured assets, and a versioned edit log. **Open project** accepts that folder or its manifest. Replaying edits restores colors, cable parameters, structural edits and available undo/redo history. Saves are staged then renamed, and never overwrite an existing folder. Keep the whole folder together; MJCF export is not a replacement for saving a project.

MJCF export preserves imported materials, textures, primitive geometry, lights, cameras and source-only scene elements, alongside edited mesh colors and opacity. Assets are bundled with the XML. The viewport currently previews solid mesh colors, not the complete MuJoCo rendering pipeline; primitive geometry is retained for export but is not yet editable/rendered in the viewport.

The bundled MyoArm source references a missing tendon site (`PECM2_PECM2-P3_r`). Project saves retain this invalid source and report its compile error. Normal export refuses it; explicitly check **Visual-only** in the export dialog to omit tendons, actuators, constraints, contacts, sensors and keyframes while preserving appearance. This does not repair the source dynamics.

This remains an editor prototype, not a simulation runner. Original simulation sections are retained on valid imports, but edits are not guaranteed to preserve dynamics and new cable parameters are not all exported. Arbitrary MuJoCo features and structural edits need further coverage before claiming universally lossless simulation round-trips. Validate exported files in MuJoCo before simulation.

Checks: `npm run build`, `npm run test:processes`; from the repository root in the devenv shell, `cargo test --features web-editor`. Appearance tests compile exports in MuJoCo and compare geometry, materials and textures, including portable external texture packaging.
