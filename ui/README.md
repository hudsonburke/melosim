# Tauri + Three.js UI prototype

This is an interaction prototype, not a replacement editor. Run `npm install && npm run dev` for the browser viewport, or `npm run tauri dev` after installing the Tauri CLI. The frontend requests a typed `model_snapshot` from Rust and renders/selects body placeholders. The next boundary is replacing the demo command with a serialized snapshot from the Bevy ECS world.
