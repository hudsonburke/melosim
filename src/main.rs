use bevy::prelude::*;
use melosim::{editor::EditorPlugin, render::RenderPlugin};

/// The editor app. It owns a model registry (via the editor plugin) and loads
/// a model at startup (default: MyoArm) or from the UI — so the editor itself
/// is the debugging surface, with consistent render-module visualization.
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EditorPlugin)
        .add_plugins(RenderPlugin)
        .run();
}
