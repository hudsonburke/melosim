use bevy::prelude::*;
use melosim::{editor::EditorPlugin, model::myoarm_skeleton, render::RenderPlugin};

/// The editor app. The default debugging surface: it loads the MyoArm model
/// (via `myoarm_skeleton`) into a Bevy-native editor shell with consistent
/// render-module visualization, selection, inspection/editing, and a transform
/// gizmo.
fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EditorPlugin)
        .add_plugins(RenderPlugin)
        // Load the canonical demo model so the editor is immediately useful.
        .add_systems(Startup, myoarm_skeleton.spawn())
        .run();
}
