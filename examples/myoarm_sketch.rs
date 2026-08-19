//! Runs the melosim editor, which loads models from the top-level `models/`
//! directory (selected via the toolbar "Model" menu). The canonical demo model
//! is MyOArm, defined in `models/myoarm.rs`.

use bevy::prelude::*;
use melosim::{editor::EditorPlugin, render::RenderPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EditorPlugin)
        .add_plugins(RenderPlugin)
        .run();
}
