//! Minimal example: build the MyoArm model and run the editor on it.
//!
//! The model itself is defined declaratively in
//! [`melosim::model::myoarm_skeleton`] (Bevy Scene Notation). This example just
//! shows the pattern — construct a model and drop it into the editor — the same
//! path a real importer/model file would follow.

use bevy::prelude::*;
use melosim::{editor::EditorPlugin, model::myoarm_skeleton, render::RenderPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EditorPlugin)
        .add_plugins(RenderPlugin)
        .add_systems(Startup, myoarm_skeleton.spawn())
        .run();
}
