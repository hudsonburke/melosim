use bevy::prelude::*;
use melosim::{editor::EditorPlugin, render::RenderPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EditorPlugin)
        .add_plugins(RenderPlugin)
        .run();
}
