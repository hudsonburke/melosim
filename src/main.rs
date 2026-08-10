use bevy::prelude::*;
use melosim::{editor::EditorPlugin, render::RenderPlugin, sim::SimPlugin};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins((SimPlugin, RenderPlugin, EditorPlugin))
        .run();
}
