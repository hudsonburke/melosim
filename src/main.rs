use bevy::prelude::*;
use melosim::editor::EditorPlugin;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EditorPlugin)
        .run();
}
