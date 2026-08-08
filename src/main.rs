use bevy_ecs::prelude::*;

pub struct MelosimPlugin;

impl Plugin for MelosimPlugin {
    fn build(&self, app: &mut App) {
        // Add your systems and resources here
    }
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugin(MelosimPlugin)
        .run();
}
