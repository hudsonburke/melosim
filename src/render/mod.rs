mod sync;
mod visualization;

use bevy::prelude::*;

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(PostUpdate, sync::sync_simulation_to_transform);
    }
}
