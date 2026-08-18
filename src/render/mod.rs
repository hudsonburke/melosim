pub mod sync;
pub mod visualize;

use bevy::prelude::*;

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PostUpdate,
            sync::sync_kinematics.before(bevy::transform::TransformSystems::Propagate),
        )
        .add_systems(Update, visualize::visualize_model);
    }
}
