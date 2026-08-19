pub mod visualize;

use bevy::prelude::*;

use crate::model::sync_kinematics;

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(
            PreUpdate,
            crate::model::ensure_coordinate_states,
        )
        .add_systems(
            PostUpdate,
            sync_kinematics.before(bevy::transform::TransformSystems::Propagate),
        )
        .add_systems(
            Update,
            (visualize::draw_muscle_paths, visualize::draw_joint_axes),
        );
    }
}
