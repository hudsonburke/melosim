pub mod visualize;

use bevy::prelude::*;

use crate::model::sync_kinematics;
use visualize::VisualizationSettings;

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<VisualizationSettings>()
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
