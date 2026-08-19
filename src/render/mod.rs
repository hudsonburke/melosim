pub mod visualize;

use bevy::prelude::*;
use bevy::transform::StaticTransformOptimizations;

use crate::model::sync_kinematics;

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        // The model is fully dynamic: `sync_kinematics` rewrites joint Transforms
        // every frame, so Bevy's static-scene transform optimization (which skips
        // propagating "unchanged" subtrees) suppresses the FK from reaching child
        // meshes. Force full propagation every frame.
        app.insert_resource(StaticTransformOptimizations::Disabled)
            .add_systems(
                PreUpdate,
                crate::model::ensure_coordinate_states,
            )
            .add_systems(
                PostUpdate,
                sync_kinematics.before(bevy::transform::TransformSystems::Propagate),
            )
            .add_systems(
                Update,
                (
                    visualize::draw_body_gizmos,
                    visualize::draw_frames,
                    visualize::draw_muscle_paths,
                    visualize::draw_joint_axes,
                ),
            );
    }
}
