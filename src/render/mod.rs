pub mod visualize;

use bevy::prelude::*;
use bevy::transform::StaticTransformOptimizations;

use crate::model::{ensure_coordinate_states, sync_kinematics};

/// Per-overlay toggles for the model visualization gizmos (driven by the
/// editor's "View" menu).
#[derive(Resource)]
pub struct RenderSettings {
    pub frames: bool,
    pub sites: bool,
    pub muscles: bool,
}

impl Default for RenderSettings {
    fn default() -> Self {
        Self {
            frames: true,
            sites: true,
            muscles: true,
        }
    }
}

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        // The model is fully dynamic: `sync_kinematics` rewrites joint Transforms
        // every frame, so Bevy's static-scene transform optimization (which skips
        // propagating "unchanged" subtrees) suppresses the FK from reaching child
        // meshes. Force full propagation every frame.
        app.insert_resource(StaticTransformOptimizations::Disabled)
            .init_resource::<RenderSettings>()
            .add_systems(
                PostUpdate,
                (
                    ensure_coordinate_states,
                    sync_kinematics,
                )
                    .chain()
                    .before(bevy::transform::TransformSystems::Propagate),
            )
            .add_systems(
                Update,
                (
                    visualize::draw_frames,
                    visualize::draw_sites,
                    visualize::draw_muscle_paths,
                ),
            );
    }
}
