pub mod mesh_loaders;
pub mod visualize;

use bevy::prelude::*;
use bevy::transform::StaticTransformOptimizations;

use crate::model::sync_kinematics;

/// Per-overlay toggles for the model visualization gizmos (driven by the
/// editor's "View" menu).
#[derive(Resource)]
pub struct RenderSettings {
    pub bodies: bool,
    pub frames: bool,
    pub sites: bool,
    pub muscles: bool,
    pub joint_axes: bool,
    pub meshes: bool,
}

impl Default for RenderSettings {
    fn default() -> Self {
        Self {
            bodies: true,
            frames: true,
            sites: true,
            muscles: true,
            joint_axes: true,
            meshes: true,
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
            // Bevy bundles a glTF loader but not OBJ/STL — add them:
            //   OBJ → `bevy_obj` (0.19); STL → our custom loader (`bevy_stl` is only at 0.18).
            .add_plugins(bevy_obj::ObjPlugin)
            .init_asset_loader::<mesh_loaders::StlLoader>()
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
                    visualize::draw_bodies,
                    visualize::draw_frames,
                    visualize::draw_sites,
                    visualize::draw_muscle_paths,
                    visualize::draw_joint_axes,
                    visualize::sync_mesh_visibility,
                ),
            );
    }
}
