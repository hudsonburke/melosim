pub mod stl;
pub mod sync;

use bevy::prelude::*;
use stl::StlPlugin;

pub struct RenderPlugin;

impl Plugin for RenderPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins(StlPlugin).add_systems(
            PostUpdate,
            (sync::sync_fixed_frames, sync::sync_kinematics)
                .chain()
                .before(bevy::transform::TransformSystems::Propagate),
        );
    }
}
