//! Flat-style elbow: the same model as `elbow.rs`, authored for scale.
//!
//! Topology is a FLAT sibling list in one `bsn_list!` — bodies, joints and
//! coordinates are roots, parented to each other by name via
//! `ChildOf(#name)`. Only a joint's own contents (its coordinates and axes)
//! nest one level under the joint. Depth never exceeds 2, no matter how
//! long the kinematic chain grows; forward references (`ChildOf(#elbow)`
//! before `#elbow` is declared) resolve at spawn time.
//!
//! Compare `elbow.rs` (nested style — fine for small models/teaching).
//!
//! Run: `cargo run --example elbow_parallel`
//!
//! Press Left/Right arrows to flex/extend the elbow.

use bevy::prelude::*;
use melosim::{
    editor::EditorPlugin,
    model::{bone, rotation_axis, Coordinate, CoordinateState, Inertia, InertialProperties, Joint},
    render::RenderPlugin,
};
use nalgebra::Vector3;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(AssetPlugin {
            file_path: "tests/fixtures".into(),
            ..default()
        }))
        .add_plugins((RenderPlugin, EditorPlugin))
        .add_systems(Startup, spawn_model)
        .add_systems(Update, control_elbow)
        .run();
}

fn spawn_model(mut commands: Commands) {
    // queue_spawn_scene_list waits for the STL dependencies to load.
    commands.queue_spawn_scene_list(elbow_model());
}

fn elbow_model() -> impl SceneList {
    bsn_list![
        (
            #humerus
            bone("humerus", "myo_sim/meshes/arm_r_humerus.stl")
            InertialProperties {
                mass: 1.865,
                mass_center: Vector3::new(0.0, -0.1805, 0.0),
                inertia: Inertia::new(0.01481, 0.00455, 0.01319, 0.0, 0.0, 0.0),
            }
        ),
        (
            #elbow Joint
            ChildOf(#humerus)
            // A joint's contents stay local: coordinates and axes nest one
            // level, so deleting the joint deletes them.
            Children [
                (
                    #r_elbow_flex
                    Coordinate { range: {(0.0, 2.269)} }
                    CoordinateState { value: 0.5 }
                ),
                rotation_axis(#r_elbow_flex, Vector3::new(0.0494, 0.0366, 0.998108)),
            ]
        ),
        (
            #forearm
            bone("forearm", "myo_sim/meshes/arm_r_ulna.stl")
            InertialProperties {
                mass: 1.534,
                mass_center: Vector3::new(0.0, -0.1815, 0.0),
                inertia: Inertia::new(0.01928, 0.00157, 0.02006, 0.0, 0.0, 0.0),
            }
            // Forward reference: #elbow is declared above, but order doesn't matter.
            ChildOf(#elbow)
        ),
    ]
}

fn control_elbow(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut coords: Query<(&Coordinate, &mut CoordinateState)>,
) {
    let dir = keys.pressed(KeyCode::ArrowRight) as i8 - keys.pressed(KeyCode::ArrowLeft) as i8;
    if dir == 0 {
        return;
    }
    for (coord, mut state) in &mut coords {
        state.value += f64::from(dir) * time.delta_secs_f64();
        if coord.clamped {
            state.value = state.value.clamp(coord.range.0, coord.range.1);
        }
    }
}
