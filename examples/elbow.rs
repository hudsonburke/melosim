//! Minimal skeleton visualization: humerus → forearm with a hinge elbow,
//! authored entirely in Bevy Scene Notation (BSN).
//!
//! Run: `cargo run --example elbow`
//!
//! Press Left/Right arrows to flex/extend the elbow.
//! Loads bone meshes from the myo_sim test fixtures.

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
    // queue_spawn_scene waits for the STL dependencies to load before spawning.
    commands.queue_spawn_scene(elbow_model());
}

/// The whole model is ONE `bsn!` invocation = one name scope, so axes can
/// reference their coordinates (and future muscles/constraints can reference
/// both) with `#Name`.
///
/// The pattern — "create bodies, then link them":
///   * bodies come from the `bone(...)` scene; its entries merge into the
///     enclosing entity, so `InertialProperties` and `Children` patches
///     written after it apply to the body;
///   * a joint is written inline as a child of its parent body, nesting
///     its coordinates, its axes (Twist + DrivesCoordinate), and the child
///     body's own `bone(...)` scene — indentation mirrors the kinematic tree.
fn elbow_model() -> impl Scene {
    bsn! {
        bone("humerus", "myo_sim/meshes/arm_r_humerus.stl")
        InertialProperties {
            mass: 1.865,
            mass_center: Vector3::new(0.0, -0.1805, 0.0),
            inertia: Inertia::new(0.01481, 0.00455, 0.01319, 0.0, 0.0, 0.0),
        }
        Children [
            (
                #elbow Joint
                Children [
                    (
                        #r_elbow_flex
                        Coordinate { range: {(0.0, 2.269)} }
                        CoordinateState { value: 0.5 }
                    ),
                    rotation_axis(#r_elbow_flex, Vector3::new(0.0494, 0.0366, 0.998108)),
                    (
                        bone("forearm", "myo_sim/meshes/arm_r_ulna.stl")
                        InertialProperties {
                            mass: 1.534,
                            mass_center: Vector3::new(0.0, -0.1815, 0.0),
                            inertia: Inertia::new(0.01928, 0.00157, 0.02006, 0.0, 0.0, 0.0),
                        }
                    ),
                ]
            )
        ]
    }
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
