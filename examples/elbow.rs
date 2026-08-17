//! Run: `cargo run --example elbow`

use bevy::prelude::*;
use melosim::{editor::EditorPlugin, model::*, render::RenderPlugin};

fn myoarm() -> impl SceneList {
    bsn_list![
        // ── Skeleton ──
        (
            #humerus Body
            InertialProperties {
                mass: 8.09341,
                mass_center: nalgebra::Vector3::new(-0.02118, -0.128888, -9.98087e-05),
                inertia: Inertia::from_diagonal(nalgebra::Vector3::new(0.0764928, 0.0573742, 0.0471)),
            }
            Children [
                (#humerus_marker Site Transform::from_xyz(-0.05613, 0.05528, -0.00151)),
                (#distal Frame Transform::from_xyz(-0.05613, 0.05528, -0.00151)),
            ]
        ),
        (
            #elbow Joint
            ChildOf(#distal)
            JointCoordinates [
                (
                    #elbow_rot Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.0494, 0.0366, 0.998108)}
                ),
                (
                    #elbow_slide Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{linear: nalgebra::Vector3::new(0.1, 0.0, 0.0)}
                ),
            ]
        ),
        (
            #ulna Body
            ChildOf(#elbow)
        ),
        // ── Muscle sites (children of bodies, for spatial hierarchy) ──
        (#bicep_origin Site ChildOf(#humerus) Transform::from_xyz(0.0, 0.05, 0.0)),
        (#bicep_via Site ChildOf(#humerus) Transform::from_xyz(0.0, 0.05, 0.0)),
        (#bicep_insertion Site ChildOf(#ulna) Transform::from_xyz(0.0, 0.05, 0.0)),
        // ── Muscles (references sites by name) ──
        (
            #bicep Muscle
            HillTypeMuscleParams::default()
            OriginSite(#bicep_origin)
            InsertionSite(#bicep_insertion)
            ViaSites [#bicep_via]
        ),
    ]
}

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(EditorPlugin)
        .add_plugins(RenderPlugin)
        .add_systems(Startup, myoarm.spawn())
        .run();
}
