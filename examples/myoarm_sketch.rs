//! MyoArm model sketch — bodies with meshes, sites, joints, and muscle paths.
//!
//! ✅ = supported    ⚠️ = needs extension    ❌ = not yet supported

use bevy::prelude::*;
use melosim::{editor::EditorPlugin, model::*, render::RenderPlugin};

fn myoarm() -> impl SceneList {
    bsn_list![
        // ══════════════════════════════════════════════════════════════
        // SKELETON — bodies, joints, sites
        // ══════════════════════════════════════════════════════════════

        // ✅ Clavicle
        (
            #clavicle Body
            InertialProperties {
                mass: 0.156,
                mass_center: nalgebra::Vector3::new(-0.011, 0.006, 0.054),
                inertia: Inertia::new(0.001, 0.001, 0.001, 0.0, 0.0, 0.0),
            }
            Children [
                (WorldAssetRoot("gltf/clavicle.glb#Scene0") Transform::IDENTITY),
                (#DELT1_P4 Site Transform::from_xyz(-0.014, 0.011, 0.080)),
                (#PECM1_P3 Site Transform::from_xyz(0.026, 0.004, 0.057)),
            ]
        ),
        (
            #sternoclavicular Joint
            ChildOf(#clavicle)
            JointCoordinates [
                (#sternoclavicular_r2 Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.015, 0.989, -0.145)}),
                (#sternoclavicular_r3 Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(-0.994, 0.0, -0.105)}),
            ]
        ),

        // ✅ Scapula
        (
            #scapula Body
            ChildOf(#sternoclavicular)
            InertialProperties {
                mass: 0.704,
                mass_center: nalgebra::Vector3::new(-0.055, -0.035, -0.044),
                inertia: Inertia::new(0.002, 0.001, 0.001, 0.0, 0.0, 0.0),
            }
            Children [
                (WorldAssetRoot("gltf/scapula.glb#Scene0") Transform::IDENTITY),
                (#DELT1_P3 Site Transform::from_xyz(0.043, -0.032, 0.005)),
                (#DELT2_P3 Site Transform::from_xyz(0.00005, 0.003, 0.022)),
                // ⚠️ Wrapping surfaces (need shape-specific components)
                // ❌ Sidesites (wrapping direction)
            ]
        ),
        (
            #acromioclavicular Joint
            ChildOf(#scapula)
            JointCoordinates [
                (#acromioclavicular_r1 Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.638, 0.119, 0.761)}),
                (#acromioclavicular_r2 Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.157, 0.947, -0.279)}),
                (#acromioclavicular_r3 Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(-0.754, 0.298, 0.585)}),
            ]
        ),

        // ✅ Humerus
        (
            #humerus Body
            ChildOf(#acromioclavicular)
            InertialProperties {
                mass: 1.998,
                mass_center: nalgebra::Vector3::new(0.018, -0.140, -0.013),
                inertia: Inertia::new(0.013, 0.012, 0.002, 0.0, 0.0, 0.0),
            }
            Children [
                (WorldAssetRoot("gltf/humerus.glb#Scene0") Transform::IDENTITY),
                (#DELT1_P1 Site Transform::from_xyz(0.025, -0.045, 0.0)),
                (#BIClong_P1 Site Transform::from_xyz(-0.01, -0.12, -0.015)),
                (#TRIlong_P1 Site Transform::from_xyz(0.005, -0.10, 0.01)),
            ]
        ),
        (
            #shoulder Joint
            ChildOf(#humerus)
            JointCoordinates [
                (#shoulder_elv Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(-0.998, 0.002, 0.059)}),
                (#shoulder_rot Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.005, 0.999, 0.042)}),
            ]
        ),

        // ✅ Ulna
        (
            #ulna Body
            ChildOf(#shoulder)
            InertialProperties {
                mass: 0.118,
                mass_center: nalgebra::Vector3::new(0.002, -0.133, 0.008),
                inertia: Inertia::new(0.0005, 0.0005, 0.00003, 0.0, 0.0, 0.0),
            }
            Children [
                (WorldAssetRoot("gltf/ulna.glb#Scene0") Transform::IDENTITY),
                (#BIClong_P8 Site Transform::from_xyz(-0.012, -0.28, -0.01)),
                (#TRIlong_P5 Site Transform::from_xyz(0.008, -0.26, 0.015)),
            ]
        ),
        (
            #elbow Joint
            ChildOf(#ulna)
            JointCoordinates [
                (#elbow_flex Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.005, 0.999, 0.042)}),
            ]
        ),

        // ══════════════════════════════════════════════════════════════
        // MUSCLES — ordered paths
        // ══════════════════════════════════════════════════════════════

        // ✅ Deltoid 1 — simple path
        (
            #DELT1 Muscle
            HillTypeMuscleParams::default()
            PathEntities [
                (#DELT1_P1 Site ChildOf(#humerus) Transform::from_xyz(0.025, -0.045, 0.0)),
                (#DELT1_P3 Site ChildOf(#scapula) Transform::from_xyz(0.043, -0.032, 0.005)),
                (#DELT1_P4 Site ChildOf(#clavicle) Transform::from_xyz(-0.014, 0.011, 0.080)),
            ]
        ),

        // ⚠️ Biceps long — needs ellipsoid wrapping
        (
            #BIClong Muscle
            HillTypeMuscleParams::default()
            PathEntities [
                (#BIClong_P1 Site ChildOf(#humerus) Transform::from_xyz(-0.01, -0.12, -0.015)),
                (#BIClong_P2 Site ChildOf(#humerus) Transform::from_xyz(-0.008, -0.15, -0.012)),
                // ❌ Ellipsoid wrapping needed
                (#BIClong_P3 Site ChildOf(#humerus) Transform::from_xyz(-0.005, -0.20, -0.008)),
                (#BIClong_P8 Site ChildOf(#ulna) Transform::from_xyz(-0.012, -0.28, -0.01)),
                // ❌ Elbow wrapping needed
                (#BIClong_P10 Site ChildOf(#ulna) Transform::from_xyz(-0.01, -0.30, -0.005)),
                (#BIClong_P11 Site ChildOf(#ulna) Transform::from_xyz(-0.008, -0.32, 0.0)),
            ]
        ),

        // ⚠️ Triceps long — needs cylinder wrapping
        (
            #TRIlong Muscle
            HillTypeMuscleParams::default()
            PathEntities [
                (#TRIlong_P1 Site ChildOf(#humerus) Transform::from_xyz(0.005, -0.10, 0.01)),
                (#TRIlong_P2 Site ChildOf(#humerus) Transform::from_xyz(0.008, -0.14, 0.012)),
                // ❌ Cylinder wrapping needed
                (#TRIlong_P5 Site ChildOf(#ulna) Transform::from_xyz(0.008, -0.26, 0.015)),
            ]
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
