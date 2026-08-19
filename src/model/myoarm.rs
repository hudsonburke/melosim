//! Canonical MyoArm demo model, defined declaratively in Bevy Scene Notation
//! (BSN). This is the model the editor loads by default and that examples
//! reference — the pattern for "build a model, import it into the editor".
//!
//! Mesh coordinate convention (MuJoCo):
//! The GLB files are direct exports of the MuJoCo STL meshes, so their vertices
//! are already expressed in the MuJoCo body frame (origin at the joint, Z-up).
//! The only transform each mesh needs is the Z-up → Y-up rotation, baked into
//! the mesh node's local `Transform`.

use bevy::prelude::*;

use crate::model::*;

/// Build the MyoArm skeleton (bodies, meshes, joints, coordinates, muscle
/// paths) as a BSN scene.
pub fn myoarm_skeleton() -> impl SceneList {
    bsn_list![
        // Clavicle
        (
            #clavicle Body
            InertialProperties {
                mass: 0.156,
                mass_center: nalgebra::Vector3::new(-0.011, 0.006, 0.054),
                inertia: Inertia::new(0.001, 0.001, 0.001, 0.0, 0.0, 0.0),
            }
            Children [
                (#clavicle_mesh
                    Mesh3d("gltf/clavicle.glb#Mesh0/Primitive0")
                    MeshMaterial3d<StandardMaterial>(asset_value(StandardMaterial::from(
                        Color::srgb(0.8, 0.7, 0.6),
                    )))
                    Transform::from_rotation(Quat::from_xyzw(-0.707107, 0.0, 0.0, 0.707107))),
                (#PECM1_P3 Site Transform::from_xyz(0.026, 0.057, -0.004)),
                (#sternoclavicular_offset Frame Transform::from_xyz(-0.01433, 0.1355, -0.02007)),
            ]
        ),
        (
            #sternoclavicular Joint
            ChildOf(#sternoclavicular_offset)
            JointCoordinates [
                (#sternoclavicular_r2 Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.015, 0.989, -0.145)}),
                (#sternoclavicular_r3 Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(-0.994, 0.0, -0.105)}),
            ]
        ),

        // Scapula
        (
            #scapula Body
            ChildOf(#sternoclavicular)
            InertialProperties {
                mass: 0.704,
                mass_center: nalgebra::Vector3::new(-0.055, -0.035, -0.044),
                inertia: Inertia::new(0.002, 0.001, 0.001, 0.0, 0.0, 0.0),
            }
            Children [
                (#scapula_mesh
                    Mesh3d("gltf/scapula.glb#Mesh0/Primitive0")
                    MeshMaterial3d<StandardMaterial>(asset_value(StandardMaterial::from(
                        Color::srgb(0.8, 0.7, 0.6),
                    )))
                    Transform::from_rotation(Quat::from_xyzw(-0.707107, 0.0, 0.0, 0.707107))),
                (#DELT2_P3 Site Transform::from_xyz(0.00005, 0.022, -0.003)),
                (#acromioclavicular_offset Frame Transform::from_xyz(-0.00955, 0.009, 0.034)),
            ]
        ),
        (
            #acromioclavicular Joint
            ChildOf(#acromioclavicular_offset)
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

        // Humerus
        (
            #humerus Body
            ChildOf(#acromioclavicular)
            InertialProperties {
                mass: 1.998,
                mass_center: nalgebra::Vector3::new(0.018, -0.140, -0.013),
                inertia: Inertia::new(0.013, 0.012, 0.002, 0.0, 0.0, 0.0),
            }
            Children [
                (#humerus_mesh
                    Mesh3d("gltf/humerus.glb#Mesh0/Primitive0")
                    MeshMaterial3d<StandardMaterial>(asset_value(StandardMaterial::from(
                        Color::srgb(0.8, 0.7, 0.6),
                    )))
                    Transform::from_rotation(Quat::from_xyzw(-0.707107, 0.0, 0.0, 0.707107))),
                ( #shoulder_offset Frame Transform::from_xyz(0.0061, -0.0123, 0.2904)),
            ]
        ),
        (
            #shoulder Joint
            ChildOf(#shoulder_offset)
            JointCoordinates [
                (#shoulder_elv Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(-0.998, 0.002, 0.059)}),
                (#shoulder_rot Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.005, 0.999, 0.042)}),
            ]
        ),

        // Ulna
        (
            #ulna Body
            ChildOf(#shoulder)
            InertialProperties {
                mass: 0.118,
                mass_center: nalgebra::Vector3::new(0.002, -0.133, 0.008),
                inertia: Inertia::new(0.0005, 0.0005, 0.00003, 0.0, 0.0, 0.0),
            }
            Children [
                (#ulna_mesh
                    Mesh3d("gltf/ulna.glb#Mesh0/Primitive0")
                    MeshMaterial3d<StandardMaterial>(asset_value(StandardMaterial::from(
                        Color::srgb(0.8, 0.7, 0.6),
                    )))
                    Transform::from_rotation(Quat::from_xyzw(-0.707107, 0.0, 0.0, 0.707107))),
                (#pro_sup_offset Frame Transform::from_xyz(0.0004, 0.020, 0.0115)),
            ]
        ),
        (
            #elbow Joint
            ChildOf(#ulna)
            JointCoordinates [
                (
                    #elbow_flex Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(0.005, 0.999, 0.042)}
                ),
            ]
        ),

        // Radius
        (
            #pro_sup Joint
            ChildOf(#pro_sup_offset)
            JointCoordinates [
                (#pro_sup Coordinate
                    InitialConditions { value: 0.0, velocity: 0.0 }
                    Twist{angular: nalgebra::Vector3::new(-0.017, 0.993, -0.120)}),
            ]
        ),
        (
            #radius Body
            ChildOf(#pro_sup)
            InertialProperties {
                mass: 0.234,
                mass_center: nalgebra::Vector3::new(0.034, -0.182, 0.016),
                inertia: Inertia::new(0.001, 0.001, 0.001, 0.0, 0.0, 0.0),
            }
            Children [
                (#radius_mesh
                    Mesh3d("gltf/radius.glb#Mesh0/Primitive0")
                    MeshMaterial3d<StandardMaterial>(asset_value(StandardMaterial::from(
                        Color::srgb(0.8, 0.7, 0.6),
                    )))
                    Transform::from_rotation(Quat::from_xyzw(-0.707107, 0.0, 0.0, 0.707107))),
            ]
        ),

        // Deltoid 1
        (
            #DELT1 Muscle
            HillTypeMuscleParams::default()
            PathEntities [
                (#DELT1_P1 Site ChildOf(#humerus) Transform::from_xyz(0.00896, 0.00585, 0.11883)),
                (#DELT1_P2 Site ChildOf(#humerus) Transform::from_xyz(0.01623, -0.00412, 0.10330)),
                (#DELT1_P3 Site ChildOf(#scapula) Transform::from_xyz(0.04347, 0.00499, 0.03202)),
                (#DELT1_P4 Site ChildOf(#clavicle) Transform::from_xyz(-0.01400, 0.08021, -0.01106)),
            ]
        ),

        // Biceps long
        (
            #BIClong Muscle
            HillTypeMuscleParams::default()
            PathEntities [
                (#BIClong_P1 Site ChildOf(#scapula) Transform::from_xyz(-0.03123, -0.01305, 0.02353)),
                (#BIClong_P2 Site ChildOf(#scapula) Transform::from_xyz(-0.02094, -0.00461, 0.01309)),
                (#BIClong_P3 Site ChildOf(#humerus) Transform::from_xyz(0.01921, 0.00828, -0.02083)),
                (#BIClong_P8 Site ChildOf(#humerus) Transform::from_xyz(0.02280, -0.00630, 0.17540)),
                (#BIClong_P10 Site ChildOf(#radius) Transform::from_xyz(-0.00670, -0.01273, 0.02916)),
                (#BIClong_P11 Site ChildOf(#radius) Transform::from_xyz(-0.00200, -0.00200, 0.03750)),
            ]
        ),

        // Triceps long
        (
            #TRIlong Muscle
            HillTypeMuscleParams::default()
            PathEntities [
                (#TRIlong_P1 Site ChildOf(#scapula) Transform::from_xyz(-0.04565, -0.01377, 0.04073)),
                (#TRIlong_P2 Site ChildOf(#humerus) Transform::from_xyz(-0.02714, -0.00664, 0.11441)),
                (#TRIlong_P5 Site ChildOf(#ulna) Transform::from_xyz(-0.02190, -0.00078, -0.01046)),
            ]
        ),
    ]
}
