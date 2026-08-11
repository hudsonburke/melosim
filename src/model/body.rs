use bevy::prelude::*;
use nalgebra::{Matrix3, Vector3};

/// Marker for a rigid body entity.
///
/// Bodies are the nodes in the kinematic tree. Joints connect them.
/// Each body should have `InertialProperties` and be connected to
/// the tree via `ChildOf` (through a joint entity).
///
/// `Transform` and `Visibility` are required so Bevy's transform and
/// visibility propagation reach every body in the tree (a child with
/// `GlobalTransform`/`InheritedVisibility` whose parent lacks them triggers
/// warning B0004); the simulation writes `Transform`, never reads them.
#[derive(Component, Clone, Debug, Default)]
#[require(Transform, Visibility)]
pub struct Body;

#[derive(Component)]
pub struct Site(Vector3<f64>);

/// Upper-triangle inertia tensor: (Ixx, Iyy, Izz, Ixy, Ixz, Iyz).
/// Matches OpenSim's storage order.
///
/// `Default` (zeros) exists only as a BSN patch base — a real body
/// must always specify its inertia.
#[derive(Component, Clone, Debug, Default)]
pub struct Inertia(pub [f64; 6]);

impl Inertia {
    pub fn new(ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> Self {
        Self([ixx, iyy, izz, ixy, ixz, iyz])
    }

    pub fn diagonal(&self) -> Vector3<f64> {
        Vector3::new(self.0[0], self.0[1], self.0[2])
    }

    pub fn to_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.0[0], self.0[3], self.0[4], self.0[3], self.0[1], self.0[5], self.0[4], self.0[5],
            self.0[2],
        )
    }
}

#[derive(Component, Clone, Debug, Default)]
pub struct InertialProperties {
    pub mass: f64,
    pub mass_center: Vector3<f64>,
    pub inertia: Inertia,
}

/// A [`Scene`] for a rigid body with a visual mesh — the "create the body"
/// half of model authoring. List it as an entry inside a larger `bsn!`;
/// its components merge onto that entity, so patches and `Children` written
/// after it apply to the body:
///
/// ```ignore
/// bsn! {
///     bone("humerus", "meshes/humerus.stl")
///     InertialProperties { mass: 1.865 /* ... */ }
///     Children [ /* joints connecting this body to its children */ ]
/// }
/// ```
pub fn bone(name: &'static str, mesh: &'static str) -> impl Scene {
    bsn! {
        Body
        Name({name})
        Mesh3d({mesh})
        MeshMaterial3d<StandardMaterial>(asset_value(Color::srgb(0.82, 0.72, 0.62)))
    }
}

/// A frame computed from anatomical landmarks (stations).
/// Used for attaching exoskeleton parts — the user places 3-4
/// stations, the system computes the frame, and the attachment
/// follows automatically when the body scales.
#[derive(Component)]
pub struct StationDefinedFrame {
    /// Station providing the frame origin.
    pub origin: Entity,
    /// Station defining the X axis direction (from origin).
    pub axis_x: Entity,
    /// Station defining the Y axis direction (from origin).
    pub axis_y: Entity,
}
