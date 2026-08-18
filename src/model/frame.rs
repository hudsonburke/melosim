use bevy::prelude::*;
use nalgebra::{Matrix3, Vector3};

/// Marker for a rigid body entity.
///
/// A Body is a marker tag for a rigid body in the model hierarchy.
/// It has an implicit default frame represented by its `Transform`
/// component (identity by default). Child `Frame` and `Site` entities
/// are positioned relative to this default frame.
#[derive(Component, Clone, Debug, Default)]
#[require(Transform, Visibility)]
pub struct Body;

/// Marker for a frame — a coordinate system offset from its parent.
///
/// The frame's offset is stored in its `Transform` component. The parent
/// is specified via `ChildOf` — either another `Frame` or a `Body`.
#[derive(Component, Clone, Debug, Default)]
pub struct Frame;

/// Marker for a site — a point on a frame.
///
/// The site's position is stored in its `Transform` component. The parent
/// frame is specified via `ChildOf` — either a `Frame` or a `Body`
/// (using the body's default frame).
///
/// To find sites belonging to a body, query `Query<&Site, With<ChildOf<Body>>>`
/// or iterate the body's `Children` and filter by `With<Site>`.
#[derive(Component, Clone, Debug, Default)]
pub struct Site;

/// Upper-triangle inertia tensor: (Ixx, Iyy, Izz, Ixy, Ixz, Iyz).
#[derive(Component, Clone, Debug, Default)]
pub struct Inertia(pub [f64; 6]);

impl Inertia {
    pub fn new(ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> Self {
        Self([ixx, iyy, izz, ixy, ixz, iyz])
    }

    pub fn from_diagonal(diagonal: Vector3<f64>) -> Self {
        Self([diagonal.x, diagonal.y, diagonal.z, 0.0, 0.0, 0.0])
    }

    pub fn diagonal(&self) -> Vector3<f64> {
        Vector3::new(self.0[0], self.0[1], self.0[2])
    }

    pub fn to_matrix(&self) -> Matrix3<f64> {
        Matrix3::new(
            self.0[0], self.0[3], self.0[4], //
            self.0[3], self.0[1], self.0[5], //
            self.0[4], self.0[5], self.0[2], //
        )
    }
}

#[derive(Component, Clone, Debug, Default)]
pub struct InertialProperties {
    pub mass: f64,
    pub mass_center: nalgebra::Vector3<f64>,
    pub inertia: Inertia,
}
