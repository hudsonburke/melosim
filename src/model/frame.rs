use bevy::prelude::*;
use nalgebra::{Matrix3, Vector3};

/// Marker for a rigid body entity.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform, Visibility)]
pub struct Body;

/// Marker for a frame.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform, Visibility)]
pub struct Frame;

/// Marker for a site — a point on a frame.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform, Visibility)]
pub struct Site;

/// Upper-triangle inertia tensor: (Ixx, Iyy, Izz, Ixy, Ixz, Iyz).
#[derive(Component, Clone, Debug, Default, Reflect)]
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
            self.0[0], self.0[3], self.0[4],
            self.0[3], self.0[1], self.0[5],
            self.0[4], self.0[5], self.0[2],
        )
    }
}

#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct InertialProperties {
    pub mass: f64,
    #[reflect(ignore)]
    pub mass_center: nalgebra::Vector3<f64>, //TODO: Maybe a convenience constructor
    pub inertia: Inertia,
}
