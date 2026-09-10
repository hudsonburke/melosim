
use bevy::prelude::*;
use nalgebra::{Matrix3, Vector3};

/// Upper-triangle inertia tensor: (Ixx, Iyy, Izz, Ixy, Ixz, Iyz).
#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct Inertia(pub [f64; 6]);

impl Inertia {
    pub fn new(ixx: f64, iyy: f64, izz: f64, ixy: f64, ixz: f64, iyz: f64) -> Self {
        Self([ixx, iyy, izz, ixy, ixz, iyz])
    }

    /// Diagonal inertia from three principal moments (Ixx, Iyy, Izz).
    pub fn diag(ixx: f64, iyy: f64, izz: f64) -> Self {
        Self([ixx, iyy, izz, 0.0, 0.0, 0.0])
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
    pub mass_center: nalgebra::Vector3<f64>,
    pub inertia: Inertia,
}

impl InertialProperties {
    pub fn new(mass: f64, mass_center: Vector3<f64>, inertia: Inertia) -> Self {
        Self { mass, mass_center, inertia }
    }
}
