use bevy_ecs::prelude::*;
use crate::math::{Vec3, Quaternion};

/// A 3D position in the parent's coordinate system.
/// Used alone for sites/landmarks (no rotation).
#[derive(Component, Clone, Copy, Debug)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn to_vec3(&self) -> Vec3 {
        Vec3 { x: self.x, y: self.y, z: self.z }
    }
}

impl From<Vec3> for Position {
    fn from(v: Vec3) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}

/// A rotation in the parent's coordinate system.
/// Combined with Position, this defines a full 6-DOF frame.
#[derive(Component, Clone, Copy, Debug)]
pub struct Rotation {
    pub quaternion: Quaternion,
}

impl Rotation {
    pub fn identity() -> Self {
        Self { quaternion: Quaternion::default() }
    }
}

impl From<Quaternion> for Rotation {
    fn from(q: Quaternion) -> Self {
        Self { quaternion: q }
    }
}
