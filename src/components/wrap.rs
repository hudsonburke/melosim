use bevy_ecs::prelude::*;
use crate::math::Transform;

/// A wrapping surface that a muscle-tendon path can wrap over.
///
/// Attached to a body at a given transform. The muscle path solver
/// computes how the muscle wraps around the surface based on the
/// current body positions and the wrap geometry type.
///
/// In Rajagopal 2015, wrapping surfaces are used for muscles that
/// wrap around bones and joints (e.g., the quadriceps around the femur).
#[derive(Component, Clone, Debug)]
pub struct WrapGeom {
    /// The body this wrap surface is rigidly attached to.
    pub body: Entity,
    /// Transform from body frame to wrap surface frame.
    pub transform: Transform,
    /// The shape of the wrapping surface.
    pub geom_type: WrapGeomType,
}

/// Supported wrapping surface types.
#[derive(Clone, Debug)]
pub enum WrapGeomType {
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
    Ellipsoid { radii: [f64; 3] },
}
