use serde::{Deserialize, Serialize};
use crate::id::EntityKey;
use crate::math::Transform;

/// A wrapping surface that a muscle-tendon path can wrap over.
///
/// Attached to a body at a given transform. The muscle path solver
/// computes how the muscle wraps around the surface based on the
/// current body positions and the wrap geometry type.
///
/// In Rajagopal 2015, wrapping surfaces are used for muscles that
/// wrap around bones and joints (e.g., the quadriceps around the femur).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WrapGeom {
    pub name: String,
    /// The body this wrap surface is rigidly attached to.
    pub body: EntityKey,
    /// Transform from body frame to wrap surface frame.
    pub transform: Transform,
    /// The shape of the wrapping surface.
    pub geom_type: WrapGeomType,
}

/// Supported wrapping surface types.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum WrapGeomType {
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
    Ellipsoid { radii: [f64; 3] },
}
