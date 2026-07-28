use serde::{Deserialize, Serialize};
use crate::id::EntityID;
use crate::math::Transform;

/// A display/mesh geometry attached to a body for visualization.
///
/// OpenSim serializes these in each Body's `<VisibleObject>` element.
/// They're purely visual — not used in simulation.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct DisplayGeometry {
    pub body: EntityID,
    pub mesh_file: Option<String>,
    pub scale: [f64; 3],
    pub color: [f64; 3],
    pub opacity: f64,
    pub transform: Transform,
}

/// A mesh geometry reference (file path).
/// Used for both display and collision geometry in some imports.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshGeometry {
    pub mesh: String,
}

/// Primitive geometry shapes.
/// These are kept as standalone structs for import/export convenience.
/// In the ECS, entities use `DisplayGeometry` for visualization.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sphere {
    pub radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Cylinder {
    pub radius: f64,
    pub length: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Capsule {
    pub radius: f64,
    pub length: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BoxGeom {
    pub half_extents: [f64; 3],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Plane;
