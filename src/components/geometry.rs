use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshGeometry {
    pub entity: EntityID,
    pub body: EntityID,
    pub mesh: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PrimitiveGeometry {
    pub entity: EntityID,
    pub body: EntityID,
    pub shape: Shape,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Shape {
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
    Capsule { radius: f64, length: f64 },
    Box { half_extents: [f64; 3] },
    Plane,
}
