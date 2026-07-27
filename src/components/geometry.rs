use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshGeometry {
    pub mesh: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sphere {
    radius: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Cylinder {
    radius: f64,
    length: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Capsule {
    radius: f64,
    length: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Box {
    half_extents: [f64; 3],
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Plane;
