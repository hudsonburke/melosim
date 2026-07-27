use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WrapGeom {
    pub geom_type: WrapGeomType,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum WrapGeomType {
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
}
