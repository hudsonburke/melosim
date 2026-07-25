use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WrapGeom {
    pub id: EntityID,
    pub body: EntityID,
    pub geom_type: WrapGeomType,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum WrapGeomType {
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WrapPoint {
    pub site: EntityID,
    pub wrap_geom: EntityID,
}
