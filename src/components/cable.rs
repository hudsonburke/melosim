use serde::{Deserialize, Serialize};
use crate::id::EntityID;
use crate::math::Vec3;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CableGuide {
    pub id: EntityID,
    pub site: EntityID,
    pub diameter: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CablePort {
    pub id: EntityID,
    pub port_type: PortType,
    pub offset: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum PortType {
    Entry,
    Exit,
    Termination,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Cable {
    pub id: EntityID,
    pub name: String,
    pub path: Vec<CableSegment>,
    pub tendon: Option<EntityID>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CableSegment {
    ViaPoint(EntityID),
    Port(EntityID),
    Wrap(EntityID),
}
