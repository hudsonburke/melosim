use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CableGuide {
    pub entity: EntityID,
    pub site: EntityID,
    pub diameter: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CablePort {
    pub entity: EntityID,
    pub port_type: PortType,
    pub offset: crate::math::Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum PortType {
    Entry,
    Exit,
    Termination,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Cable {
    pub entity: EntityID,
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
