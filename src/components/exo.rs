use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExoPart {
    pub id: EntityID,
    pub name: String,
    pub part_type: ExoPartType,
    pub body: EntityID,
    pub offset: crate::math::Vec3,
    pub ports: Vec<crate::components::CablePort>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ExoPartType {
    Cuff { coverage: f64 },
    Brace { length: f64 },
    MotorMount { torque: f64 },
    CableGuide { diameter: f64 },
}
