use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Actuator {
    pub entity: EntityID,
    pub name: String,
    pub actuator_type: ActuatorType,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ActuatorType {
    JointMotor {
        joint: EntityID,
        gear: f64,
        max_torque: f64,
    },
    PositionMotor {
        joint: EntityID,
        kp: f64,
    },
    CableMotor {
        cable: EntityID,
        gear: f64,
        max_force: f64,
        speed: f64,
    },
    MuscleActuator {
        muscle: EntityID,
        model: MuscleModelType,
    },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum MuscleModelType {
    Hill,
    Millard,
    Thelen,
    Schutte,
}
