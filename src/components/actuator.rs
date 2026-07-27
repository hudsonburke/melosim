use serde::{Deserialize, Serialize};
use crate::id::EntityKey;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Actuator {
    pub name: String,
    pub actuator_type: ActuatorType,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ActuatorType {
    JointMotor {
        joint: EntityKey,
        gear: f64,
        max_torque: f64,
    },
    PositionMotor {
        joint: EntityKey,
        kp: f64,
    },
    CableMotor {
        cable: EntityKey,
        gear: f64,
        max_force: f64,
        speed: f64,
    },
    MuscleActuator {
        muscle: EntityKey,
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
