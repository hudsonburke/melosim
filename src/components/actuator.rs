#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Actuator {
    pub id: EntityID,
    pub name: String,
    pub actuator_type: ActuatorType,
}

pub enum ActuatorType {
    // Motor directly on a joint
    JointMotor {
        joint: EntityID,
        gear: f64,
        max_torque: f64,
    },
    // Position-controlled joint
    PositionMotor {
        joint: EntityID,
        kp: f64,
    },
    // Motor pulling on a cable
    CableMotor {
        cable: EntityID,
        gear: f64,
        max_force: f64,
        speed: f64,
    },
    // Muscle actuator
    MuscleActuator {
        muscle: EntityID,
        model: MuscleModelType,
    },
}

pub enum MuscleModelType {
    Hill,
    Millard,
    Thelen,
    Schutte,
}
