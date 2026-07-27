use serde::{Deserialize, Serialize};

use slotmap::new_key_type;

new_key_type! {
    pub struct MuscleKey;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MuscleState {
    pub fiber_length: f64,
    pub fiber_velocity: f64,
    pub activation: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HillTypeMuscleParams {
    pub max_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub pcsa: f64,
    pub pennation_angle: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ForceLengthCurve {}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ForceVelocityCurve {}
