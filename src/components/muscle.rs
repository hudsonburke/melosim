use serde::{Deserialize, Serialize};
use crate::id::EntityID;
use crate::math::Vec3;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Muscle {
    pub entity: EntityID,
    pub name: String,
    pub path: Vec<MusclePoint>,
    pub max_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub pcsa: f64,
    pub pennation_angle: f64,
}
