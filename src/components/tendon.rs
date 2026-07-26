use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Tendon {
    pub entity: EntityID,
    pub name: String,
    pub spring_length: f64,
    pub width: f64,
    pub via_points: Vec<EntityID>,
}
