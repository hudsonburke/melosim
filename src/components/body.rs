use serde::{Deserialize, Serialize};
use crate::math::{Transform, Vec3};
use crate::id::EntityKey;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InertialProperties {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub parent: EntityKey,
    pub transform: Transform,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Site {
    pub parent: EntityKey,
    pub offset: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Landmark {
    pub site: EntityKey,
    pub name: String,
}
