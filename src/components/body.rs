use crate::math::{Transform, Vec3};
use serde::{Deserialize, Serialize};
use slotmap::new_key_type;

new_key_type! {
    pub struct BodyKey;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InertialProperties {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub parent: BodyKey,
    pub transform: Transform,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Site {
    pub parent: BodyKey,
    pub offset: Vec3,
}
