use serde::{Deserialize, Serialize};
use crate::id::EntityID;
use crate::math::{Transform, Vec3};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Body {
    pub id: EntityID,
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub id: EntityID,
    pub body: EntityID,
    pub transform: Transform,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Site {
    pub id: EntityID,
    pub body: EntityID,
    pub offset: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Landmark {
    pub site: EntityID,
    pub name: String,
}
