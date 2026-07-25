use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint {
    pub id: EntityID,
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub joint_type: JointType,
    pub limits: Option<JointLimits>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum JointType {
    Hinge { axis: [f64; 3] },
    Slide { axis: [f64; 3] },
    Ball,
    Free,
    Fixed,
}
