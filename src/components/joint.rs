use crate::components::body::BodyKey;
use serde::{Deserialize, Serialize};
use slotmap::new_key_type;

new_key_type! {
    pub struct JointKey;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint {
    pub body_a: BodyKey,
    pub body_b: BodyKey,
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
