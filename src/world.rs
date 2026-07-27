use crate::components::*;
use serde::{Deserialize, Serialize};

use slotmap::SlotMap;
use slotmap::new_key_type;

new_key_type! {
    pub struct InertialKey;
    pub struct FrameKey;
    pub struct JointKey;
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct World {
    next_id: u32,
    pub inertials: SlotMap<InertialKey, InertialProperties>,
    pub frames: SlotMap<FrameKey, Frame>,
    pub joints: SlotMap<JointKey, Joint>,
}

pub struct WorldState {}

impl World {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn validate(&self) {}
}
