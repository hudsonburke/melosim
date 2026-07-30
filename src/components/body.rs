use serde::{Deserialize, Serialize};
use crate::id::EntityID;
use crate::math::{Transform, Vec3};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InertialProperties {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub parent: EntityID,
    pub transform: Transform,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Site {
    pub parent: EntityID,
    pub offset: Vec3,
}

// ── Validation ────────────────────────────────────────

use super::Validate;
use crate::world::World;
use crate::systems::{System, validate_all};

impl Validate for Frame {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        if world.get::<InertialProperties>(self.parent).is_none() {
            vec![format!("Frame {:?} references missing parent {:?}", entity.0, self.parent.0)]
        } else {
            Vec::new()
        }
    }
}

impl Validate for Site {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        if world.get::<InertialProperties>(self.parent).is_none() {
            vec![format!("Site {:?} references missing parent {:?}", entity.0, self.parent.0)]
        } else {
            Vec::new()
        }
    }
}

inventory::submit! { System::new("validate_frame", |w| validate_all::<Frame>(w)) }
inventory::submit! { System::new("validate_site", |w| validate_all::<Site>(w)) }
