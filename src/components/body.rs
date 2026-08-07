use serde::{Deserialize, Serialize};
use crate::id::EntityID;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InertialProperties {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

/// A frame computed from anatomical landmarks (stations).
/// Used for attaching exoskeleton parts — the user places 3-4
/// stations, the system computes the frame, and the attachment
/// follows automatically when the body scales.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StationDefinedFrame {
    /// Station providing the frame origin.
    pub origin: EntityID,
    /// Station defining the X axis direction (from origin).
    pub axis_x: EntityID,
    /// Station defining the Y axis direction (from origin).
    pub axis_y: EntityID,
}

// ── Validation ────────────────────────────────────────

use super::Validate;
use crate::world::World;
use crate::systems::{System, validate_all, check_has};
use super::spatial::Position;

impl Validate for StationDefinedFrame {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        [
            check_has::<Position>(world, entity, "origin", self.origin),
            check_has::<Position>(world, entity, "axis_x", self.axis_x),
            check_has::<Position>(world, entity, "axis_y", self.axis_y),
        ].into_iter().flatten().collect()
    }
}

inventory::submit! { System::new("validate_station_defined_frame", |w| validate_all::<StationDefinedFrame>(w)) }
