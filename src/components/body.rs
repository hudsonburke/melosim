use bevy_ecs::prelude::*;
use super::Validate;
use crate::world::World;
use crate::systems::check_has;
use super::spatial::Position;

#[derive(Component, Clone, Debug)]
pub struct InertialProperties {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

/// A frame computed from anatomical landmarks (stations).
/// Used for attaching exoskeleton parts — the user places 3-4
/// stations, the system computes the frame, and the attachment
/// follows automatically when the body scales.
#[derive(Component, Clone, Debug)]
pub struct StationDefinedFrame {
    /// Station providing the frame origin.
    pub origin: Entity,
    /// Station defining the X axis direction (from origin).
    pub axis_x: Entity,
    /// Station defining the Y axis direction (from origin).
    pub axis_y: Entity,
}

// ── Validation ────────────────────────────────────────

impl Validate for StationDefinedFrame {
    fn validate(&self, entity: Entity, world: &World) -> Vec<String> {
        [
            check_has::<Position>(world, entity, "origin", self.origin),
            check_has::<Position>(world, entity, "axis_x", self.axis_x),
            check_has::<Position>(world, entity, "axis_y", self.axis_y),
        ].into_iter().flatten().collect()
    }
}
