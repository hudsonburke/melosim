use super::{JointCoordinate, Validate};
use crate::systems::check_has;
use crate::world::World;
use bevy_ecs::prelude::*;

/// An actuator that drives a single coordinate (degree of freedom).
///
/// In OpenSim this is a `CoordinateActuator` — a torque actuator that
/// acts on one coordinate. The `optimal_force` scales the control signal
/// to produce the generalized force.
#[derive(Component, Clone, Debug)]
pub struct CoordinateActuator {
    /// Force per unit control signal (N or N·m).
    pub optimal_force: f64,
    /// Minimum control signal (default: -1.0).
    pub min_control: f64,
    /// Maximum control signal (default: 1.0).
    pub max_control: f64,
}

// ── Validation ────────────────────────────────────────

impl Validate for CoordinateActuator {
    fn validate(&self, entity: Entity, world: &World) -> Vec<String> {
        check_has::<JointCoordinate>(world, entity, "coordinate", self.coordinate)
            .into_iter()
            .collect()
    }
}
