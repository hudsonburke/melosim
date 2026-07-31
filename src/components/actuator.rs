use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// An actuator that drives a single coordinate (degree of freedom).
///
/// In OpenSim this is a `CoordinateActuator` — a torque actuator that
/// acts on one coordinate. The `optimal_force` scales the control signal
/// to produce the generalized force.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CoordinateActuator {
    /// The coordinate (degree of freedom) this actuator drives.
    pub coordinate: EntityID,
    /// Force per unit control signal (N or N·m).
    pub optimal_force: f64,
    /// Minimum control signal (default: -1.0).
    pub min_control: f64,
    /// Maximum control signal (default: 1.0).
    pub max_control: f64,
}

// ── Validation ────────────────────────────────────────

use super::{Validate, JointCoordinate};
use crate::world::World;
use crate::systems::{System, validate_all, check_has};

impl Validate for CoordinateActuator {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        check_has::<JointCoordinate>(world, entity, "coordinate", self.coordinate)
            .into_iter().collect()
    }
}

inventory::submit! { System::new("validate_coordinate_actuator", |w| validate_all::<CoordinateActuator>(w)) }
