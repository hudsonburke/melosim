use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// Common fields shared by all joint types.
/// Inlined into each type-specific joint component.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
}

/// A unified joint component that replaces all type-specific joint structs.
///
/// The `joint_type` field distinguishes between joint kinds:
/// - "PinJoint" (hinge) — 1 rotational DOF along an axis
/// - "WeldJoint" (fixed) — no DOFs, rigidly connects two bodies
/// - "BallJoint" — 3 rotational DOFs
/// - "FreeJoint" — 6 DOFs (3 rotation + 3 translation)
/// - "UniversalJoint" — 2 rotational DOFs on orthogonal axes
/// - "CustomJoint" — arbitrary DOFs defined by CoordinateEffects
///
/// Coordinates (DOFs) are separate entities referenced by `coordinates`.
/// For simple joints (Pin/Weld/Ball/Free), coordinates are created by
/// convenience builders on World. For CustomJoint, coordinates are provided
/// by the caller.
///
/// CoordinateEffect components on separate entities define how each
/// coordinate drives the spatial transform. SpatialTransform components
/// group the effects for a joint.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
    pub joint_type: &'static str,
    pub coordinates: Vec<EntityID>,
}

// ── Validation ────────────────────────────────────────

use super::{Validate, InertialProperties, JointCoordinate};
use crate::world::World;
use crate::systems::{System, validate_all, check_has, check_exists};

impl Validate for Joint {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        let mut e: Vec<String> = [
            check_has::<InertialProperties>(world, entity, "body_a", self.body_a),
            check_has::<InertialProperties>(world, entity, "body_b", self.body_b),
        ].into_iter().flatten().collect();
        for (i, coord_key) in self.coordinates.iter().enumerate() {
            if let Some(err) = check_has::<JointCoordinate>(world, entity, &format!("coordinates[{i}]"), *coord_key) {
                e.push(err);
            }
        }
        e
    }
}

inventory::submit! { System::new("validate_joint", |w| validate_all::<Joint>(w)) }
