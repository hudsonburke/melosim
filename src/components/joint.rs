use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// Common fields shared by all joint types.
/// Inlined into each type-specific joint component.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
}

// ── Type-specific joint components ────────────────────
// Each joint type is a standalone component that carries its own
// base fields (body_a, body_b, limits). A joint is an entity with
// exactly one joint component.
//
// A new joint type = a new struct + a system. No other code changes.

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HingeJoint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
    pub axis: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SlideJoint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
    pub axis: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BallJoint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FreeJoint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FixedJoint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
}

// ── OpenSim-compatible joint types ────────────────────
// These correspond to OpenSim's UniversalJoint and CustomJoint.
// CustomJoint is the general case: 1-6 DOFs with SpatialTransform
// encoded by CoordinateEffect components on separate entities.

/// Two rotational DOFs on orthogonal axes.
/// Corresponds to OpenSim's `UniversalJoint`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct UniversalJoint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
    /// First rotation axis (in parent frame).
    pub axis1: [f64; 3],
    /// Second rotation axis (orthogonal to axis1, in child frame).
    pub axis2: [f64; 3],
}

/// A joint defined by a SpatialTransform with up to 6 coordinates.
///
/// The joint's spatial transform is the composition of CoordinateEffect
/// components on separate entities. Each CoordinateEffect maps one
/// coordinate to one of the six transform components (rotX/Y/Z, transX/Y/Z)
/// via a JointFunction (Constant, Linear, or Polynomial).
///
/// Corresponds to OpenSim's `CustomJoint`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CustomJoint {
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub limits: Option<JointLimits>,
    /// The coordinates (DOFs) of this joint, in order.
    /// Each is an EntityID referencing a JointCoordinate component.
    pub coordinates: Vec<EntityID>,
}

// ── Validation ────────────────────────────────────────

use super::{Validate, Frame};
use crate::world::World;
use crate::systems::{System, validate_all, check_has};

macro_rules! impl_validate_body_refs {
    ($ty:ident) => {
        impl Validate for $ty {
            fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
                [
                    check_has::<Frame>(world, entity, "body_a", self.body_a),
                    check_has::<Frame>(world, entity, "body_b", self.body_b),
                ].into_iter().flatten().collect()
            }
        }
    };
}

impl_validate_body_refs!(HingeJoint);
impl_validate_body_refs!(SlideJoint);
impl_validate_body_refs!(BallJoint);
impl_validate_body_refs!(FreeJoint);
impl_validate_body_refs!(FixedJoint);
impl_validate_body_refs!(UniversalJoint);

impl Validate for CustomJoint {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        let mut e: Vec<String> = [
            check_has::<Frame>(world, entity, "body_a", self.body_a),
            check_has::<Frame>(world, entity, "body_b", self.body_b),
        ].into_iter().flatten().collect();
        for (i, coord_key) in self.coordinates.iter().enumerate() {
            if let Some(err) = check_has::<super::JointCoordinate>(world, entity, &format!("coordinates[{i}]"), *coord_key) {
                e.push(err);
            }
        }
        e
    }
}

inventory::submit! { System::new("validate_hinge", |w| validate_all::<HingeJoint>(w)) }
inventory::submit! { System::new("validate_slide", |w| validate_all::<SlideJoint>(w)) }
inventory::submit! { System::new("validate_ball", |w| validate_all::<BallJoint>(w)) }
inventory::submit! { System::new("validate_free", |w| validate_all::<FreeJoint>(w)) }
inventory::submit! { System::new("validate_fixed", |w| validate_all::<FixedJoint>(w)) }
inventory::submit! { System::new("validate_universal", |w| validate_all::<UniversalJoint>(w)) }
inventory::submit! { System::new("validate_custom", |w| validate_all::<CustomJoint>(w)) }
