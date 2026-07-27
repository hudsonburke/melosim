use serde::{Deserialize, Serialize};
use crate::id::EntityKey;

/// Common fields shared by all joint types.
/// Inlined into each type-specific joint component.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
}

// ── Type-specific joint components ────────────────────
// Each joint type is a standalone component that carries its own
// base fields (body_a, body_b, limits). This keeps each entity
// mapped to exactly one component, which is the slotmap pattern.
//
// A new joint type = a new struct + a system. No other code changes.

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HingeJoint {
    pub body_a: EntityKey,
    pub body_b: EntityKey,
    pub limits: Option<JointLimits>,
    pub axis: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SlideJoint {
    pub body_a: EntityKey,
    pub body_b: EntityKey,
    pub limits: Option<JointLimits>,
    pub axis: [f64; 3],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BallJoint {
    pub body_a: EntityKey,
    pub body_b: EntityKey,
    pub limits: Option<JointLimits>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FreeJoint {
    pub body_a: EntityKey,
    pub body_b: EntityKey,
    pub limits: Option<JointLimits>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FixedJoint {
    pub body_a: EntityKey,
    pub body_b: EntityKey,
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
    pub body_a: EntityKey,
    pub body_b: EntityKey,
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
    pub body_a: EntityKey,
    pub body_b: EntityKey,
    pub limits: Option<JointLimits>,
    /// The coordinates (DOFs) of this joint, in order.
    /// Each is an EntityKey referencing a JointCoordinate component.
    pub coordinates: Vec<EntityKey>,
}
