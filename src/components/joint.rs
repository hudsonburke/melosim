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
