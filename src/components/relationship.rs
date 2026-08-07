use serde::{Deserialize, Serialize};
use crate::id::EntityID;

// ── Generic relationship ──────────────────────────────

/// A relationship: this entity is a child of `parent`.
/// Fallback for cases where no typed relationship applies.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChildOf {
    pub parent: EntityID,
}

/// Bidirectional relationship target: auto-synced list of children.
/// Maintained by World when ChildOf is attached/detached.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Children {
    pub entities: Vec<EntityID>,
}

// ── Spatial relationships (positioning) ───────────────

/// "This entity is positioned relative to this frame."
/// Used by bodies, sites, joint entities, geometry.
/// The FK solver walks InFrame to compute world poses.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct InFrame(pub EntityID);

/// Auto-synced list of entities positioned in this frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct FrameContents {
    pub entities: Vec<EntityID>,
}

// ── Behavioral relationships (motion) ─────────────────

/// "This joint connects to this frame (child side)."
/// The FK solver uses this to find which frame each joint drives.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Connects(pub EntityID);

/// Auto-synced list of joints that connect to this frame.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct ConnectedJoints {
    pub entities: Vec<EntityID>,
}

/// "This coordinate belongs to this joint."
/// Groups DOFs under a joint entity.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HasDOF(pub EntityID);

/// Auto-synced list of coordinates (DOFs) in this joint.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct JointDOFs {
    pub entities: Vec<EntityID>,
}

/// "This effect reads from this coordinate."
/// Maps coordinate value to spatial transform component.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Drives(pub EntityID);

/// Auto-synced list of effects driven by this coordinate.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CoordinateEffects {
    pub entities: Vec<EntityID>,
}
