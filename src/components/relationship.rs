use bevy_ecs::prelude::*;

// ── Generic relationship ──────────────────────────────

/// A relationship: this entity is a child of `parent`.
/// Fallback for cases where no typed relationship applies.
#[derive(Component, Clone, Debug)]
pub struct ChildOf {
    pub parent: Entity,
}

/// Bidirectional relationship target: auto-synced list of children.
/// Maintained by World when ChildOf is attached/detached.
#[derive(Component, Clone, Debug, Default)]
pub struct Children {
    pub entities: Vec<Entity>,
}

// ── Spatial relationships (positioning) ───────────────

/// "This entity is positioned relative to this frame."
/// Used by bodies, sites, joint entities, geometry.
/// The FK solver walks InFrame to compute world poses.
#[derive(Component, Clone, Debug)]
pub struct InFrame(pub Entity);

/// Auto-synced list of entities positioned in this frame.
#[derive(Component, Clone, Debug, Default)]
pub struct FrameContents {
    pub entities: Vec<Entity>,
}

// ── Behavioral relationships (motion) ─────────────────

/// "This joint connects to this frame (child side)."
/// The FK solver uses this to find which frame each joint drives.
#[derive(Component, Clone, Debug)]
pub struct Connects(pub Entity);

/// Auto-synced list of joints that connect to this frame.
#[derive(Component, Clone, Debug, Default)]
pub struct ConnectedJoints {
    pub entities: Vec<Entity>,
}

/// "This coordinate belongs to this joint."
/// Groups DOFs under a joint entity.
#[derive(Component, Clone, Debug)]
pub struct HasDOF(pub Entity);

/// Auto-synced list of coordinates (DOFs) in this joint.
#[derive(Component, Clone, Debug, Default)]
pub struct JointDOFs {
    pub entities: Vec<Entity>,
}

/// "This effect reads from this coordinate."
/// Maps coordinate value to spatial transform component.
#[derive(Component, Clone, Debug)]
pub struct Drives(pub Entity);

/// Auto-synced list of effects driven by this coordinate.
#[derive(Component, Clone, Debug, Default)]
pub struct CoordinateEffects {
    pub entities: Vec<Entity>,
}
