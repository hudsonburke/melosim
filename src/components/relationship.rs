use bevy_ecs::prelude::*;

// ── Spatial relationships (positioning) ───────────────

/// "This entity is positioned relative to this frame."
/// Used by bodies, sites, joint entities, geometry.
/// The FK solver walks InFrame to compute world poses.
#[derive(Component, Debug)]
#[relationship(relationship_target = FrameContents)]
pub struct InFrame(Entity);

/// Auto-synced list of entities positioned in this frame.
#[derive(Component, Debug)]
pub struct FrameContents(Vec<Entity>);

// ── Behavioral relationships (motion) ─────────────────

/// "This joint connects to this frame (child side)."
/// The FK solver uses this to find which frame each joint drives.
#[derive(Component, Debug)]
#[relationship(relationship_target = ConnectedJoints)]
pub struct Connects(Entity);

/// Auto-synced list of joints that connect to this frame.
#[derive(Component, Debug)]
pub struct ConnectedJoints(Vec<Entity>);

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
