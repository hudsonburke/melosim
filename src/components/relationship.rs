use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// A relationship: this entity is a child of `parent`.
/// Used for spatial hierarchy (frames, sites, joints in the tree)
/// and for grouping (coordinate effects under coordinates, coordinates under joints).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChildOf {
    pub parent: EntityID,
}

/// Bidirectional relationship target: auto-synced list of children.
/// Maintained by World when ChildOf is attached/detached.
/// Modeled after Bevy's Children component.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Children {
    pub entities: Vec<EntityID>,
}
