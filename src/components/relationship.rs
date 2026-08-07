use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// A relationship: this entity is a child of `parent`.
/// Replaces `Frame.parent` and `Site.parent` fields.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChildOf {
    pub parent: EntityID,
}

/// A joint connects two frames.
/// ParentFrame points to the frame that serves as the joint's parent.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParentFrame {
    pub frame: EntityID,
}

/// A joint connects two frames.
/// ChildFrame points to the frame that moves relative to the parent.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChildFrame {
    pub frame: EntityID,
}
