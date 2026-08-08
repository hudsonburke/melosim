use bevy_ecs::prelude::*;

/// A human-readable name for any entity.
///
/// Names are metadata — used for import/export, logging, and debugging.
/// Solvers never access names during simulation.
///
/// Attach to any entity that needs identity:
/// ```ignore
/// let entity = world.spawn(Name::new("pelvis")).id();
/// ```
#[derive(Component, Clone, Debug)]
pub struct Name {
    pub value: String,
}

impl Name {
    pub fn new(value: impl Into<String>) -> Self {
        Self { value: value.into() }
    }
}

impl From<&str> for Name {
    fn from(s: &str) -> Self {
        Self { value: s.to_string() }
    }
}

impl From<String> for Name {
    fn from(s: String) -> Self {
        Self { value: s }
    }
}
