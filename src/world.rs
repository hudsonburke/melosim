use bevy_ecs::prelude::*;

// Re-export bevy_ecs::World as melosim's World type.
// No wrapper — use Bevy's API directly.
pub use bevy_ecs::world::World;

/// Resource to hold validation errors.
#[derive(Resource, Default, Clone)]
pub struct ErrorList(pub Vec<String>);
