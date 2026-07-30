mod actuator;
mod body;
mod coordinate;
mod geometry;
mod joint;
mod material;
mod muscle;
mod name;
mod path;
mod tendon;
mod wrap;

pub use actuator::*;
pub use body::*;
pub use coordinate::*;
pub use geometry::*;
pub use joint::*;
pub use material::*;
pub use muscle::*;
pub use name::*;
pub use path::*;
pub use tendon::*;
pub use wrap::*;

use crate::id::EntityID;
use crate::world::World;

/// Validation trait for components that have invariants.
///
/// Implement this on a component to define its validation rules.
/// The generic `validate_all::<T>()` function iterates all instances
/// and collects errors into the world's error resource.
pub trait Validate {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String>;
}
