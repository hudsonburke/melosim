// ── System infrastructure ──────────────────────────────
//
// Systems are named functions that operate on the World.
// Validation is just a system that checks invariants.

use crate::world::{World, ErrorList};
use crate::world::WorldExt;
use bevy_ecs::prelude::*;

/// A named system that operates on the World.
pub struct System {
    pub name: &'static str,
    pub run: fn(&mut World),
}

impl System {
    pub const fn new(name: &'static str, run: fn(&mut World)) -> Self {
        Self { name, run }
    }
}

// ── Validation helpers ────────────────────────────────

use crate::components::Validate;

/// Check that a referenced entity exists AND has component T.
pub fn check_has<T: Component>(
    world: &World,
    _entity: Entity,
    field: &str,
    reference: Entity,
) -> Option<String> {
    if world.get::<T>(reference).is_none() {
        Some(format!(
            "{} references entity that is missing {}",
            field, std::any::type_name::<T>()
        ))
    } else {
        None
    }
}

/// Iterate all instances of T, call validate on each, collect errors.
pub fn validate_all<T: Validate + Component + Clone>(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, component) in world.iter::<T>() {
        local_errors.extend(component.validate(key, world));
    }
    let mut errors = world.get_resource_or_insert_with(ErrorList::default);
    errors.0.extend(local_errors);
}

/// Run all registered systems (validation, etc.)
pub fn run_systems(world: &mut World) {
    // Clear previous errors
    world.insert_resource(ErrorList::default());

    // Run validation systems
    validate_all::<crate::components::StationDefinedFrame>(world);
    validate_all::<crate::components::JointCoordinate>(world);
    validate_all::<crate::components::CoordinateEffect>(world);
    validate_all::<crate::components::CoordinateActuator>(world);
}

/// Print accumulated validation errors. Run this last.
pub fn print_errors(world: &mut World) {
    let errors = world.get_resource::<ErrorList>();
    let count = errors.as_ref().map_or(0, |e| e.0.len());
    if count == 0 {
        println!("Validation: World is valid");
    } else {
        for e in &errors.unwrap().0 {
            println!("VALIDATION ERROR: {}", e);
        }
    }
}
