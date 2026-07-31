// ── System infrastructure ──────────────────────────────
//
// Systems are named functions that operate on the World.
// Plugins register systems via `inventory::submit!`.
// Running `run_systems()` executes all registered systems.
//
// To add a system:
//   inventory::submit! { System::new("my_system", |w| { ... }) }
//
// Validation is just a system that checks invariants.
// Rendering will be a system that draws components.

use crate::world::World;

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

inventory::collect!(System);

/// Run all registered systems.
pub fn run_systems(world: &mut World) {
    for system in inventory::iter::<System> {
        (system.run)(world);
    }
}

// ── Validation helpers ────────────────────────────────
//
// Two kinds of reference checks:
// 1. check_exists — entity must have been spawned (any entity)
// 2. check_has::<T> — entity must exist AND have component T

use crate::components::Validate;
use crate::id::EntityID;

/// Check that a referenced entity was spawned.
/// Use for: Frame.parent, Site.parent, StationDefinedFrame.origin, etc.
pub fn check_exists(
    world: &World,
    entity: EntityID,
    field: &str,
    reference: EntityID,
) -> Option<String> {
    if reference.0 >= world.next_id {
        Some(format!(
            "{:?} {} references non-existent entity {:?}",
            entity.0, field, reference.0
        ))
    } else {
        None
    }
}

/// Check that a referenced entity exists AND has component T.
/// Use for: Joint.body_a (needs InertialProperties), etc.
pub fn check_has<T: 'static>(
    world: &World,
    entity: EntityID,
    field: &str,
    reference: EntityID,
) -> Option<String> {
    if world.get::<T>(reference).is_none() {
        Some(format!(
            "{:?} {} references entity {:?} missing {}",
            entity.0, field, reference.0, std::any::type_name::<T>()
        ))
    } else {
        None
    }
}

/// Iterate all instances of T, call validate on each, collect errors.
pub fn validate_all<T: Validate + 'static>(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, component) in world.iter::<T>() {
        local_errors.extend(component.validate(key, world));
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

/// Print accumulated validation errors. Run this last.
pub fn print_errors(world: &mut World) {
    let errors = world.get_resource::<Vec<String>>();
    let count = errors.map_or(0, |e| e.len());
    if count == 0 {
        println!("Validation: World is valid");
    } else {
        for e in errors.unwrap() {
            println!("VALIDATION ERROR: {}", e);
        }
    }
}
