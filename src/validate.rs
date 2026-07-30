// ── Generic validation infrastructure ─────────────────
//
// Components implement `Validate` to define their own invariants.
// The generic `validate_all::<T>()` iterates all instances of T
// and collects errors into the world's error resource.
//
// To add validation for a new component:
//   1. impl Validate for YourComponent { ... }
//   2. registry.add("validate_your", |w| validate_all::<YourComponent>(w));

use crate::components::Validate;
use crate::world::World;

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
