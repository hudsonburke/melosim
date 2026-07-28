// ── Per-type validation systems ───────────────────────
// Each validation system checks one component type's invariants
// using local error accumulation to avoid borrow conflicts,
// then merges into the shared Vec<String> resource.
//
// A downstream crate adds its own validation by writing a system
// and registering it — no changes to melosim core.

use crate::components::*;
use crate::id::EntityID;
use crate::world::World;

/// Check that a body reference exists. Returns an error string if missing.
fn check_body(
    world: &World,
    entity: EntityID,
    label: &str,
    body: EntityID,
) -> Option<String> {
    if world.get::<InertialProperties>(body).is_none() {
        Some(format!(
            "{:?} {} references missing body {:?}",
            entity.0, label, body.0
        ))
    } else {
        None
    }
}

// ── Joint validation ──

pub fn validate_hinge(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, hinge) in world.iter::<HingeJoint>() {
        if let Some(err) = check_body(world, key, "HingeJoint body_a", hinge.body_a) {
            local_errors.push(err);
        }
        if let Some(err) = check_body(world, key, "HingeJoint body_b", hinge.body_b) {
            local_errors.push(err);
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

pub fn validate_slide(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, slide) in world.iter::<SlideJoint>() {
        if let Some(err) = check_body(world, key, "SlideJoint body_a", slide.body_a) {
            local_errors.push(err);
        }
        if let Some(err) = check_body(world, key, "SlideJoint body_b", slide.body_b) {
            local_errors.push(err);
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

pub fn validate_ball(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, ball) in world.iter::<BallJoint>() {
        if let Some(err) = check_body(world, key, "BallJoint body_a", ball.body_a) {
            local_errors.push(err);
        }
        if let Some(err) = check_body(world, key, "BallJoint body_b", ball.body_b) {
            local_errors.push(err);
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

pub fn validate_free(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, free) in world.iter::<FreeJoint>() {
        if let Some(err) = check_body(world, key, "FreeJoint body_a", free.body_a) {
            local_errors.push(err);
        }
        if let Some(err) = check_body(world, key, "FreeJoint body_b", free.body_b) {
            local_errors.push(err);
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

pub fn validate_fixed(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, fixed) in world.iter::<FixedJoint>() {
        if let Some(err) = check_body(world, key, "FixedJoint body_a", fixed.body_a) {
            local_errors.push(err);
        }
        if let Some(err) = check_body(world, key, "FixedJoint body_b", fixed.body_b) {
            local_errors.push(err);
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── UniversalJoint validation ──

pub fn validate_universal(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, univ) in world.iter::<UniversalJoint>() {
        if let Some(err) = check_body(world, key, "UniversalJoint body_a", univ.body_a) {
            local_errors.push(err);
        }
        if let Some(err) = check_body(world, key, "UniversalJoint body_b", univ.body_b) {
            local_errors.push(err);
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── CustomJoint validation ──

pub fn validate_custom(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, custom) in world.iter::<CustomJoint>() {
        if let Some(err) = check_body(world, key, "CustomJoint body_a", custom.body_a) {
            local_errors.push(err);
        }
        if let Some(err) = check_body(world, key, "CustomJoint body_b", custom.body_b) {
            local_errors.push(err);
        }
        for (i, coord_key) in custom.coordinates.iter().enumerate() {
            if world.get::<JointCoordinate>(*coord_key).is_none() {
                local_errors.push(format!(
                    "{:?} CustomJoint coordinate[{}] {:?} references missing JointCoordinate",
                    key.0, i, coord_key.0
                ));
            }
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── Coordinate validation ──

pub fn validate_coordinate(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, coord) in world.iter::<JointCoordinate>() {
        if coord.clamped && coord.range_min > coord.range_max {
            local_errors.push(format!(
                "{:?} JointCoordinate '{}' has invalid range [{}, {}]",
                key.0, coord.name, coord.range_min, coord.range_max
            ));
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── CoordinateEffect validation ──

pub fn validate_coordinate_effect(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, effect) in world.iter::<CoordinateEffect>() {
        if world.get::<JointCoordinate>(effect.coordinate).is_none() {
            local_errors.push(format!(
                "{:?} CoordinateEffect references missing coordinate {:?}",
                key.0, effect.coordinate.0
            ));
        }
        if world.get::<CustomJoint>(effect.joint).is_none()
            && world.get::<HingeJoint>(effect.joint).is_none()
            && world.get::<UniversalJoint>(effect.joint).is_none()
        {
            local_errors.push(format!(
                "{:?} CoordinateEffect references missing joint {:?}",
                key.0, effect.joint.0
            ));
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── SpatialTransform validation ──

pub fn validate_spatial_transform(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, st) in world.iter::<SpatialTransform>() {
        if world.get::<CustomJoint>(st.joint).is_none() {
            local_errors.push(format!(
                "{:?} SpatialTransform references missing CustomJoint {:?}",
                key.0, st.joint.0
            ));
        }
        for (i, effect_key) in st.effects.iter().enumerate() {
            if world.get::<CoordinateEffect>(*effect_key).is_none() {
                local_errors.push(format!(
                    "{:?} SpatialTransform effect[{}] {:?} references missing CoordinateEffect",
                    key.0, i, effect_key.0
                ));
            }
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── Frame validation ──

pub fn validate_frame(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, frame) in world.iter::<Frame>() {
        if world.get::<InertialProperties>(frame.parent).is_none() {
            local_errors.push(format!(
                "Frame {:?} references missing parent {:?}",
                key.0, frame.parent.0
            ));
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── Site validation ──

pub fn validate_site(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, site) in world.iter::<Site>() {
        if world.get::<InertialProperties>(site.parent).is_none() {
            local_errors.push(format!(
                "Site {:?} references missing parent {:?}",
                key.0, site.parent.0
            ));
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── Print accumulated errors ──
// Run this last to show all validation results.

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
