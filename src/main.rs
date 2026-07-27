use melosim::components::*;
use melosim::id::EntityKey;
use melosim::math::{Transform, Vec3};
use melosim::system::SystemRegistry;
use melosim::validate;
use melosim::world::World;
use slotmap::Key;

// ── Example FK systems ────────────────────────────────
// Each reads only the concrete types it needs.

fn hinge_fk_system(world: &mut World) {
    for (_key, _hinge) in world.iter::<HingeJoint>() {
        // stub
    }
}

fn ball_fk_system(world: &mut World) {
    for (_key, _ball) in world.iter::<BallJoint>() {
        // stub
    }
}

fn free_fk_system(world: &mut World) {
    for (_key, _free) in world.iter::<FreeJoint>() {
        // stub
    }
}

// ── Example: custom joint from a downstream crate ─────
// No changes to melosim core.

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct PrismaticJoint {
    body_a: EntityKey,
    body_b: EntityKey,
    limits: Option<JointLimits>,
    axis: [f64; 3],
}

fn prismatic_system(world: &mut World) {
    for (_key, _prismatic) in world.iter::<PrismaticJoint>() {
        // stub
    }
}

// Custom validation for the downstream crate's type.
// Registered the same way — no core changes needed.
fn validate_prismatic(world: &mut World) {
    let mut local_errors = Vec::new();
    for (key, prismatic) in world.iter::<PrismaticJoint>() {
        if world.get::<InertialProperties>(prismatic.body_a).is_none() {
            local_errors.push(format!(
                "PrismaticJoint {:?}: missing body_a {:?}",
                key.data().as_ffi(),
                prismatic.body_a.data().as_ffi()
            ));
        }
    }
    let errors = world.get_resource_or_default::<Vec<String>>();
    errors.extend(local_errors);
}

// ── Main ──────────────────────────────────────────────

fn main() {
    let mut world = World::new();

    // ── Create bodies ──
    let ground = world.insert(InertialProperties {
        mass: 0.0,
        com: [0.0, 0.0, 0.0],
        inertia: [0.0; 6],
    });

    let pelvis = world.insert(InertialProperties {
        mass: 11.78,
        com: [0.0, 0.0, 0.0],
        inertia: [0.18, 0.22, 0.20, 0.0, 0.0, 0.0],
    });
    world.insert(Frame {
        parent: ground,
        transform: Transform::default(),
    });

    let femur = world.insert(InertialProperties {
        mass: 9.3,
        com: [0.0, -0.17, 0.0],
        inertia: [0.12, 0.12, 0.02, 0.0, 0.0, 0.0],
    });

    // ── Create joints ──
    let _hip = world.insert(HingeJoint {
        body_a: pelvis,
        body_b: femur,
        limits: Some(JointLimits {
            lower: -2.0,
            upper: 2.0,
        }),
        axis: [1.0, 0.0, 0.0],
    });

    let _pelvis_free = world.insert(FreeJoint {
        body_a: ground,
        body_b: pelvis,
        limits: None,
    });

    // ── Create a site ──
    let _asis = world.insert(Site {
        parent: pelvis,
        offset: Vec3::new(0.01, 0.02, 0.13),
    });

    // ── Register systems ──
    let mut registry = SystemRegistry::new();

    // Validation phase: run before FK
    registry.add("validate_hinge", validate::validate_hinge);
    registry.add("validate_ball", validate::validate_ball);
    registry.add("validate_slide", validate::validate_slide);
    registry.add("validate_free", validate::validate_free);
    registry.add("validate_fixed", validate::validate_fixed);
    registry.add("validate_frame", validate::validate_frame);
    registry.add("validate_site", validate::validate_site);
    // Downstream crate's validation — same registry, no core change
    registry.add("validate_prismatic", validate_prismatic);

    // FK phase
    registry.add("hinge_fk", hinge_fk_system);
    registry.add("ball_fk", ball_fk_system);
    registry.add("free_fk", free_fk_system);
    registry.add("prismatic_fk", prismatic_system);

    // Print errors last
    registry.add("print_errors", validate::print_errors);

    println!("Registered {} systems:\n  {:?}", registry.len(), registry);

    // ── Run systems in order ──
    registry.run(&mut world);

    // ── Summary ──
    println!("{:?}", world);
}
