use melosim::components::*;
use melosim::id::EntityKey;
use melosim::math::{Transform, Vec3};
use melosim::system::SystemRegistry;
use melosim::world::World;

// ── Example systems ───────────────────────────────────
// Each system reads only the concrete types it needs.
// Systems iterate a single component type — the join between
// component types happens via EntityKey references in fields.

fn hinge_fk_system(world: &mut World) {
    for (_key, hinge) in world.iter::<HingeJoint>() {
        // In a real FK solver:
        //   - read joint position from state (not shown)
        //   - compute transform from hinge axis + angle
        //   - look up body_b's Frame and apply the transform
        let _ = hinge; // stub
    }
}

fn ball_fk_system(world: &mut World) {
    for (_key, ball) in world.iter::<BallJoint>() {
        // Ball joint: rotation about any axis through the joint origin
        let _ = ball; // stub
    }
}

fn free_fk_system(world: &mut World) {
    for (_key, free) in world.iter::<FreeJoint>() {
        // Free joint: 6-DOF motion
        let _ = free; // stub
    }
}

// ── Example: custom joint from a downstream crate ─────
// No changes to melosim core. Define the struct + system + register.

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct PrismaticJoint {
    body_a: EntityKey,
    body_b: EntityKey,
    limits: Option<JointLimits>,
    axis: [f64; 3],
}

fn prismatic_system(world: &mut World) {
    for (_key, _prismatic) in world.iter::<PrismaticJoint>() {
        // Prismatic: translation along axis
    }
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
    // Each joint is a single component referencing body_a and body_b.
    let _hip = world.insert(HingeJoint {
        body_a: pelvis,
        body_b: femur,
        limits: Some(JointLimits {
            lower: -2.0,
            upper: 2.0,
        }),
        axis: [1.0, 0.0, 0.0], // flexion/extension
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
    registry.add("hinge_fk", hinge_fk_system);
    registry.add("ball_fk", ball_fk_system);
    registry.add("free_fk", free_fk_system);
    // Custom joint from downstream crate — registered the same way:
    registry.add("prismatic_fk", prismatic_system);

    println!("Registered systems: {:?}", registry);

    // ── Run systems ──
    registry.run(&mut world);

    // ── Validate ──
    let errors = world.validate();
    if errors.is_empty() {
        println!("World is valid");
    } else {
        for e in &errors {
            println!("ERROR: {}", e);
        }
    }

    // ── Summary ──
    println!("{:?}", world);
}
