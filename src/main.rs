use melosim::components::*;
use melosim::id::EntityKey;
use melosim::math::{Transform, Vec3};
use melosim::system::SystemRegistry;
use melosim::validate;
use melosim::world::World;
use slotmap::Key;

// ── Example FK systems ────────────────────────────────

#[allow(dead_code)]
fn hinge_fk_system(_world: &mut World) {}
#[allow(dead_code)]
fn ball_fk_system(_world: &mut World) {}
#[allow(dead_code)]
fn free_fk_system(_world: &mut World) {}

// ── Example: custom joint from a downstream crate ─────

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
struct PrismaticJoint {
    body_a: EntityKey,
    body_b: EntityKey,
    limits: Option<JointLimits>,
    axis: [f64; 3],
}

#[allow(dead_code)]
fn prismatic_system(_world: &mut World) {}

#[allow(dead_code)]
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

// ── FlatWorld solver: runs on frozen snapshot ─────────
// No AnyMap lookups, no SlotMap generation checks.
// Direct Vec indexing: flat.inertials[id]

fn solve_fk(flat: &melosim::flat::FlatWorld) {
    for (id, hinge) in flat.iter::<HingeJoint>() {
        // flat.inertials[id.as_usize()] — single load
        let _body_b = hinge.body_b;
        let _id = id;
    }
}

// ── Main ──────────────────────────────────────────────

fn main() {
    // ── Phase 1: Build World (extensible, dynamic) ──
    let mut world = World::new();

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

    let _hip = world.insert(HingeJoint {
        body_a: pelvis,
        body_b: femur,
        limits: Some(JointLimits { lower: -2.0, upper: 2.0 }),
        axis: [1.0, 0.0, 0.0],
    });

    let _pelvis_free = world.insert(FreeJoint {
        body_a: ground,
        body_b: pelvis,
        limits: None,
    });

    let _asis = world.insert(Site {
        parent: pelvis,
        offset: Vec3::new(0.01, 0.02, 0.13),
    });

    // Register and run validation systems
    let mut registry = SystemRegistry::new();
    registry.add("validate_hinge", validate::validate_hinge);
    registry.add("validate_ball", validate::validate_ball);
    registry.add("validate_free", validate::validate_free);
    registry.add("validate_frame", validate::validate_frame);
    registry.add("validate_site", validate::validate_site);
    registry.add("print_errors", validate::print_errors);
    registry.run(&mut world);

    println!("\nBuild World:\n  {:?}", world);
    println!("  component count: {}", world.components.len());

    // ── Phase 2: Freeze → FlatWorld (dense, GPU-ready) ──
    let flat = world.freeze();

    println!("\nFlatWorld snapshot:");
    println!("  entities: {}", flat.len());
    println!("  inertials: {}", flat.inertials.iter().filter_map(|x| x.as_ref()).count());
    println!("  frames: {}", flat.frames.iter().filter_map(|x| x.as_ref()).count());
    println!("  hinge_joints: {}", flat.hinge_joints.iter().filter_map(|x| x.as_ref()).count());

    // Demonstrate dense indexing
    if let Some(hip) = &flat.hinge_joints[0] {
        println!("\nHinge at index 0: body_a={:?}, axis={:?}", hip.body_a, hip.axis);
    }

    // ── Simulate on FlatWorld ──
    solve_fk(&flat);

    // ── Custom type example (downstream) ──
    // Build phase: insert into World
    world.insert(PrismaticJoint {
        body_a: pelvis,
        body_b: femur,
        limits: None,
        axis: [0.0, 1.0, 0.0],
    });

    // Freeze again (or freeze once after all insertions)
    let flat2 = world.freeze();
    // Custom types are not in the built-in Vecs — they go in extensions
    // flat2.extensions.insert::<Vec<Option<PrismaticJoint>>>(...);
    // For now, downstream solvers iterate via flat2.extensions:
    //   let joints: &Vec<Option<PrismaticJoint>> = flat2.extensions.get().unwrap();

    println!("\nFlatWorld with custom prismatic:");
    println!("  hinge_joints: {}, prismatic: (in extensions)", 
        flat2.hinge_joints.iter().filter_map(|x| x.as_ref()).count());
    println!("  extensions: {} types registered", flat2.extensions.len());
}
