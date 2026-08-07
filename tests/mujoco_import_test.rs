#![cfg(feature = "mujoco")]

use melosim::importer::mujoco::import_mjcf;
use melosim::components::*;
use melosim::world::World;
use melosim::id::EntityID;
use std::path::Path;
use std::process::Command;

/// Ensure the myo_sim test fixtures are available.
/// Clones the repo if not present.
fn ensure_myo_sim() {
    let fixture_dir = "tests/fixtures/myo_sim";
    if !Path::new(fixture_dir).exists() {
        println!("Downloading myo_sim test fixtures...");
        let status = Command::new("git")
            .args(["clone", "--depth", "1", "https://github.com/MyoHub/myo_sim.git", fixture_dir])
            .status()
            .expect("Failed to run git clone");
        assert!(status.success(), "git clone failed");
    }
}

/// Infer joint kind from coordinate/effect configuration.
fn infer_joint_kind(world: &World, joint_entity: EntityID) -> &'static str {
    let coords: Vec<EntityID> = world.children_of(joint_entity).iter()
        .filter(|&&c| world.get::<JointCoordinate>(c).is_some())
        .copied()
        .collect();
    match coords.len() {
        0 => "WeldJoint",
        1 => {
            for effect_entity in world.children_of(coords[0]) {
                if let Some(effect) = world.get::<CoordinateEffect>(effect_entity) {
                    match &effect.component {
                        TransformComponent::TranslationAlongAxis(_)
                        | TransformComponent::TranslationX
                        | TransformComponent::TranslationY
                        | TransformComponent::TranslationZ => return "SlideJoint",
                        _ => {}
                    }
                }
            }
            "PinJoint"
        }
        2 => "UniversalJoint",
        3 => "BallJoint",
        6 => "FreeJoint",
        _ => "CustomJoint",
    }
}

/// Collect all unique joint entities from the world.
fn collect_joint_entities(world: &World) -> Vec<EntityID> {
    let mut joints: Vec<EntityID> = Vec::new();
    for (coord_eid, _) in world.iter::<JointCoordinate>() {
        if let Some(co) = world.get::<ChildOf>(coord_eid) {
            let joint = co.parent;
            if !joints.contains(&joint) && world.get::<InertialProperties>(joint).is_none() {
                joints.push(joint);
            }
        }
    }
    joints
}

/// Count joints of a specific kind.
fn count_joints_by_kind(world: &World, kind: &str) -> usize {
    collect_joint_entities(world).iter()
        .filter(|&&j| infer_joint_kind(world, j) == kind)
        .count()
}

/// Count site entities (entities with Position but without InertialProperties or Rotation).
fn count_sites(world: &World) -> usize {
    world.iter::<Position>()
        .filter(|(eid, _)| {
            world.get::<InertialProperties>(*eid).is_none()
                && world.get::<Rotation>(*eid).is_none()
        })
        .count()
}

#[test]
fn test_myoelbow_import() {
    ensure_myo_sim();

    let model_path = "tests/fixtures/myo_sim/elbow/myoelbow_1dof6muscles.xml";
    let (mut world, _body_map) = import_mjcf(model_path)
        .expect("Failed to import myoelbow MJCF");

    // ── Bodies ──
    // MuJoCo body 0 = worldbody (mapped to ground entity)
    // Body 1 = full_body (root)
    // Body 2 = base (pos 0 .08 1.4)
    // Body 3 = r_humerus (mass 1.864572)
    // Body 4 = r_ulna_radius_hand (mass 1.534315)
    let n_inertials = world.count::<InertialProperties>();
    println!("Bodies (InertialProperties): {}", n_inertials);

    // ── Joints ──
    // 1 hinge joint: r_elbow_flex
    let n_hinge = count_joints_by_kind(&world, "PinJoint");
    println!("Hinge joints: {}", n_hinge);
    assert_eq!(n_hinge, 1, "Expected 1 hinge joint (r_elbow_flex)");

    // ── Coordinates ──
    let n_coords = world.count::<JointCoordinate>();
    println!("Coordinates: {}", n_coords);
    assert_eq!(n_coords, 1, "Expected 1 coordinate for the hinge joint");

    // ── Sites ──
    let n_sites = count_sites(&world);
    println!("Sites: {}", n_sites);
    assert!(n_sites > 10, "Expected many path point sites (>10), got {}", n_sites);

    // ── Display geometries ──
    let n_geoms = world.count::<DisplayGeometry>();
    println!("Display geometries: {}", n_geoms);

    // ── Muscles ──
    let n_muscles = world.count::<Muscle>();
    println!("Muscles: {}", n_muscles);
    assert_eq!(n_muscles, 6, "Expected 6 muscles");

    // ── Muscle paths ──
    let n_paths = world.count::<MusclePath>();
    println!("Muscle paths: {}", n_paths);
    assert_eq!(n_paths, 6, "Expected 6 muscle paths");

    // ── Validation ──
    let errors = world.validate();
    if !errors.is_empty() {
        for e in &errors {
            println!("VALIDATION: {}", e);
        }
    }

    println!("\nImport summary:");
    println!("  Bodies: {}", n_inertials);
    println!("  Hinge joints: {}", n_hinge);
    println!("  Coordinates: {}", n_coords);
    println!("  Sites: {}", n_sites);
    println!("  Display geoms: {}", n_geoms);
    println!("  Muscles: {}", n_muscles);
    println!("  Muscle paths: {}", n_paths);
    println!("  Total entities: {}", world.next_id);
}
