#![cfg(feature = "mujoco")]

use melosim::importer::mujoco::import_mjcf;
use melosim::components::*;
use melosim::world::World;
use melosim::world::WorldExt;
use bevy_ecs::prelude::Entity;
use std::path::Path;
use std::process::Command;

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

fn infer_joint_kind(world: &mut World, joint_entity: Entity) -> &'static str {
    let coords: Vec<Entity> = world.children_of(joint_entity).iter()
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

fn collect_joint_entities(world: &mut World) -> Vec<Entity> {
    let mut joints: Vec<Entity> = Vec::new();
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

fn count_joints_by_kind(world: &mut World, kind: &str) -> usize {
    collect_joint_entities(world).iter()
        .filter(|&&j| infer_joint_kind(world, j) == kind)
        .count()
}

fn count_sites(world: &mut World) -> usize {
    world.iter::<Position>().into_iter()
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

    let n_inertials = world.count::<InertialProperties>();
    println!("Bodies (InertialProperties): {}", n_inertials);
    let n_hinge = count_joints_by_kind(&mut world, "PinJoint");
    println!("Hinge joints: {}", n_hinge);
    assert_eq!(n_hinge, 1, "Expected 1 hinge joint (r_elbow_flex)");
    let n_coords = world.count::<JointCoordinate>();
    println!("Coordinates: {}", n_coords);
    assert_eq!(n_coords, 1, "Expected 1 coordinate for the hinge joint");
    let n_sites = count_sites(&mut world);
    println!("Sites: {}", n_sites);
    assert!(n_sites > 10, "Expected many path point sites (>10), got {}", n_sites);
    let n_geoms = world.count::<DisplayGeometry>();
    println!("Display geometries: {}", n_geoms);
    let n_muscles = world.count::<Muscle>();
    println!("Muscles: {}", n_muscles);
    assert_eq!(n_muscles, 6, "Expected 6 muscles");
    let n_paths = world.count::<MusclePath>();
    println!("Muscle paths: {}", n_paths);
    assert_eq!(n_paths, 6, "Expected 6 muscle paths");
    let errors = world.validate();
    if !errors.is_empty() {
        for e in &errors { println!("VALIDATION: {}", e); }
    }
    println!("\nImport summary:");
    println!("  Bodies: {}", n_inertials);
    println!("  Hinge joints: {}", n_hinge);
    println!("  Coordinates: {}", n_coords);
    println!("  Sites: {}", n_sites);
    println!("  Display geoms: {}", n_geoms);
    println!("  Muscles: {}", n_muscles);
    println!("  Muscle paths: {}", n_paths);
}
