#![cfg(feature = "mujoco")]

use melosim::importer::mujoco_spec::import_mjcf_spec;
use melosim::exporter::mujoco_spec::world_to_mjcf_spec;
use melosim::components::*;
use melosim::world::World;
use bevy_ecs::prelude::Entity;
use std::path::Path;
use std::process::Command;

fn ensure_myo_sim() {
    let fixture_dir = "tests/fixtures/myo_sim";
    if !Path::new(fixture_dir).exists() {
        let status = Command::new("git")
            .args(["clone", "--depth", "1", "https://github.com/MyoHub/myo_sim.git", fixture_dir])
            .status()
            .expect("Failed to run git clone");
        assert!(status.success(), "git clone failed");
    }
}

fn infer_joint_kind(world: &World, joint_entity: Entity) -> &'static str {
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

fn collect_joint_entities(world: &World) -> Vec<Entity> {
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

fn count_joints_by_kind(world: &World, kind: &str) -> usize {
    collect_joint_entities(world).iter()
        .filter(|&&j| infer_joint_kind(world, j) == kind)
        .count()
}

fn count_sites(world: &World) -> usize {
    world.iter::<Position>().into_iter()
        .filter(|(eid, _)| {
            world.get::<InertialProperties>(*eid).is_none()
                && world.get::<Rotation>(*eid).is_none()
        })
        .count()
}

#[test]
fn test_myohand_mjspec_roundtrip() {
    ensure_myo_sim();
    let model_path = "tests/fixtures/myo_sim/hand/myohand.xml";
    let (world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoHand via MjSpec");
    let n_bodies = world.count::<InertialProperties>();
    let n_hinge = count_joints_by_kind(&world, "PinJoint");
    let n_slide = count_joints_by_kind(&world, "SlideJoint");
    let n_ball = count_joints_by_kind(&world, "BallJoint");
    let n_free = count_joints_by_kind(&world, "FreeJoint");
    let n_sites = count_sites(&world);
    let n_geoms = world.count::<DisplayGeometry>();
    let n_muscles = world.count::<Muscle>();
    let n_paths = world.count::<MusclePath>();
    println!("\nmyoHand MjSpec import:");
    println!("  Bodies: {}", n_bodies);
    println!("  Hinge joints: {}", n_hinge);
    println!("  Slide joints: {}", n_slide);
    println!("  Ball joints: {}", n_ball);
    println!("  Free joints: {}", n_free);
    println!("  Sites: {}", n_sites);
    println!("  Display geoms: {}", n_geoms);
    println!("  Muscles: {}", n_muscles);
    println!("  Muscle paths: {}", n_paths);
    println!("  Body map entries: {}", body_map.len());
    let exported_xml = world_to_mjcf_spec(&mut world, "MyoHand_v0.1.7")
        .expect("Failed to export myoHand via MjSpec");
    let tmp_path = "/tmp/melosim_myohand_roundtrip.xml";
    std::fs::write(tmp_path, &exported_xml).expect("Failed to write");
    let (world2, _) = import_mjcf_spec(tmp_path)
        .expect("Failed to re-import myoHand");
    assert_eq!(world.count::<InertialProperties>(), world2.count::<InertialProperties>(), "Body count mismatch");
    assert_eq!(count_joints_by_kind(&world, "PinJoint"), count_joints_by_kind(&world2, "PinJoint"), "Hinge count mismatch");
    assert_eq!(world.count::<Muscle>(), world2.count::<Muscle>(), "Muscle count mismatch");
    let _ = std::fs::remove_file(tmp_path);
    println!("\n  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}", n_bodies, world2.count::<InertialProperties>());
    println!("  Muscles: {} -> {}", n_muscles, world2.count::<Muscle>());
    println!("  myoHand roundtrip: PASSED");
}

#[test]
fn test_myoleg_mjspec_roundtrip() {
    ensure_myo_sim();
    let model_path = "tests/fixtures/myo_sim/leg/myolegs.xml";
    let (world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoLeg via MjSpec");
    let n_bodies = world.count::<InertialProperties>();
    let n_hinge = count_joints_by_kind(&world, "PinJoint");
    let n_muscles = world.count::<Muscle>();
    println!("\nmyoLeg MjSpec import:");
    println!("  Bodies: {}", n_bodies);
    println!("  Hinge joints: {}", n_hinge);
    println!("  Muscles: {}", n_muscles);
    println!("  Body map entries: {}", body_map.len());
    let exported_xml = world_to_mjcf_spec(&mut world, "myoLeg_v0.1")
        .expect("Failed to export myoLeg via MjSpec");
    let tmp_path = "/tmp/melosim_myoleg_roundtrip.xml";
    std::fs::write(tmp_path, &exported_xml).expect("Failed to write");
    let (world2, _) = import_mjcf_spec(tmp_path)
        .expect("Failed to re-import myoLeg");
    assert_eq!(world.count::<InertialProperties>(), world2.count::<InertialProperties>(), "Body count mismatch");
    assert_eq!(world.count::<Muscle>(), world2.count::<Muscle>(), "Muscle count mismatch");
    let _ = std::fs::remove_file(tmp_path);
    println!("\n  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}", n_bodies, world2.count::<InertialProperties>());
    println!("  Muscles: {} -> {}", n_muscles, world2.count::<Muscle>());
    println!("  myoLeg roundtrip: PASSED");
}
