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

fn children_of(world: &mut World, entity: Entity) -> Vec<Entity> {
    let mut query = world.query::<(Entity, &ChildOf)>();
    query.iter(world)
        .filter(|(_, co)| co.parent == entity)
        .map(|(e, _)| e)
        .collect()
}

fn infer_joint_kind(world: &mut World, joint_entity: Entity) -> &'static str {
    let child_entities = children_of(world, joint_entity);
    let coords: Vec<Entity> = child_entities.iter()
        .filter(|&&c| world.get::<JointCoordinate>(c).is_some())
        .copied()
        .collect();
    match coords.len() {
        0 => "WeldJoint",
        1 => {
            let coord_children = children_of(world, coords[0]);
            for effect_entity in coord_children {
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
    let coord_entities: Vec<(Entity, JointCoordinate)> = {
        let mut query = world.query::<(Entity, &JointCoordinate)>();
        query.iter(world).map(|(e, c)| (e, c.clone())).collect()
    };
    for (coord_eid, _) in coord_entities {
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
    let site_entities: Vec<(Entity, Position)> = {
        let mut query = world.query::<(Entity, &Position)>();
        query.iter(world).map(|(e, p)| (e, p.clone())).collect()
    };
    site_entities.iter()
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
    let (mut world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoHand via MjSpec");
    let n_bodies = world.query::<&InertialProperties>().iter(&world).count();
    let n_hinge = count_joints_by_kind(&mut world, "PinJoint");
    let n_slide = count_joints_by_kind(&mut world, "SlideJoint");
    let n_ball = count_joints_by_kind(&mut world, "BallJoint");
    let n_free = count_joints_by_kind(&mut world, "FreeJoint");
    let n_sites = count_sites(&mut world);
    let n_geoms = world.query::<&DisplayGeometry>().iter(&world).count();
    let n_muscles = world.query::<&Muscle>().iter(&world).count();
    let n_paths = world.query::<&MusclePath>().iter(&world).count();
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
    let (mut world2, _) = import_mjcf_spec(tmp_path)
        .expect("Failed to re-import myoHand");
    assert_eq!(world.query::<&InertialProperties>().iter(&world).count(), world2.query::<&InertialProperties>().iter(&world2).count(), "Body count mismatch");
    assert_eq!(count_joints_by_kind(&mut world, "PinJoint"), count_joints_by_kind(&mut world2, "PinJoint"), "Hinge count mismatch");
    assert_eq!(world.query::<&Muscle>().iter(&world).count(), world2.query::<&Muscle>().iter(&world2).count(), "Muscle count mismatch");
    let _ = std::fs::remove_file(tmp_path);
    println!("\n  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}", n_bodies, world2.query::<&InertialProperties>().iter(&world2).count());
    println!("  Muscles: {} -> {}", n_muscles, world2.query::<&Muscle>().iter(&world2).count());
    println!("  myoHand roundtrip: PASSED");
}

#[test]
fn test_myoleg_mjspec_roundtrip() {
    ensure_myo_sim();
    let model_path = "tests/fixtures/myo_sim/leg/myolegs.xml";
    let (mut world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoLeg via MjSpec");
    let n_bodies = world.query::<&InertialProperties>().iter(&world).count();
    let n_hinge = count_joints_by_kind(&mut world, "PinJoint");
    let n_muscles = world.query::<&Muscle>().iter(&world).count();
    println!("\nmyoLeg MjSpec import:");
    println!("  Bodies: {}", n_bodies);
    println!("  Hinge joints: {}", n_hinge);
    println!("  Muscles: {}", n_muscles);
    println!("  Body map entries: {}", body_map.len());
    let exported_xml = world_to_mjcf_spec(&mut world, "myoLeg_v0.1")
        .expect("Failed to export myoLeg via MjSpec");
    let tmp_path = "/tmp/melosim_myoleg_roundtrip.xml";
    std::fs::write(tmp_path, &exported_xml).expect("Failed to write");
    let (mut world2, _) = import_mjcf_spec(tmp_path)
        .expect("Failed to re-import myoLeg");
    assert_eq!(world.query::<&InertialProperties>().iter(&world).count(), world2.query::<&InertialProperties>().iter(&world2).count(), "Body count mismatch");
    assert_eq!(world.query::<&Muscle>().iter(&world).count(), world2.query::<&Muscle>().iter(&world2).count(), "Muscle count mismatch");
    let _ = std::fs::remove_file(tmp_path);
    println!("\n  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}", n_bodies, world2.query::<&InertialProperties>().iter(&world2).count());
    println!("  Muscles: {} -> {}", n_muscles, world2.query::<&Muscle>().iter(&world2).count());
    println!("  myoLeg roundtrip: PASSED");
}
