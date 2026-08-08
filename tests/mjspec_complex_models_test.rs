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
fn test_mjspec_import() {
    ensure_myo_sim();
    let model_path = "tests/fixtures/myo_sim/elbow/myoelbow_1dof6muscles.xml";
    let (mut world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoelbow via MjSpec");
    let n_bodies = world.query::<&InertialProperties>().iter(&world).count();
    let n_hinge = count_joints_by_kind(&mut world, "PinJoint");
    let n_sites = count_sites(&mut world);
    let n_muscles = world.query::<&Muscle>().iter(&world).count();
    let n_paths = world.query::<&MusclePath>().iter(&world).count();
    println!("MjSpec import:");
    println!("  Bodies: {}", n_bodies);
    println!("  Hinge joints: {}", n_hinge);
    println!("  Sites: {}", n_sites);
    println!("  Muscles: {}", n_muscles);
    println!("  Muscle paths: {}", n_paths);
    println!("  Body map entries: {}", body_map.len());
    assert_eq!(n_hinge, 1, "Expected 1 hinge joint");
    assert_eq!(n_muscles, 6, "Expected 6 muscles");
    assert!(n_sites > 10, "Expected many sites");
}

#[test]
fn test_mjspec_lossless_roundtrip() {
    ensure_myo_sim();
    let model_path = "tests/fixtures/myo_sim/elbow/myoelbow_1dof6muscles.xml";
    let original_xml = std::fs::read_to_string(model_path)
        .expect("Failed to read original MJCF");
    let (mut world, _) = import_mjcf_spec(model_path).expect("Failed to import");
    let exported_xml = world_to_mjcf_spec(&mut world, "MyoElbow_v0.1.7")
        .expect("Failed to export via MjSpec");
    let tmp_path = "/tmp/melosim_mjspec_roundtrip.xml";
    std::fs::write(tmp_path, &exported_xml).expect("Failed to write");
    let (mut world2, _) = import_mjcf_spec(tmp_path)
        .expect(&format!("Failed to re-import exported MJCF.\nExported:\n{}", &exported_xml[..2000]));
    assert_eq!(world.query::<&InertialProperties>().iter(&world).count(), world2.query::<&InertialProperties>().iter(&world2).count(), "Body count mismatch after roundtrip");
    assert_eq!(count_joints_by_kind(&mut world, "PinJoint"), count_joints_by_kind(&mut world2, "PinJoint"), "Hinge joint count mismatch");
    assert_eq!(world.query::<&Muscle>().iter(&world).count(), world2.query::<&Muscle>().iter(&world2).count(), "Muscle count mismatch");
    println!("Exported XML (first 2000 chars):");
    println!("{}", &exported_xml[..exported_xml.len().min(2000)]);
    assert!(exported_xml.contains("<mujoco"), "Missing <mujoco> root");
    assert!(exported_xml.contains("<worldbody>"), "Missing <worldbody>");
    assert!(exported_xml.contains("<actuator>"), "Missing <actuator>");
    let _ = std::fs::remove_file(tmp_path);
    println!("\nMjSpec lossless roundtrip:");
    println!("  Original: {} bytes", original_xml.len());
    println!("  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}", world.query::<&InertialProperties>().iter(&world).count(), world2.query::<&InertialProperties>().iter(&world2).count());
    println!("  Muscles: {} -> {}", world.query::<&Muscle>().iter(&world).count(), world2.query::<&Muscle>().iter(&world2).count());
    println!("  PASSED");
}
