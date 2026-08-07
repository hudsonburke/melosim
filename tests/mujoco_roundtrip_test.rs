#![cfg(feature = "mujoco")]

use melosim::importer::mujoco::import_mjcf;
use melosim::exporter::mujoco::world_to_mjcf;
use melosim::components::*;
use melosim::world::World;
use melosim::id::EntityID;
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

/// Find all hinge joint entities and their names.
fn find_hinge_joints(world: &World) -> Vec<(EntityID, Option<String>)> {
    collect_joint_entities(world).iter()
        .filter(|&&j| infer_joint_kind(world, j) == "PinJoint")
        .map(|&j| (j, world.get::<Name>(j).map(|n| n.value.clone())))
        .collect()
}

#[test]
fn test_myoelbow_export() {
    ensure_myo_sim();

    let model_path = "tests/fixtures/myo_sim/elbow/myoelbow_1dof6muscles.xml";
    let (world, _) = import_mjcf(model_path).expect("Failed to import");

    let xml = world_to_mjcf(&world, "MyoElbow_export");

    assert!(xml.contains("<mujoco model=\"MyoElbow_export\">"));
    assert!(xml.contains("<worldbody>"));
    assert!(xml.contains("<tendon>"));
    assert!(xml.contains("<actuator>"));
    assert!(xml.contains("<muscle name=\"TRIlong\""));
    assert!(xml.contains("<muscle name=\"BRA\""));
    assert!(xml.contains("<joint name=\"r_elbow_flex\""));
    assert!(xml.contains("type=\"hinge\""));
    assert!(xml.contains("axis=\""));

    println!("Exported MJCF: {} bytes", xml.len());
}

#[test]
fn test_myoelbow_roundtrip_structural() {
    ensure_myo_sim();

    let model_path = "tests/fixtures/myo_sim/elbow/myoelbow_1dof6muscles.xml";
    let (world1, _) = import_mjcf(model_path).expect("Failed to import");

    let xml = world_to_mjcf(&world1, "MyoElbow_roundtrip");
    let tmp_path = "/tmp/melosim_roundtrip_test.xml";
    std::fs::write(tmp_path, &xml).expect("Failed to write temp file");

    let (world2, _) = import_mjcf(tmp_path).expect(
        &format!("Failed to re-import exported MJCF.\nExported XML:\n{}", xml)
    );

    // ── Structural comparison ──
    let checks = [
        ("Bodies", world1.count::<InertialProperties>(),
                     world2.count::<InertialProperties>()),
        ("Hinge joints", count_joints_by_kind(&world1, "PinJoint"),
                            count_joints_by_kind(&world2, "PinJoint")),
        ("Coordinates", world1.count::<JointCoordinate>(),
                           world2.count::<JointCoordinate>()),
        ("Muscles", world1.count::<Muscle>(),
                       world2.count::<Muscle>()),
        ("Muscle paths", world1.count::<MusclePath>(),
                            world2.count::<MusclePath>()),
    ];

    for (label, c1, c2) in &checks {
        println!("  {}: {} -> {}", label, c1, c2);
        assert_eq!(c1, c2, "{} count mismatch: {} vs {}", label, c1, c2);
    }

    // Sites may differ (export only emits sites with unique positions)
    let n_sites1 = count_sites(&world1);
    let n_sites2 = count_sites(&world2);
    println!("  Sites: {} -> {} (may differ due to deduplication)", n_sites1, n_sites2);

    // ── Joint axis comparison ──
    let hinges1 = find_hinge_joints(&world1);
    let hinges2 = find_hinge_joints(&world2);

    for (key1, name1) in &hinges1 {
        // Get the coordinate entity for this joint
        let coord1 = world1.children_of(*key1).iter()
            .find(|&&c| world1.get::<JointCoordinate>(c).is_some())
            .copied();

        // Find axis from RotationAboutAxis effect
        let mut axis1 = [0.0f64; 3];
        if let Some(coord_key) = coord1 {
            for effect_entity in world1.children_of(coord_key) {
                if let Some(effect) = world1.get::<CoordinateEffect>(effect_entity) {
                    if let TransformComponent::RotationAboutAxis(a) = effect.component {
                        axis1 = a;
                    }
                }
            }
        }

        for (key2, name2) in &hinges2 {
            if name1 == name2 {
                let coord2 = world2.children_of(*key2).iter()
                    .find(|&&c| world2.get::<JointCoordinate>(c).is_some())
                    .copied();

                let mut axis2 = [0.0f64; 3];
                if let Some(coord_key) = coord2 {
                    for effect_entity in world2.children_of(coord_key) {
                        if let Some(effect) = world2.get::<CoordinateEffect>(effect_entity) {
                            if let TransformComponent::RotationAboutAxis(a) = effect.component {
                                axis2 = a;
                            }
                        }
                    }
                }

                let axis_diff: f64 = axis1.iter().zip(axis2.iter())
                    .map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt();
                assert!(axis_diff < 1e-6,
                    "Joint '{:?}': axis mismatch {:?} vs {:?}", name1, axis1, axis2);
                println!("  Joint '{:?}': axis preserved", name1);
            }
        }
    }

    // ── Joint limits comparison ──
    for (key1, name1) in &hinges1 {
        let coord1 = world1.children_of(*key1).iter()
            .find(|&&c| world1.get::<JointCoordinate>(c).is_some())
            .copied()
            .and_then(|c| world1.get::<JointCoordinate>(c));

        for (key2, name2) in &hinges2 {
            if name1 == name2 {
                let coord2 = world2.children_of(*key2).iter()
                    .find(|&&c| world2.get::<JointCoordinate>(c).is_some())
                    .copied()
                    .and_then(|c| world2.get::<JointCoordinate>(c));

                match (coord1, coord2) {
                    (Some(c1), Some(c2)) => {
                        let lower_diff = (c1.range_min - c2.range_min).abs();
                        let upper_diff = (c1.range_max - c2.range_max).abs();
                        if lower_diff > 1e-3 || upper_diff > 1e-3 {
                            println!("  Joint '{:?}': LIMIT MISMATCH: [{}, {}] vs [{}, {}] (diff: {}, {})",
                                name1, c1.range_min, c1.range_max, c2.range_min, c2.range_max, lower_diff, upper_diff);
                        }
                        assert!(lower_diff < 1e-3,
                            "Joint '{:?}': lower limit mismatch {} vs {}", name1, c1.range_min, c2.range_min);
                        assert!(upper_diff < 1e-3,
                            "Joint '{:?}': upper limit mismatch {} vs {}", name1, c1.range_max, c2.range_max);
                        println!("  Joint '{:?}': limits [{}, {}] preserved", name1, c1.range_min, c1.range_max);
                    }
                    (None, None) => println!("  Joint '{:?}': no limits (preserved)", name1),
                    _ => panic!("Joint '{:?}': limits presence mismatch", name1),
                }
            }
        }
    }

    // ── Coordinate damping/stiffness comparison ──
    for (key1, coord1) in world1.iter::<JointCoordinate>() {
        let name1 = world1.get::<Name>(key1).map(|n| n.value.clone());
        for (key2, coord2) in world2.iter::<JointCoordinate>() {
            let name2 = world2.get::<Name>(key2).map(|n| n.value.clone());
            if name1 == name2 {
                assert!((coord1.damping - coord2.damping).abs() < 1e-6,
                    "Coord '{:?}': damping mismatch", name1);
                println!("  Coord '{:?}': damping={} preserved", name1, coord1.damping);
            }
        }
    }

    // ── Muscle name preservation ──
    let mut muscle_names1: Vec<String> = world1.iter::<Muscle>()
        .filter_map(|(k, _)| world1.get::<Name>(k).map(|n| n.value.clone()))
        .collect();
    let mut muscle_names2: Vec<String> = world2.iter::<Muscle>()
        .filter_map(|(k, _)| world2.get::<Name>(k).map(|n| n.value.clone()))
        .collect();
    muscle_names1.sort();
    muscle_names2.sort();
    assert_eq!(muscle_names1, muscle_names2, "Muscle names mismatch");
    println!("  Muscles: {:?} preserved", muscle_names1);

    let _ = std::fs::remove_file(tmp_path);

    println!("\nStructural roundtrip: PASSED");
}
