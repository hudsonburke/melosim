use melosim::importer::mujoco::import_mjcf;
use melosim::exporter::mujoco::world_to_mjcf;
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
        ("Bodies", world1.count::<melosim::components::InertialProperties>(),
                     world2.count::<melosim::components::InertialProperties>()),
        ("Hinge joints", world1.count::<melosim::components::HingeJoint>(),
                            world2.count::<melosim::components::HingeJoint>()),
        ("Coordinates", world1.count::<melosim::components::JointCoordinate>(),
                           world2.count::<melosim::components::JointCoordinate>()),
        ("Muscles", world1.count::<melosim::components::Muscle>(),
                       world2.count::<melosim::components::Muscle>()),
        ("Muscle paths", world1.count::<melosim::components::MusclePath>(),
                            world2.count::<melosim::components::MusclePath>()),
    ];

    for (label, c1, c2) in &checks {
        println!("  {}: {} -> {}", label, c1, c2);
        assert_eq!(c1, c2, "{} count mismatch: {} vs {}", label, c1, c2);
    }

    // Sites may differ (export only emits sites with unique positions)
    let n_sites1 = world1.count::<melosim::components::Site>();
    let n_sites2 = world2.count::<melosim::components::Site>();
    println!("  Sites: {} -> {} (may differ due to deduplication)", n_sites1, n_sites2);

    // ── Joint axis comparison ──
    for (key1, hinge1) in world1.iter::<melosim::components::HingeJoint>() {
        let name1 = world1.get::<melosim::components::Name>(key1).map(|n| n.value.clone());
        for (key2, hinge2) in world2.iter::<melosim::components::HingeJoint>() {
            let name2 = world2.get::<melosim::components::Name>(key2).map(|n| n.value.clone());
            if name1 == name2 {
                let axis_diff: f64 = hinge1.axis.iter().zip(hinge2.axis.iter())
                    .map(|(a, b)| (a - b).powi(2)).sum::<f64>().sqrt();
                assert!(axis_diff < 1e-6,
                    "Joint '{:?}': axis mismatch {:?} vs {:?}", name1, hinge1.axis, hinge2.axis);
                println!("  Joint '{:?}': axis preserved", name1);
            }
        }
    }

    // ── Joint limits comparison ──
    for (key1, hinge1) in world1.iter::<melosim::components::HingeJoint>() {
        let name1 = world1.get::<melosim::components::Name>(key1).map(|n| n.value.clone());
        for (key2, hinge2) in world2.iter::<melosim::components::HingeJoint>() {
            let name2 = world2.get::<melosim::components::Name>(key2).map(|n| n.value.clone());
            if name1 == name2 {
                match (&hinge1.limits, &hinge2.limits) {
                    (Some(l1), Some(l2)) => {
                        let lower_diff = (l1.lower - l2.lower).abs();
                        let upper_diff = (l1.upper - l2.upper).abs();
                        if lower_diff > 1e-3 || upper_diff > 1e-3 {
                            println!("  Joint '{:?}': LIMIT MISMATCH: [{}, {}] vs [{}, {}] (diff: {}, {})",
                                name1, l1.lower, l1.upper, l2.lower, l2.upper, lower_diff, upper_diff);
                        }
                        assert!(lower_diff < 1e-3,
                            "Joint '{:?}': lower limit mismatch {} vs {}", name1, l1.lower, l2.lower);
                        assert!(upper_diff < 1e-3,
                            "Joint '{:?}': upper limit mismatch {} vs {}", name1, l1.upper, l2.upper);
                        println!("  Joint '{:?}': limits [{}, {}] preserved", name1, l1.lower, l1.upper);
                    }
                    (None, None) => println!("  Joint '{:?}': no limits (preserved)", name1),
                    _ => panic!("Joint '{:?}': limits presence mismatch", name1),
                }
            }
        }
    }

    // ── Coordinate damping/stiffness comparison ──
    for (key1, coord1) in world1.iter::<melosim::components::JointCoordinate>() {
        let name1 = world1.get::<melosim::components::Name>(key1).map(|n| n.value.clone());
        for (key2, coord2) in world2.iter::<melosim::components::JointCoordinate>() {
            let name2 = world2.get::<melosim::components::Name>(key2).map(|n| n.value.clone());
            if name1 == name2 {
                assert!((coord1.damping - coord2.damping).abs() < 1e-6,
                    "Coord '{:?}': damping mismatch", name1);
                println!("  Coord '{:?}': damping={} preserved", name1, coord1.damping);
            }
        }
    }

    // ── Muscle name preservation ──
    let mut muscle_names1: Vec<String> = world1.iter::<melosim::components::Muscle>()
        .filter_map(|(k, _)| world1.get::<melosim::components::Name>(k).map(|n| n.value.clone()))
        .collect();
    let mut muscle_names2: Vec<String> = world2.iter::<melosim::components::Muscle>()
        .filter_map(|(k, _)| world2.get::<melosim::components::Name>(k).map(|n| n.value.clone()))
        .collect();
    muscle_names1.sort();
    muscle_names2.sort();
    assert_eq!(muscle_names1, muscle_names2, "Muscle names mismatch");
    println!("  Muscles: {:?} preserved", muscle_names1);

    let _ = std::fs::remove_file(tmp_path);

    println!("\nStructural roundtrip: PASSED");
}
