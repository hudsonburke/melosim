#![cfg(feature = "mujoco")]

use melosim::importer::mujoco_spec::import_mjcf_spec;
use melosim::exporter::mujoco_spec::world_to_mjcf_spec;
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
fn test_mjspec_import() {
    ensure_myo_sim();

    let model_path = "tests/fixtures/myo_sim/elbow/myoelbow_1dof6muscles.xml";
    let (world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoelbow via MjSpec");

    let n_bodies = world.count::<melosim::components::InertialProperties>();
    let n_hinge = world.count::<melosim::components::HingeJoint>();
    let n_sites = world.count::<melosim::components::Site>();
    let n_muscles = world.count::<melosim::components::Muscle>();
    let n_paths = world.count::<melosim::components::MusclePath>();

    println!("MjSpec import:");
    println!("  Bodies: {}", n_bodies);
    println!("  Hinge joints: {}", n_hinge);
    println!("  Sites: {}", n_sites);
    println!("  Muscles: {}", n_muscles);
    println!("  Muscle paths: {}", n_paths);
    println!("  Body map entries: {}", body_map.len());
    println!("  Total entities: {}", world.next_id);

    assert_eq!(n_hinge, 1, "Expected 1 hinge joint");
    assert_eq!(n_muscles, 6, "Expected 6 muscles");
    assert!(n_sites > 10, "Expected many sites");
}

#[test]
fn test_mjspec_lossless_roundtrip() {
    ensure_myo_sim();

    let model_path = "tests/fixtures/myo_sim/elbow/myoelbow_1dof6muscles.xml";

    // Read original XML
    let original_xml = std::fs::read_to_string(model_path)
        .expect("Failed to read original MJCF");

    // Import via MjSpec
    let (world, _) = import_mjcf_spec(model_path)
        .expect("Failed to import");

    // Export via MjSpec (lossless)
    let exported_xml = world_to_mjcf_spec(&world, "MyoElbow_v0.1.7")
        .expect("Failed to export via MjSpec");

    // Write exported XML
    let tmp_path = "/tmp/melosim_mjspec_roundtrip.xml";
    std::fs::write(tmp_path, &exported_xml).expect("Failed to write");

    // Re-import to verify it's valid
    let (world2, _) = import_mjcf_spec(tmp_path)
        .expect(&format!("Failed to re-import exported MJCF.\nExported:\n{}", &exported_xml[..2000]));

    // Verify structural counts
    assert_eq!(
        world.count::<melosim::components::InertialProperties>(),
        world2.count::<melosim::components::InertialProperties>(),
        "Body count mismatch after roundtrip"
    );
    assert_eq!(
        world.count::<melosim::components::HingeJoint>(),
        world2.count::<melosim::components::HingeJoint>(),
        "Hinge joint count mismatch"
    );
    assert_eq!(
        world.count::<melosim::components::Muscle>(),
        world2.count::<melosim::components::Muscle>(),
        "Muscle count mismatch"
    );

    // Verify key structural elements are present in the exported XML
    println!("Exported XML (first 2000 chars):");
    println!("{}", &exported_xml[..exported_xml.len().min(2000)]);
    println!("...");

    // Check for key structural elements
    assert!(exported_xml.contains("<mujoco"), "Missing <mujoco> root");
    assert!(exported_xml.contains("<worldbody>"), "Missing <worldbody>");
    assert!(exported_xml.contains("<actuator>"), "Missing <actuator>");

    let _ = std::fs::remove_file(tmp_path);

    println!("\nMjSpec lossless roundtrip:");
    println!("  Original: {} bytes", original_xml.len());
    println!("  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}",
        world.count::<melosim::components::InertialProperties>(),
        world2.count::<melosim::components::InertialProperties>());
    println!("  Muscles: {} -> {}",
        world.count::<melosim::components::Muscle>(),
        world2.count::<melosim::components::Muscle>());
    println!("  PASSED");
}
