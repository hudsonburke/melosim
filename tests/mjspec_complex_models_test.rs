use melosim::importer::mujoco_spec::import_mjcf_spec;
use melosim::exporter::mujoco_spec::world_to_mjcf_spec;
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

#[test]
fn test_myohand_mjspec_roundtrip() {
    ensure_myo_sim();

    let model_path = "tests/fixtures/myo_sim/hand/myohand.xml";
    let (world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoHand via MjSpec");

    let n_bodies = world.count::<melosim::components::InertialProperties>();
    let n_hinge = world.count::<melosim::components::HingeJoint>();
    let n_slide = world.count::<melosim::components::SlideJoint>();
    let n_ball = world.count::<melosim::components::BallJoint>();
    let n_free = world.count::<melosim::components::FreeJoint>();
    let n_sites = world.count::<melosim::components::Site>();
    let n_geoms = world.count::<melosim::components::DisplayGeometry>();
    let n_muscles = world.count::<melosim::components::Muscle>();
    let n_paths = world.count::<melosim::components::MusclePath>();

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
    println!("  Total entities: {}", world.next_id);

    // Export via MjSpec (lossless)
    let exported_xml = world_to_mjcf_spec(&world, "MyoHand_v0.1.7")
        .expect("Failed to export myoHand via MjSpec");

    let tmp_path = "/tmp/melosim_myohand_roundtrip.xml";
    std::fs::write(tmp_path, &exported_xml).expect("Failed to write");

    // Re-import to verify
    let (world2, _) = import_mjcf_spec(tmp_path)
        .expect("Failed to re-import myoHand");

    // Verify structural counts
    assert_eq!(
        world.count::<melosim::components::InertialProperties>(),
        world2.count::<melosim::components::InertialProperties>(),
        "Body count mismatch"
    );
    assert_eq!(
        world.count::<melosim::components::HingeJoint>(),
        world2.count::<melosim::components::HingeJoint>(),
        "Hinge count mismatch"
    );
    assert_eq!(
        world.count::<melosim::components::Muscle>(),
        world2.count::<melosim::components::Muscle>(),
        "Muscle count mismatch"
    );

    let _ = std::fs::remove_file(tmp_path);

    println!("\n  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}", n_bodies, world2.count::<melosim::components::InertialProperties>());
    println!("  Muscles: {} -> {}", n_muscles, world2.count::<melosim::components::Muscle>());
    println!("  myoHand roundtrip: PASSED");
}

#[test]
fn test_myoleg_mjspec_roundtrip() {
    ensure_myo_sim();

    let model_path = "tests/fixtures/myo_sim/leg/myolegs.xml";
    let (world, body_map) = import_mjcf_spec(model_path)
        .expect("Failed to import myoLeg via MjSpec");

    let n_bodies = world.count::<melosim::components::InertialProperties>();
    let n_hinge = world.count::<melosim::components::HingeJoint>();
    let n_slide = world.count::<melosim::components::SlideJoint>();
    let n_ball = world.count::<melosim::components::BallJoint>();
    let n_free = world.count::<melosim::components::FreeJoint>();
    let n_sites = world.count::<melosim::components::Site>();
    let n_geoms = world.count::<melosim::components::DisplayGeometry>();
    let n_muscles = world.count::<melosim::components::Muscle>();
    let n_paths = world.count::<melosim::components::MusclePath>();

    println!("\nmyoLeg MjSpec import:");
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
    println!("  Total entities: {}", world.next_id);

    // Export via MjSpec (lossless)
    let exported_xml = world_to_mjcf_spec(&world, "myoLeg_v0.1")
        .expect("Failed to export myoLeg via MjSpec");

    let tmp_path = "/tmp/melosim_myoleg_roundtrip.xml";
    std::fs::write(tmp_path, &exported_xml).expect("Failed to write");

    // Re-import to verify
    let (world2, _) = import_mjcf_spec(tmp_path)
        .expect("Failed to re-import myoLeg");

    assert_eq!(
        world.count::<melosim::components::InertialProperties>(),
        world2.count::<melosim::components::InertialProperties>(),
        "Body count mismatch"
    );
    assert_eq!(
        world.count::<melosim::components::Muscle>(),
        world2.count::<melosim::components::Muscle>(),
        "Muscle count mismatch"
    );

    let _ = std::fs::remove_file(tmp_path);

    println!("\n  Exported: {} bytes", exported_xml.len());
    println!("  Bodies: {} -> {}", n_bodies, world2.count::<melosim::components::InertialProperties>());
    println!("  Muscles: {} -> {}", n_muscles, world2.count::<melosim::components::Muscle>());
    println!("  myoLeg roundtrip: PASSED");
}
