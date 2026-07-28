use melosim::importer::opensim::{import_opensim_model, OpenSimModelData};
use melosim::exporter::mujoco::world_to_mjcf;
use melosim::world::World;
use std::fs;

#[test]
fn test_opensim_to_mujoco_conversion() {
    let json_path = "rj.json";
    if !std::path::Path::new(json_path).exists() {
        eprintln!("Skipping: rj.json not found");
        return;
    }

    let json_str = fs::read_to_string(json_path).expect("Failed to read rj.json");
    let model_data: OpenSimModelData =
        serde_json::from_str(&json_str).expect("Failed to parse JSON");

    let mut world = World::new();
    let result = import_opensim_model(&mut world, &model_data);
    if let Err(errs) = &result {
        for e in errs { println!("IMPORT ERROR: {}", e); }
    }

    println!("OpenSim import (Rajagopal 2015):");
    println!("  Bodies: {}", world.count::<melosim::components::InertialProperties>());
    println!("  Hinge joints: {}", world.count::<melosim::components::HingeJoint>());
    println!("  Custom joints: {}", world.count::<melosim::components::CustomJoint>());
    println!("  Universal joints: {}", world.count::<melosim::components::UniversalJoint>());
    println!("  Coordinates: {}", world.count::<melosim::components::JointCoordinate>());
    println!("  Muscles: {}", world.count::<melosim::components::Muscle>());
    println!("  Sites: {}", world.count::<melosim::components::Site>());

    // Export to MJCF
    let mjcf_xml = world_to_mjcf(&world, "Rajagopal2015");
    let out_path = "/tmp/rajagopal_from_opensim.xml";
    fs::write(out_path, &mjcf_xml).expect("Failed to write MJCF");

    let body_count = mjcf_xml.matches("<body name=").count();
    let joint_count = mjcf_xml.matches("<joint ").count() + mjcf_xml.matches("<freejoint ").count();
    let site_count = mjcf_xml.matches("<site ").count();
    let muscle_count = mjcf_xml.matches("<muscle ").count();

    println!("\nMJCF export ({} bytes):", mjcf_xml.len());
    println!("  Bodies in XML: {}", body_count);
    println!("  Joints in XML: {}", joint_count);
    println!("  Sites in XML: {}", site_count);
    println!("  Muscles in XML: {}", muscle_count);

    assert!(body_count > 0, "No bodies exported");
    assert!(joint_count > 0, "No joints exported");
    assert!(muscle_count > 0, "No muscles exported");

    println!("\n  OpenSim -> MuJoCo conversion: PASSED");
}
