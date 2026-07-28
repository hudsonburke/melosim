// ── Exporter tests ────────────────────────────────────
// Validates that the exporter produces valid .osim XML from a World.
// These tests run without OpenSim — they use JSON fixture data.

use melosim::exporter::opensim::world_to_osim;
use melosim::importer::opensim::{import_opensim_model, load_opensim_json};
use melosim::world::World;

/// Basic checks on the XML output structure.
fn check_xml_structure(xml: &str) {
    assert!(xml.starts_with("<?xml"), "XML should start with declaration");
    assert!(
        xml.contains("<OpenSimDocument Version=\"30000\">"),
        "Should have OpenSimDocument root"
    );
    assert!(xml.contains("<Model"), "Should have Model element");
    assert!(xml.contains("</Model>"), "Should close Model");
    assert!(xml.contains("</OpenSimDocument>"), "Should close OpenSimDocument");
}

/// Count occurrences of a substring in XML.
fn count_xml_tags(xml: &str, tag: &str) -> usize {
    xml.matches(&format!("<{} ", tag)).count()
        + xml.matches(&format!("<{}>", tag)).count()
}

#[test]
fn test_export_simple_hip() {
    let path = "tests/fixtures/simple_hip.json";
    let model = load_opensim_json(path).expect("Failed to load fixture");
    let mut world = World::new();
    import_opensim_model(&mut world, &model).expect("Import failed");

    let xml = world_to_osim(&world, "SimpleHipTest");

    // Debug: show marker section
    if let Some(midx) = xml.find("MarkerSet") {
        let end = xml[midx..].find("</MarkerSet>").map(|e| midx + e + 12).unwrap_or(xml.len());
        eprintln!("Marker XML:\n{}", &xml[midx..end.min(midx+1000)]);
    }

    // Check basic structure
    check_xml_structure(&xml);
    assert!(xml.contains("SimpleHipTest"), "Model name should be in output");

    // Check bodies have real names from fixture (not auto-generated body_N)
    assert!(xml.contains("<Body name=\"pelvis\""), "Should contain body pelvis");
    assert!(xml.contains("<Body name=\"femur_r\""), "Should contain body femur_r");

    // Check joints
    assert!(xml.contains("<FreeJoint"), "Should contain FreeJoint");
    assert!(xml.contains("<PinJoint"), "Should contain PinJoint");

    // Check markers have correct names from Landmark component
    assert!(xml.contains("<Marker name=\"RASI\""), "Should contain RASI marker");
    assert!(xml.contains("<Marker name=\"RTHI\""), "Should contain RTHI marker");

    // Check body properties
    assert!(xml.contains("<mass>11.78"), "Should contain pelvis mass");

    // Verify XML is well-formed (basic check — matching open/close tags)
    let open_models = xml.matches("<Model").count();
    let close_models = xml.matches("</Model>").count();
    assert_eq!(open_models, close_models, "Model tag mismatch");

    let open_bodies = count_xml_tags(&xml, "Body");
    let close_bodies = xml.matches("</Body>").count();
    assert_eq!(open_bodies, close_bodies, "Body tag mismatch");
}

#[test]
fn test_export_simple_muscle() {
    let path = "tests/fixtures/simple_muscle.json";
    let model = load_opensim_json(path).expect("Failed to load fixture");
    let mut world = World::new();
    import_opensim_model(&mut world, &model).expect("Import failed");

    let xml = world_to_osim(&world, "SimpleMuscleTest");

    // Check basic structure
    check_xml_structure(&xml);

    // Check ForceSet with muscle
    assert!(xml.contains("<ForceSet>"), "Should contain ForceSet");
    assert!(
        xml.contains("Millard2012EquilibriumMuscle"),
        "Should contain muscle type"
    );
    assert!(
        xml.contains("rectus_femoris_r"),
        "Should contain muscle name"
    );

    // Check muscle parameters
    assert!(xml.contains("<max_isometric_force>1169"), "Should contain force");
    assert!(xml.contains("<optimal_fiber_length>0.114"), "Should contain fiber length");
    assert!(xml.contains("<tendon_slack_length>0.344"), "Should contain slack length");

    // Check geometry path
    assert!(xml.contains("<GeometryPath>"), "Should contain GeometryPath");
    assert!(xml.contains("<PathPointSet>"), "Should contain PathPointSet");

    // Check wrap objects
    assert!(xml.contains("<WrapCylinder"), "Should contain WrapCylinder");

    // Check display geometry inside bodies
    assert!(xml.contains("<DisplayGeometry"), "Should contain DisplayGeometry");
    assert!(xml.contains("femur.vtp"), "Should contain mesh file ref");

    // Verify tag matching
    let open_forces = count_xml_tags(&xml, "ForceSet");
    let close_forces = xml.matches("</ForceSet>").count();
    assert_eq!(open_forces, close_forces, "ForceSet tag mismatch");
}
