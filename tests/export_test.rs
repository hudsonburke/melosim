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

    // Check markers have correct names from Name component on Site
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

    // Check ForceSet with muscle + actuator
    assert!(xml.contains("<ForceSet>"), "Should contain ForceSet");
    assert!(
        xml.contains("Millard2012EquilibriumMuscle"),
        "Should contain muscle type"
    );
    assert!(
        xml.contains("rectus_femoris_r"),
        "Should contain muscle name"
    );
    assert!(
        xml.contains("CoordinateActuator"),
        "Should contain CoordinateActuator"
    );
    assert!(
        xml.contains("knee_actuator"),
        "Should contain actuator name"
    );
    assert!(
        xml.contains("<optimal_force>50</optimal_force>"),
        "Should contain optimal_force"
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

// ── Model editing API tests ───────────────────────────

use melosim::components::*;
use melosim::id::EntityID;
use melosim::math::{Transform, Vec3};

#[test]
fn test_find_by_name() {
    let mut world = World::new();
    let e1 = world.spawn();
    let e2 = world.spawn();
    world.attach(e1, Name { value: "forearm".into() });
    world.attach(e2, Name { value: "cuff".into() });

    assert_eq!(world.find_by_name("forearm"), Some(e1));
    assert_eq!(world.find_by_name("cuff"), Some(e2));
    assert_eq!(world.find_by_name("missing"), None);
}

#[test]
fn test_attach_mesh() {
    let mut world = World::new();

    // Create a forearm body
    let forearm = world.spawn();
    world.attach(forearm, InertialProperties {
        mass: 1.5,
        com: [0.0; 3],
        inertia: [0.0; 6],
    });
    world.attach(forearm, Frame {
        parent: EntityID(0),
        transform: Transform::default(),
    });
    world.attach(forearm, Name { value: "r_forearm".into() });

    // Attach a mesh to it
    let cuff = world.attach_mesh(
        forearm,
        "assets/arm_cuff.stl",
        "arm_cuff",
        Vec3::new(0.0, 0.0, -0.15),
    );

    // Verify the mesh entity has the right components
    let frame = world.get::<Frame>(cuff).expect("cuff should have Frame");
    assert_eq!(frame.parent, forearm);
    assert_eq!(frame.transform.translation.z, -0.15);

    let mesh = world.get::<MeshGeometry>(cuff).expect("cuff should have MeshGeometry");
    assert_eq!(mesh.mesh, "assets/arm_cuff.stl");

    let name = world.get::<Name>(cuff).expect("cuff should have Name");
    assert_eq!(name.value, "arm_cuff");
}

#[test]
fn test_body_builder() {
    let mut world = World::new();

    // Create a forearm body
    let forearm = world.spawn();
    world.attach(forearm, InertialProperties {
        mass: 1.5,
        com: [0.0; 3],
        inertia: [0.0; 6],
    });
    world.attach(forearm, Frame {
        parent: EntityID(0),
        transform: Transform::default(),
    });
    world.attach(forearm, Name { value: "r_forearm".into() });

    // Build a cuff with mass and mesh
    let cuff = world.body_builder("r_forearm")
        .name("arm_cuff")
        .mesh("assets/cuff.stl")
        .mass(0.5)
        .offset(Vec3::new(0.0, 0.0, -0.15))
        .color([0.8, 0.2, 0.1])
        .build(&mut world)
        .expect("should find parent");

    // Verify all components
    let inertial = world.get::<InertialProperties>(cuff).expect("cuff should have InertialProperties");
    assert_eq!(inertial.mass, 0.5);

    let frame = world.get::<Frame>(cuff).expect("cuff should have Frame");
    assert_eq!(frame.parent, forearm);

    let mesh = world.get::<MeshGeometry>(cuff).expect("cuff should have MeshGeometry");
    assert_eq!(mesh.mesh, "assets/cuff.stl");

    let name = world.get::<Name>(cuff).expect("cuff should have Name");
    assert_eq!(name.value, "arm_cuff");
}

#[test]
fn test_get_mut() {
    let mut world = World::new();
    let e = world.spawn();
    world.attach(e, Name { value: "test".into() });

    // Modify the name
    if let Some(name) = world.get_mut::<Name>(e) {
        name.value = "modified".into();
    }

    let name = world.get::<Name>(e).expect("should have Name");
    assert_eq!(name.value, "modified");
}
