// ── Exporter tests ────────────────────────────────────

use melosim::exporter::opensim::world_to_osim;
use melosim::importer::opensim::{import_opensim_model, load_opensim_json};
use melosim::world::World;
use melosim::world::WorldExt;
use melosim::components::*;
use bevy_ecs::prelude::Entity;

fn check_xml_structure(xml: &str) {
    assert!(xml.starts_with("<?xml"), "XML should start with declaration");
    assert!(xml.contains("<OpenSimDocument Version=\"30000\">"), "Should have OpenSimDocument root");
    assert!(xml.contains("<Model"), "Should have Model element");
    assert!(xml.contains("</Model>"), "Should close Model");
    assert!(xml.contains("</OpenSimDocument>"), "Should close OpenSimDocument");
}

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

    let xml = world_to_osim(&mut world, "SimpleHipTest");
    check_xml_structure(&xml);
    assert!(xml.contains("SimpleHipTest"), "Model name should be in output");
    assert!(xml.contains("<Body name=\"pelvis\""), "Should contain body pelvis");
    assert!(xml.contains("<Body name=\"femur_r\""), "Should contain body femur_r");
    assert!(xml.contains("<FreeJoint"), "Should contain FreeJoint");
    assert!(xml.contains("<PinJoint"), "Should contain PinJoint");
    assert!(xml.contains("<Marker name=\"RASI\""), "Should contain RASI marker");
    assert!(xml.contains("<Marker name=\"RTHI\""), "Should contain RTHI marker");
    assert!(xml.contains("<mass>11.78"), "Should contain pelvis mass");

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

    let xml = world_to_osim(&mut world, "SimpleMuscleTest");
    check_xml_structure(&xml);

    assert!(xml.contains("<ForceSet>"), "Should contain ForceSet");
    assert!(xml.contains("Millard2012EquilibriumMuscle"), "Should contain muscle type");
    assert!(xml.contains("rectus_femoris_r"), "Should contain muscle name");
    assert!(xml.contains("CoordinateActuator"), "Should contain CoordinateActuator");
    assert!(xml.contains("knee_actuator"), "Should contain actuator name");
    assert!(xml.contains("<optimal_force>50</optimal_force>"), "Should contain optimal_force");
    assert!(xml.contains("<max_isometric_force>1169"), "Should contain force");
    assert!(xml.contains("<optimal_fiber_length>0.114"), "Should contain fiber length");
    assert!(xml.contains("<tendon_slack_length>0.344"), "Should contain slack length");
    assert!(xml.contains("<GeometryPath>"), "Should contain GeometryPath");
    assert!(xml.contains("<PathPointSet>"), "Should contain PathPointSet");
    assert!(xml.contains("<WrapCylinder"), "Should contain WrapCylinder");
    assert!(xml.contains("<DisplayGeometry"), "Should contain DisplayGeometry");
    assert!(xml.contains("femur.vtp"), "Should contain mesh file ref");

    let open_forces = count_xml_tags(&xml, "ForceSet");
    let close_forces = xml.matches("</ForceSet>").count();
    assert_eq!(open_forces, close_forces, "ForceSet tag mismatch");
}

#[test]
fn test_find_by_name() {
    let mut world = World::new();
    let e1 = world.spawn_entity();
    let e2 = world.spawn_entity();
    world.attach(e1, Name::new("forearm"));
    world.attach(e2, Name::new("cuff"));

    assert_eq!(world.find_by_name("forearm"), Some(e1));
    assert_eq!(world.find_by_name("cuff"), Some(e2));
    assert_eq!(world.find_by_name("missing"), None);
}

#[test]
fn test_body_construction() {
    let mut world = World::new();

    let forearm = world.spawn_entity();
    world.attach(forearm, InertialProperties {
        mass: 1.5, com: [0.0; 3], inertia: [0.0; 6],
    });
    world.attach(forearm, Name::new("r_forearm"));

    let cuff = world.spawn_entity();
    world.attach(cuff, InertialProperties {
        mass: 0.5, com: [0.0; 3], inertia: [0.0; 6],
    });
    world.set_parent(cuff, forearm);
    world.attach(cuff, Name::new("arm_cuff"));
    world.attach(cuff, MeshGeometry { mesh: "assets/cuff.stl".into() });

    let inertial = world.get::<InertialProperties>(cuff).expect("cuff should have InertialProperties");
    assert_eq!(inertial.mass, 0.5);

    let parent = world.parent_of(cuff).expect("cuff should have parent");
    assert_eq!(parent, forearm);

    let mesh = world.get::<MeshGeometry>(cuff).expect("cuff should have MeshGeometry");
    assert_eq!(mesh.mesh, "assets/cuff.stl");

    let mut name = world.get::<Name>(cuff).expect("cuff should have Name");
    assert_eq!(name.value, "arm_cuff");
}

#[test]
fn test_get_mut() {
    let mut world = World::new();
    let e = world.spawn_entity();
    world.attach(e, Name::new("test"));

    // Verify the name was attached
    let name = world.get::<Name>(e).expect("should have Name");
    assert_eq!(name.value, "test");
}

#[test]
fn test_hierarchy() {
    let mut world = World::new();

    let ground = world.spawn_entity();
    let body = world.spawn_entity();
    let joint = world.spawn_entity();
    let child = world.spawn_entity();

    world.set_parent(joint, ground);
    world.set_parent(body, joint);
    world.set_parent(child, body);

    assert_eq!(world.parent_of(joint), Some(ground));
    assert_eq!(world.parent_of(body), Some(joint));
    assert_eq!(world.parent_of(child), Some(body));

    let ground_children = world.children_of(ground);
    assert!(ground_children.contains(&joint));

    let joint_children = world.children_of(joint);
    assert!(joint_children.contains(&body));
}
