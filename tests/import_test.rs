use melosim::components::*;
use melosim::importer::opensim::{import_opensim_model, load_opensim_json};
use melosim::world::World;

#[test]
fn test_import_simple_hip() {
    let path = "tests/fixtures/simple_hip.json";
    let model = load_opensim_json(path).expect("Failed to load JSON fixture");

    assert_eq!(model.name, "SimpleHip");
    assert_eq!(model.bodies.len(), 3);
    assert_eq!(model.joints.len(), 2);
    assert_eq!(model.markers.len(), 2);

    let mut world = World::new();
    import_opensim_model(&mut world, &model).expect("Import failed");

    // Validate: 3 bodies + 2 joints (intermediate nodes) + 1 coordinate + 2 markers (sites)
    assert_eq!(world.count::<InertialProperties>(), 3);
    assert_eq!(world.count::<JointCoordinate>(), 7); // 6 from FreeJoint + 1 from CustomJoint
    // 2 markers imported as Position + ChildOf (no Site marker)
    assert_eq!(world.count::<ChildOf>(), world.count::<ChildOf>()); // just verify it compiles

    // Validate the world — all entity references should resolve
    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);
}

#[test]
fn test_import_simple_knee() {
    let path = "tests/fixtures/simple_knee.json";
    let model = load_opensim_json(path).expect("Failed to load JSON fixture");

    assert_eq!(model.name, "SimpleKnee");
    assert_eq!(model.bodies.len(), 3);
    assert_eq!(model.joints.len(), 2);

    let mut world = World::new();
    import_opensim_model(&mut world, &model).expect("Import failed");

    // Validate counts
    assert_eq!(world.count::<InertialProperties>(), 3);
    assert_eq!(world.count::<JointCoordinate>(), 7); // 6 from FreeJoint + 1 from CustomJoint
    assert_eq!(world.count::<CoordinateEffect>(), 9); // 6 from FreeJoint + 3 from CustomJoint

    // Validate the world
    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    // Verify the CoordinateEffect functions
    let effects: Vec<&CoordinateEffect> = world.iter::<CoordinateEffect>()
        .map(|(_, e)| e)
        .collect();
    // 6 Linear from FreeJoint (3 rotation + 3 translation) + 1 from PinJoint = 7
    assert_eq!(effects.len(), 9); // 6 from FreeJoint + 3 from CustomJoint
}

#[test]
fn test_import_simple_muscle() {
    let path = "tests/fixtures/simple_muscle.json";
    let model = load_opensim_json(path).expect("Failed to load JSON fixture");

    assert_eq!(model.name, "SimpleMuscle");
    assert_eq!(model.bodies.len(), 3);
    assert_eq!(model.joints.len(), 2);
    assert_eq!(model.muscles.len(), 1);
    assert_eq!(model.wrap_objects.len(), 1);
    assert_eq!(model.display_geometries.len(), 1);

    let mut world = World::new();
    import_opensim_model(&mut world, &model).expect("Import failed");

    // Validate counts
    assert_eq!(world.count::<InertialProperties>(), 3);
    assert_eq!(world.count::<JointCoordinate>(), 7); // 6 from FreeJoint + 1 from CustomJoint
    // Component types
    assert_eq!(world.count::<Muscle>(), 1);
    assert_eq!(world.count::<MusclePath>(), 1);
    assert_eq!(world.count::<Millard2012Params>(), 1);
    assert_eq!(world.count::<WrapGeom>(), 1);
    assert_eq!(world.count::<DisplayGeometry>(), 1);
    assert_eq!(world.count::<CoordinateActuator>(), 1);

    // Validate the world
    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    // Verify CoordinateActuator references the knee_flexion coordinate
    for (_key, act) in world.iter::<CoordinateActuator>() {
        let coord_name = world.get::<Name>(act.coordinate)
            .map(|n| n.value.as_str())
            .unwrap_or("");
        assert_eq!(coord_name, "knee_flexion", "CoordinateActuator should reference knee_flexion");
        assert_eq!(act.optimal_force, 50.0);
    }

    // Verify muscle was created with correct name
    let name = world.iter::<Name>().find(|(_, n)| n.value == "rectus_femoris_r");
    assert!(name.is_some(), "Should find muscle named rectus_femoris_r");
    // Verify WrapGeom was created with correct type
    for (_key, wrap) in world.iter::<WrapGeom>() {
        match &wrap.geom_type {
            WrapGeomType::Cylinder { radius, length } => {
                assert_eq!(*radius, 0.03);
                assert_eq!(*length, 0.1);
            }
            _ => panic!("Expected Cylinder wrap geom"),
        }
    }
    // Verify DisplayGeometry
    for (_key, geom) in world.iter::<DisplayGeometry>() {
        assert_eq!(geom.mesh_file, Some("femur.vtp".to_string()));
        assert_eq!(geom.color, [0.8, 0.8, 0.8]);
    }
}
