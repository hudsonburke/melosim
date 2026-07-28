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

    // Validate: 3 bodies + 2 joints + 1 coordinate + 2 markers (sites with names)
    assert_eq!(world.count::<InertialProperties>(), 3);
    assert_eq!(world.count::<Frame>(), 3);
    assert_eq!(world.count::<HingeJoint>(), 1);
    assert_eq!(world.count::<FreeJoint>(), 1);
    assert_eq!(world.count::<JointCoordinate>(), 1);
    assert_eq!(world.count::<Site>(), 2);

    // Validate the world — all entity references should resolve
    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    // Freeze — all components should transfer
    let flat = world.freeze();
    assert_eq!(flat.len(), world.next_id as usize);
    assert_eq!(
        flat.hinge_joints.iter().filter_map(|x| x.as_ref()).count(),
        1
    );
    assert_eq!(
        flat.free_joints.iter().filter_map(|x| x.as_ref()).count(),
        1
    );
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
    assert_eq!(world.count::<CustomJoint>(), 1);
    assert_eq!(world.count::<FreeJoint>(), 1);
    assert_eq!(world.count::<JointCoordinate>(), 1);
    assert_eq!(world.count::<CoordinateEffect>(), 3);
    assert_eq!(world.count::<SpatialTransform>(), 1);
    assert_eq!(world.count::<Site>(), 2);

    // Validate the world
    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    // Freeze — all components should transfer
    let flat = world.freeze();
    assert_eq!(flat.len(), world.next_id as usize);
    assert_eq!(
        flat.custom_joints.iter().filter_map(|x| x.as_ref()).count(),
        1
    );
    assert_eq!(
        flat.coordinate_effects
            .iter()
            .filter_map(|x| x.as_ref())
            .count(),
        3
    );

    // Verify the CoordinateEffect functions match what we put in
    let effects: Vec<&CoordinateEffect> = flat
        .coordinate_effects
        .iter()
        .filter_map(|x| x.as_ref())
        .collect();
    assert_eq!(effects.len(), 3);

    // Should have: RotationY (Linear), TranslationX (Polynomial), TranslationZ (Polynomial)
    let linear_count = effects
        .iter()
        .filter(|e| matches!(e.function, JointFunction::Linear { .. }))
        .count();
    let poly_count = effects
        .iter()
        .filter(|e| matches!(e.function, JointFunction::Polynomial { .. }))
        .count();
    assert_eq!(linear_count, 1);
    assert_eq!(poly_count, 2);
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
    assert_eq!(world.count::<CustomJoint>(), 1);
    assert_eq!(world.count::<FreeJoint>(), 1);
    assert_eq!(world.count::<JointCoordinate>(), 1);
    assert_eq!(world.count::<Site>(), 1);
    // Component types
    assert_eq!(world.count::<Muscle>(), 1);
    assert_eq!(world.count::<MusclePath>(), 1);
    assert_eq!(world.count::<Millard2012Params>(), 1);
    assert_eq!(world.count::<WrapGeom>(), 1);
    assert_eq!(world.count::<DisplayGeometry>(), 1);

    // Validate the world
    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    // Freeze — verify counts transfer
    let flat = world.freeze();
    assert_eq!(flat.len(), world.next_id as usize);

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
