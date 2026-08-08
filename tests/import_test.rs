use melosim::components::*;
use melosim::importer::opensim::{import_opensim_model, load_opensim_json};
use melosim::world::World;
use melosim::world::WorldExt;
use bevy_ecs::prelude::Entity;

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

    assert_eq!(world.count::<InertialProperties>(), 3);
    assert_eq!(world.count::<JointCoordinate>(), 7);
    assert_eq!(world.count::<ChildOf>(), world.count::<ChildOf>());

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

    assert_eq!(world.count::<InertialProperties>(), 3);
    assert_eq!(world.count::<JointCoordinate>(), 7);
    assert_eq!(world.count::<CoordinateEffect>(), 9);

    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    let effects: Vec<CoordinateEffect> = world.iter::<CoordinateEffect>()
        .into_iter()
        .map(|(_, e)| e)
        .collect();
    assert_eq!(effects.len(), 9);
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

    assert_eq!(world.count::<InertialProperties>(), 3);
    assert_eq!(world.count::<JointCoordinate>(), 7);
    assert_eq!(world.count::<Muscle>(), 1);
    assert_eq!(world.count::<MusclePath>(), 1);
    assert_eq!(world.count::<Millard2012Params>(), 1);
    assert_eq!(world.count::<WrapGeom>(), 1);
    assert_eq!(world.count::<DisplayGeometry>(), 1);
    assert_eq!(world.count::<CoordinateActuator>(), 1);

    let errors = world.validate();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    for (_key, act) in world.iter::<CoordinateActuator>().into_iter() {
        let coord_name = world.get::<Name>(act.coordinate)
            .map(|n| n.value.as_str())
            .unwrap_or("");
        assert_eq!(coord_name, "knee_flexion", "CoordinateActuator should reference knee_flexion");
        assert_eq!(act.optimal_force, 50.0);
    }

    let name = world.iter::<Name>().into_iter().find(|(_, n)| n.value == "rectus_femoris_r");
    assert!(name.is_some(), "Should find muscle named rectus_femoris_r");

    for (_key, wrap) in world.iter::<WrapGeom>().into_iter() {
        match &wrap.geom_type {
            WrapGeomType::Cylinder { radius, length } => {
                assert_eq!(*radius, 0.03);
                assert_eq!(*length, 0.1);
            }
            _ => panic!("Expected Cylinder wrap geom"),
        }
    }

    for (_key, geom) in world.iter::<DisplayGeometry>().into_iter() {
        assert_eq!(geom.mesh_file, Some("femur.vtp".to_string()));
        assert_eq!(geom.color, [0.8, 0.8, 0.8]);
    }
}
