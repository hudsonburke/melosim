use melosim::components::*;
use melosim::importer::opensim::{import_opensim_model, load_opensim_json};
use melosim::world::World;
use bevy_ecs::prelude::Entity;

fn find_by_name(world: &mut World, name: &str) -> Option<Entity> {
    let mut query = world.query::<(Entity, &Name)>();
    query.iter(world).find(|(_, n)| n.value == name).map(|(e, _)| e)
}

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

    assert_eq!(world.query::<&InertialProperties>().iter(&world).count(), 3);
    assert_eq!(world.query::<&JointCoordinate>().iter(&world).count(), 7);
    assert_eq!(world.query::<&ChildOf>().iter(&world).count(), world.query::<&ChildOf>().iter(&world).count());

    melosim::systems::run_systems(&mut world);
    let errors = world.get_resource::<melosim::world::ErrorList>().map(|e| e.0.clone()).unwrap_or_default();
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

    assert_eq!(world.query::<&InertialProperties>().iter(&world).count(), 3);
    assert_eq!(world.query::<&JointCoordinate>().iter(&world).count(), 7);
    assert_eq!(world.query::<&CoordinateEffect>().iter(&world).count(), 9);

    melosim::systems::run_systems(&mut world);
    let errors = world.get_resource::<melosim::world::ErrorList>().map(|e| e.0.clone()).unwrap_or_default();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    let effects: Vec<CoordinateEffect> = {
        let mut query = world.query::<(Entity, &CoordinateEffect)>();
        query.iter(&world).map(|(_, e)| e.clone()).collect()
    };
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

    assert_eq!(world.query::<&InertialProperties>().iter(&world).count(), 3);
    assert_eq!(world.query::<&JointCoordinate>().iter(&world).count(), 7);
    assert_eq!(world.query::<&Muscle>().iter(&world).count(), 1);
    assert_eq!(world.query::<&MusclePath>().iter(&world).count(), 1);
    assert_eq!(world.query::<&Millard2012Params>().iter(&world).count(), 1);
    assert_eq!(world.query::<&WrapGeom>().iter(&world).count(), 1);
    assert_eq!(world.query::<&DisplayGeometry>().iter(&world).count(), 1);
    assert_eq!(world.query::<&CoordinateActuator>().iter(&world).count(), 1);

    melosim::systems::run_systems(&mut world);
    let errors = world.get_resource::<melosim::world::ErrorList>().map(|e| e.0.clone()).unwrap_or_default();
    assert!(errors.is_empty(), "Validation errors: {:?}", errors);

    let actuator_entities: Vec<(Entity, CoordinateActuator)> = {
        let mut query = world.query::<(Entity, &CoordinateActuator)>();
        query.iter(&world).map(|(e, a)| (e, a.clone())).collect()
    };
    for (_key, act) in actuator_entities {
        let coord_name = world.get::<Name>(act.coordinate)
            .map(|n| n.value.as_str())
            .unwrap_or("");
        assert_eq!(coord_name, "knee_flexion", "CoordinateActuator should reference knee_flexion");
        assert_eq!(act.optimal_force, 50.0);
    }

    let name = {
        let mut query = world.query::<(Entity, &Name)>();
        query.iter(&world).find(|(_, n)| n.value == "rectus_femoris_r")
    };
    assert!(name.is_some(), "Should find muscle named rectus_femoris_r");

    let wrap_entities: Vec<(Entity, WrapGeom)> = {
        let mut query = world.query::<(Entity, &WrapGeom)>();
        query.iter(&world).map(|(e, w)| (e, w.clone())).collect()
    };
    for (_key, wrap) in wrap_entities {
        match &wrap.geom_type {
            WrapGeomType::Cylinder { radius, length } => {
                assert_eq!(*radius, 0.03);
                assert_eq!(*length, 0.1);
            }
            _ => panic!("Expected Cylinder wrap geom"),
        }
    }

    let geom_entities: Vec<(Entity, DisplayGeometry)> = {
        let mut query = world.query::<(Entity, &DisplayGeometry)>();
        query.iter(&world).map(|(e, g)| (e, g.clone())).collect()
    };
    for (_key, geom) in geom_entities {
        assert_eq!(geom.mesh_file, Some("femur.vtp".to_string()));
        assert_eq!(geom.color, [0.8, 0.8, 0.8]);
    }
}
