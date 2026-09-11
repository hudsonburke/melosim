#![cfg(feature = "mujoco")]

use bevy::prelude::*;
use melosim::{
    exporter::{save_mjcf, to_mjcf},
    importer::import_mjcf,
    model::MeshGeometry,
};
use mujoco_rs::wrappers::mj_editing::MjSpec;
use std::{path::Path, sync::Arc};

fn app() -> App {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    app
}

#[test]
fn native_mesh_rendering_and_portable_export() {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let path = fixtures.join("mesh_sources.xml");
    let mut spec = MjSpec::from_xml(&path).unwrap();
    for mesh in spec.mesh_iter_mut() {
        mesh.set_file(&fixtures.join(mesh.file()).to_string_lossy());
    }
    let compiled = spec.compile().unwrap();
    let mut app = app();
    let root = import_mjcf(app.world_mut(), &path).unwrap();
    app.update();
    let geoms = app
        .world_mut()
        .query::<(&MeshGeometry, &Transform)>()
        .iter(app.world())
        .map(|(g, t)| (g.clone(), *t))
        .collect::<Vec<_>>();
    assert_eq!(geoms.len(), 2, "unnamed geoms must render too");
    assert!(Arc::ptr_eq(&geoms[0].0.source, &geoms[1].0.source));
    assert_eq!(geoms[0].0.source.vertices, compiled.mesh_vert());
    assert_eq!(geoms[0].0.source.faces, compiled.mesh_face());
    let exported = to_mjcf(app.world_mut(), root).unwrap().compile().unwrap();
    assert_eq!(exported.nmesh(), 1);
    assert_eq!(exported.ngeom(), 2);
    for i in 0..2 {
        assert_eq!(compiled.geom_contype()[i], exported.geom_contype()[i]);
        assert_eq!(
            compiled.geom_conaffinity()[i],
            exported.geom_conaffinity()[i]
        );
        for (a, b) in compiled.geom_pos()[i].iter().zip(exported.geom_pos()[i]) {
            assert!((a - b).abs() < 2e-6);
        }
        let qa = Quat::from_array([
            compiled.geom_quat()[i][1] as f32,
            compiled.geom_quat()[i][2] as f32,
            compiled.geom_quat()[i][3] as f32,
            compiled.geom_quat()[i][0] as f32,
        ]);
        let qb = Quat::from_array([
            exported.geom_quat()[i][1] as f32,
            exported.geom_quat()[i][2] as f32,
            exported.geom_quat()[i][3] as f32,
            exported.geom_quat()[i][0] as f32,
        ]);
        assert!(qa.dot(qb).abs() > 0.99999);
    }
    let dir = std::env::temp_dir().join(format!(
        "melosim_mesh_package_{}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::create_dir(&dir).unwrap();
    let output = dir.join("model.xml");
    save_mjcf(app.world_mut(), root, &output).unwrap();
    let xml = std::fs::read_to_string(&output).unwrap();
    assert!(!xml.contains(&fixtures.to_string_lossy().to_string()));
    let mut reloaded = World::new();
    import_mjcf(&mut reloaded, &output).unwrap();
    let sources = reloaded
        .query::<&MeshGeometry>()
        .iter(&reloaded)
        .collect::<Vec<_>>();
    assert_eq!(sources.len(), 2);
    let copy = sources[0].source.file.as_ref().unwrap();
    assert!(copy.starts_with(&dir));
    assert_eq!(
        std::fs::read(copy).unwrap(),
        std::fs::read(fixtures.join("asymmetric.obj")).unwrap()
    );
    std::fs::remove_dir_all(dir).unwrap();
}

#[test]
fn bundled_arm_renders_stl_without_gltf_assets() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("assets/myo_sim/myo_sim/models/arm/myoarm_r.xml");
    let mut app = app();
    import_mjcf(app.world_mut(), &path).unwrap();
    let world = app.world_mut();
    let count = world.query::<&MeshGeometry>().iter(world).count();
    // The arm declares 33 assets but instantiates 32; thorax is unused.
    assert_eq!(count, 32);
    assert!(
        world.query::<&MeshGeometry>().iter(world).all(|g| g
            .source
            .file
            .as_ref()
            .unwrap()
            .extension()
            .unwrap()
            == "stl")
    );
}
