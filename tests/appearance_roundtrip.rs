#![cfg(feature = "mujoco")]
use bevy::prelude::*;
use melosim::{
    exporter::{save_mjcf, to_mjcf},
    importer::import_mjcf,
};
use mujoco_rs::wrappers::mj_editing::MjSpec;
use mujoco_rs::wrappers::mj_model::{MjModel, MjtObj};
use std::path::Path;

fn close(a: &[f64], b: &[f64]) {
    assert_eq!(a.len(), b.len());
    for (a, b) in a.iter().zip(b) {
        assert!((a - b).abs() < 2e-6, "{a} != {b}");
    }
}
fn rotation(a: [f64; 4], mut b: [f64; 4]) {
    if a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>() < 0.0 {
        b = b.map(|v| -v);
    }
    close(&a, &b);
}

#[test]
fn edited_gray_overrides_imported_material() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/appearance.xml");
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    let root = import_mjcf(app.world_mut(), &path).unwrap();
    app.update();
    for mut geometry in app
        .world_mut()
        .query::<&mut melosim::model::MeshGeometry>()
        .iter_mut(app.world_mut())
    {
        geometry.rgba = [0.5, 0.5, 0.5, 1.0];
    }
    let exported = to_mjcf(app.world_mut(), root).unwrap().compile().unwrap();
    let geom = exported
        .name_to_id(MjtObj::mjOBJ_GEOM, "bone_geom")
        .unwrap();
    let material = exported.geom_matid()[geom] as usize;
    assert_eq!(exported.mat_rgba()[material], [0.5, 0.5, 0.5, 1.0]);
}
fn appearance(a: &MjModel, b: &MjModel) {
    assert_eq!(a.ngeom(), b.ngeom());
    assert_eq!(a.nmat(), b.nmat());
    assert_eq!(a.ntex(), b.ntex());
    assert_eq!(a.ncam(), b.ncam());
    assert_eq!(a.nlight(), b.nlight());
    assert_eq!(a.tex_data(), b.tex_data());
    assert_eq!(a.nbody(), b.nbody());
    for i in 1..a.nbody() as usize {
        let name = a.id_to_name(MjtObj::mjOBJ_BODY, i).unwrap();
        let j = b.name_to_id(MjtObj::mjOBJ_BODY, name).unwrap();
        close(&a.body_pos()[i], &b.body_pos()[j]);
        rotation(a.body_quat()[i], b.body_quat()[j]);
    }
    for i in 0..a.ncam() as usize {
        close(&a.cam_pos()[i], &b.cam_pos()[i]);
        rotation(a.cam_quat()[i], b.cam_quat()[i]);
    }
    close(a.cam_fovy(), b.cam_fovy());
    assert_eq!(a.light_pos(), b.light_pos());
    assert_eq!(a.light_dir(), b.light_dir());
    assert_eq!(a.light_diffuse(), b.light_diffuse());
    for i in 0..a.ngeom() as usize {
        let name = a.id_to_name(MjtObj::mjOBJ_GEOM, i).unwrap();
        let j = b.name_to_id(MjtObj::mjOBJ_GEOM, name).unwrap();
        assert_eq!(a.geom_type()[i], b.geom_type()[j]);
        close(&a.geom_size()[i], &b.geom_size()[j]);
        close(&a.geom_pos()[i], &b.geom_pos()[j]);
        rotation(a.geom_quat()[i], b.geom_quat()[j]);
        assert_eq!(a.geom_matid()[i], b.geom_matid()[j]);
        close(
            &a.geom_rgba()[i].map(f64::from),
            &b.geom_rgba()[j].map(f64::from),
        );
    }
    for i in 0..a.nmat() as usize {
        let name = a.id_to_name(MjtObj::mjOBJ_MATERIAL, i).unwrap();
        let j = b.name_to_id(MjtObj::mjOBJ_MATERIAL, name).unwrap();
        assert_eq!(a.mat_rgba()[i], b.mat_rgba()[j]);
        assert_eq!(a.mat_shininess()[i], b.mat_shininess()[j]);
        assert_eq!(a.mat_specular()[i], b.mat_specular()[j]);
        assert_eq!(a.mat_reflectance()[i], b.mat_reflectance()[j]);
        assert_eq!(a.mat_texrepeat()[i], b.mat_texrepeat()[j]);
        assert_eq!(a.mat_emission()[i], b.mat_emission()[j]);
        assert_eq!(a.mat_texid()[i], b.mat_texid()[j]);
    }
}

#[test]
fn bundled_myoarm_preserves_all_scene_geoms_and_materials() {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("assets/myo_sim/myo_sim/models/arm/myoarm_r.xml");
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    let root = import_mjcf(app.world_mut(), &path).unwrap();
    app.update();
    let document = app
        .world()
        .get::<melosim::mjcf_document::MjcfDocument>(root)
        .unwrap();
    assert!(document.warning.is_some());
    assert!(document.xml.contains("PECM2_PECM2-P3_r"));
    let source = document.visual_only().unwrap().xml;
    let original = MjSpec::from_xml_string(&source).unwrap().compile().unwrap();
    assert!(to_mjcf(app.world_mut(), root).is_err());
    let exported = melosim::exporter::to_mjcf_visual(app.world_mut(), root)
        .unwrap()
        .compile()
        .unwrap();
    appearance(&original, &exported);
    assert_eq!(original.ntendon(), exported.ntendon());
    assert_eq!(original.nu(), exported.nu());
}

#[test]
fn packages_external_textures_and_meshes() {
    let directory = std::env::temp_dir().join(format!("melosim-texture-{}", std::process::id()));
    let source = directory.join("input");
    let output = directory.join("output");
    std::fs::create_dir_all(&source).unwrap();
    std::fs::create_dir_all(&output).unwrap();
    // A valid 1x1 RGB PNG. No image renderer dependency is needed by the tests.
    let png = [
        137, 80, 78, 71, 13, 10, 26, 10, 0, 0, 0, 13, 73, 72, 68, 82, 0, 0, 0, 1, 0, 0, 0, 1, 8, 2,
        0, 0, 0, 144, 119, 83, 222, 0, 0, 0, 12, 73, 68, 65, 84, 120, 156, 99, 248, 207, 192, 0, 0,
        3, 1, 1, 0, 201, 254, 146, 239, 0, 0, 0, 0, 73, 69, 78, 68, 174, 66, 96, 130,
    ];
    std::fs::write(source.join("red.png"), png).unwrap();
    std::fs::write(source.join("model.xml"), r#"<mujoco><asset><texture name="red" type="2d" file="red.png"/><material name="paint" texture="red"/></asset><worldbody><body name="box"><geom name="box_geom" type="box" size="0.1 0.2 0.3" material="paint"/></body></worldbody></mujoco>"#).unwrap();
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    let root = import_mjcf(app.world_mut(), &source.join("model.xml")).unwrap();
    app.update();
    let xml = app
        .world()
        .get::<melosim::mjcf_document::MjcfDocument>(root)
        .unwrap()
        .xml
        .clone();
    let original = MjSpec::from_xml_string(&xml).unwrap().compile().unwrap();
    save_mjcf(app.world_mut(), root, &output.join("model.xml")).unwrap();
    std::fs::remove_dir_all(&source).unwrap();
    let mut reloaded = App::new();
    reloaded.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    let root = import_mjcf(reloaded.world_mut(), &output.join("model.xml")).unwrap();
    reloaded.update();
    appearance(
        &original,
        &to_mjcf(reloaded.world_mut(), root)
            .unwrap()
            .compile()
            .unwrap(),
    );
    std::fs::remove_dir_all(directory).unwrap();
}
#[test]
fn keeps_scene_appearance() {
    let fixtures = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let path = fixtures.join("appearance.xml");
    let mut spec = MjSpec::from_xml(&path).unwrap();
    for mesh in spec.mesh_iter_mut() {
        mesh.set_file(&fixtures.join(mesh.file()).to_string_lossy());
    }
    let original = spec.compile().unwrap();
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    let root = import_mjcf(app.world_mut(), &path).unwrap();
    app.update();
    let exported = to_mjcf(app.world_mut(), root).unwrap().compile().unwrap();
    appearance(&original, &exported);
    let dir = std::env::temp_dir().join(format!("melosim-appearance-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let output = dir.join("model.xml");
    save_mjcf(app.world_mut(), root, &output).unwrap();
    let mut reloaded = App::new();
    reloaded.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    let r = import_mjcf(reloaded.world_mut(), &output).unwrap();
    reloaded.update();
    appearance(
        &original,
        &to_mjcf(reloaded.world_mut(), r).unwrap().compile().unwrap(),
    );
    std::fs::remove_dir_all(dir).unwrap();
}
