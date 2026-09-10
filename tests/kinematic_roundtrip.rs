#![cfg(feature = "mujoco")]

use bevy::prelude::*;
use melosim::{exporter::to_mjcf, importer::import_mjcf};
use mujoco_rs::wrappers::{mj_editing::MjSpec, mj_model::MjtObj};

fn close(actual: &[f64], expected: &[f64]) {
    assert_eq!(actual.len(), expected.len());
    for (a, e) in actual.iter().zip(expected) {
        assert!((a - e).abs() < 2e-6, "{actual:?} != {expected:?}");
    }
}

#[test]
fn preserves_rotated_body_joint_site_and_inertia_frames() {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("tests/fixtures/rotated_arm.xml");
    let source = MjSpec::from_xml(&path).unwrap().compile().unwrap();
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    let root = import_mjcf(app.world_mut(), &path).unwrap();
    app.update();
    let result = to_mjcf(app.world_mut(), root).unwrap().compile().unwrap();
    assert_eq!(source.nbody(), result.nbody());
    assert_eq!(source.njnt(), result.njnt());
    assert_eq!(source.nsite(), result.nsite());
    for i in 1..source.nbody() as usize {
        assert_eq!(source.id_to_name(MjtObj::mjOBJ_BODY, i), result.id_to_name(MjtObj::mjOBJ_BODY, i));
        close(&result.body_pos()[i], &source.body_pos()[i]);
        close(&result.body_quat()[i], &source.body_quat()[i]);
        close(&result.body_ipos()[i], &source.body_ipos()[i]);
        // Compare tensors, since principal-axis quaternion signs are arbitrary.
        let tensor = |q: [f64; 4], d: [f64; 3]| {
            let r = nalgebra::UnitQuaternion::new_normalize(nalgebra::Quaternion::new(q[0], q[1], q[2], q[3])).to_rotation_matrix();
            r.matrix() * nalgebra::Matrix3::from_diagonal(&nalgebra::Vector3::from(d)) * r.matrix().transpose()
        };
        close(tensor(result.body_iquat()[i], result.body_inertia()[i]).as_slice(),
            tensor(source.body_iquat()[i], source.body_inertia()[i]).as_slice());
    }
    for i in 0..source.njnt() as usize {
        assert_eq!(source.jnt_bodyid()[i], result.jnt_bodyid()[i]);
        assert_eq!(source.id_to_name(MjtObj::mjOBJ_JOINT, i), result.id_to_name(MjtObj::mjOBJ_JOINT, i));
        close(&result.jnt_pos()[i], &source.jnt_pos()[i]);
        close(&result.jnt_axis()[i], &source.jnt_axis()[i]);
        close(&result.jnt_range()[i], &source.jnt_range()[i]);
    }
    for i in 0..source.nsite() as usize {
        close(&result.site_pos()[i], &source.site_pos()[i]);
    }
}
