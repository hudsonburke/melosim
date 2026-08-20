//! Headless import mode for debugging: imports an MJCF into a real App with
//! `TransformPlugin` (needed for `GlobalTransform` propagation), then dumps
//! every `Body` entity's `Name` + `GlobalTransform` as a table for comparison
//! against MuJoCo's `mj_printData` / viewer.
//!
//! Run with `--features mujoco`:
//!
//! ```sh
//! nix develop --command cargo test --features mujoco -- --nocapture headless_import
//! ```

#![cfg(feature = "mujoco")]

use bevy::app::App;
use bevy::prelude::*;

use melosim::importer::import_mjcf;
use melosim::model::Body;

#[test]
fn headless_import() {
    // ── Build a minimal model that exercises the transforms we care about ──
    //
    //  root (worldbody)
    //    body_A  pos=[0 0 1]
    //      body_B  pos=[0 0 0.5]
    //
    // After import + one transform-propagation step, dump body_A and body_B
    // global positions.  If the import is correct:
    //   body_A ≈ [0, 1, 0]   (zup: x, z, -y  →  [0, 1, 0])
    //   body_B ≈ [0, 1.5, 0] (zup: [0, 1.5, 0])
    //
    // For a REAL model test, set `TEST_MJCF` to the path of your .xml file.

    let xml = if let Ok(path) = std::env::var("TEST_MJCF") {
        eprintln!("headless import: using {path}");
        std::fs::read_to_string(&path).expect("read TEST_MJCF")
    } else {
        r#"<mujoco model="headless_test">
          <worldbody>
            <body name="A" pos="0 0 1">
              <inertial pos="0 0 0" mass="0.1" diaginertia="1e-5 1e-5 1e-5"/>
              <body name="B" pos="0 0 0.5">
                <inertial pos="0 0 0" mass="0.1" diaginertia="1e-5 1e-5 1e-5"/>
              </body>
            </body>
          </worldbody>
        </mujoco>"#
            .to_owned()
    };

    let dir = std::env::temp_dir().join("melosim_headless");
    std::fs::create_dir_all(&dir).ok();
    let xml_path = dir.join("model.xml");
    std::fs::write(&xml_path, &xml).expect("write xml");

    // ── Create a real App so TransformPlugin propagates GlobalTransform ──
    let mut app = App::new();
    app.add_plugins(bevy::MinimalPlugins);
    app.add_plugins(bevy::transform::TransformPlugin::default());

    {
        let mut world = app.world_mut();
        let _root = import_mjcf(&mut world, &xml_path).expect("import_mjcf failed");
    }

    // One update step to propagate GlobalTransform.
    app.update();

    // ── Dump every Body entity ──
    let mut world = app.world_mut();
    let mut query = world.query_filtered::<(Entity, &Name, &GlobalTransform), With<Body>>();
    eprintln!("\n--- headless import: body transforms ---");
    for (_ent, name, gt) in query.iter(world) {
        let t = gt.translation();
        let r = gt.rotation();
        eprintln!(
            "  {name:30}  pos=[{:8.4}, {:8.4}, {:8.4}]  quat=[{:8.5}, {:8.5}, {:8.5}, {:8.5}]",
            t.x, t.y, t.z, r.w, r.x, r.y, r.z,
        );
    }
    eprintln!("--- done ({})\n", xml_path.display());

    // ── Basic sanity checks ──
    assert!(
        query.iter(world).count() >= 2,
        "expected at least 2 bodies (A, B)"
    );
}
