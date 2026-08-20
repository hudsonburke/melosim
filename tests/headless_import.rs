//! Headless import mode for debugging: imports an MJCF into a real App with
//! `TransformPlugin` (needed for `GlobalTransform` propagation), then dumps
//! every `Body` entity's `Name` + `GlobalTransform` and every `Muscle` entity's
//! `PathEntities` for comparison against MuJoCo's viewer.
//!
//! Run with `--features mujoco`:
//!
//! ```sh
//! # Use a real model (set TEST_MJCF to the path of your .xml file):
//! TEST_MJCF=/path/to/model.xml cargo test --features mujoco --test headless_import -- --nocapture
//!
//! # Or run with the built-in 2-body test:
//! cargo test --features mujoco --test headless_import -- --nocapture
//! ```

#![cfg(feature = "mujoco")]

use bevy::app::App;
use bevy::prelude::*;

use melosim::importer::import_mjcf;
use melosim::model::Body;

#[test]
fn headless_import() {
    let xml_path = if let Ok(path) = std::env::var("TEST_MJCF") {
        eprintln!("headless import: using {path}");
        let abs = std::path::PathBuf::from(&path)
            .canonicalize()
            .expect("canonicalize");
        // MjSpec resolves `<include>` relative to the CWD.
        let parent = abs.parent().unwrap_or(std::path::Path::new("."));
        std::env::set_current_dir(parent).expect("set_current_dir");
        abs
    } else {
        let dir = std::env::temp_dir().join("melosim_headless");
        std::fs::create_dir_all(&dir).ok();
        let p = dir.join("model.xml");
        std::fs::write(
            &p,
            r#"<mujoco model="headless_test">
          <worldbody>
            <body name="A" pos="0 0 1">
              <inertial pos="0 0 0" mass="0.1" diaginertia="1e-5 1e-5 1e-5"/>
              <body name="B" pos="0 0 0.5">
                <inertial pos="0 0 0" mass="0.1" diaginertia="1e-5 1e-5 1e-5"/>
              </body>
            </body>
          </worldbody>
        </mujoco>"#,
        )
        .expect("write xml");
        p
    };

    // ── Create a real App so TransformPlugin propagates GlobalTransform ──
    let mut app = App::new();
    app.add_plugins(bevy::MinimalPlugins);
    app.add_plugins(bevy::transform::TransformPlugin::default());

    {
        let mut world = app.world_mut();
        let _root = import_mjcf(&mut world, &xml_path).expect("import_mjcf failed");
    }
    app.update();

    // ── Dump every Body entity ──
    let mut world = app.world_mut();
    let mut body_q = world.query_filtered::<(Entity, &Name, &GlobalTransform), With<Body>>();
    eprintln!("\n--- bodies ({}) ---", body_q.iter(&world).count());
    for (_ent, name, gt) in body_q.iter(world) {
        let t = gt.translation();
        eprintln!("  {name:30}  pos=[{:8.4}, {:8.4}, {:8.4}]", t.x, t.y, t.z);
    }

    // ── Dump every Muscle entity and its PathEntities ──
    use melosim::model::{Muscle, PathEntities};
    let muscle_data: Vec<(String, Vec<Entity>)> = {
        let mut mq = world.query::<(&Name, &PathEntities)>();
        mq.iter(world)
            .map(|(n, p)| (n.as_str().to_owned(), p.iter().collect()))
            .collect()
    };
    eprintln!("\n--- muscles ({}) ---", muscle_data.len());
    for (name, path) in &muscle_data {
        eprintln!("  '{name}' → {} sites:", path.len());
        for &s in path {
            if let Ok((sname, sgt)) = world.query::<(&Name, &GlobalTransform)>().get(world, s) {
                let t = sgt.translation();
                eprintln!("    site '{sname}' @ [{:.4}, {:.4}, {:.4}]", t.x, t.y, t.z);
            }
        }
    }
    eprintln!("--- done ({})\n", xml_path.display());
}
