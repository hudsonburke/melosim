//! Round-trip test: import a myo_sim MJCF into the melosim World, export it
//! back via `to_mjcf`, save to XML, reload in MjSpec, and compare counts.
//!
//! Requires `--features mujoco` and `TEST_MJCF` pointing to a myo_sim model.
//!
//! ```sh
//! TEST_MJCF=/path/to/myolegs_osl.xml cargo test --features mujoco --test roundtrip -- --nocapture
//! ```

#![cfg(feature = "mujoco")]

use bevy::app::App;
use bevy::ecs::component::Component;
use bevy::prelude::*;
use melosim::exporter::to_mjcf;
use melosim::importer::import_mjcf;
use melosim::model::{Body, Coordinate, Joint, Muscle, Site};
use mujoco_rs::wrappers::mj_editing::MjSpec;

fn count<T: Component>(world: &mut World) -> usize {
    world.query_filtered::<Entity, With<T>>().iter(world).count()
}

#[test]
fn roundtrip_myo_sim() {
    let path = std::env::var("TEST_MJCF").unwrap_or_else(|_| format!("{}/assets/myo_sim/myo_sim/models/arm/myoarm_r.xml", env!("CARGO_MANIFEST_DIR")));
    eprintln!("roundtrip: importing {path}");

    let mut app = App::new();
    app.add_plugins(bevy::MinimalPlugins);
    app.add_plugins(bevy::transform::TransformPlugin::default());

    // ── 1. Import ── (import_mjcf handles CWD switching internally)
    let root = import_mjcf(app.world_mut(), std::path::Path::new(&path)).expect("import failed");
    app.update();

    // Count entities.
    let (bodies_i, joints_i, sites_i, muscles_i, coords_i) = {
        let mut w = app.world_mut();
        (
            count::<Body>(&mut w),
            count::<Joint>(&mut w),
            count::<Site>(&mut w),
            count::<Muscle>(&mut w),
            count::<Coordinate>(&mut w),
        )
    };
    eprintln!("import: bodies={bodies_i} joints={joints_i} sites={sites_i} muscles={muscles_i} coords={coords_i}");

    // ── 2. Export to MjSpec ──
    let mut spec = to_mjcf(app.world_mut(), root).expect("export failed");

    // ── 3. Compile + save ──
    spec.compile().expect("MjSpec compile failed");
    let xml = spec.save_xml_string(1 << 20).expect("save_xml_string failed");
    eprintln!("exported XML: {} bytes", xml.len());

    let dir = std::env::temp_dir().join("melosim_roundtrip");
    std::fs::create_dir_all(&dir).ok();
    let out_path = dir.join("roundtrip.xml");
    std::fs::write(&out_path, &xml).expect("write XML");

    // ── 4. Reload + compare ──
    let mut reimported = MjSpec::from_xml(&out_path).expect("reimport failed");
    let recompiled = reimported.compile().expect("recompile failed");
    let bodies_o = recompiled.nbody() as usize;
    let joints_o = recompiled.njnt() as usize;
    let sites_o = recompiled.nsite() as usize;
    assert_eq!(bodies_o, bodies_i + 1, "body count including world");
    assert_eq!(joints_o, coords_i, "every scalar coordinate must survive export");
    assert_eq!(sites_o, sites_i, "every site must survive export");

    eprintln!("reimport: bodies={bodies_o} joints={joints_o} sites={sites_o}");
    eprintln!("original: bodies={bodies_i} joints={joints_i} sites={sites_i}");

    // Compare bodies (allow +1 for a possible ground/worldbody).
    eprintln!("bodies: {bodies_i} -> {bodies_o} (MuJoCo compilation may simplify)");
    eprintln!("joint count: import={joints_i} reimport={joints_o}");
    eprintln!("site count:  import={sites_i} reimport={sites_o}");

    // Dump the exported XML for debugging.
    if let Ok(content) = std::fs::read_to_string(&out_path) {
        eprintln!("--- exported XML (first 2000 chars) ---");
        eprintln!("{}", &content[..content.len().min(2000)]);
    }

    eprintln!("roundtrip check complete — {}", out_path.display());
}
