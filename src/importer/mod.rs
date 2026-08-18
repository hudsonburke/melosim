//! Model import: external formats (OpenSim `.osim`, MuJoCo `.xml`) → ECS.
//!
//! Files are parsed by the *reference implementations* — OpenSim's Python
//! API (PyO3, `opensim` feature) and MuJoCo's `MjSpec` (`mujoco` feature) —
//! never by hand-rolled XML parsing. Each loader produces the format-neutral
//! [`ModelData`] asset; [`spawn_model`] then builds the same entity
//! structure that hand-authored `bsn!` models use:
//!
//! ```text
//! ModelSource entity
//!   └─ Body
//!        ├─ FixedFrame (geometry, with Mesh3d when an AssetServer exists)
//!        └─ FixedFrame (joint parent offset)
//!             └─ Joint
//!                  ├─ Coordinate + CoordinateState (per coordinate)
//!                  ├─ Twist + DrivesCoordinate (+ Function) (per axis)
//!                  └─ FixedFrame (child offset)
//!                       └─ Body (child, reparented here)
//! ```
//!
//! Usage: put `ModelSource(asset_server.load("model.osim"))` on an entity;
//! the spawn system builds the model as its children once the asset loads.

pub mod ir;
pub use ir::*;

#[cfg(feature = "mujoco")]
pub mod mujoco;
#[cfg(feature = "opensim")]
pub mod opensim;

use bevy::prelude::*;

/// Registers the `ModelData` asset, the format loaders, and the spawn
/// system. `asset_root` must match `AssetPlugin.file_path` — the loaders
/// join it with asset paths to get real filesystem paths for the
/// path-bound reference parsers (OpenSim's `Model`, MuJoCo's `MjSpec`).
pub struct ImporterPlugin {
    asset_root: std::path::PathBuf,
}

impl ImporterPlugin {
    pub fn new(asset_root: impl Into<std::path::PathBuf>) -> Self {
        Self { asset_root: asset_root.into() }
    }
}

impl Default for ImporterPlugin {
    fn default() -> Self {
        Self::new("assets")
    }
}

impl Plugin for ImporterPlugin {
    fn build(&self, app: &mut App) {
        app.insert_resource(AssetRoot(self.asset_root.clone()))
            .init_asset::<ModelData>()
            .add_systems(Update, (spawn_loaded_models, handle_file_drop));
        #[cfg(feature = "mujoco")]
        app.register_asset_loader(mujoco::MjcfLoader {
            asset_root: self.asset_root.clone(),
        });
        #[cfg(feature = "opensim")]
        app.register_asset_loader(opensim::OpensimLoader {
            asset_root: self.asset_root.clone(),
        });
    }
}

/// The asset root directory, for resolving drag-and-drop file paths.
#[derive(Resource)]
pub struct AssetRoot(pub std::path::PathBuf);

/// Put this on an entity to populate it with a loaded model's structure.
///
/// `Transform`/`Visibility` are required so the model root is a proper
/// propagation node — without them the spawned tree would never receive
/// `GlobalTransform` updates (warning B0004, one level up).
#[derive(Component)]
#[require(Transform, Visibility)]
pub struct ModelSource(pub Handle<ModelData>);

/// Marker: this `ModelSource` entity has already been populated.
#[derive(Component)]
struct ModelSpawned;

fn spawn_loaded_models(world: &mut World) {
    let mut sources =
        world.query_filtered::<(Entity, &ModelSource), Without<ModelSpawned>>();
    let pending: Vec<(Entity, Handle<ModelData>)> = sources
        .iter(world)
        .map(|(e, s)| (e, s.0.clone()))
        .collect();

    for (entity, handle) in pending {
        let spawned = world.resource_scope(|world, models: Mut<Assets<ModelData>>| {
            models.get(&handle).map(|model| spawn_model(world, entity, model))
        });
        if spawned.is_some() {
            world.entity_mut(entity).insert(ModelSpawned);
        }
    }
}

/// Handle drag-and-drop: when a `.osim` or `.xml` file is dropped onto
/// the window, load it as a `ModelData` asset and spawn a `ModelSource`.
fn handle_file_drop(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    asset_root: Res<AssetRoot>,
    window_events: Option<
        bevy::ecs::message::MessageReader<bevy::window::WindowEvent>,
    >,
) {
    let Some(mut window_events) = window_events else {
        warn_once!("file drop: WindowEvent message not available");
        return;
    };
    for event in window_events.read() {
        match event {
            bevy::window::WindowEvent::FileDragAndDrop(
                bevy::window::FileDragAndDrop::DroppedFile { path_buf, .. },
            ) => {
                info!("file dropped: {}", path_buf.display());

                // Resolve the absolute dropped path to an asset-root-relative
                // path. Canonicalize the asset root first so strip_prefix
                // works against the absolute path from the OS.
                let canon_root = std::fs::canonicalize(&asset_root.0)
                    .unwrap_or_else(|e| {
                        warn!("file drop: cannot canonicalize asset root {:?}: {e}", asset_root.0);
                        asset_root.0.clone()
                    });
                info!("asset root: {}", canon_root.display());

                let asset_path = path_buf
                    .strip_prefix(&canon_root)
                    .map_err(|e| {
                        warn!(
                            "file drop: {} is not under asset root {}: {e}",
                            path_buf.display(),
                            canon_root.display(),
                        );
                    })
                    .map(|p| p.to_string_lossy().into_owned());

                let Ok(asset_path) = asset_path else {
                    continue;
                };

                if !asset_path.ends_with(".osim") && !asset_path.ends_with(".xml") {
                    warn!("file drop: unsupported file type: {asset_path}");
                    continue;
                }

                info!("file drop: loading {asset_path}");
                commands.spawn(ModelSource(asset_server.load(asset_path.clone())));
            }
            bevy::window::WindowEvent::FileDragAndDrop(
                bevy::window::FileDragAndDrop::HoveredFile { path_buf, .. },
            ) => {
                info!("file hover: {}", path_buf.display());
            }
            _ => {}
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Body;

    fn pipeline_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            AssetPlugin {
                file_path: "tests/fixtures".into(),
                ..default()
            },
        ))
        .add_plugins(ImporterPlugin::new("tests/fixtures"));
        app
    }

    fn count_bodies(app: &mut App) -> usize {
        app.world_mut()
            .query_filtered::<Entity, With<Body>>()
            .iter(app.world())
            .count()
    }

    /// Pump updates until the model under `handle` has spawned bodies
    /// (async asset loads complete in wall-clock time, not frame counts).
    fn wait_for_spawn(app: &mut App, handle: &Handle<ModelData>) -> usize {
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        loop {
            app.update();
            let bodies = count_bodies(app);
            if bodies > 0 {
                return bodies;
            }
            assert!(
                std::time::Instant::now() < deadline,
                "model never spawned; load state: {:?}",
                app.world().resource::<AssetServer>().load_state(handle)
            );
            std::thread::sleep(std::time::Duration::from_millis(10));
        }
    }

    /// End-to-end: `ModelSource` + asset path → registered loader parses the
    /// file asynchronously → spawn system builds the body tree.
    #[test]
    fn loads_and_spawns_mjcf_via_asset_pipeline() {
        let mut app = pipeline_app();
        let handle = app
            .world()
            .resource::<AssetServer>()
            .load("myo_sim/osl/myolegs_osl.xml");
        app.world_mut().spawn(ModelSource(handle.clone()));

        assert!(wait_for_spawn(&mut app, &handle) > 10);
    }

    #[cfg(feature = "opensim")]
    #[test]
    fn loads_and_spawns_osim_via_asset_pipeline() {
        let mut app = pipeline_app();
        let handle = app
            .world()
            .resource::<AssetServer>()
            .load("Rajagopal/Rajagopal2015.osim");
        app.world_mut().spawn(ModelSource(handle.clone()));

        assert_eq!(wait_for_spawn(&mut app, &handle), 23, "22 bodies + ground");
    }
}
