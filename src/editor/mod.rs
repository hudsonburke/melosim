pub mod dock;
pub mod gizmo;
pub mod mesh_import;
pub mod models;
pub mod panels;
pub mod path_editor;
pub mod selection;
pub mod ui;
pub mod viewport;

use std::path::PathBuf;

/// A `.xml` model the user chose to import; processed by `process_model_imports`.
/// (Resource, not a `World`-param button system, so the egui pass tuple stays
/// chainable.)
#[derive(Default, Resource)]
pub struct PendingModelImport(pub Option<PathBuf>);

/// A save path the user chose for MuJoCo export; processed by `process_mujoco_export`.
#[derive(Default, Resource)]
pub struct PendingMujocoExport(pub Option<PathBuf>);

/// Toggleble helper windows. Both are hidden by default so they don't sit in
/// the middle of the viewport unless the user opens them from the View menu.
#[derive(Default, Resource)]
pub struct ToolPanels {
    pub import_mesh: bool,
    pub path_editor: bool,
}

fn process_model_imports(world: &mut World) {
    let Some(path) = world.resource_mut::<PendingModelImport>().0.take() else {
        return;
    };
    #[cfg(feature = "mujoco")]
    match crate::importer::import_mjcf(world, &path) {
        Ok(_) => info!("imported model: {}", path.display()),
        Err(e) => error!("model import failed: {e:?}"),
    }
    #[cfg(not(feature = "mujoco"))]
    {
        let _ = (&path, world);
        warn!("MuJoCo model import requires the `mujoco` feature");
    }
}

/// When `PendingMujocoExport` has a path, export the current model to MJCF
/// and write it to that file.
fn process_mujoco_export(world: &mut World) {
    let Some(path) = world.resource_mut::<PendingMujocoExport>().0.take() else {
        return;
    };
    #[cfg(feature = "mujoco")]
    {
        // Canonicalize the output path so CWD changes don't break it.
        let abs_path = path.canonicalize().unwrap_or_else(|_| path.clone());
        let parent = abs_path.parent().unwrap_or(std::path::Path::new(".")).to_path_buf();
        let mesh_dir = parent.join("assets");
        let _ = std::fs::create_dir_all(&mesh_dir);

        // MuJoCo resolves `meshdir` relative to CWD, not the XML's directory.
        // Temporarily switch CWD so compile() and save_xml() find the assets/.
        let saved_cwd = std::env::current_dir().ok();
        let _ = std::env::set_current_dir(&parent);

        let _dummy_root = Entity::PLACEHOLDER;
        match crate::exporter::to_mjcf_with_meshes(world, Some(&mesh_dir)) {
            Ok(mut spec) => {
                let compiled = match spec.compile() {
                    Ok(m) => m,
                    Err(e) => {
                        error!("MuJoCo compile failed: {e}");
                        if let Some(cwd) = saved_cwd { let _ = std::env::set_current_dir(&cwd); }
                        return;
                    }
                };
                match spec.save_xml(&abs_path) {
                    Ok(()) => info!("exported MuJoCo model to {} (meshes in {})", abs_path.display(), mesh_dir.display()),
                    Err(e) => error!("failed to write MuJoCo XML: {e}"),
                }
                std::mem::drop(compiled);
            }
            Err(e) => error!("MuJoCo export failed: {e:?}"),
        }

        // Restore original CWD.
        if let Some(cwd) = saved_cwd { let _ = std::env::set_current_dir(&cwd); }
    }
    #[cfg(not(feature = "mujoco"))]
    {
        let _ = &path;
        warn!("MuJoCo export requires the `mujoco` feature");
    }
}

use bevy::{
    camera_controller::free_camera::{FreeCamera, FreeCameraPlugin},
    dev_tools::infinite_grid::{InfiniteGrid, InfiniteGridPlugin, InfiniteGridSettings},
    gizmos::transform_gizmo::{TransformGizmoCamera, TransformGizmoPlugin},
    prelude::*,
};
use bevy_inspector_egui::bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

use selection::{clear_selection_on_escape, sync_selection_markers, Selection};
use ui::HierarchyClick;
use viewport::{frame_camera_to_model, select_on_click};

pub struct MelosimEditorPlugin;

impl Plugin for MelosimEditorPlugin {
    fn build(&self, app: &mut App) {
        // Plugins: native Bevy (grid, camera, picking, transform gizmo) + egui shell.
        app.add_plugins((
            InfiniteGridPlugin,
            FreeCameraPlugin,
            EguiPlugin::default(),
            TransformGizmoPlugin,
        ));

        // bevy_picking needs the mesh-picking backend to actually emit
        // `Pointer<Click>` events for our `select_on_click` observer (the
        // transform-gizmo example adds this explicitly).
        if !app.is_plugin_added::<bevy::picking::mesh_picking::MeshPickingPlugin>() {
            app.add_plugins(bevy::picking::mesh_picking::MeshPickingPlugin);
        }

        // Resources
        app.init_resource::<Selection>();
        app.init_resource::<models::ModelRegistry>();
        app.init_resource::<path_editor::PathEditor>();
        app.init_resource::<PendingModelImport>();
        app.init_resource::<PendingMujocoExport>();
        app.init_resource::<ToolPanels>();
        app.init_resource::<dock::EditorDockState>();
        app.init_resource::<HierarchyClick>();
        // Start with NO model loaded; the user picks one from the Model menu or
        // imports a MuJoCo model (so importing doesn't stack on top of MyoArm).
        app.insert_resource(models::SelectedModel(None));

        // Register model types for reflection so the inspector can edit them.
        app.register_type::<crate::model::Body>()
            .register_type::<crate::model::Frame>()
            .register_type::<crate::model::Site>()
            .register_type::<crate::model::Inertia>()
            .register_type::<crate::model::InertialProperties>()
            .register_type::<crate::model::Joint>()
            .register_type::<crate::model::Coordinate>()
            .register_type::<crate::model::CoordinateProperties>()
            .register_type::<crate::model::InitialConditions>()
            .register_type::<crate::model::CoordinateState>()
            .register_type::<crate::model::Twist>()
            .register_type::<crate::model::Coupling>()
            .register_type::<crate::model::CouplingKind>()
            .register_type::<crate::model::Muscle>()
            .register_type::<crate::model::HillTypeMuscleParams>()
            .register_type::<crate::model::Millard2012Params>()
            .register_type::<crate::model::MuscleState>()
            .register_type::<crate::model::PathEntities>()
            .register_type::<crate::model::PathElement>()
            .register_type::<crate::model::WrappingSurface>()
            .register_type::<crate::model::WrapRadius>()
            .register_type::<crate::model::Function>();

        // Systems
        app.add_systems(
            Update,
            (
                clear_selection_on_escape,
                sync_selection_markers,
                gizmo::sync_focus,
                gizmo::gizmo_mode_keys,
                gizmo::sync_freecam_to_egui,
                frame_camera_to_model,
                mesh_import::import_dropped_mesh,
                models::spawn_selected_model,
                models::tag_model_roots,
            ),
        );

        // Systems that need exclusive `World` access run standalone
        // (not in the Update tuple, which has a 16-param cap).
        app.add_systems(Update, process_model_imports);
        app.add_systems(Update, process_mujoco_export);

        // egui UI runs inside the egui primary context pass (after egui begins
        // the frame) — running it in `Update` panics because egui's fonts /
        // available-rect aren't set up before `Context::run()`.
        //
        // The former monolithic `editor_ui` is split into narrow-scoped systems:
        //   1. toolbar_panel      — top toolbar (5 params)
        //   2. hierarchy_panel    — left hierarchy tree (7 params)
        //   3. inspector_panel    — right inspector (11 params)
        //   4. apply_hierarchy_selection — reads click result, updates Selection
        // Each system queries only the resources/components it needs.
        app.add_systems(
            EguiPrimaryContextPass,
            (
                ui::toolbar_panel,
                ui::hierarchy_panel,
                ui::inspector_panel,
                ui::apply_hierarchy_selection,
                mesh_import::mesh_import_ui,
                path_editor::path_editor_ui,
                ui::tool_windows_toggle,
            )
                .chain(),
        );

        // 3D selection via bevy_picking. bevy_egui's `picking` feature
        // suppresses these events over egui windows (capture_pointer_input), so
        // interacting with panels/sliders never changes the 3D selection.
        app.add_observer(select_on_click);

        // Startup: scene setup + editor camera
        app.add_systems(Startup, setup_editor_scene);
    }
}

/// Editor scene: grid, camera, light. The 3D model renders in the central
/// viewport (whole window); egui panels overlay on top.
fn setup_editor_scene(mut commands: Commands) {
    commands.spawn((InfiniteGrid, InfiniteGridSettings::default()));

    commands.spawn((
        Camera3d::default(),
        // A sensible default view of the model near the origin; `F` reframes to
        // the exact model bounds.
        Transform::from_xyz(3.0, 2.0, 4.0).looking_at(Vec3::ZERO, Vec3::Y),
        FreeCamera::default(),
        TransformGizmoCamera,
    ));

    commands.spawn((
        DirectionalLight {
            illuminance: 10000.0,
            ..default()
        },
        Transform::from_translation(Vec3::X * 15. + Vec3::Y * 20.)
            .looking_at(Vec3::ZERO, Vec3::Y),
    ));
}

// Re-export the old name for backwards compatibility
pub use MelosimEditorPlugin as EditorPlugin;
