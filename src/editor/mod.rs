pub mod gizmo;
pub mod mesh_import;
pub mod models;
pub mod selection;
pub mod ui;
pub mod viewport;

use bevy::{
    camera_controller::free_camera::{FreeCamera, FreeCameraPlugin},
    dev_tools::infinite_grid::{InfiniteGrid, InfiniteGridPlugin, InfiniteGridSettings},
    gizmos::transform_gizmo::{TransformGizmoCamera, TransformGizmoPlugin},
    prelude::*,
};
use bevy_inspector_egui::bevy_egui::{EguiPlugin, EguiPrimaryContextPass};

use selection::{clear_selection_on_escape, sync_selection_markers, Selection};
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
        app.init_resource::<mesh_import::MeshImport>();
        // Load the default model (MyoArm) at startup; the UI can change this.
        app.insert_resource(models::SelectedModel(Some(0)));

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

        // egui UI runs inside the egui primary context pass (after egui begins
        // the frame) — running it in `Update` panics because egui's fonts /
        // available-rect aren't set up before `Context::run()`.
        app.add_systems(
            EguiPrimaryContextPass,
            (ui::editor_ui, mesh_import::mesh_import_ui).chain(),
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
