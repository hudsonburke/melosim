pub mod hierarchy;
pub mod inspector;
pub mod selection;
pub mod toolbar;
pub mod viewport;

use bevy::{
    camera_controller::free_camera::{FreeCamera, FreeCameraPlugin},
    dev_tools::infinite_grid::{InfiniteGrid, InfiniteGridPlugin, InfiniteGridSettings},
    feathers::{
        dark_theme::create_dark_theme,
        theme::UiTheme,
        FeathersPlugins,
    },
    prelude::*,
};

use selection::{clear_selection_on_escape, sync_selection_markers, Selection};
use viewport::{click_to_select, draw_selection_highlight};

pub struct MelosimEditorPlugin;

impl Plugin for MelosimEditorPlugin {
    fn build(&self, app: &mut App) {
        // Plugins
        app.add_plugins((
            InfiniteGridPlugin,
            FreeCameraPlugin,
            FeathersPlugins,
        ));

        // Theme
        app.insert_resource(UiTheme(create_dark_theme()));

        // Resources
        app.init_resource::<Selection>();

        // Register model types for reflection
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

        // Systems: Update schedule
        app.add_systems(
            Update,
            (
                click_to_select,
                clear_selection_on_escape,
                sync_selection_markers,
                toolbar::update_toolbar,
            ),
        );
        app.add_systems(
            Update,
            hierarchy::rebuild_hierarchy,
        );
        app.add_systems(
            Update,
            hierarchy::update_hierarchy_selection,
        );
        app.add_systems(
            Update,
            (
                inspector::gather_inspector_data,
                inspector::render_inspector,
            ),
        );

        // Systems: PostUpdate (gizmos)
        app.add_systems(PostUpdate, draw_selection_highlight);

        // Startup: scene setup + UI
        app.add_systems(Startup, setup_editor_scene);
        app.add_systems(Startup, toolbar::setup_toolbar);
        app.add_systems(Startup, hierarchy::setup_hierarchy);
        app.add_systems(Startup, inspector::setup_inspector);

        // Observers: clicking a hierarchy row selects the model entity.
        app.add_observer(hierarchy::on_hierarchy_row_click);
    }
}

fn setup_editor_scene(mut commands: Commands) {
    // Infinite grid
    commands.spawn((InfiniteGrid, InfiniteGridSettings::default()));

    // Editor camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-12.5, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        FreeCamera::default(),
    ));

    // Directional light
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
