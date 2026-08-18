use bevy::prelude::*;

/// Marker for the toolbar root entity.
#[derive(Component)]
pub struct ToolbarRoot;

/// Marker for hierarchy toggle checkbox state.
#[derive(Resource)]
pub struct HierarchyVisible(pub bool);

impl Default for HierarchyVisible {
    fn default() -> Self {
        Self(true)
    }
}

/// Marker for inspector toggle checkbox state.
#[derive(Resource)]
pub struct InspectorVisible(pub bool);

impl Default for InspectorVisible {
    fn default() -> Self {
        Self(true)
    }
}

/// Setup the toolbar UI.
pub fn setup_toolbar(mut commands: Commands) {
    commands.insert_resource(HierarchyVisible::default());
    commands.insert_resource(InspectorVisible::default());

    // Spawn toolbar as a top bar
    commands.spawn((
        ToolbarRoot,
        Node {
            width: percent(100),
            height: px(32),
            flex_direction: FlexDirection::Row,
            align_items: AlignItems::Center,
            padding: UiRect::horizontal(px(8)),
            column_gap: px(4),
            ..default()
        },
        BackgroundColor(bevy::color::palettes::css::DARK_GRAY.into()),
    ));
}

/// Update system (placeholder for dynamic toolbar updates).
pub fn update_toolbar() {
    // Toolbar is static for now
}
