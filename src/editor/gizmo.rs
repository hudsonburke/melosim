//! Transform gizmo integration using Bevy's built-in
//! `bevy_gizmos::transform_gizmo` (translate / rotate / scale, world or local).
//!
//! The editor mirrors its `Selection` onto the `TransformGizmoFocus` component,
//! marks the editor camera with `TransformGizmoCamera`, and exposes mode keys
//! (1=Translate, 2=Rotate, 3=Scale, X=World/Local). The plugin handles all the
//! gizmo dragging itself.

use bevy::{
    gizmos::transform_gizmo::{
        TransformGizmoFocus, TransformGizmoMode, TransformGizmoSettings, TransformGizmoSpace,
    },
    prelude::*,
};

use super::selection::Selection;

/// Keep exactly one `TransformGizmoFocus` matching the current selection.
/// Runs on selection change (not every frame).
pub fn sync_focus(
    selection: Res<Selection>,
    mut commands: Commands,
    focus_query: Query<Entity, With<TransformGizmoFocus>>,
    all: Query<()>,
) {
    if !selection.is_changed() {
        return;
    }
    for entity in &focus_query {
        commands.entity(entity).remove::<TransformGizmoFocus>();
    }
    if let Some(entity) = selection.primary() {
        if all.get(entity).is_ok() {
            commands.entity(entity).insert(TransformGizmoFocus);
        }
    }
}

/// Keyboard shortcuts for gizmo mode / space (1/2/3, X) — same as the Bevy example.
pub fn gizmo_mode_keys(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut settings: ResMut<TransformGizmoSettings>,
) {
    if keyboard.just_pressed(KeyCode::Digit1) {
        settings.mode = TransformGizmoMode::Translate;
    }
    if keyboard.just_pressed(KeyCode::Digit2) {
        settings.mode = TransformGizmoMode::Rotate;
    }
    if keyboard.just_pressed(KeyCode::Digit3) {
        settings.mode = TransformGizmoMode::Scale;
    }
    if keyboard.just_pressed(KeyCode::KeyX) {
        settings.space = match settings.space {
            TransformGizmoSpace::World => TransformGizmoSpace::Local,
            TransformGizmoSpace::Local => TransformGizmoSpace::World,
        };
    }
}
