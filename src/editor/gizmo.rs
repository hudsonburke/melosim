//! Transform gizmo integration using Bevy's built-in
//! `bevy_gizmos::transform_gizmo` (translate / rotate / scale, world or local).
//!
//! The editor mirrors its `Selection` onto the `TransformGizmoFocus` component,
//! marks the editor camera with `TransformGizmoCamera`, and exposes mode keys
//! (1=Translate, 2=Rotate, 3=Scale, X=World/Local). The plugin handles all the
//! gizmo dragging itself.

use bevy::{
    camera_controller::free_camera::FreeCameraState,
    gizmos::transform_gizmo::{
        TransformGizmoFocus, TransformGizmoMode, TransformGizmoSettings, TransformGizmoSpace,
    },
    prelude::*,
};
use bevy_inspector_egui::bevy_egui::EguiContexts;

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
/// Gated so keys meant for egui (e.g. typing a name) don't switch gizmo mode.
pub fn gizmo_mode_keys(
    mut contexts: EguiContexts,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut settings: ResMut<TransformGizmoSettings>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    if ctx.is_pointer_over_egui() || ctx.egui_wants_keyboard_input() {
        return;
    }
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

/// Disable `FreeCamera` while the pointer is over egui (or egui wants keyboard,
/// e.g. typing in a text field) so the 3D view doesn't react to clicks / scroll /
/// keys meant for the panels.
pub fn sync_freecam_to_egui(
    mut contexts: EguiContexts,
    mut cameras: Query<&mut FreeCameraState, With<Camera>>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    let egui_active = ctx.is_pointer_over_egui() || ctx.egui_wants_keyboard_input();
    for mut state in &mut cameras {
        state.enabled = !egui_active;
    }
}
