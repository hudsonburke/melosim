//! Minimal 3D transform gizmo (v1: translate via dragging on a camera-parallel
//! plane). Draws RGB axis handles at the selected spatial entity and lets you
//! drag it, mapping the world-space drag delta back into the entity's
//! parent-local frame. Rotate/scale modes can be added later.

use bevy::{
    picking::{hover::HoverMap, pointer::PointerId},
    prelude::*,
};
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::selection::Selection;
use crate::model::{Body, Frame, Site};

const AXIS_LEN: f32 = 0.12;

/// Transform gizmo state.
#[derive(Resource, Default)]
pub struct TransformGizmo {
    /// Whether translate-dragging is enabled.
    pub translate: bool,
    dragging: bool,
    last_hit: Option<Vec3>,
}

/// Draw translate axis handles (RGB = XYZ) at the selected spatial entity.
pub fn draw_transform_handles(
    mut gizmos: Gizmos,
    selection: Res<Selection>,
    global: Query<&GlobalTransform>,
    spatial: Query<(), Or<(With<Body>, With<Frame>, With<Site>)>>,
) {
    let Some(entity) = selection.primary() else { return };
    if spatial.get(entity).is_err() {
        return;
    }
    let Ok(gt) = global.get(entity) else { return };

    let origin = gt.translation();
    let axes = [
        (Vec3::X, Color::srgb(1.0, 0.0, 0.0)),
        (Vec3::Y, Color::srgb(0.0, 1.0, 0.0)),
        (Vec3::Z, Color::srgb(0.0, 0.0, 1.0)),
    ];
    for (axis, color) in axes {
        gizmos.line(origin, origin + axis * AXIS_LEN, color);
    }
}

/// Drag the selected spatial entity across a camera-parallel plane.
///
/// A drag starts only when you press on the already-selected entity itself
/// (via bevy_picking's hover map), so a normal click still selects a new entity
/// through `select_on_click`. Runs in `EguiPrimaryContextPass` after the editor
/// panels, and bails if the pointer is over an egui window.
pub fn drag_transform(
    mouse: Res<ButtonInput<MouseButton>>,
    mut gizmo: ResMut<TransformGizmo>,
    mut contexts: EguiContexts,
    hover: Res<HoverMap>,
    selection: Res<Selection>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    child_of: Query<&ChildOf>,
    global: Query<&GlobalTransform>,
    mut local: Query<&mut Transform>,
    spatial: Query<(), Or<(With<Body>, With<Frame>, With<Site>)>>,
) {
    if !gizmo.translate {
        return;
    }
    let Some(entity) = selection.primary() else { return };
    if spatial.get(entity).is_err() {
        return;
    }
    if contexts.ctx_mut().expect("one primary egui context").is_pointer_over_egui() {
        return;
    }

    let Ok((camera, camera_gt)) = cameras.single() else { return };
    let Ok(window) = windows.single() else { return };
    let Some(cursor) = window.cursor_position() else { return };
    let Ok(entity_gt) = global.get(entity) else { return };

    // Drag plane: through the entity's world position, facing the camera.
    let plane_normal = camera_gt.rotation() * Vec3::NEG_Z;
    let plane_origin = entity_gt.translation();

    let intersect = |cursor: Vec2| -> Option<Vec3> {
        let ray = camera.viewport_to_world(camera_gt, cursor).ok()?;
        let denom = ray.direction.dot(plane_normal);
        if denom.abs() < 1e-6 {
            return None;
        }
        let t = (plane_origin - ray.origin).dot(plane_normal) / denom;
        Some(ray.origin + ray.direction * t)
    };

    if mouse.just_pressed(MouseButton::Left) {
        let hovering_self = hover
            .0
            .get(&PointerId::Mouse)
            .is_some_and(|m| m.contains_key(&entity));
        if hovering_self {
            gizmo.dragging = true;
            gizmo.last_hit = intersect(cursor);
        }
        return;
    }

    if mouse.just_released(MouseButton::Left) {
        gizmo.dragging = false;
        gizmo.last_hit = None;
        return;
    }

    if !gizmo.dragging {
        return;
    }

    let Some(hit) = intersect(cursor) else { return };
    let Some(last) = gizmo.last_hit else { return };
    let delta_world = hit - last;
    gizmo.last_hit = Some(hit);
    if delta_world.length_squared() < 1e-12 {
        return;
    }

    // Map the world-space delta into the parent-local frame.
    let parent_rot = match child_of.get(entity) {
        Ok(c) => global.get(c.parent()).map(|g| g.rotation()).unwrap_or(Quat::IDENTITY),
        Err(_) => Quat::IDENTITY,
    };
    let delta_local = parent_rot.inverse() * delta_world;

    if let Ok(mut t) = local.get_mut(entity) {
        t.translation += delta_local;
    }
}
