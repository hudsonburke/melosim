//! 3D viewport: model selection via bevy_picking + auto-framed editor camera.
//!
//! Selection is driven by a `bevy_picking` `Pointer<Click>` observer. bevy_egui's
//! `picking` feature (capture_pointer_input) suppresses these events while the
//! pointer is over an egui window, so clicking/dragging a panel or slider never
//! selects or deselects a model entity.

use bevy::picking::events::{Click, Pointer};
use bevy::prelude::*;

use super::selection::Selection;

use crate::model::{Body, Frame, Joint, Muscle, Site};

/// Whether the editor camera has been auto-framed to the model yet.
#[derive(Resource, Default)]
pub struct CameraFramed(pub bool);

/// Direction the camera sits at, relative to the model center (before scaling).
const FRAME_DIR: Vec3 = Vec3::new(-1.0, 0.6, 1.2);

/// Auto-position the editor camera to fit the whole model (first frame + on `F`).
///
/// Bounding sphere is computed from the spatial entities' (Body/Frame/Site)
/// world positions; distance is chosen so the sphere fits the camera's vertical
/// FOV. Runs in `PostUpdate` (after transform propagation) so world positions
/// are settled. If there are no spatial entities yet, it frames the origin so
/// the camera is never left at a degenerate position.
pub fn frame_camera_to_model(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut framed: ResMut<CameraFramed>,
    mut cameras: Query<&mut Transform, With<Camera3d>>,
    spatial: Query<&GlobalTransform, Or<(With<Body>, With<Frame>, With<Site>)>>,
) {
    if framed.0 && !keyboard.just_pressed(KeyCode::KeyF) {
        return;
    }

    let Ok(mut cam) = cameras.single_mut() else {
        return;
    };

    let mut min = Vec3::splat(f32::INFINITY);
    let mut max = Vec3::splat(f32::NEG_INFINITY);
    let mut any = false;
    for gt in &spatial {
        let p = gt.translation();
        min = min.min(p);
        max = max.max(p);
        any = true;
    }

    // Frame the model if present, otherwise a sensible default around the origin.
    let (center, radius) = if any {
        ((min + max) * 0.5, (max - min).length() * 0.5 + 0.5)
    } else {
        (Vec3::ZERO, 2.5)
    };

    let fov_half = 45f32.to_radians() * 0.5;
    let distance = (radius / fov_half.sin()).max(2.0);
    let eye = center + FRAME_DIR.normalize() * distance;

    *cam = Transform::from_translation(eye).looking_at(center, Vec3::Y);
    framed.0 = true;
}

/// Select the model entity owning the clicked mesh. Walks up `ChildOf` to the
/// nearest Body/Joint/Site/Frame/Muscle. Nothing under a model entity changes
/// the selection, so clicking egui or non-model meshes is safe.
pub fn select_on_click(
    trigger: On<Pointer<Click>>,
    mut selection: ResMut<Selection>,
    child_of_query: Query<&ChildOf>,
    bodies: Query<Entity, With<Body>>,
    joints: Query<Entity, With<Joint>>,
    sites: Query<Entity, With<Site>>,
    frames: Query<Entity, With<Frame>>,
    muscles: Query<Entity, With<Muscle>>,
) {
    let mut current = trigger.original_event_target();
    loop {
        if bodies.get(current).is_ok()
            || joints.get(current).is_ok()
            || sites.get(current).is_ok()
            || frames.get(current).is_ok()
            || muscles.get(current).is_ok()
        {
            selection.select_single(current);
            return;
        }
        match child_of_query.get(current) {
            Ok(parent) => current = parent.parent(),
            Err(_) => return, // not under a model entity; leave selection unchanged
        }
    }
}
