//! 3D viewport: model selection via bevy_picking.
//!
//! Selection is driven by a `bevy_picking` `Pointer<Click>` observer. bevy_egui's
//! `picking` feature (capture_pointer_input) suppresses these events while the
//! pointer is over an egui window, so clicking/dragging a panel or slider never
//! selects or deselects a model entity.

use bevy::picking::events::{Click, Pointer};
use bevy::prelude::*;

use super::selection::Selection;

use crate::model::{Body, Frame, Joint, Muscle, Site};

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
