use bevy::picking::mesh_picking::ray_cast::MeshRayCast;
use bevy::prelude::*;

use super::selection::Selection;

use crate::model::{Body, Frame, Joint, Muscle, Site};

/// Click-to-select system using MeshRayCast.
pub fn click_to_select(
    mouse: Res<ButtonInput<MouseButton>>,
    cameras: Query<(&Camera, &GlobalTransform)>,
    windows: Query<&Window>,
    mut ray_cast: MeshRayCast,
    mut selection: ResMut<Selection>,
    child_of_query: Query<&ChildOf>,
    bodies: Query<Entity, With<Body>>,
    joints: Query<Entity, With<Joint>>,
    sites: Query<Entity, With<Site>>,
    frames: Query<Entity, With<Frame>>,
    muscles: Query<Entity, With<Muscle>>,
) {
    if !mouse.just_pressed(MouseButton::Left) {
        return;
    }

    let Ok(window) = windows.single() else {
        return;
    };
    let Some(cursor_pos) = window.cursor_position() else {
        return;
    };
    let Ok((camera, camera_transform)) = cameras.single() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_pos) else {
        return;
    };

    let settings = bevy::picking::mesh_picking::ray_cast::MeshRayCastSettings::default()
        .with_visibility(bevy::picking::mesh_picking::ray_cast::RayCastVisibility::VisibleInView);

    let hits = ray_cast.cast_ray(ray, &settings);

    if let Some(&(hit_entity, ref _hit_data)) = hits.first() {
        // Walk up hierarchy to find the model entity
        let mut current = hit_entity;
        loop {
            // Check if current entity is a model entity
            if bodies.get(current).is_ok()
                || joints.get(current).is_ok()
                || sites.get(current).is_ok()
                || frames.get(current).is_ok()
                || muscles.get(current).is_ok()
            {
                selection.select_single(current);
                return;
            }
            if let Ok(parent) = child_of_query.get(current) {
                current = parent.parent();
            } else {
                // No model entity found; select the hit entity itself
                selection.select_single(hit_entity);
                return;
            }
        }
    } else {
        // Clicked on empty space -- clear selection
        selection.clear();
    }
}

/// Draw a visual highlight around the selected entity.
pub fn draw_selection_highlight(
    mut gizmos: Gizmos,
    selection: Res<Selection>,
    transforms: Query<&GlobalTransform>,
) {
    for &entity in &selection.entities {
        if let Ok(gt) = transforms.get(entity) {
            let pos = gt.translation();
            let scale = gt.compute_transform().scale;
            let radius = scale.x.max(scale.y).max(scale.z) * 0.1;

            // Yellow wireframe sphere for selection
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                radius.max(0.01),
                Color::srgb(1.0, 1.0, 0.0),
            );
        }
    }
}
