//! Visualization resources and gizmo systems.

use bevy::prelude::*;

use crate::render::RenderSettings;

/// Standard RGB = XYZ axis visualization colors/lengths shared by Body & Frame.
const AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];
const COLORS: [Color; 3] = [
    Color::srgb(1.0, 0.0, 0.0),
    Color::srgb(0.0, 1.0, 0.0),
    Color::srgb(0.0, 0.0, 1.0),
];

// ── Gizmo systems ────────────────────────────────────

/// Draw each `Frame` as a small local RGB axis triplet (its own coordinate
/// system), so attachment frames are visible and aimable.
pub fn draw_frames(
    mut gizmos: Gizmos,
    settings: Res<RenderSettings>,
    frames: Query<&GlobalTransform, With<crate::model::Frame>>,
) {
    if !settings.frames {
        return;
    }
    for gt in &frames {
        let pos = gt.translation();
        let rot = gt.rotation();
        for (axis, color) in AXES.iter().zip(COLORS.iter()) {
            gizmos.line(pos, pos + rot * *axis * 0.015, *color);
        }
    }
}

/// Draw each `Site` as a small cyan sphere (cable-routing / landmark points).
pub fn draw_sites(
    mut gizmos: Gizmos,
    settings: Res<RenderSettings>,
    sites: Query<&GlobalTransform, With<crate::model::Site>>,
) {
    if !settings.sites {
        return;
    }
    for gt in &sites {
        gizmos.sphere(
            Isometry3d::from_translation(gt.translation()),
            0.003,
            Color::srgb(0.0, 1.0, 1.0),
        );
    }
}

/// Draw muscle paths as line segments between path entities.
pub fn draw_muscle_paths(
    mut gizmos: Gizmos,
    settings: Res<RenderSettings>,
    muscles: Query<&crate::model::PathEntities>,
    transforms: Query<&GlobalTransform>,
) {
    if !settings.muscles {
        return;
    }
    for path_entities in &muscles {
        let points: Vec<Vec3> = path_entities
            .iter()
            .filter_map(|e| transforms.get(e).ok().map(|gt| gt.translation()))
            .collect();

        if points.len() >= 2 {
            for pair in points.windows(2) {
                gizmos.line(pair[0], pair[1], Color::srgb(0.9, 0.2, 0.2));
            }
        }
    }
}
