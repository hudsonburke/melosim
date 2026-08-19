//! Visualization resources and gizmo systems.

use bevy::prelude::*;

// ── Gizmo systems ────────────────────────────────────

/// Draw body coordinate axes (RGB) and small spheres at body origins, plus
/// small spheres at site positions — the canonical per-component viz so every
/// model renders consistently regardless of how it was authored.
pub fn draw_body_gizmos(
    mut gizmos: Gizmos,
    bodies: Query<(&Name, &GlobalTransform), With<crate::model::Body>>,
    sites: Query<(&Name, &GlobalTransform), With<crate::model::Site>>,
) {
    for (_name, gt) in &bodies {
        let pos = gt.translation();
        let rot = gt.rotation();

        gizmos.line(pos, pos + rot * Vec3::X * 0.02, Color::srgb(1.0, 0.0, 0.0));
        gizmos.line(pos, pos + rot * Vec3::Y * 0.02, Color::srgb(0.0, 1.0, 0.0));
        gizmos.line(pos, pos + rot * Vec3::Z * 0.02, Color::srgb(0.0, 0.0, 1.0));

        gizmos.sphere(
            Isometry3d::from_translation(pos),
            0.005,
            Color::srgb(1.0, 1.0, 0.0),
        );
    }

    for (_name, gt) in &sites {
        let pos = gt.translation();
        gizmos.sphere(
            Isometry3d::from_translation(pos),
            0.003,
            Color::srgb(0.0, 1.0, 1.0),
        );
    }
}

/// Draw each `Frame` as a small local RGB axis triplet — the frame's own
/// coordinate system — so attachment frames are visible and aimable.
pub fn draw_frames(
    mut gizmos: Gizmos,
    frames: Query<&GlobalTransform, With<crate::model::Frame>>,
) {
    const AXIS_LEN: f32 = 0.015;
    for gt in &frames {
        let pos = gt.translation();
        let rot = gt.rotation();
        gizmos.line(pos, pos + rot * Vec3::X * AXIS_LEN, Color::srgb(1.0, 0.0, 0.0));
        gizmos.line(pos, pos + rot * Vec3::Y * AXIS_LEN, Color::srgb(0.0, 1.0, 0.0));
        gizmos.line(pos, pos + rot * Vec3::Z * AXIS_LEN, Color::srgb(0.0, 0.0, 1.0));
    }
}

/// Draw muscle paths as line segments between path entities.
pub fn draw_muscle_paths(
    mut gizmos: Gizmos,
    muscles: Query<&crate::model::PathEntities>,
    transforms: Query<&GlobalTransform>,
) {
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

/// Draw joint axes as short lines showing rotation/translation directions.
pub fn draw_joint_axes(
    mut gizmos: Gizmos,
    axes: Query<(&crate::model::Twist, &ChildOf)>,
    transforms: Query<&GlobalTransform>,
) {

    for (twist, child_of) in &axes {
        let Ok(joint_gt) = transforms.get(child_of.0) else {
            continue;
        };
        let pos = joint_gt.translation();

        // Rotation axis (blue)
        if twist.angular.norm() > 1e-6 {
            let dir = twist.angular.normalize() * 0.05;
            gizmos.line(
                pos,
                pos + Vec3::new(dir.x as f32, dir.y as f32, dir.z as f32),
                Color::srgb(0.2, 0.4, 0.9),
            );
        }

        // Translation axis (green)
        if twist.linear.norm() > 1e-6 {
            let dir = twist.linear.normalize() * 0.05;
            gizmos.line(
                pos,
                pos + Vec3::new(dir.x as f32, dir.y as f32, dir.z as f32),
                Color::srgb(0.2, 0.9, 0.4),
            );
        }
    }
}
