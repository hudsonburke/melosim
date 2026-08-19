//! Visualization resources and gizmo systems.

use bevy::prelude::*;

// ── Gizmo systems ────────────────────────────────────

/// Draw the "own-transform" model visuals in one pass: Body → RGB axes + origin
/// sphere, Frame → smaller RGB axes, Site → cyan sphere.
///
/// These three only need the entity's own `GlobalTransform`, so they fit one
/// `Option<>` query. (Muscle paths and joint axes stay separate because they
/// depend on *other* entities' transforms — path points / the parent frame.)
pub fn draw_model_gizmos(
    mut gizmos: Gizmos,
    model: Query<(
        &GlobalTransform,
        Option<&crate::model::Body>,
        Option<&crate::model::Frame>,
        Option<&crate::model::Site>,
    )>,
) {
    const AXES: [Vec3; 3] = [Vec3::X, Vec3::Y, Vec3::Z];
    const COLORS: [Color; 3] = [
        Color::srgb(1.0, 0.0, 0.0),
        Color::srgb(0.0, 1.0, 0.0),
        Color::srgb(0.0, 0.0, 1.0),
    ];

    for (gt, body, frame, site) in &model {
        let pos = gt.translation();
        let rot = gt.rotation();

        if body.is_some() {
            for (axis, color) in AXES.iter().zip(COLORS.iter()) {
                gizmos.line(pos, pos + rot * *axis * 0.02, *color);
            }
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                0.005,
                Color::srgb(1.0, 1.0, 0.0),
            );
        } else if frame.is_some() {
            for (axis, color) in AXES.iter().zip(COLORS.iter()) {
                gizmos.line(pos, pos + rot * *axis * 0.015, *color);
            }
        } else if site.is_some() {
            gizmos.sphere(
                Isometry3d::from_translation(pos),
                0.003,
                Color::srgb(0.0, 1.0, 1.0),
            );
        }
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
