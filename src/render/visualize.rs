//! Visualization resources and gizmo systems.

use bevy::prelude::*;

// ── Toggle resource ──────────────────────────────────

/// Controls which visualization layers are active.
#[derive(Resource, Clone, Debug)]
pub struct VisualizationSettings {
    pub muscle_paths: bool,
    pub joint_axes: bool,
}

impl Default for VisualizationSettings {
    fn default() -> Self {
        Self {
            muscle_paths: true,
            joint_axes: false,
        }
    }
}

// ── Gizmo systems ────────────────────────────────────

/// Draw muscle paths as line segments between path entities.
pub fn draw_muscle_paths(
    mut gizmos: Gizmos,
    settings: Res<VisualizationSettings>,
    muscles: Query<&crate::model::PathEntities>,
    transforms: Query<&GlobalTransform>,
) {
    if !settings.muscle_paths {
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

/// Draw joint axes as short lines showing rotation/translation directions.
pub fn draw_joint_axes(
    mut gizmos: Gizmos,
    settings: Res<VisualizationSettings>,
    axes: Query<(&crate::model::Twist, &ChildOf)>,
    transforms: Query<&GlobalTransform>,
) {
    if !settings.joint_axes {
        return;
    }

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
