use bevy::prelude::*;

use crate::model::{Body, PathEntities, Site, WrappingSurface};

/// Draws bodies as spheres, sites as small spheres, and muscle paths as lines.
/// Uses Bevy's built-in gizmos for immediate-mode visualization.
pub fn visualize_model(
    mut gizmos: Gizmos,
    bodies: Query<&GlobalTransform, With<Body>>,
    sites: Query<&GlobalTransform, With<Site>>,
    wrapping: Query<&GlobalTransform, With<WrappingSurface>>,
    muscles: Query<&PathEntities>,
    transforms: Query<&GlobalTransform>,
) {
    // ── Bodies (blue spheres) ──
    for gt in &bodies {
        let pos = gt.translation();
        gizmos.sphere(Isometry3d::from_translation(pos), 0.03, Color::srgb(0.3, 0.5, 0.9));
    }

    // ── Sites (green dots) ──
    for gt in &sites {
        let pos = gt.translation();
        gizmos.sphere(Isometry3d::from_translation(pos), 0.008, Color::srgb(0.2, 0.9, 0.3));
    }

    // ── Wrapping surfaces (orange outlines) ──
    for gt in &wrapping {
        let pos = gt.translation();
        gizmos.sphere(Isometry3d::from_translation(pos), 0.04, Color::srgba(1.0, 0.6, 0.2, 0.4));
    }

    // ── Muscle paths (red lines) ──
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
