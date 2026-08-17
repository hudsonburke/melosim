use bevy::prelude::*;

// ── Muscle/Cable path definition ──

/// Ordered list of path entities for a muscle or cable.
///
/// Each element is either a `Site` (attachment point) or a
/// `WrappingSurface` (geometry to wrap around). The evaluation system
/// computes the path through space:
///
/// - Site → Site: straight line
/// - Site → Wrap: approach vector to surface
/// - Wrap → Site: departure vector from surface
/// - Wrap → Wrap: geodesic arc between surfaces
///
/// First element = origin (force enters bone).
/// Last element = insertion (force exits bone).
#[derive(Component, Clone, Debug)]
#[relationship_target(relationship = PathElement)]
pub struct PathEntities(Vec<Entity>);

impl PathEntities {
    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }
}

/// Relationship: this entity is part of a muscle/cable path.
#[derive(Component, Clone, Debug, FromTemplate)]
#[relationship(relationship_target = PathEntities)]
pub struct PathElement(pub Entity);

// ── Wrapping surface marker ──

/// Marker for wrapping surface entities (spheres, cylinders, ellipsoids).
///
/// These are children of bodies (like `Site`), but the path evaluation
/// system treats them as surfaces to wrap around rather than points
/// to pass through.
#[derive(Component, Clone, Debug, Default)]
pub struct WrappingSurface;

/// Radius of a wrapping surface (for spheres and cylinders).
#[derive(Component, Clone, Debug)]
pub struct WrapRadius(pub f32);

// ── Path evaluation ──

/// A point along an evaluated path, with its world-space position.
#[derive(Clone, Debug)]
pub struct PathPoint {
    pub entity: Entity,
    pub position: Vec3,
}

/// Evaluate the full path: iterate PathEntities and compute world-space positions.
///
/// For sites, this is just their GlobalTransform translation.
/// For wrapping surfaces, this would compute approach/departure points
/// and geodesic arcs (future work).
pub fn evaluate_path(
    path_entities: &PathEntities,
    transforms: &Query<&GlobalTransform>,
    surfaces: Query<&WrappingSurface>,
) -> Option<Vec<PathPoint>> {
    let mut path = Vec::new();

    for e in path_entities.iter() {
        if surfaces.get(e).is_ok() {
            // Wrapping surface: for now, use center position.
            // Future: compute approach/departure/geodesic points.
            let pos = transforms.get(e).ok()?;
            path.push(PathPoint {
                entity: e,
                position: pos.translation(),
            });
        } else {
            // Site: straightforward position.
            let pos = transforms.get(e).ok()?;
            path.push(PathPoint {
                entity: e,
                position: pos.translation(),
            });
        }
    }

    Some(path)
}

/// Compute the total length of a path.
pub fn path_length(points: &[PathPoint]) -> f32 {
    points
        .windows(2)
        .map(|w| (w[1].position - w[0].position).length())
        .sum()
}
