use bevy::prelude::*;

// ── Muscle/Cable path definition ──

/// Ordered list of path entities for a muscle or cable.
#[derive(Component, Clone, Debug, Reflect)]
#[relationship_target(relationship = PathElement)]
pub struct PathEntities(Vec<Entity>);

impl PathEntities {
    pub fn new(entities: Vec<Entity>) -> Self {
        Self(entities)
    }

    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Replace the whole path (used by the editor to author a new path).
    pub fn set(&mut self, entities: Vec<Entity>) {
        self.0 = entities;
    }
}

/// Relationship: this entity is part of a muscle/cable path.
///
/// Injected by the relationship machinery from the owner's `PathEntities` (like
/// Bevy's own `ChildOf`), not authored in scene notation.
#[derive(Component, Clone, Debug, FromTemplate, Reflect)]
#[relationship(relationship_target = PathEntities)]
pub struct PathElement(pub Entity);

// ── Wrapping surface marker ──

/// Marker for wrapping surface entities.
#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct WrappingSurface;

/// Radius of a wrapping surface.
#[derive(Component, Clone, Debug, Reflect)]
pub struct WrapRadius(pub f32);

// ── Path evaluation ──

/// A point along an evaluated path.
#[derive(Clone, Debug)]
pub struct PathPoint {
    pub entity: Entity,
    pub position: Vec3,
}

/// Evaluate the full path: compute world-space positions.
pub fn evaluate_path(
    path_entities: &PathEntities,
    transforms: &Query<&GlobalTransform>,
    surfaces: Query<&WrappingSurface>,
) -> Option<Vec<PathPoint>> {
    let mut path = Vec::new();

    for e in path_entities.iter() {
        if surfaces.get(e).is_ok() {
            let pos = transforms.get(e).ok()?;
            path.push(PathPoint {
                entity: e,
                position: pos.translation(),
            });
        } else {
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
