use bevy::prelude::*;

// ── Muscle/Cable → Site references ──

/// The origin site of a path (muscle or cable).
///
/// Single entity — every path has exactly one origin. This is a plain
/// component, not a relationship target, because it's always one entity.
/// The corresponding `HasOrigin` on the site points back to the muscle/cable.
#[derive(Component, Clone, Debug, FromTemplate)]
pub struct OriginSite(pub Entity);

/// The insertion site of a path (muscle or cable).
///
/// Single entity — every path has exactly one insertion. This is a plain
/// component, not a relationship target, because it's always one entity.
/// The corresponding `HasInsertion` on the site points back to the muscle/cable.
#[derive(Component, Clone, Debug, FromTemplate)]
pub struct InsertionSite(pub Entity);

/// Ordered via points between origin and insertion.
///
/// Via points redirect the path around obstacles (e.g., wrapping surfaces).
/// Zero or more; the order defines the path geometry.
/// The full path is: OriginSite → ViaSites → InsertionSite.
#[derive(Component, Clone, Debug)]
#[relationship_target(relationship = ViaSite)]
pub struct ViaSites(Vec<Entity>);

impl ViaSites {
    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }

    pub fn len(&self) -> usize {
        self.0.len()
    }
}

// ── Site → Muscle/Cable back-references ──

/// This site is the origin of a muscle or cable.
///
/// A site can be the origin of multiple muscles/cables simultaneously.
#[derive(Component, Clone, Debug)]
pub struct HasOrigin(pub Entity);

/// This site is the insertion of a muscle or cable.
///
/// A site can be the insertion of multiple muscles/cables simultaneously.
#[derive(Component, Clone, Debug)]
pub struct HasInsertion(pub Entity);

/// Relationship: this site is a via point of a muscle or cable.
///
/// A site can be a via point for multiple muscles/cables simultaneously.
#[derive(Component, Clone, Debug, FromTemplate)]
#[relationship(relationship_target = ViaSites)]
pub struct ViaSite(pub Entity);

// ── Path evaluation ──

/// A point along an evaluated path, with its world-space position.
#[derive(Clone, Debug)]
pub struct PathPoint {
    pub entity: Entity,
    pub position: Vec3,
}

/// Evaluate the full path: origin → via points → insertion.
///
/// Returns an ordered list of world-space positions. The caller can use
/// these for wrapping, length computation, moment arm calculation, etc.
pub fn evaluate_path(
    origin_site: &OriginSite,
    via_sites: &ViaSites,
    insertion_site: &InsertionSite,
    transforms: &Query<&GlobalTransform>,
) -> Option<Vec<PathPoint>> {
    let mut path = Vec::new();

    let origin_pos = transforms.get(origin_site.0).ok()?;
    path.push(PathPoint {
        entity: origin_site.0,
        position: origin_pos.translation(),
    });

    for e in via_sites.iter() {
        let pos = transforms.get(e).ok()?;
        path.push(PathPoint {
            entity: e,
            position: pos.translation(),
        });
    }

    let insertion_pos = transforms.get(insertion_site.0).ok()?;
    path.push(PathPoint {
        entity: insertion_site.0,
        position: insertion_pos.translation(),
    });

    Some(path)
}

/// Compute the total length of a path.
pub fn path_length(points: &[PathPoint]) -> f32 {
    points
        .windows(2)
        .map(|w| (w[1].position - w[0].position).length())
        .sum()
}
