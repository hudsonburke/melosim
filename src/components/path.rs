use bevy_ecs::prelude::*;

/// A muscle's geometric path through the body.
///
/// Each muscle entity can have a `MusclePath` component that defines the
/// routing of the muscle-tendon unit through path points on bodies.
/// Path points can be fixed on a body or move dynamically with a coordinate
/// (e.g., a point that shifts with knee flexion to model wrapping).
#[derive(Component, Clone, Debug)]
pub struct MusclePath {
    pub muscle: Entity,
    pub points: Vec<PathPoint>,
}

/// A single point along a muscle-tendon path.
///
/// - `BodyFixed`: fixed location on a body (most common)
/// - `Moving`: location changes with a coordinate (used for wrapping
///    around joints like the knee)
#[derive(Clone, Debug)]
pub enum PathPoint {
    /// A point fixed on a body, specified in the body's local frame.
    BodyFixed {
        body: Entity,
        location: [f64; 3],
    },
    /// A point whose location depends on a coordinate value.
    /// The function maps coordinate value → location offset in body frame.
    Moving {
        body: Entity,
        coordinate: Entity,
        /// Map from coordinate value to the 3D location offset.
        /// Stored as 3 Polynomial functions, one per axis (X, Y, Z).
        location_functions: [Vec<f64>; 3],
    },
}
