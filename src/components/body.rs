use bevy_ecs::prelude::*;

#[derive(Component)]
pub struct InertialProperties {
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

// TODO: Could maybe a function or a macro
// I believe I pulled this idea from opensimcreator
/// A frame computed from anatomical landmarks (stations).
/// Used for attaching exoskeleton parts — the user places 3-4
/// stations, the system computes the frame, and the attachment
/// follows automatically when the body scales.
#[derive(Component)]
pub struct StationDefinedFrame {
    /// Station providing the frame origin.
    pub origin: Entity,
    /// Station defining the X axis direction (from origin).
    pub axis_x: Entity,
    /// Station defining the Y axis direction (from origin).
    pub axis_y: Entity,
}
