
use bevy::prelude::*;
use super::InertialProperties;

/// Marker for a rigid body entity.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform, InertialProperties)]
pub struct Body;

/// Marker for a frame.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform)]
pub struct Frame;

/// Marker for a site — a point on a frame (ignores rotation).
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform)]
pub struct Site;

