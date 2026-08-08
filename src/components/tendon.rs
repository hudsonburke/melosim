use bevy_ecs::prelude::*;

#[derive(Component, Clone, Debug)]
pub struct TendonParams {
    pub spring_length: f64,
    pub width: f64,
}
