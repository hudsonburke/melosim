use bevy_ecs::prelude::*;

#[derive(Component, Clone, Debug)]
pub struct Material {
    pub density: f64,
    pub youngs_modulus: f64,
    pub poissons_ratio: f64,
}
