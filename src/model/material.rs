use bevy::prelude::*;

#[derive(Component)]
pub struct MaterialPropertites {
    pub density: f64,
    pub youngs_modulus: f64,
    pub poissons_ratio: f64,
}
