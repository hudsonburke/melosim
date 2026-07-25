use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Material {
    pub id: EntityID,
    pub body: EntityID,
    pub density: f64,
    pub youngs_modulus: f64,
    pub poissons_ratio: f64,
}
