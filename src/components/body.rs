use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Body {
    pub id: EntityID,
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub id: EntityID,
    pub body: EntityID,
    pub transform: Transform,
}

use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Site {
    pub id: EntityID,
    pub body: EntityID,
    pub offset: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Landmark {
    pub site: EntityID,
    pub name: String,
}
