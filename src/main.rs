use serde::{Deserialize, Serialize};

#[derive(Serialize, Deserialize)]
pub struct EntityID(u32);

impl EntityID {
    pub fn next(&self) -> Self {
        EntityID(self.0 + 1)
    }
}
#[derive(Serialize, Deserialize)]
pub struct World {
    next_id: EntityID,
    bodies: Vec<Body>,
    joints: Vec<Joint>,
    sites: Vec<Site>,
    materials: Vec<Material>,
    geometries: Vec<Geometry>,
}

impl World {
    pub fn spawn(&mut self) -> EntityID {
        let id = self.next_id;
        self.next_id.next();
        id
    }

    pub fn add_body(&mut self, mass: f64, com: [f64; 3], inertia: [f64; 6]) -> EntityID {
        let id = self.spawn();
        self.bodies.push(Body {
            id,
            mass,
            com,
            inertia,
        });
        id
    }

    pub fn get_body(&self, id: u32) -> Option<&Body> {
        self.bodies.iter().find(|b| b.id == id)
    }
}

#[derive(Serialize, Deserialize)]
pub struct Joint {}

#[derive(Serialize, Deserialize)]
pub struct Geometry {
    body: EntityID,
    mesh: String,
    role: GeometryRole,
}

#[derive(Serialize, Deserialize)]
pub enum GeometryRole {
    Collision,
    Visualization,
    Simulation,
}

#[derive(Serialize, Deserialize)]
pub struct Body {
    id: EntityID,
    mass: f64,
    com: [f64; 3],
    inertia: [f64; 6],
}

#[derive(Serialize, Deserialize)]
pub struct Material {
    body: EntityID,
    density: f64,
    youngs_modulus: f64,
    poissons_ration: f64,
}

// TODO: Use quaternion crate

#[derive(Serialize, Deserialize)]
pub struct Quaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

#[derive(Serialize, Deserialize)]
pub struct Translation {
    x: f64,
    y: f64,
    z: f64,
}

#[derive(Serialize, Deserialize)]
pub struct Transform {
    position: Translation,
    rotation: Quaternion,
}

#[derive(Serialize, Deserialize)]
pub struct Frame {
    body: EntityID,
    transform: Transform,
}

#[derive(Serialize, Deserialize)]
pub struct Site {
    body: EntityID,
    offset: Translation,
}

#[derive(Serialize, Deserialize)]
pub enum SiteRole {
    Landmark,
    Attachment,
    WrapPoint,
}

#[derive(Serialize, Deserialize)]
pub enum JointType {
    Hinge { axis: [f64; 3] },
    Slide { axis: [f64; 3] },
    Ball,
    Free,
    Fixed,
    Custom,
}

#[derive(Serialize, Deserialize)]
pub enum ActuatorType {
    Motor,
    Muscle,
    Coordinate,
}

fn main() {
    println!("Hello, world!");
}
