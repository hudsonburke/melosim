use serde::{Deserialize, Serialize};

pub struct EntityID(u32);

#[derive(Serialize, Deserialize)]
pub struct World {
    next_id: u32,
    bodies: Vec<Body>,
    joints: Vec<Joint>,
    sites: Vec<Site>,
    materials: Vec<Material>,
    geometries: Vec<Geometry>,
}

impl World {
    pub fn spawn(&mut self) -> u32 {
        let id = self.next_id;
        self.next_id += 1;
        id
    }

    pub fn add_body(&mut self, mass: f64, com: [f64; 3], inertia: [f64; 6]) -> u32 {
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

pub struct Joint {}
pub struct Geometry {
    body: EntityID,
    mesh: String,
    role: GeometryRole,
}

pub enum GeometryRole {
    Collision,
    Visualization,
    Simulation,
}

pub struct Body {
    id: EntityID,
    mass: f64,
    com: [f64; 3],
    inertia: [f64; 6],
}

pub struct Material {
    body: EntityID,
    density: f64,
    youngs_modulus: f64,
    poissons_ration: f64,
}

// TODO: Use quaternion crate
pub struct Quaternion {
    x: f64,
    y: f64,
    z: f64,
    w: f64,
}

pub struct Translation {
    x: f64,
    y: f64,
    z: f64,
}

pub struct Transform {
    position: Translation,
    rotation: Quaternion,
}

pub struct Frame {
    body: EntityID,
    transform: Transform,
}

pub struct Site {
    body: EntityID,
    offset: Translation,
}

pub enum SiteRole {
    Landmark,
    Attachment,
    WrapPoint,
}

pub enum JointType {
    Hinge { axis: [f64; 3] },
    Slide { axis: [f64; 3] },
    Ball,
    Free,
    Fixed,
    Custom,
}

pub enum ActuatorType {
    Motor,
    Muscle,
    Coordinate,
}

fn main() {
    println!("Hello, world!");
}
