use serde::{Deserialize, Serialize};

// ── Entity ID ────────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EntityID(u32);

impl EntityID {
    pub fn new(id: u32) -> Self {
        Self(id)
    }

    pub fn inner(&self) -> u32 {
        self.0
    }

    pub fn next(&self) -> Self {
        Self(self.0 + 1)
    }
}

// ── Math types ───────────────────────────────────────────────────────

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Vec3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn as_array(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Quaternion {
    pub w: f64,
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Default for Quaternion {
    fn default() -> Self {
        Self { w: 1.0, x: 0.0, y: 0.0, z: 0.0 }
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Transform {
    pub translation: Vec3,
    pub rotation: Quaternion,
}

// ── World ────────────────────────────────────────────────────────────

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct World {
    next_id: u32,
    pub bodies: Vec<Body>,
    pub joints: Vec<Joint>,
    pub muscles: Vec<Muscle>,
    pub tendons: Vec<Tendon>,
    pub sites: Vec<Site>,
    pub materials: Vec<Material>,
    pub geometries: Vec<Geometry>,
    pub cable_guides: Vec<CableGuide>,
    pub exo_parts: Vec<ExoPart>,
    pub cables: Vec<Cable>,
    pub wrap_geoms: Vec<WrapGeom>,
    pub transforms: Vec<Frame>,
}

impl World {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn spawn(&mut self) -> EntityID {
        let id = EntityID::new(self.next_id);
        self.next_id += 1;
        id
    }

    // ── Body ─────────────────────────────────────────────────────

    pub fn add_body(&mut self, mass: f64, com: [f64; 3], inertia: [f64; 6]) -> EntityID {
        let id = self.spawn();
        self.bodies.push(Body { id, mass, com, inertia });
        id
    }

    pub fn get_body(&self, id: EntityID) -> Option<&Body> {
        self.bodies.iter().find(|b| b.id == id)
    }

    pub fn get_body_mut(&mut self, id: EntityID) -> Option<&mut Body> {
        self.bodies.iter_mut().find(|b| b.id == id)
    }

    // ── Joint ────────────────────────────────────────────────────

    pub fn add_joint(
        &mut self,
        body_a: EntityID,
        body_b: EntityID,
        joint_type: JointType,
        limits: Option<JointLimits>,
    ) -> EntityID {
        let id = self.spawn();
        self.joints.push(Joint {
            id,
            body_a,
            body_b,
            joint_type,
            limits,
        });
        id
    }

    pub fn get_joint(&self, id: EntityID) -> Option<&Joint> {
        self.joints.iter().find(|j| j.id == id)
    }

    // ── Muscle ───────────────────────────────────────────────────

    pub fn add_muscle(
        &mut self,
        name: String,
        path: Vec<MusclePoint>,
        max_force: f64,
        optimal_fiber_length: f64,
        tendon_slack_length: f64,
        pcsa: f64,
        pennation_angle: f64,
    ) -> EntityID {
        let id = self.spawn();
        self.muscles.push(Muscle {
            id,
            name,
            path,
            max_force,
            optimal_fiber_length,
            tendon_slack_length,
            pcsa,
            pennation_angle,
        });
        id
    }

    pub fn get_muscle(&self, id: EntityID) -> Option<&Muscle> {
        self.muscles.iter().find(|m| m.id == id)
    }

    // ── Site ─────────────────────────────────────────────────────

    pub fn add_site(&mut self, body: EntityID, offset: Vec3) -> EntityID {
        let id = self.spawn();
        self.sites.push(Site { id, body, offset });
        id
    }

    pub fn get_site(&self, id: EntityID) -> Option<&Site> {
        self.sites.iter().find(|s| s.id == id)
    }

    // ── Material ─────────────────────────────────────────────────

    pub fn add_material(
        &mut self,
        body: EntityID,
        density: f64,
        youngs_modulus: f64,
        poissons_ratio: f64,
    ) -> EntityID {
        let id = self.spawn();
        self.materials.push(Material {
            id,
            body,
            density,
            youngs_modulus,
            poissons_ratio,
        });
        id
    }

    pub fn get_material(&self, id: EntityID) -> Option<&Material> {
        self.materials.iter().find(|m| m.id == id)
    }

    // ── Geometry ─────────────────────────────────────────────────

    pub fn add_geometry(
        &mut self,
        body: EntityID,
        mesh: String,
        role: GeometryRole,
    ) -> EntityID {
        let id = self.spawn();
        self.geometries.push(Geometry {
            id,
            body,
            mesh,
            role,
        });
        id
    }

    // ── Transform ────────────────────────────────────────────────

    pub fn add_transform(&mut self, body: EntityID, transform: Transform) -> EntityID {
        let id = self.spawn();
        self.transforms.push(Frame {
            id,
            body,
            transform,
        });
        id
    }

    pub fn get_transform(&self, body: EntityID) -> Option<&Transform> {
        self.transforms.iter().find(|f| f.body == body).map(|f| &f.transform)
    }

    // ── Muscle ───────────────────────────────────────────────────

    pub fn add_muscle2(
        &mut self,
        name: String,
        path: Vec<MusclePoint>,
        max_force: f64,
        optimal_fiber_length: f64,
        tendon_slack_length: f64,
        pcsa: f64,
        pennation_angle: f64,
    ) -> EntityID {
        let id = self.spawn();
        self.muscles.push(Muscle {
            id,
            name,
            path,
            max_force,
            optimal_fiber_length,
            tendon_slack_length,
            pcsa,
            pennation_angle,
        });
        id
    }

    // ── Tendon ───────────────────────────────────────────────────

    pub fn add_tendon(
        &mut self,
        name: String,
        spring_length: f64,
        width: f64,
        via_points: Vec<EntityID>,
    ) -> EntityID {
        let id = self.spawn();
        self.tendons.push(Tendon {
            id,
            name,
            spring_length,
            width,
            via_points,
        });
        id
    }

    // ── CableGuide ───────────────────────────────────────────────

    pub fn add_cable_guide(
        &mut self,
        site: EntityID,
        diameter: f64,
    ) -> EntityID {
        let id = self.spawn();
        self.cable_guides.push(CableGuide { id, site, diameter });
        id
    }

    // ── ExoPart ──────────────────────────────────────────────────

    pub fn add_exo_part(
        &mut self,
        name: String,
        part_type: ExoPartType,
        body: EntityID,
        offset: Vec3,
        ports: Vec<CablePort>,
    ) -> EntityID {
        let id = self.spawn();
        self.exo_parts.push(ExoPart {
            id,
            name,
            part_type,
            body,
            offset,
            ports,
        });
        id
    }

    // ── Cable ────────────────────────────────────────────────────

    pub fn add_cable(
        &mut self,
        name: String,
        path: Vec<CableSegment>,
        tendon: Option<EntityID>,
    ) -> EntityID {
        let id = self.spawn();
        self.cables.push(Cable { id, name, path, tendon });
        id
    }

    // ── WrapGeom ─────────────────────────────────────────────────

    pub fn add_wrap_geom(
        &mut self,
        body: EntityID,
        geom_type: WrapGeomType,
    ) -> EntityID {
        let id = self.spawn();
        self.wrap_geoms.push(WrapGeom { id, body, geom_type });
        id
    }

    // ── Validation ───────────────────────────────────────────────

    pub fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();

        for joint in &self.joints {
            if self.get_body(joint.body_a).is_none() {
                errors.push(format!(
                    "Joint {:?} references missing body_a {:?}",
                    joint.id.inner(), joint.body_a.inner()
                ));
            }
            if self.get_body(joint.body_b).is_none() {
                errors.push(format!(
                    "Joint {:?} references missing body_b {:?}",
                    joint.id.inner(), joint.body_b.inner()
                ));
            }
        }

        for muscle in &self.muscles {
            for point in &muscle.path {
                if self.get_body(point.body).is_none() {
                    errors.push(format!(
                        "Muscle '{}' path references missing body {:?}",
                        muscle.name, point.body.inner()
                    ));
                }
            }
        }

        for material in &self.materials {
            if self.get_body(material.body).is_none() {
                errors.push(format!(
                    "Material {:?} references missing body {:?}",
                    material.id.inner(), material.body.inner()
                ));
            }
        }

        errors
    }
}

// ── Body ─────────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Body {
    pub id: EntityID,
    pub mass: f64,
    pub com: [f64; 3],
    pub inertia: [f64; 6],
}

// ── Joint ────────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint {
    pub id: EntityID,
    pub body_a: EntityID,
    pub body_b: EntityID,
    pub joint_type: JointType,
    pub limits: Option<JointLimits>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum JointType {
    Hinge { axis: [f64; 3] },
    Slide { axis: [f64; 3] },
    Ball,
    Free,
    Fixed,
}

// ── Muscle ───────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Muscle {
    pub id: EntityID,
    pub name: String,
    pub path: Vec<MusclePoint>,
    pub max_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub pcsa: f64,
    pub pennation_angle: f64,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MusclePoint {
    pub body: EntityID,
    pub offset: Vec3,
}

// ── Tendon ───────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Tendon {
    pub id: EntityID,
    pub name: String,
    pub spring_length: f64,
    pub width: f64,
    pub via_points: Vec<EntityID>,
}

// ── Site ─────────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Site {
    pub id: EntityID,
    pub body: EntityID,
    pub offset: Vec3,
}

// ── Material ─────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Material {
    pub id: EntityID,
    pub body: EntityID,
    pub density: f64,
    pub youngs_modulus: f64,
    pub poissons_ratio: f64,
}

// ── Geometry ─────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Geometry {
    pub id: EntityID,
    pub body: EntityID,
    pub mesh: String,
    pub role: GeometryRole,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum GeometryRole {
    Collision,
    Visualization,
    Simulation,
}

// ── Transform / Frame ────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Frame {
    pub id: EntityID,
    pub body: EntityID,
    pub transform: Transform,
}

// ── CableGuide ───────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CableGuide {
    pub id: EntityID,
    pub site: EntityID,
    pub diameter: f64,
}

// ── ExoPart ──────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ExoPart {
    pub id: EntityID,
    pub name: String,
    pub part_type: ExoPartType,
    pub body: EntityID,
    pub offset: Vec3,
    pub ports: Vec<CablePort>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ExoPartType {
    Cuff { coverage: f64 },
    Brace { length: f64 },
    MotorMount { torque: f64 },
    CableGuide { diameter: f64 },
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CablePort {
    pub id: EntityID,
    pub port_type: PortType,
    pub offset: Vec3,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum PortType {
    Entry,
    Exit,
    Termination,
}

// ── Cable ────────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Cable {
    pub id: EntityID,
    pub name: String,
    pub path: Vec<CableSegment>,
    pub tendon: Option<EntityID>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum CableSegment {
    ViaPoint(EntityID),
    Port(EntityID),
    Wrap(EntityID),
}

// ── WrapGeom ─────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WrapGeom {
    pub id: EntityID,
    pub body: EntityID,
    pub geom_type: WrapGeomType,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum WrapGeomType {
    Sphere { radius: f64 },
    Cylinder { radius: f64, length: f64 },
}

// ── Actuator ─────────────────────────────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ActuatorType {
    Motor { gear: f64 },
    Position { kp: f64 },
    Muscle,
}

// ── Site roles (separate components) ─────────────────────────────────

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MuscleAttachment {
    pub site: EntityID,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Landmark {
    pub site: EntityID,
    pub name: String,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WrapPoint {
    pub site: EntityID,
    pub wrap_geom: EntityID,
}

// ── Main ─────────────────────────────────────────────────────────────

fn main() {
    let mut world = World::new();

    // Create ground
    let _ground = world.add_body(0.0, [0.0, 0.0, 0.0], [0.0; 6]);

    // Create pelvis
    let pelvis = world.add_body(11.78, [0.0, 0.0, 0.0], [0.18, 0.22, 0.20, 0.0, 0.0, 0.0]);
    world.add_transform(pelvis, Transform::default());

    // Create femur
    let femur = world.add_body(9.3, [0.0, -0.17, 0.0], [0.12, 0.12, 0.02, 0.0, 0.0, 0.0]);

    // Create hip joint
    let _hip = world.add_joint(
        pelvis,
        femur,
        JointType::Ball,
        Some(JointLimits { lower: -2.0, upper: 2.0 }),
    );

    // Create a site for ASIS landmark
    let asis = world.add_site(pelvis, Vec3::new(0.01, 0.02, 0.13));

    // Attach landmark role to the site
    let mut landmarks = vec![];
    landmarks.push(Landmark { site: asis, name: "ASIS".to_string() });

    // Create a muscle
    let _muscle_id = world.add_muscle(
        "iliopsoas".to_string(),
        vec![
            MusclePoint { body: pelvis, offset: Vec3::new(0.0, 0.0, 0.1) },
            MusclePoint { body: femur, offset: Vec3::new(0.0, -0.2, 0.0) },
        ],
        2000.0,
        0.11,
        0.13,
        30.0,
        0.1,
    );

    // Validate
    let errors = world.validate();
    if errors.is_empty() {
        println!("World is valid");
    } else {
        for e in &errors {
            println!("ERROR: {}", e);
        }
    }

    // Serialize to JSON
    let json = serde_json::to_string_pretty(&world).unwrap();
    println!("World JSON ({} bytes)", json.len());
}
