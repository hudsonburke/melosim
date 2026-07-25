use serde::{Deserialize, Serialize};
use crate::components::*;
use crate::id::EntityID;
use crate::math::{Transform, Vec3};

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
        self.joints.push(Joint { id, body_a, body_b, joint_type, limits });
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
            id, name, path, max_force, optimal_fiber_length,
            tendon_slack_length, pcsa, pennation_angle,
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
        &mut self, body: EntityID, density: f64,
        youngs_modulus: f64, poissons_ratio: f64,
    ) -> EntityID {
        let id = self.spawn();
        self.materials.push(Material {
            id, body, density, youngs_modulus, poissons_ratio,
        });
        id
    }

    pub fn get_material(&self, id: EntityID) -> Option<&Material> {
        self.materials.iter().find(|m| m.id == id)
    }

    // ── Geometry ─────────────────────────────────────────────────

    pub fn add_geometry(&mut self, body: EntityID, mesh: String, role: GeometryRole) -> EntityID {
        let id = self.spawn();
        self.geometries.push(Geometry { id, body, mesh, role });
        id
    }

    // ── Transform ────────────────────────────────────────────────

    pub fn add_transform(&mut self, body: EntityID, transform: Transform) -> EntityID {
        let id = self.spawn();
        self.transforms.push(Frame { id, body, transform });
        id
    }

    pub fn get_transform(&self, body: EntityID) -> Option<&Transform> {
        self.transforms.iter().find(|f| f.body == body).map(|f| &f.transform)
    }

    // ── Tendon ───────────────────────────────────────────────────

    pub fn add_tendon(
        &mut self, name: String, spring_length: f64,
        width: f64, via_points: Vec<EntityID>,
    ) -> EntityID {
        let id = self.spawn();
        self.tendons.push(Tendon { id, name, spring_length, width, via_points });
        id
    }

    // ── CableGuide ───────────────────────────────────────────────

    pub fn add_cable_guide(&mut self, site: EntityID, diameter: f64) -> EntityID {
        let id = self.spawn();
        self.cable_guides.push(CableGuide { id, site, diameter });
        id
    }

    // ── Cable ────────────────────────────────────────────────────

    pub fn add_cable(
        &mut self, name: String,
        path: Vec<CableSegment>, tendon: Option<EntityID>,
    ) -> EntityID {
        let id = self.spawn();
        self.cables.push(Cable { id, name, path, tendon });
        id
    }

    // ── WrapGeom ─────────────────────────────────────────────────

    pub fn add_wrap_geom(&mut self, body: EntityID, geom_type: WrapGeomType) -> EntityID {
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
