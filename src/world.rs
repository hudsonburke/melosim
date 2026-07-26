use serde::{Deserialize, Serialize};
use crate::components::*;
use crate::id::EntityID;
use crate::math::{Transform, Vec3};

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct World {
    next_id: u32,
    pub inertials: Vec<InertialProperties>,
    pub frames: Vec<Frame>,
    pub joints: Vec<Joint>,
    pub muscles: Vec<Muscle>,
    pub tendons: Vec<Tendon>,
    pub sites: Vec<Site>,
    pub materials: Vec<Material>,
    pub mesh_geometries: Vec<MeshGeometry>,
    pub primitive_geometries: Vec<PrimitiveGeometry>,
    pub cable_guides: Vec<CableGuide>,
    pub cables: Vec<Cable>,
    pub wrap_geoms: Vec<WrapGeom>,
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

    // ── InertialProperties ───────────────────────────────────────

    pub fn add_inertial(&mut self, mass: f64, com: [f64; 3], inertia: [f64; 6]) -> EntityID {
        let entity = self.spawn();
        self.inertials.push(InertialProperties { entity, mass, com, inertia });
        entity
    }

    pub fn get_inertial(&self, entity: EntityID) -> Option<&InertialProperties> {
        self.inertials.iter().find(|i| i.entity == entity)
    }

    pub fn get_inertial_mut(&mut self, entity: EntityID) -> Option<&mut InertialProperties> {
        self.inertials.iter_mut().find(|i| i.entity == entity)
    }

    // ── Frame ────────────────────────────────────────────────────

    pub fn add_frame(&mut self, body: EntityID, transform: Transform) -> EntityID {
        let entity = self.spawn();
        self.frames.push(Frame { entity, body, transform });
        entity
    }

    pub fn get_frame(&self, body: EntityID) -> Option<&Transform> {
        self.frames.iter().find(|f| f.body == body).map(|f| &f.transform)
    }

    // ── Joint ────────────────────────────────────────────────────

    pub fn add_joint(
        &mut self,
        body_a: EntityID,
        body_b: EntityID,
        joint_type: JointType,
        limits: Option<JointLimits>,
    ) -> EntityID {
        let entity = self.spawn();
        self.joints.push(Joint { entity, body_a, body_b, joint_type, limits });
        entity
    }

    pub fn get_joint(&self, entity: EntityID) -> Option<&Joint> {
        self.joints.iter().find(|j| j.entity == entity)
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
        let entity = self.spawn();
        self.muscles.push(Muscle {
            entity, name, path, max_force, optimal_fiber_length,
            tendon_slack_length, pcsa, pennation_angle,
        });
        entity
    }

    pub fn get_muscle(&self, entity: EntityID) -> Option<&Muscle> {
        self.muscles.iter().find(|m| m.entity == entity)
    }

    // ── Site ─────────────────────────────────────────────────────

    pub fn add_site(&mut self, body: EntityID, offset: Vec3) -> EntityID {
        let entity = self.spawn();
        self.sites.push(Site { entity, body, offset });
        entity
    }

    pub fn get_site(&self, entity: EntityID) -> Option<&Site> {
        self.sites.iter().find(|s| s.entity == entity)
    }

    // ── Material ─────────────────────────────────────────────────

    pub fn add_material(
        &mut self, body: EntityID, density: f64,
        youngs_modulus: f64, poissons_ratio: f64,
    ) -> EntityID {
        let entity = self.spawn();
        self.materials.push(Material {
            entity, body, density, youngs_modulus, poissons_ratio,
        });
        entity
    }

    pub fn get_material(&self, entity: EntityID) -> Option<&Material> {
        self.materials.iter().find(|m| m.entity == entity)
    }

    // ── MeshGeometry ─────────────────────────────────────────────

    pub fn add_mesh_geometry(&mut self, body: EntityID, mesh: String) -> EntityID {
        let entity = self.spawn();
        self.mesh_geometries.push(MeshGeometry { entity, body, mesh });
        entity
    }

    // ── PrimitiveGeometry ────────────────────────────────────────

    pub fn add_primitive_geometry(&mut self, body: EntityID, shape: Shape) -> EntityID {
        let entity = self.spawn();
        self.primitive_geometries.push(PrimitiveGeometry { entity, body, shape });
        entity
    }

    // ── Tendon ───────────────────────────────────────────────────

    pub fn add_tendon(
        &mut self, name: String, spring_length: f64,
        width: f64, via_points: Vec<EntityID>,
    ) -> EntityID {
        let entity = self.spawn();
        self.tendons.push(Tendon { entity, name, spring_length, width, via_points });
        entity
    }

    // ── CableGuide ───────────────────────────────────────────────

    pub fn add_cable_guide(&mut self, site: EntityID, diameter: f64) -> EntityID {
        let entity = self.spawn();
        self.cable_guides.push(CableGuide { entity, site, diameter });
        entity
    }

    // ── Cable ────────────────────────────────────────────────────

    pub fn add_cable(
        &mut self, name: String,
        path: Vec<CableSegment>, tendon: Option<EntityID>,
    ) -> EntityID {
        let entity = self.spawn();
        self.cables.push(Cable { entity, name, path, tendon });
        entity
    }

    // ── WrapGeom ─────────────────────────────────────────────────

    pub fn add_wrap_geom(&mut self, body: EntityID, geom_type: WrapGeomType) -> EntityID {
        let entity = self.spawn();
        self.wrap_geoms.push(WrapGeom { entity, body, geom_type });
        entity
    }

    // ── Validation ───────────────────────────────────────────────

    pub fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();

        for joint in &self.joints {
            if self.get_inertial(joint.body_a).is_none() {
                errors.push(format!(
                    "Joint {:?} references missing body_a {:?}",
                    joint.entity.inner(), joint.body_a.inner()
                ));
            }
            if self.get_inertial(joint.body_b).is_none() {
                errors.push(format!(
                    "Joint {:?} references missing body_b {:?}",
                    joint.entity.inner(), joint.body_b.inner()
                ));
            }
        }

        for muscle in &self.muscles {
            for point in &muscle.path {
                if self.get_inertial(point.body).is_none() {
                    errors.push(format!(
                        "Muscle '{}' path references missing body {:?}",
                        muscle.name, point.body.inner()
                    ));
                }
            }
        }

        for material in &self.materials {
            if self.get_inertial(material.body).is_none() {
                errors.push(format!(
                    "Material {:?} references missing body {:?}",
                    material.entity.inner(), material.body.inner()
                ));
            }
        }

        errors
    }
}
