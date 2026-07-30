use crate::components::*;
use crate::id::EntityID;
use crate::math::*;
use anymap2::AnyMap;

/// Type alias for per-type component storage.
/// Dense `Vec<Option<T>>` indexed by EntityID (u32).
/// `None` means the entity does not have this component type.
pub type ComponentStorage<T> = Vec<Option<T>>;

/// The dynamic Build World. Components are stored in AnyMap-keyed
/// `Vec<Option<T>>` vectors, indexed by dense EntityID (u32).
///
/// - `spawn()` allocates a new entity ID.
/// - `attach::<T>(entity, component)` stores a component on an entity.
/// - `get::<T>(entity)` retrieves a component.
///
/// Resources (singletons) are stored separately in `resources: AnyMap`.
///
/// Adding a new component type does NOT require modifying this struct:
/// downstream crates just call `world.attach::<MyType>(entity, val)`.
pub struct World {
    pub components: AnyMap,
    pub resources: AnyMap,
    /// Monotonic counter for EntityID assignment.
    /// Each spawn() increments this.
    pub next_id: u32,
}

impl World {
    pub fn new() -> Self {
        Self {
            components: AnyMap::new(),
            resources: AnyMap::new(),
            next_id: 0,
        }
    }

    // ── Entity lifecycle ──

    /// Spawn a new entity. Returns its unique EntityID.
    pub fn spawn(&mut self) -> EntityID {
        let id = self.next_id;
        self.next_id += 1;
        EntityID(id)
    }

    // ── Component access ──

    fn ensure_storage<T: 'static>(&mut self) -> &mut ComponentStorage<T> {
        self.components
            .entry::<ComponentStorage<T>>()
            .or_insert_with(Vec::new)
    }

    /// Attach a component to an entity.
    pub fn attach<T: 'static>(&mut self, entity: EntityID, component: T) {
        let storage = self.ensure_storage::<T>();
        let idx = entity.0 as usize;
        if idx >= storage.len() {
            storage.reserve(idx + 1 - storage.len());
            while storage.len() <= idx {
                storage.push(None);
            }
        }
        storage[idx] = Some(component);
    }

    /// Get a component by EntityID.
    pub fn get<T: 'static>(&self, entity: EntityID) -> Option<&T> {
        self.components
            .get::<ComponentStorage<T>>()
            .and_then(|storage| storage.get(entity.0 as usize)?.as_ref())
    }

    /// Get a mutable component by EntityID.
    pub fn get_mut<T: 'static>(&mut self, entity: EntityID) -> Option<&mut T> {
        self.components
            .get_mut::<ComponentStorage<T>>()
            .and_then(|storage| storage.get_mut(entity.0 as usize)?.as_mut())
    }

    /// Iterate over all entities that have component T.
    pub fn iter<T: 'static>(&self) -> impl Iterator<Item = (EntityID, &T)> {
        self.components
            .get::<ComponentStorage<T>>()
            .into_iter()
            .flat_map(|storage| {
                storage
                    .iter()
                    .enumerate()
                    .filter_map(|(i, opt)| opt.as_ref().map(|c| (EntityID(i as u32), c)))
            })
    }

    /// Get the raw storage for type T (creates if missing for mutable access).
    pub fn storage<T: 'static>(&self) -> Option<&ComponentStorage<T>> {
        self.components.get::<ComponentStorage<T>>()
    }

    /// Get the mutable storage for type T (creates if missing).
    pub fn storage_mut<T: 'static>(&mut self) -> &mut ComponentStorage<T> {
        self.ensure_storage::<T>()
    }

    /// Count entities that have component T.
    pub fn count<T: 'static>(&self) -> usize {
        self.components
            .get::<ComponentStorage<T>>()
            .map_or(0, |storage| storage.iter().filter_map(|x| x.as_ref()).count())
    }

    /// Remove a component from an entity. Returns the component if it existed.
    pub fn remove<T: 'static>(&mut self, entity: EntityID) -> Option<T> {
        self.components
            .get_mut::<ComponentStorage<T>>()
            .and_then(|storage| {
                let idx = entity.0 as usize;
                if idx < storage.len() {
                    storage[idx].take()
                } else {
                    None
                }
            })
    }

    // ── Queries ──

    /// Find an entity by its Name component value.
    /// Returns the first entity with a matching name.
    pub fn find_by_name(&self, name: &str) -> Option<EntityID> {
        self.iter::<Name>()
            .find(|(_, n)| n.value == name)
            .map(|(eid, _)| eid)
    }

    /// Find all entities with a given Name component value.
    /// Useful when multiple entities share a name (e.g., bilateral models).
    pub fn find_all_by_name(&self, name: &str) -> Vec<EntityID> {
        self.iter::<Name>()
            .filter(|(_, n)| n.value == name)
            .map(|(eid, _)| eid)
            .collect()
    }

    // ── Convenience: model editing ──

    /// Attach a mesh geometry to a parent body.
    ///
    /// Creates a new entity with:
    /// - `Frame { parent, transform }` — position relative to parent
    /// - `MeshGeometry { mesh: path, scale }` — mesh reference
    /// - `Name { value }` — entity name
    ///
    /// Returns the new entity's ID.
    pub fn attach_mesh(
        &mut self,
        parent: EntityID,
        mesh_path: &str,
        name: &str,
        offset: Vec3,
    ) -> EntityID {
        let entity = self.spawn();
        self.attach(entity, Frame {
            parent,
            transform: Transform {
                translation: offset,
                rotation: Quaternion::default(),
            },
        });
        self.attach(entity, MeshGeometry {
            mesh: mesh_path.to_string(),
        });
        self.attach(entity, Name { value: name.to_string() });
        entity
    }

    /// Create a new body fixed to a parent.
    ///
    /// Creates a new entity with:
    /// - `InertialProperties { mass, com, inertia }` — dynamics
    /// - `Frame { parent, transform }` — position relative to parent
    /// - `Name { value }` — entity name
    ///
    /// Returns the new entity's ID.
    pub fn attach_body(
        &mut self,
        parent: EntityID,
        name: &str,
        mass: f64,
        offset: Vec3,
    ) -> EntityID {
        let entity = self.spawn();
        self.attach(entity, InertialProperties {
            mass,
            com: [0.0; 3],
            inertia: [0.0; 6],
        });
        self.attach(entity, Frame {
            parent,
            transform: Transform {
                translation: offset,
                rotation: Quaternion::default(),
            },
        });
        self.attach(entity, Name { value: name.to_string() });
        entity
    }

    /// Start building a body attached to a parent.
    ///
    /// Returns a `BodyBuilder` for fluent configuration.
    ///
    /// # Example
    /// ```ignore
    /// let forearm = world.find_by_name("r_forearm").unwrap();
    /// let cuff = world.body_builder("r_forearm")
    ///     .name("arm_cuff")
    ///     .mesh("assets/cuff.stl")
    ///     .mass(0.5)
    ///     .offset(Vec3::new(0.0, 0.0, -0.15))
    ///     .build(&mut world);
    /// ```
    pub fn body_builder(&self, parent_name: &str) -> BodyBuilder {
        BodyBuilder {
            parent_name: parent_name.to_string(),
            name: String::new(),
            mesh: None,
            mass: 0.0,
            offset: Vec3::ZERO,
            rotation: Quaternion::default(),
            display_color: None,
            display_opacity: 1.0,
        }
    }

    // ── Resource access ──

    pub fn get_resource<T: 'static>(&self) -> Option<&T> {
        self.resources.get::<T>()
    }

    pub fn get_resource_mut<T: 'static>(&mut self) -> Option<&mut T> {
        self.resources.get_mut::<T>()
    }

    pub fn insert_resource<T: 'static>(&mut self, resource: T) {
        self.resources.insert(resource);
    }

    pub fn get_resource_or_default<T: Default + 'static>(&mut self) -> &mut T {
        self.resources.entry::<T>().or_insert_with(T::default)
    }

    // ── Validation ──

    pub fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();

        let check_joint = |body_a: EntityID, body_b: EntityID| -> Vec<String> {
            let mut errs = Vec::new();
            if self.get::<InertialProperties>(body_a).is_none() {
                errs.push(format!(
                    "Joint references missing body_a {:?}",
                    body_a.0
                ));
            }
            if self.get::<InertialProperties>(body_b).is_none() {
                errs.push(format!(
                    "Joint references missing body_b {:?}",
                    body_b.0
                ));
            }
            errs
        };

        for (_key, hinge) in self.iter::<HingeJoint>() {
            errors.extend(check_joint(hinge.body_a, hinge.body_b));
        }
        for (_key, slide) in self.iter::<SlideJoint>() {
            errors.extend(check_joint(slide.body_a, slide.body_b));
        }
        for (_key, ball) in self.iter::<BallJoint>() {
            errors.extend(check_joint(ball.body_a, ball.body_b));
        }
        for (_key, free) in self.iter::<FreeJoint>() {
            errors.extend(check_joint(free.body_a, free.body_b));
        }
        for (_key, fixed) in self.iter::<FixedJoint>() {
            errors.extend(check_joint(fixed.body_a, fixed.body_b));
        }
        for (_key, univ) in self.iter::<UniversalJoint>() {
            errors.extend(check_joint(univ.body_a, univ.body_b));
        }
        for (key, custom) in self.iter::<CustomJoint>() {
            errors.extend(check_joint(custom.body_a, custom.body_b));
            for (i, coord_key) in custom.coordinates.iter().enumerate() {
                if self.get::<JointCoordinate>(*coord_key).is_none() {
                    errors.push(format!(
                        "CustomJoint {:?} coordinate[{}] {:?} references missing JointCoordinate",
                        key.0, i, coord_key.0
                    ));
                }
            }
        }
        for (key, effect) in self.iter::<CoordinateEffect>() {
            if self.get::<JointCoordinate>(effect.coordinate).is_none() {
                errors.push(format!(
                    "CoordinateEffect {:?} references missing coordinate {:?}",
                    key.0, effect.coordinate.0
                ));
            }
        }

        for (key, frame) in self.iter::<Frame>() {
            if self.get::<InertialProperties>(frame.parent).is_none() {
                errors.push(format!(
                    "Frame {:?} references missing parent {:?}",
                    key.0, frame.parent.0
                ));
            }
        }

        for (key, site) in self.iter::<Site>() {
            if self.get::<InertialProperties>(site.parent).is_none() {
                errors.push(format!(
                    "Site {:?} references missing parent {:?}",
                    key.0, site.parent.0
                ));
            }
        }

        for (key, act) in self.iter::<CoordinateActuator>() {
            if self.get::<JointCoordinate>(act.coordinate).is_none() {
                errors.push(format!(
                    "CoordinateActuator {:?} references missing coordinate {:?}",
                    key.0, act.coordinate.0
                ));
            }
        }

        errors
    }
}

impl std::fmt::Debug for World {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = f.debug_struct("World");
        s.field("inertials", &self.count::<InertialProperties>())
            .field("frames", &self.count::<Frame>())
            .field("hinge_joints", &self.count::<HingeJoint>())
            .field("slide_joints", &self.count::<SlideJoint>())
            .field("ball_joints", &self.count::<BallJoint>())
            .field("free_joints", &self.count::<FreeJoint>())
            .field("fixed_joints", &self.count::<FixedJoint>())
            .field("universal_joints", &self.count::<UniversalJoint>())
            .field("custom_joints", &self.count::<CustomJoint>())
            .field("coordinates", &self.count::<JointCoordinate>())
            .field("coordinate_effects", &self.count::<CoordinateEffect>())
            .field("spatial_transforms", &self.count::<SpatialTransform>())
            .field("sites", &self.count::<Site>())
            .field("materials", &self.count::<Material>())
            .field("muscles", &self.count::<Muscle>())
            .field("coordinate_actuators", &self.count::<CoordinateActuator>())
            .field("millard_params", &self.count::<Millard2012Params>())
            .field("wraps", &self.count::<WrapGeom>())
            .field("display_geoms", &self.count::<DisplayGeometry>())
            .field("muscle_params", &self.count::<HillTypeMuscleParams>());
        s.field("next_id", &self.next_id);
        s.finish()
    }
}

// ── Body builder ──

/// Fluent builder for creating a body entity attached to a parent.
///
/// Use `world.body_builder("parent_name")` to start, then chain
/// configuration methods, and call `.build(&mut world)` to finalize.
pub struct BodyBuilder {
    parent_name: String,
    name: String,
    mesh: Option<String>,
    mass: f64,
    offset: Vec3,
    rotation: Quaternion,
    display_color: Option<[f64; 3]>,
    display_opacity: f64,
}

impl BodyBuilder {
    /// Set the entity name.
    pub fn name(mut self, name: &str) -> Self {
        self.name = name.to_string();
        self
    }

    /// Attach a mesh file (STL, OBJ, etc.) for visualization.
    pub fn mesh(mut self, path: &str) -> Self {
        self.mesh = Some(path.to_string());
        self
    }

    /// Set the body mass (for dynamics). Defaults to 0.0 (fixed body).
    pub fn mass(mut self, mass: f64) -> Self {
        self.mass = mass;
        self
    }

    /// Set the offset from the parent body.
    pub fn offset(mut self, offset: Vec3) -> Self {
        self.offset = offset;
        self
    }

    /// Set the rotation relative to the parent body.
    pub fn rotation(mut self, rotation: Quaternion) -> Self {
        self.rotation = rotation;
        self
    }

    /// Set display color [r, g, b] (0.0–1.0). Only used if mesh is set.
    pub fn color(mut self, color: [f64; 3]) -> Self {
        self.display_color = Some(color);
        self
    }

    /// Set display opacity (0.0–1.0). Defaults to 1.0.
    pub fn opacity(mut self, opacity: f64) -> Self {
        self.display_opacity = opacity;
        self
    }

    /// Build the body entity and attach it to the parent.
    ///
    /// Returns the new entity ID, or `None` if the parent name wasn't found.
    pub fn build(self, world: &mut World) -> Option<EntityID> {
        let parent = world.find_by_name(&self.parent_name)?;

        let entity = world.spawn();

        // InertialProperties (mass=0 means fixed to parent)
        world.attach(entity, InertialProperties {
            mass: self.mass,
            com: [0.0; 3],
            inertia: [0.0; 6],
        });

        // Frame parented to the target body
        world.attach(entity, Frame {
            parent,
            transform: Transform {
                translation: self.offset,
                rotation: self.rotation,
            },
        });

        // Name
        if !self.name.is_empty() {
            world.attach(entity, Name { value: self.name });
        }

        // Mesh geometry (if provided)
        if let Some(mesh_path) = self.mesh {
            world.attach(entity, MeshGeometry { mesh: mesh_path });

            // Display geometry for visualization
            if let Some(color) = self.display_color {
                world.attach(entity, DisplayGeometry {
                    body: entity,
                    mesh_file: None,
                    scale: [1.0; 3],
                    color,
                    opacity: self.display_opacity,
                    transform: Transform::default(),
                });
            }
        }

        Some(entity)
    }
}
