use crate::components::*;
use crate::flat::FlatWorld;
use crate::id::EntityID;
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

    // ── Freeze: Build World → FlatWorld ──

    /// Freeze the current World into a FlatWorld for simulation.
    ///
    /// Each known component type's storage Vec is cloned directly
    /// (already indexed by EntityID, so no translation needed).
    ///
    /// Custom types are NOT collected automatically. After freeze, add them:
    /// ```rust
    /// # use melosim::world::World;
    /// # let mut world = World::new();
    /// # let e = world.spawn();
    /// # world.attach(e, melosim::components::InertialProperties {
    /// #     mass: 1.0, com: [0.0; 3], inertia: [0.0; 6],
    /// # });
    /// # world.attach(e, melosim::components::Name { value: "test".into() });
    /// let mut flat = world.freeze();
    /// flat.extensions.insert::<Vec<Option<f64>>>(vec![None, Some(3.14)]);
    /// ```
    pub fn freeze(&self) -> FlatWorld {
        FlatWorld {
            inertials: extract::<InertialProperties>(self),
            frames: extract::<Frame>(self),
            sites: extract::<Site>(self),
            hinge_joints: extract::<HingeJoint>(self),
            slide_joints: extract::<SlideJoint>(self),
            ball_joints: extract::<BallJoint>(self),
            free_joints: extract::<FreeJoint>(self),
            fixed_joints: extract::<FixedJoint>(self),
            universal_joints: extract::<UniversalJoint>(self),
            custom_joints: extract::<CustomJoint>(self),
            coordinates: extract::<JointCoordinate>(self),
            coordinate_effects: extract::<CoordinateEffect>(self),
            spatial_transforms: extract::<SpatialTransform>(self),
            muscles: extract::<Muscle>(self),
            millard_params: extract::<Millard2012Params>(self),
            wraps: extract::<WrapGeom>(self),
            display_geoms: extract::<DisplayGeometry>(self),
            coordinate_actuators: extract::<CoordinateActuator>(self),
            extensions: AnyMap::new(),
            num_entities: self.next_id,
        }
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

// ── Helper: extract component Vec from World ──

fn extract<T: Clone + 'static>(world: &World) -> Vec<Option<T>> {
    world
        .components
        .get::<ComponentStorage<T>>()
        .cloned()
        .unwrap_or_default()
}
