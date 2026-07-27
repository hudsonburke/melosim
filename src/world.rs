use crate::components::*;
use crate::id::EntityKey;
use crate::flat::FlatWorld;
use anymap2::AnyMap;
use slotmap::{Key, SlotMap};

/// Type alias for the component storage pattern.
/// Each component type T is stored in a SlotMap<EntityKey, T>.
pub type ComponentMap<T> = SlotMap<EntityKey, T>;

/// The World stores all components and resources in separate AnyMaps.
/// - Components: per-entity data, keyed by EntityKey in SlotMaps
/// - Resources: singleton data (sim config, error accumulators, etc.)
///
/// Adding a new component type does NOT require modifying this struct.
pub struct World {
    pub components: AnyMap,
    pub resources: AnyMap,
    /// Monotonic counter for EntityID assignment during freeze.
    /// Each insert() increments this. EntityIDs map to slotmap key indices.
    pub next_id: u64,
}

impl World {
    pub fn new() -> Self {
        Self {
            components: AnyMap::new(),
            resources: AnyMap::new(),
            next_id: 0,
        }
    }

    // ── Component access ──

    fn ensure_map<T: 'static>(&mut self) -> &mut ComponentMap<T> {
        self.components
            .entry::<ComponentMap<T>>()
            .or_insert_with(SlotMap::with_key)
    }

    /// Insert a component for a new entity. Returns the EntityKey.
    pub fn insert<T: 'static>(&mut self, component: T) -> EntityKey {
        let key = self.ensure_map::<T>().insert(component);
        // Track next_id from slotmap key's lower 32 bits (slot index)
        let slot_idx = (key.data().as_ffi() & 0xFFFF_FFFF) as u64;
        self.next_id = self.next_id.max(slot_idx + 1);
        key
    }

    /// Get a component by EntityKey.
    pub fn get<T: 'static>(&self, key: EntityKey) -> Option<&T> {
        self.components.get::<ComponentMap<T>>()?.get(key)
    }

    /// Get a mutable component by EntityKey.
    pub fn get_mut<T: 'static>(&mut self, key: EntityKey) -> Option<&mut T> {
        self.components.get_mut::<ComponentMap<T>>()?.get_mut(key)
    }

    /// Iterate over all components of type T.
    pub fn iter<T: 'static>(&self) -> impl Iterator<Item = (EntityKey, &T)> {
        self.components
            .get::<ComponentMap<T>>()
            .into_iter()
            .flat_map(|map| map.iter())
    }

    /// Get the ComponentMap for type T (creates if missing).
    pub fn map<T: 'static>(&self) -> Option<&ComponentMap<T>> {
        self.components.get::<ComponentMap<T>>()
    }

    /// Get the mutable ComponentMap for type T (creates if missing).
    pub fn map_mut<T: 'static>(&mut self) -> &mut ComponentMap<T> {
        self.ensure_map::<T>()
    }

    /// Count entities of a component type.
    pub fn count<T: 'static>(&self) -> usize {
        self.components
            .get::<ComponentMap<T>>()
            .map_or(0, |m| m.len())
    }

    /// Remove a component from an entity. Returns the component if it existed.
    pub fn remove<T: 'static>(&mut self, key: EntityKey) -> Option<T> {
        self.components
            .get_mut::<ComponentMap<T>>()
            .and_then(|map| map.remove(key))
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
    /// Iterates each known component type's SlotMap and copies data into
    /// dense `Vec<Option<T>>` indexed by the key's slot index (as EntityID).
    ///
    /// Custom types are NOT collected automatically. After freeze, add them:
    /// ```rust
    /// # use melosim::world::World;
    /// # let mut world = World::new();
    /// # world.insert(melosim::components::InertialProperties {
    /// #     mass: 1.0, com: [0.0; 3], inertia: [0.0; 6],
    /// # });
    /// let mut flat = world.freeze();
    /// // Custom types go into extensions:
    /// let custom_vec: Vec<Option<f64>> = vec![None, Some(3.14)];
    /// flat.extensions.insert(custom_vec);
    /// ```
    pub fn freeze(&self) -> FlatWorld {
        let count = self.next_id as usize;

        FlatWorld {
            inertials: collect_dense::<InertialProperties>(self, count),
            frames: collect_dense::<Frame>(self, count),
            sites: collect_dense::<Site>(self, count),
            hinge_joints: collect_dense::<HingeJoint>(self, count),
            slide_joints: collect_dense::<SlideJoint>(self, count),
            ball_joints: collect_dense::<BallJoint>(self, count),
            free_joints: collect_dense::<FreeJoint>(self, count),
            fixed_joints: collect_dense::<FixedJoint>(self, count),
            universal_joints: collect_dense::<UniversalJoint>(self, count),
            custom_joints: collect_dense::<CustomJoint>(self, count),
            coordinates: collect_dense::<JointCoordinate>(self, count),
            coordinate_effects: collect_dense::<CoordinateEffect>(self, count),
            spatial_transforms: collect_dense::<SpatialTransform>(self, count),
            muscles: collect_dense::<Muscle>(self, count),
            millard_params: collect_dense::<Millard2012Params>(self, count),
            wraps: collect_dense::<WrapGeom>(self, count),
            display_geoms: collect_dense::<DisplayGeometry>(self, count),
            extensions: AnyMap::new(),
            num_entities: self.next_id as u32,
        }
    }

    // ── Validation ──

    pub fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();

        let check_joint = |body_a: EntityKey, body_b: EntityKey| -> Vec<String> {
            let mut errs = Vec::new();
            if self.get::<InertialProperties>(body_a).is_none() {
                errs.push(format!(
                    "Joint references missing body_a {:?}",
                    body_a.data().as_ffi()
                ));
            }
            if self.get::<InertialProperties>(body_b).is_none() {
                errs.push(format!(
                    "Joint references missing body_b {:?}",
                    body_b.data().as_ffi()
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
            for coord_key in &custom.coordinates {
                if self.get::<JointCoordinate>(*coord_key).is_none() {
                    errors.push(format!(
                        "CustomJoint {:?} references missing coordinate {:?}",
                        key.data().as_ffi(),
                        coord_key.data().as_ffi()
                    ));
                }
            }
        }
        for (key, effect) in self.iter::<CoordinateEffect>() {
            if self.get::<JointCoordinate>(effect.coordinate).is_none() {
                errors.push(format!(
                    "CoordinateEffect {:?} references missing coordinate {:?}",
                    key.data().as_ffi(),
                    effect.coordinate.data().as_ffi()
                ));
            }
        }

        for (key, frame) in self.iter::<Frame>() {
            if self.get::<InertialProperties>(frame.parent).is_none() {
                errors.push(format!(
                    "Frame {:?} references missing parent {:?}",
                    key.data().as_ffi(),
                    frame.parent.data().as_ffi()
                ));
            }
        }

        for (key, site) in self.iter::<Site>() {
            if self.get::<InertialProperties>(site.parent).is_none() {
                errors.push(format!(
                    "Site {:?} references missing parent {:?}",
                    key.data().as_ffi(),
                    site.parent.data().as_ffi()
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
            .field("millard_params", &self.count::<Millard2012Params>())
            .field("wraps", &self.count::<WrapGeom>())
            .field("display_geoms", &self.count::<DisplayGeometry>())
            .field("muscle_params", &self.count::<HillTypeMuscleParams>());
        s.field("next_id", &self.next_id);
        s.finish()
    }
}

// ── Helper: collect SlotMap into dense Vec ──

fn collect_dense<T: Clone + 'static>(world: &World, count: usize) -> Vec<Option<T>> {
    let mut vec = vec![None; count];
    if let Some(slotmap) = world.components.get::<ComponentMap<T>>() {
        for (key, component) in slotmap.iter() {
            let idx = (key.data().as_ffi() & 0xFFFF_FFFF) as usize;
            if idx < count {
                vec[idx] = Some(component.clone());
            }
        }
    }
    vec
}
