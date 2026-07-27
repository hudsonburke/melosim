use crate::components::*;
use crate::id::EntityKey;
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
}

impl World {
    pub fn new() -> Self {
        Self {
            components: AnyMap::new(),
            resources: AnyMap::new(),
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
        self.ensure_map::<T>().insert(component)
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
    // Resources are singletons stored by type, not per-entity.
    // Useful for configuration, error accumulators, solver parameters.

    /// Get a reference to a resource by type.
    pub fn get_resource<T: 'static>(&self) -> Option<&T> {
        self.resources.get::<T>()
    }

    /// Get a mutable reference to a resource by type.
    pub fn get_resource_mut<T: 'static>(&mut self) -> Option<&mut T> {
        self.resources.get_mut::<T>()
    }

    /// Insert a resource (replaces existing).
    pub fn insert_resource<T: 'static>(&mut self, resource: T) {
        self.resources.insert(resource);
    }

    /// Get or create a resource with Default::default().
    pub fn get_resource_or_default<T: Default + 'static>(&mut self) -> &mut T {
        self.resources.entry::<T>().or_insert_with(T::default)
    }

    // ── Validation (legacy wrapper) ──
    // Runs the built-in validation and returns accumulated errors.
    // Prefer calling validate systems through the SystemRegistry instead.

    pub fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();

        // Joint body reference checks
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
            .field("sites", &self.count::<Site>())
            .field("materials", &self.count::<Material>())
            .field("muscle_params", &self.count::<HillTypeMuscleParams>());
        s.finish()
    }
}
