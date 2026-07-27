use crate::components::*;
use crate::id::EntityKey;
use anymap2::AnyMap;
use slotmap::{Key, SlotMap};

/// Type alias for the component storage pattern.
/// Each component type T is stored in a SlotMap<EntityKey, T>.
pub type ComponentMap<T> = SlotMap<EntityKey, T>;

/// The World stores all components in an AnyMap.
/// Adding a new component type does NOT require modifying this struct.
pub struct World {
    pub components: AnyMap,
}

impl World {
    pub fn new() -> Self {
        Self {
            components: AnyMap::new(),
        }
    }

    /// Ensure a ComponentMap<T> exists in the AnyMap, then return a mutable ref.
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

    /// Validate cross-entity references.
    pub fn validate(&self) -> Vec<String> {
        let mut errors = Vec::new();

        // Check that Joint body references exist
        for (key, joint) in self.iter::<Joint>() {
            if self.get::<InertialProperties>(joint.body_a).is_none() {
                errors.push(format!(
                    "Joint {:?} references missing body_a {:?}",
                    key.data().as_ffi(),
                    joint.body_a.data().as_ffi()
                ));
            }
            if self.get::<InertialProperties>(joint.body_b).is_none() {
                errors.push(format!(
                    "Joint {:?} references missing body_b {:?}",
                    key.data().as_ffi(),
                    joint.body_b.data().as_ffi()
                ));
            }
        }

        // Check that Frame parent references exist
        for (key, frame) in self.iter::<Frame>() {
            if self.get::<InertialProperties>(frame.parent).is_none() {
                errors.push(format!(
                    "Frame {:?} references missing parent {:?}",
                    key.data().as_ffi(),
                    frame.parent.data().as_ffi()
                ));
            }
        }

        // Check that Site parent references exist
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
        f.debug_struct("World")
            .field("inertials", &self.count::<InertialProperties>())
            .field("frames", &self.count::<Frame>())
            .field("joints", &self.count::<Joint>())
            .field("sites", &self.count::<Site>())
            .field("materials", &self.count::<Material>())
            .field("muscles", &self.count::<HillTypeMuscleParams>())
            .finish()
    }
}
