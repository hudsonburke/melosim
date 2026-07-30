use crate::components::*;
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
            .map_or(0, |storage| {
                storage.iter().filter_map(|x| x.as_ref()).count()
            })
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

    pub fn validate(&mut self) -> Vec<String> {
        crate::systems::run_systems(self);
        self.get_resource::<Vec<String>>()
            .cloned()
            .unwrap_or_default()
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
