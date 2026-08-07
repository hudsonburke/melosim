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

    // ── Relationship queries ──

    /// Get the parent entity via ChildOf relationship.
    pub fn parent_of(&self, entity: EntityID) -> Option<EntityID> {
        self.get::<ChildOf>(entity).map(|c| c.parent)
    }

    /// Get all children of an entity.
    pub fn children_of(&self, entity: EntityID) -> Vec<EntityID> {
        self.iter::<ChildOf>()
            .filter(|(_, c)| c.parent == entity)
            .map(|(eid, _)| eid)
            .collect()
    }

    /// Get the parent frame of a joint.
    pub fn joint_parent_frame(&self, joint: EntityID) -> Option<EntityID> {
        self.get::<ParentFrame>(joint).map(|p| p.frame)
    }

    /// Get the child frame of a joint.
    pub fn joint_child_frame(&self, joint: EntityID) -> Option<EntityID> {
        self.get::<ChildFrame>(joint).map(|c| c.frame)
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

    // ── Convenience joint builders ──

    /// Add a hinge (pin) joint between two frame entities.
    /// Creates a Joint entity with 1 coordinate entity, a RotationAboutAxis
    /// effect, and a SpatialTransform.
    /// Returns the joint entity.
    pub fn add_hinge(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        axis: [f64; 3],
        limits: Option<JointLimits>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Relationship components
        self.attach(joint_entity, ParentFrame { frame: parent_frame });
        self.attach(joint_entity, ChildFrame { frame: child_frame });

        // Create coordinate
        let coord_entity = self.spawn();
        self.attach(coord_entity, JointCoordinate {
            range_min: limits.as_ref().map_or(-1e10, |l| l.lower),
            range_max: limits.as_ref().map_or(1e10, |l| l.upper),
            default_value: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            clamped: limits.is_some(),
            locked: false,
            prescribed_function: None,
        });

        // Create CoordinateEffect: rotation about the specified axis
        let effect_entity = self.spawn();
        self.attach(effect_entity, CoordinateEffect {
            coordinate: coord_entity,
            joint: joint_entity,
            component: TransformComponent::RotationAboutAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });

        // Create SpatialTransform
        let st_entity = self.spawn();
        self.attach(st_entity, SpatialTransform {
            joint: joint_entity,
            effects: vec![effect_entity],
        });

        // Create the joint
        self.attach(joint_entity, Joint {
            limits,
            coordinates: vec![coord_entity],
        });

        joint_entity
    }

    /// Add a slide (prismatic) joint between two frame entities.
    /// Creates a Joint entity with 1 coordinate entity, a TranslationAlongAxis
    /// effect, and a SpatialTransform.
    pub fn add_slide(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        axis: [f64; 3],
        limits: Option<JointLimits>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Relationship components
        self.attach(joint_entity, ParentFrame { frame: parent_frame });
        self.attach(joint_entity, ChildFrame { frame: child_frame });

        // Create coordinate
        let coord_entity = self.spawn();
        self.attach(coord_entity, JointCoordinate {
            range_min: limits.as_ref().map_or(-1e10, |l| l.lower),
            range_max: limits.as_ref().map_or(1e10, |l| l.upper),
            default_value: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            clamped: limits.is_some(),
            locked: false,
            prescribed_function: None,
        });

        // Create CoordinateEffect: translation along the specified axis
        let effect_entity = self.spawn();
        self.attach(effect_entity, CoordinateEffect {
            coordinate: coord_entity,
            joint: joint_entity,
            component: TransformComponent::TranslationAlongAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });

        // Create SpatialTransform
        let st_entity = self.spawn();
        self.attach(st_entity, SpatialTransform {
            joint: joint_entity,
            effects: vec![effect_entity],
        });

        // Create the joint
        self.attach(joint_entity, Joint {
            limits,
            coordinates: vec![coord_entity],
        });

        joint_entity
    }

    /// Add a ball (spherical) joint between two frame entities.
    /// Creates a Joint entity with 3 coordinate entities, 3 RotationAboutAxis
    /// effects, and a SpatialTransform.
    pub fn add_ball(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        limits: Option<JointLimits>,
    ) -> EntityID {
        let joint_entity = self.spawn();
        let mut coords = Vec::new();
        let mut effects = Vec::new();

        // Relationship components
        self.attach(joint_entity, ParentFrame { frame: parent_frame });
        self.attach(joint_entity, ChildFrame { frame: child_frame });

        let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        for axis in &axes {
            let coord_entity = self.spawn();
            self.attach(coord_entity, JointCoordinate {
                range_min: limits.as_ref().map_or(-1e10, |l| l.lower),
                range_max: limits.as_ref().map_or(1e10, |l| l.upper),
                default_value: 0.0,
                stiffness: 0.0,
                damping: 0.0,
                clamped: limits.is_some(),
                locked: false,
                prescribed_function: None,
            });

            let effect_entity = self.spawn();
            self.attach(effect_entity, CoordinateEffect {
                coordinate: coord_entity,
                joint: joint_entity,
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });

            coords.push(coord_entity);
            effects.push(effect_entity);
        }

        let st_entity = self.spawn();
        self.attach(st_entity, SpatialTransform {
            joint: joint_entity,
            effects,
        });

        self.attach(joint_entity, Joint {
            limits,
            coordinates: coords,
        });

        joint_entity
    }

    /// Add a free (6-DOF) joint between two frame entities.
    /// Creates a Joint entity with 6 coordinate entities (3 rotation + 3 translation),
    /// effects, and SpatialTransform.
    pub fn add_free(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
    ) -> EntityID {
        let joint_entity = self.spawn();
        let mut coords = Vec::new();
        let mut effects = Vec::new();

        // Relationship components
        self.attach(joint_entity, ParentFrame { frame: parent_frame });
        self.attach(joint_entity, ChildFrame { frame: child_frame });

        // 3 rotation axes
        let rot_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for axis in &rot_axes {
            let coord_entity = self.spawn();
            self.attach(coord_entity, JointCoordinate {
                range_min: -1e10,
                range_max: 1e10,
                default_value: 0.0,
                stiffness: 0.0,
                damping: 0.0,
                clamped: false,
                locked: false,
                prescribed_function: None,
            });
            let effect_entity = self.spawn();
            self.attach(effect_entity, CoordinateEffect {
                coordinate: coord_entity,
                joint: joint_entity,
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
            coords.push(coord_entity);
            effects.push(effect_entity);
        }

        // 3 translation axes
        let trans_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for axis in &trans_axes {
            let coord_entity = self.spawn();
            self.attach(coord_entity, JointCoordinate {
                range_min: -1e10,
                range_max: 1e10,
                default_value: 0.0,
                stiffness: 0.0,
                damping: 0.0,
                clamped: false,
                locked: false,
                prescribed_function: None,
            });
            let effect_entity = self.spawn();
            self.attach(effect_entity, CoordinateEffect {
                coordinate: coord_entity,
                joint: joint_entity,
                component: TransformComponent::TranslationAlongAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
            coords.push(coord_entity);
            effects.push(effect_entity);
        }

        let st_entity = self.spawn();
        self.attach(st_entity, SpatialTransform {
            joint: joint_entity,
            effects,
        });

        self.attach(joint_entity, Joint {
            limits: None,
            coordinates: coords,
        });

        joint_entity
    }

    /// Add a fixed (weld) joint between two frame entities.
    /// Creates a Joint entity with no coordinates or effects.
    pub fn add_fixed(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Relationship components
        self.attach(joint_entity, ParentFrame { frame: parent_frame });
        self.attach(joint_entity, ChildFrame { frame: child_frame });

        self.attach(joint_entity, Joint {
            limits: None,
            coordinates: vec![],
        });

        joint_entity
    }

    /// Add a universal joint between two frame entities.
    /// Creates a Joint entity with 2 coordinate entities, 2 RotationAboutAxis
    /// effects, and a SpatialTransform.
    pub fn add_universal(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        axis1: [f64; 3],
        axis2: [f64; 3],
        limits: Option<JointLimits>,
    ) -> EntityID {
        let joint_entity = self.spawn();
        let mut coords = Vec::new();
        let mut effects = Vec::new();

        // Relationship components
        self.attach(joint_entity, ParentFrame { frame: parent_frame });
        self.attach(joint_entity, ChildFrame { frame: child_frame });

        for axis in &[axis1, axis2] {
            let coord_entity = self.spawn();
            self.attach(coord_entity, JointCoordinate {
                range_min: limits.as_ref().map_or(-1e10, |l| l.lower),
                range_max: limits.as_ref().map_or(1e10, |l| l.upper),
                default_value: 0.0,
                stiffness: 0.0,
                damping: 0.0,
                clamped: limits.is_some(),
                locked: false,
                prescribed_function: None,
            });
            let effect_entity = self.spawn();
            self.attach(effect_entity, CoordinateEffect {
                coordinate: coord_entity,
                joint: joint_entity,
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
            coords.push(coord_entity);
            effects.push(effect_entity);
        }

        let st_entity = self.spawn();
        self.attach(st_entity, SpatialTransform {
            joint: joint_entity,
            effects,
        });

        self.attach(joint_entity, Joint {
            limits,
            coordinates: coords,
        });

        joint_entity
    }

    /// Add a custom joint between two frame entities with pre-created coordinates.
    /// The caller is responsible for creating CoordinateEffect entities.
    pub fn add_custom(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        coordinates: Vec<EntityID>,
        limits: Option<JointLimits>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Relationship components
        self.attach(joint_entity, ParentFrame { frame: parent_frame });
        self.attach(joint_entity, ChildFrame { frame: child_frame });

        self.attach(joint_entity, Joint {
            limits,
            coordinates,
        });

        joint_entity
    }
}

impl std::fmt::Debug for World {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = f.debug_struct("World");
        s.field("inertials", &self.count::<InertialProperties>())
            .field("joints", &self.count::<Joint>())
            .field("coordinates", &self.count::<JointCoordinate>())
            .field("coordinate_effects", &self.count::<CoordinateEffect>())
            .field("spatial_transforms", &self.count::<SpatialTransform>())
            .field("child_of", &self.count::<ChildOf>())
            .field("positions", &self.count::<Position>())
            .field("rotations", &self.count::<Rotation>())
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
