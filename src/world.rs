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

    // ── Parent-child relationships ──

    /// Set the parent of an entity. Attaches `ChildOf` and maintains
    /// the `Children` list on the parent.
    ///
    /// If the entity already has a parent, removes it from the old
    /// parent's Children list first.
    pub fn set_parent(&mut self, child: EntityID, parent: EntityID) {
        // If child already has a parent, remove from old parent's Children
        if let Some(old_parent) = self.get::<ChildOf>(child).map(|c| c.parent) {
            if old_parent != parent {
                self.remove_child(old_parent, child);
            } else {
                return; // Already parented to this entity
            }
        }

        // Set ChildOf on child
        self.attach(child, ChildOf { parent });

        // Add to parent's Children list
        let children = self
            .components
            .entry::<ComponentStorage<Children>>()
            .or_insert_with(Vec::new);
        let idx = parent.0 as usize;
        if idx >= children.len() {
            children.reserve(idx + 1 - children.len());
            while children.len() <= idx {
                children.push(None);
            }
        }
        if let Some(ref mut list) = children[idx] {
            if !list.entities.contains(&child) {
                list.entities.push(child);
            }
        } else {
            children[idx] = Some(Children {
                entities: vec![child],
            });
        }
    }

    /// Remove a child from a parent's Children list.
    fn remove_child(&mut self, parent: EntityID, child: EntityID) {
        if let Some(children) = self.get_mut::<Children>(parent) {
            children.entities.retain(|&e| e != child);
        }
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
    /// Uses the Children component (O(1)) if available, falls back to scanning.
    pub fn children_of(&self, entity: EntityID) -> Vec<EntityID> {
        if let Some(children) = self.get::<Children>(entity) {
            children.entities.clone()
        } else {
            // Fallback: scan ChildOf components
            self.iter::<ChildOf>()
                .filter(|(_, c)| c.parent == entity)
                .map(|(eid, _)| eid)
                .collect()
        }
    }

    // ── Typed relationships ──

    /// "entity is positioned relative to frame."
    /// Maintains InFrame on child and FrameContents on frame.
    pub fn set_in_frame(&mut self, entity: EntityID, frame: EntityID) {
        // Remove from old frame's contents if already in one
        if let Some(old_frame) = self.get::<InFrame>(entity).map(|r| r.0) {
            if old_frame != frame {
                self.remove_from_frame_contents(old_frame, entity);
            } else {
                return;
            }
        }
        self.attach(entity, InFrame(frame));
        self.add_to_frame_contents(frame, entity);
    }

    /// "joint connects to this frame (child side)."
    /// Maintains Connects on joint and ConnectedJoints on frame.
    pub fn set_connects(&mut self, joint: EntityID, frame: EntityID) {
        if let Some(old_frame) = self.get::<Connects>(joint).map(|r| r.0) {
            if old_frame != frame {
                self.remove_from_connected_joints(old_frame, joint);
            } else {
                return;
            }
        }
        self.attach(joint, Connects(frame));
        self.add_to_connected_joints(frame, joint);
    }

    /// "coordinate belongs to this joint."
    /// Maintains HasDOF on coord and JointDOFs on joint.
    pub fn set_has_dof(&mut self, coord: EntityID, joint: EntityID) {
        if let Some(old_joint) = self.get::<HasDOF>(coord).map(|r| r.0) {
            if old_joint != joint {
                self.remove_from_joint_dofs(old_joint, coord);
            } else {
                return;
            }
        }
        self.attach(coord, HasDOF(joint));
        self.add_to_joint_dofs(joint, coord);
    }

    /// "effect reads from this coordinate."
    /// Maintains Drives on effect and CoordinateEffects on coord.
    pub fn set_drives(&mut self, effect: EntityID, coord: EntityID) {
        if let Some(old_coord) = self.get::<Drives>(effect).map(|r| r.0) {
            if old_coord != coord {
                self.remove_from_coordinate_effects(old_coord, effect);
            } else {
                return;
            }
        }
        self.attach(effect, Drives(coord));
        self.add_to_coordinate_effects(coord, effect);
    }

    // ── Typed relationship queries ──

    /// Get the frame this entity is positioned in.
    pub fn in_frame(&self, entity: EntityID) -> Option<EntityID> {
        self.get::<InFrame>(entity).map(|r| r.0)
    }

    /// Get all entities positioned in this frame.
    pub fn frame_contents(&self, frame: EntityID) -> Vec<EntityID> {
        self.get::<FrameContents>(frame)
            .map(|c| c.entities.clone())
            .unwrap_or_default()
    }

    /// Get the frame this joint connects to (child side).
    pub fn connects_to(&self, joint: EntityID) -> Option<EntityID> {
        self.get::<Connects>(joint).map(|r| r.0)
    }

    /// Get all joints that connect to this frame.
    pub fn connected_joints(&self, frame: EntityID) -> Vec<EntityID> {
        self.get::<ConnectedJoints>(frame)
            .map(|c| c.entities.clone())
            .unwrap_or_default()
    }

    /// Get the joint this coordinate belongs to.
    pub fn joint_of(&self, coord: EntityID) -> Option<EntityID> {
        self.get::<HasDOF>(coord).map(|r| r.0)
    }

    /// Get all coordinates (DOFs) in this joint.
    pub fn joint_dofs(&self, joint: EntityID) -> Vec<EntityID> {
        self.get::<JointDOFs>(joint)
            .map(|d| d.entities.clone())
            .unwrap_or_default()
    }

    /// Get the coordinate this effect reads from.
    pub fn drives(&self, effect: EntityID) -> Option<EntityID> {
        self.get::<Drives>(effect).map(|r| r.0)
    }

    /// Get all effects driven by this coordinate.
    pub fn coordinate_effects(&self, coord: EntityID) -> Vec<EntityID> {
        self.get::<CoordinateEffects>(coord)
            .map(|e| e.entities.clone())
            .unwrap_or_default()
    }

    // ── Internal helpers for typed relationships ──

    fn add_to_frame_contents(&mut self, frame: EntityID, entity: EntityID) {
        let storage = self.components
            .entry::<ComponentStorage<FrameContents>>()
            .or_insert_with(Vec::new);
        let idx = frame.0 as usize;
        if idx >= storage.len() {
            storage.reserve(idx + 1 - storage.len());
            while storage.len() <= idx { storage.push(None); }
        }
        if let Some(ref mut list) = storage[idx] {
            if !list.entities.contains(&entity) { list.entities.push(entity); }
        } else {
            storage[idx] = Some(FrameContents { entities: vec![entity] });
        }
    }

    fn remove_from_frame_contents(&mut self, frame: EntityID, entity: EntityID) {
        if let Some(contents) = self.get_mut::<FrameContents>(frame) {
            contents.entities.retain(|&e| e != entity);
        }
    }

    fn add_to_connected_joints(&mut self, frame: EntityID, joint: EntityID) {
        let storage = self.components
            .entry::<ComponentStorage<ConnectedJoints>>()
            .or_insert_with(Vec::new);
        let idx = frame.0 as usize;
        if idx >= storage.len() {
            storage.reserve(idx + 1 - storage.len());
            while storage.len() <= idx { storage.push(None); }
        }
        if let Some(ref mut list) = storage[idx] {
            if !list.entities.contains(&joint) { list.entities.push(joint); }
        } else {
            storage[idx] = Some(ConnectedJoints { entities: vec![joint] });
        }
    }

    fn remove_from_connected_joints(&mut self, frame: EntityID, joint: EntityID) {
        if let Some(cj) = self.get_mut::<ConnectedJoints>(frame) {
            cj.entities.retain(|&e| e != joint);
        }
    }

    fn add_to_joint_dofs(&mut self, joint: EntityID, coord: EntityID) {
        let storage = self.components
            .entry::<ComponentStorage<JointDOFs>>()
            .or_insert_with(Vec::new);
        let idx = joint.0 as usize;
        if idx >= storage.len() {
            storage.reserve(idx + 1 - storage.len());
            while storage.len() <= idx { storage.push(None); }
        }
        if let Some(ref mut list) = storage[idx] {
            if !list.entities.contains(&coord) { list.entities.push(coord); }
        } else {
            storage[idx] = Some(JointDOFs { entities: vec![coord] });
        }
    }

    fn remove_from_joint_dofs(&mut self, joint: EntityID, coord: EntityID) {
        if let Some(dofs) = self.get_mut::<JointDOFs>(joint) {
            dofs.entities.retain(|&e| e != coord);
        }
    }

    fn add_to_coordinate_effects(&mut self, coord: EntityID, effect: EntityID) {
        let storage = self.components
            .entry::<ComponentStorage<CoordinateEffects>>()
            .or_insert_with(Vec::new);
        let idx = coord.0 as usize;
        if idx >= storage.len() {
            storage.reserve(idx + 1 - storage.len());
            while storage.len() <= idx { storage.push(None); }
        }
        if let Some(ref mut list) = storage[idx] {
            if !list.entities.contains(&effect) { list.entities.push(effect); }
        } else {
            storage[idx] = Some(CoordinateEffects { entities: vec![effect] });
        }
    }

    fn remove_from_coordinate_effects(&mut self, coord: EntityID, effect: EntityID) {
        if let Some(effects) = self.get_mut::<CoordinateEffects>(coord) {
            effects.entities.retain(|&e| e != effect);
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

    pub fn validate(&mut self) -> Vec<String> {
        crate::systems::run_systems(self);
        self.get_resource::<Vec<String>>()
            .cloned()
            .unwrap_or_default()
    }

    // ── Convenience joint builders ──

    /// Add a hinge (pin) joint between two frame entities.
    /// Creates a joint entity as an intermediate node in the ChildOf hierarchy,
    /// with 1 coordinate entity and a RotationAboutAxis effect.
    /// Returns the joint entity.
    pub fn add_hinge(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        axis: [f64; 3],
        limits: Option<(f64, f64)>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Hierarchy: joint is child of parent frame, child frame is child of joint
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        // Create coordinate (child of joint)
        let coord_entity = self.spawn();
        self.set_parent(coord_entity, joint_entity);
        self.attach(coord_entity, JointCoordinate {
            range_min: limits.map_or(-1e10, |l| l.0),
            range_max: limits.map_or(1e10, |l| l.1),
            default_value: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            clamped: limits.is_some(),
            locked: false,
            prescribed_function: None,
        });

        // Create CoordinateEffect (child of coordinate)
        let effect_entity = self.spawn();
        self.set_parent(effect_entity, coord_entity);
        self.attach(effect_entity, CoordinateEffect {
            component: TransformComponent::RotationAboutAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });

        joint_entity
    }

    /// Add a slide (prismatic) joint between two frame entities.
    /// Creates a joint entity as an intermediate node in the ChildOf hierarchy,
    /// with 1 coordinate entity and a TranslationAlongAxis effect.
    pub fn add_slide(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        axis: [f64; 3],
        limits: Option<(f64, f64)>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Hierarchy
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        // Create coordinate (child of joint)
        let coord_entity = self.spawn();
        self.set_parent(coord_entity, joint_entity);
        self.attach(coord_entity, JointCoordinate {
            range_min: limits.map_or(-1e10, |l| l.0),
            range_max: limits.map_or(1e10, |l| l.1),
            default_value: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            clamped: limits.is_some(),
            locked: false,
            prescribed_function: None,
        });

        // Create CoordinateEffect (child of coordinate)
        let effect_entity = self.spawn();
        self.set_parent(effect_entity, coord_entity);
        self.attach(effect_entity, CoordinateEffect {
            component: TransformComponent::TranslationAlongAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });

        joint_entity
    }

    /// Add a ball (spherical) joint between two frame entities.
    /// Creates a joint entity as an intermediate node in the ChildOf hierarchy,
    /// with 3 coordinate entities and 3 RotationAboutAxis effects.
    pub fn add_ball(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        limits: Option<(f64, f64)>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Hierarchy
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

        for axis in &axes {
            let coord_entity = self.spawn();
            self.set_parent(coord_entity, joint_entity);
            self.attach(coord_entity, JointCoordinate {
                range_min: limits.map_or(-1e10, |l| l.0),
                range_max: limits.map_or(1e10, |l| l.1),
                default_value: 0.0,
                stiffness: 0.0,
                damping: 0.0,
                clamped: limits.is_some(),
                locked: false,
                prescribed_function: None,
            });

            let effect_entity = self.spawn();
            self.set_parent(effect_entity, coord_entity);
            self.attach(effect_entity, CoordinateEffect {
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }

        joint_entity
    }

    /// Add a free (6-DOF) joint between two frame entities.
    /// Creates a joint entity as an intermediate node in the ChildOf hierarchy,
    /// with 6 coordinate entities (3 rotation + 3 translation) and effects.
    pub fn add_free(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Hierarchy
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        // 3 rotation axes
        let rot_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for axis in &rot_axes {
            let coord_entity = self.spawn();
            self.set_parent(coord_entity, joint_entity);
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
            self.set_parent(effect_entity, coord_entity);
            self.attach(effect_entity, CoordinateEffect {
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }

        // 3 translation axes
        let trans_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for axis in &trans_axes {
            let coord_entity = self.spawn();
            self.set_parent(coord_entity, joint_entity);
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
            self.set_parent(effect_entity, coord_entity);
            self.attach(effect_entity, CoordinateEffect {
                component: TransformComponent::TranslationAlongAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }

        joint_entity
    }

    /// Add a fixed (weld) joint between two frame entities.
    /// Creates a joint entity as an intermediate node in the ChildOf hierarchy
    /// with no coordinates or effects.
    pub fn add_fixed(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Hierarchy
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        joint_entity
    }

    /// Add a universal joint between two frame entities.
    /// Creates a joint entity as an intermediate node in the ChildOf hierarchy,
    /// with 2 coordinate entities and 2 RotationAboutAxis effects.
    pub fn add_universal(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        axis1: [f64; 3],
        axis2: [f64; 3],
        limits: Option<(f64, f64)>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Hierarchy
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        for axis in &[axis1, axis2] {
            let coord_entity = self.spawn();
            self.set_parent(coord_entity, joint_entity);
            self.attach(coord_entity, JointCoordinate {
                range_min: limits.map_or(-1e10, |l| l.0),
                range_max: limits.map_or(1e10, |l| l.1),
                default_value: 0.0,
                stiffness: 0.0,
                damping: 0.0,
                clamped: limits.is_some(),
                locked: false,
                prescribed_function: None,
            });
            let effect_entity = self.spawn();
            self.set_parent(effect_entity, coord_entity);
            self.attach(effect_entity, CoordinateEffect {
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }

        joint_entity
    }

    /// Add a custom joint between two frame entities with pre-created coordinates.
    /// The caller is responsible for creating CoordinateEffect entities.
    /// Coordinates are set as children of the joint entity.
    pub fn add_custom(
        &mut self,
        parent_frame: EntityID,
        child_frame: EntityID,
        coordinates: Vec<EntityID>,
    ) -> EntityID {
        let joint_entity = self.spawn();

        // Hierarchy
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        // Set coordinates as children of the joint
        for &coord in &coordinates {
            self.set_parent(coord, joint_entity);
        }

        joint_entity
    }
}

impl std::fmt::Debug for World {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = f.debug_struct("World");
        s.field("inertials", &self.count::<InertialProperties>())
            .field("coordinates", &self.count::<JointCoordinate>())
            .field("coordinate_effects", &self.count::<CoordinateEffect>())
            .field("child_of", &self.count::<ChildOf>())
            .field("children", &self.count::<Children>())
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
