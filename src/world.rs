use bevy_ecs::prelude::*;
use bevy_ecs::world::World as BevyWorld;
use crate::components::*;
use std::ops::{Deref, DerefMut};

/// Resource to hold validation errors.
#[derive(Resource, Default, Clone)]
pub struct ErrorList(pub Vec<String>);

impl Deref for World {
    type Target = BevyWorld;
    fn deref(&self) -> &Self::Target {
        &self.ecs
    }
}

impl DerefMut for World {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.ecs
    }
}

/// The melosim World — wraps `bevy_ecs::World` with convenience methods.
///
/// Components are stored in Bevy's archetype-based storage.
/// Resources (singletons) are stored in Bevy's resource storage.
///
/// Adding a new component type does NOT require modifying this struct:
/// downstream crates just call `world.ecs.spawn(MyComponent { ... })`.
pub struct World {
    pub ecs: BevyWorld,
}

impl World {
    pub fn new() -> Self {
        Self { ecs: BevyWorld::new() }
    }

    // ── Entity lifecycle ──

    /// Spawn a new entity. Returns its unique Entity.
    pub fn spawn(&mut self) -> Entity {
        self.ecs.spawn(()).id()
    }

    /// Spawn an entity with a bundle of components. Returns its Entity.
    pub fn spawn_with<B: Bundle>(&mut self, bundle: B) -> Entity {
        self.ecs.spawn(bundle).id()
    }

    // ── Component access ──

    /// Attach a component to an entity.
    pub fn attach<T: Component>(&mut self, entity: Entity, component: T) {
        self.ecs.entity_mut(entity).insert(component);
    }

    /// Get a component by Entity.
    pub fn get<T: Component>(&self, entity: Entity) -> Option<&T> {
        self.ecs.get::<T>(entity)
    }

    /// Get a mutable component by Entity.
    pub fn get_mut<T: Component>(&mut self, entity: Entity) -> Option<&mut T> {
        self.ecs.get_mut::<T>(entity).map(|m| m.into_inner())
    }

    /// Count entities that have component T.
    pub fn count<T: Component>(&mut self) -> usize {
        self.ecs.query::<&T>().iter(&self.ecs).count()
    }

    /// Iterate over all entities that have component T.
    pub fn iter<T: Component>(&mut self) -> Vec<(Entity, &T)> {
        self.ecs.query::<(Entity, &T)>().iter(&self.ecs).collect()
    }

    /// Get all entity IDs that have component T (returns owned values, no borrow held).
    pub fn entities_with<T: Component>(&mut self) -> Vec<Entity> {
        self.ecs.query_filtered::<Entity, &T>().iter(&self.ecs).collect()
    }

    /// Remove a component from an entity. Returns the component if it existed.
    pub fn remove<T: Component>(&mut self, entity: Entity) {
        self.ecs.entity_mut(entity).remove::<T>();
    }

    // ── Parent-child relationships ──

    /// Set the parent of an entity. Attaches `ChildOf` and maintains
    /// the `Children` list on the parent.
    pub fn set_parent(&mut self, child: Entity, parent: Entity) {
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
        let mut children = self.ecs.entity_mut(parent);
        if let Some(mut children_comp) = children.get_mut::<Children>() {
            if !children_comp.entities.contains(&child) {
                children_comp.entities.push(child);
            }
        } else {
            children.insert(Children { entities: vec![child] });
        }
    }

    /// Remove a child from a parent's Children list.
    fn remove_child(&mut self, parent: Entity, child: Entity) {
        if let Some(children) = self.get_mut::<Children>(parent) {
            children.entities.retain(|&e| e != child);
        }
    }

    // ── Queries ──

    /// Find an entity by its Name component value.
    /// Returns the first entity with a matching name.
    pub fn find_by_name(&mut self, name: &str) -> Option<Entity> {
        self.iter::<Name>()
            .into_iter()
            .find(|(_, n)| n.value == name)
            .map(|(eid, _)| eid)
    }

    /// Find all entities with a given Name component value.
    pub fn find_all_by_name(&mut self, name: &str) -> Vec<Entity> {
        self.iter::<Name>()
            .into_iter()
            .filter(|(_, n)| n.value == name)
            .map(|(eid, _)| eid)
            .collect()
    }

    // ── Relationship queries ──

    /// Get the parent entity via ChildOf relationship.
    pub fn parent_of(&self, entity: Entity) -> Option<Entity> {
        self.get::<ChildOf>(entity).map(|c| c.parent)
    }

    /// Get all children of an entity.
    pub fn children_of(&mut self, entity: Entity) -> Vec<Entity> {
        if let Some(children) = self.get::<Children>(entity) {
            children.entities.clone()
        } else {
            // Fallback: scan ChildOf components
            self.iter::<ChildOf>()
                .into_iter()
                .filter(|(_, c)| c.parent == entity)
                .map(|(eid, _)| eid)
                .collect()
        }
    }

    // ── Typed relationships ──

    /// "entity is positioned relative to frame."
    pub fn set_in_frame(&mut self, entity: Entity, frame: Entity) {
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
    pub fn set_connects(&mut self, joint: Entity, frame: Entity) {
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
    pub fn set_has_dof(&mut self, coord: Entity, joint: Entity) {
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
    pub fn set_drives(&mut self, effect: Entity, coord: Entity) {
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

    pub fn in_frame(&self, entity: Entity) -> Option<Entity> {
        self.get::<InFrame>(entity).map(|r| r.0)
    }

    pub fn frame_contents(&self, frame: Entity) -> Vec<Entity> {
        self.get::<FrameContents>(frame)
            .map(|c| c.entities.clone())
            .unwrap_or_default()
    }

    pub fn connects_to(&self, joint: Entity) -> Option<Entity> {
        self.get::<Connects>(joint).map(|r| r.0)
    }

    pub fn connected_joints(&self, frame: Entity) -> Vec<Entity> {
        self.get::<ConnectedJoints>(frame)
            .map(|c| c.entities.clone())
            .unwrap_or_default()
    }

    pub fn joint_of(&self, coord: Entity) -> Option<Entity> {
        self.get::<HasDOF>(coord).map(|r| r.0)
    }

    pub fn joint_dofs(&self, joint: Entity) -> Vec<Entity> {
        self.get::<JointDOFs>(joint)
            .map(|d| d.entities.clone())
            .unwrap_or_default()
    }

    pub fn drives(&self, effect: Entity) -> Option<Entity> {
        self.get::<Drives>(effect).map(|r| r.0)
    }

    pub fn coordinate_effects(&self, coord: Entity) -> Vec<Entity> {
        self.get::<CoordinateEffects>(coord)
            .map(|e| e.entities.clone())
            .unwrap_or_default()
    }

    // ── Internal helpers for typed relationships ──

    fn add_to_frame_contents(&mut self, frame: Entity, entity: Entity) {
        let mut parent_entity = self.ecs.entity_mut(frame);
        if let Some(mut contents) = parent_entity.get_mut::<FrameContents>() {
            if !contents.entities.contains(&entity) {
                contents.entities.push(entity);
            }
        } else {
            parent_entity.insert(FrameContents { entities: vec![entity] });
        }
    }

    fn remove_from_frame_contents(&mut self, frame: Entity, entity: Entity) {
        if let Some(contents) = self.get_mut::<FrameContents>(frame) {
            contents.entities.retain(|&e| e != entity);
        }
    }

    fn add_to_connected_joints(&mut self, frame: Entity, joint: Entity) {
        let mut parent_entity = self.ecs.entity_mut(frame);
        if let Some(mut list) = parent_entity.get_mut::<ConnectedJoints>() {
            if !list.entities.contains(&joint) {
                list.entities.push(joint);
            }
        } else {
            parent_entity.insert(ConnectedJoints { entities: vec![joint] });
        }
    }

    fn remove_from_connected_joints(&mut self, frame: Entity, joint: Entity) {
        if let Some(cj) = self.get_mut::<ConnectedJoints>(frame) {
            cj.entities.retain(|&e| e != joint);
        }
    }

    fn add_to_joint_dofs(&mut self, joint: Entity, coord: Entity) {
        let mut parent_entity = self.ecs.entity_mut(joint);
        if let Some(mut list) = parent_entity.get_mut::<JointDOFs>() {
            if !list.entities.contains(&coord) {
                list.entities.push(coord);
            }
        } else {
            parent_entity.insert(JointDOFs { entities: vec![coord] });
        }
    }

    fn remove_from_joint_dofs(&mut self, joint: Entity, coord: Entity) {
        if let Some(dofs) = self.get_mut::<JointDOFs>(joint) {
            dofs.entities.retain(|&e| e != coord);
        }
    }

    fn add_to_coordinate_effects(&mut self, coord: Entity, effect: Entity) {
        let mut parent_entity = self.ecs.entity_mut(coord);
        if let Some(mut list) = parent_entity.get_mut::<CoordinateEffects>() {
            if !list.entities.contains(&effect) {
                list.entities.push(effect);
            }
        } else {
            parent_entity.insert(CoordinateEffects { entities: vec![effect] });
        }
    }

    fn remove_from_coordinate_effects(&mut self, coord: Entity, effect: Entity) {
        if let Some(effects) = self.get_mut::<CoordinateEffects>(coord) {
            effects.entities.retain(|&e| e != effect);
        }
    }

    // ── Resource access ──

    pub fn get_resource<T: Resource>(&self) -> Option<&T> {
        self.ecs.get_resource::<T>()
    }

    pub fn get_resource_mut<T: Resource>(&mut self) -> Option<&mut T> {
        self.ecs.get_resource_mut::<T>().map(|m| m.into_inner())
    }

    pub fn insert_resource<T: Resource>(&mut self, resource: T) {
        self.ecs.insert_resource(resource);
    }

    pub fn get_resource_or_default<T: Resource + Default>(&mut self) -> &mut T {
        if self.ecs.get_resource::<T>().is_none() {
            self.ecs.insert_resource(T::default());
        }
        self.ecs.get_resource_mut::<T>().unwrap().into_inner()
    }

    // ── Validation ──

    pub fn validate(&mut self) -> Vec<String> {
        crate::systems::run_systems(self);
        self.get_resource::<ErrorList>()
            .map(|e| e.0.clone())
            .unwrap_or_default()
    }

    // ── Convenience joint builders ──

    /// Add a hinge (pin) joint between two frame entities.
    pub fn add_hinge(
        &mut self,
        parent_frame: Entity,
        child_frame: Entity,
        axis: [f64; 3],
        limits: Option<(f64, f64)>,
    ) -> Entity {
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
    pub fn add_slide(
        &mut self,
        parent_frame: Entity,
        child_frame: Entity,
        axis: [f64; 3],
        limits: Option<(f64, f64)>,
    ) -> Entity {
        let joint_entity = self.spawn();
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

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
            component: TransformComponent::TranslationAlongAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });

        joint_entity
    }

    /// Add a ball (spherical) joint between two frame entities.
    pub fn add_ball(
        &mut self,
        parent_frame: Entity,
        child_frame: Entity,
        limits: Option<(f64, f64)>,
    ) -> Entity {
        let joint_entity = self.spawn();
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
    pub fn add_free(
        &mut self,
        parent_frame: Entity,
        child_frame: Entity,
    ) -> Entity {
        let joint_entity = self.spawn();
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
    pub fn add_fixed(
        &mut self,
        parent_frame: Entity,
        child_frame: Entity,
    ) -> Entity {
        let joint_entity = self.spawn();
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);
        joint_entity
    }

    /// Add a universal joint between two frame entities.
    pub fn add_universal(
        &mut self,
        parent_frame: Entity,
        child_frame: Entity,
        axis1: [f64; 3],
        axis2: [f64; 3],
        limits: Option<(f64, f64)>,
    ) -> Entity {
        let joint_entity = self.spawn();
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
    pub fn add_custom(
        &mut self,
        parent_frame: Entity,
        child_frame: Entity,
        coordinates: Vec<Entity>,
    ) -> Entity {
        let joint_entity = self.spawn();
        self.set_parent(joint_entity, parent_frame);
        self.set_parent(child_frame, joint_entity);

        for &coord in &coordinates {
            self.set_parent(coord, joint_entity);
        }

        joint_entity
    }
}

impl World {
    pub fn debug_summary(&mut self) -> String {
        format!(
            "World {{ inertials: {}, coordinates: {}, effects: {}, muscles: {}, geoms: {} }}",
            self.count::<InertialProperties>(),
            self.count::<JointCoordinate>(),
            self.count::<CoordinateEffect>(),
            self.count::<Muscle>(),
            self.count::<DisplayGeometry>(),
        )
    }
}
