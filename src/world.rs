use bevy_ecs::prelude::*;
use crate::components::*;

// Re-export bevy_ecs::World as melosim's World type.
// No wrapper — use Bevy's API directly.
pub use bevy_ecs::world::World;

/// Resource to hold validation errors.
#[derive(Resource, Default, Clone)]
pub struct ErrorList(pub Vec<String>);

/// Extension trait providing convenient helper methods on World.
///
/// These replace the old wrapper methods. The only name change is
/// `spawn()` → `spawn_entity()` because Bevy's inherent `spawn()`
/// returns `EntityCommands`, not `Entity`.
pub trait WorldExt {
    /// Spawn an entity and return its Entity id.
    fn spawn_entity(&mut self) -> Entity;

    /// Attach a component to an entity (inserts via entity_mut).
    fn attach(&mut self, entity: Entity, component: impl Component);

    /// Iterate all entities with component T, returning owned (Entity, T) pairs.
    fn iter<T: Component + Clone>(&mut self) -> Vec<(Entity, T)>;

    /// Count entities with component T.
    fn count<T: Component>(&mut self) -> usize;

    /// Get children of an entity (entities whose ChildOf.parent == entity).
    fn children_of(&mut self, entity: Entity) -> Vec<Entity>;

    /// Get the parent of an entity via ChildOf.
    fn parent_of(&self, entity: Entity) -> Option<Entity>;

    /// Set the parent of an entity (inserts ChildOf component).
    fn set_parent(&mut self, child: Entity, parent: Entity);

    /// Find an entity by its Name component.
    fn find_by_name(&mut self, name: &str) -> Option<Entity>;

    /// Validate the world and return accumulated errors.
    fn validate(&mut self) -> Vec<String>;

    /// Get a debug summary of the world contents.
    fn debug_summary(&mut self) -> String;

    /// Add a hinge (pin) joint between two frame entities.
    fn add_hinge(&mut self, parent_frame: Entity, child_frame: Entity, axis: [f64; 3], limits: Option<(f64, f64)>) -> Entity;

    /// Add a free (6-DOF) joint between two frame entities.
    fn add_free(&mut self, parent_frame: Entity, child_frame: Entity) -> Entity;

    /// Add a universal joint between two frame entities.
    fn add_universal(&mut self, parent_frame: Entity, child_frame: Entity, axis1: [f64; 3], axis2: [f64; 3], limits: Option<(f64, f64)>) -> Entity;

    /// Add a custom joint between two frame entities with pre-created coordinates.
    fn add_custom(&mut self, parent_frame: Entity, child_frame: Entity, coordinates: Vec<Entity>, limits: Option<(f64, f64)>) -> Entity;
}

impl WorldExt for World {
    fn spawn_entity(&mut self) -> Entity {
        self.spawn(()).id()
    }

    fn attach(&mut self, entity: Entity, component: impl Component) {
        self.entity_mut(entity).insert(component);
    }

    fn iter<T: Component + Clone>(&mut self) -> Vec<(Entity, T)> {
        let mut query = self.query::<(Entity, &T)>();
        query.iter(&*self).map(|(e, t)| (e, t.clone())).collect()
    }

    fn count<T: Component>(&mut self) -> usize {
        let mut query = self.query::<&T>();
        query.iter(&*self).count()
    }

    fn children_of(&mut self, entity: Entity) -> Vec<Entity> {
        let mut query = self.query::<(Entity, &ChildOf)>();
        query
            .iter(self)
            .filter(|(_, co)| co.parent == entity)
            .map(|(e, _)| e)
            .collect()
    }

    fn parent_of(&self, entity: Entity) -> Option<Entity> {
        self.get::<ChildOf>(entity).map(|co| co.parent)
    }

    fn set_parent(&mut self, child: Entity, parent: Entity) {
        self.entity_mut(child).insert(ChildOf { parent });
    }

    fn find_by_name(&mut self, name: &str) -> Option<Entity> {
        let mut query = self.query::<(Entity, &Name)>();
        query
            .iter(self)
            .find(|(_, n)| n.value == name)
            .map(|(e, _)| e)
    }

    fn validate(&mut self) -> Vec<String> {
        crate::systems::run_systems(self);
        self.get_resource::<ErrorList>()
            .map(|e| e.0.clone())
            .unwrap_or_default()
    }

    fn debug_summary(&mut self) -> String {
        let bodies = self.count::<InertialProperties>();
        let coords = self.count::<JointCoordinate>();
        let muscles = self.count::<Muscle>();
        let effects = self.count::<CoordinateEffect>();
        format!(
            "Bodies: {}, Coordinates: {}, Muscles: {}, Effects: {}",
            bodies, coords, muscles, effects
        )
    }

    fn add_hinge(&mut self, parent_frame: Entity, child_frame: Entity, axis: [f64; 3], limits: Option<(f64, f64)>) -> Entity {
        let joint = self.spawn_entity();
        self.set_parent(joint, parent_frame);
        self.set_parent(child_frame, joint);

        let coord = self.spawn_entity();
        self.set_parent(coord, joint);
        self.attach(coord, JointCoordinate {
            range_min: limits.map_or(-1e10, |l| l.0),
            range_max: limits.map_or(1e10, |l| l.1),
            default_value: 0.0,
            stiffness: 0.0,
            damping: 0.0,
            clamped: limits.is_some(),
            locked: false,
            prescribed_function: None,
        });

        let effect = self.spawn_entity();
        self.set_parent(effect, coord);
        self.attach(effect, CoordinateEffect {
            component: TransformComponent::RotationAboutAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });

        joint
    }

    fn add_free(&mut self, parent_frame: Entity, child_frame: Entity) -> Entity {
        let joint = self.spawn_entity();
        self.set_parent(joint, parent_frame);
        self.set_parent(child_frame, joint);

        let rot_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for axis in &rot_axes {
            let coord = self.spawn_entity();
            self.set_parent(coord, joint);
            self.attach(coord, JointCoordinate {
                range_min: -1e10, range_max: 1e10, default_value: 0.0,
                stiffness: 0.0, damping: 0.0, clamped: false, locked: false, prescribed_function: None,
            });
            let effect = self.spawn_entity();
            self.set_parent(effect, coord);
            self.attach(effect, CoordinateEffect {
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }
        let trans_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        for axis in &trans_axes {
            let coord = self.spawn_entity();
            self.set_parent(coord, joint);
            self.attach(coord, JointCoordinate {
                range_min: -1e10, range_max: 1e10, default_value: 0.0,
                stiffness: 0.0, damping: 0.0, clamped: false, locked: false, prescribed_function: None,
            });
            let effect = self.spawn_entity();
            self.set_parent(effect, coord);
            self.attach(effect, CoordinateEffect {
                component: TransformComponent::TranslationAlongAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }
        joint
    }

    fn add_universal(&mut self, parent_frame: Entity, child_frame: Entity, axis1: [f64; 3], axis2: [f64; 3], limits: Option<(f64, f64)>) -> Entity {
        let joint = self.spawn_entity();
        self.set_parent(joint, parent_frame);
        self.set_parent(child_frame, joint);

        for axis in &[axis1, axis2] {
            let coord = self.spawn_entity();
            self.set_parent(coord, joint);
            self.attach(coord, JointCoordinate {
                range_min: limits.map_or(-1e10, |l| l.0),
                range_max: limits.map_or(1e10, |l| l.1),
                default_value: 0.0,
                stiffness: 0.0, damping: 0.0,
                clamped: limits.is_some(), locked: false, prescribed_function: None,
            });
            let effect = self.spawn_entity();
            self.set_parent(effect, coord);
            self.attach(effect, CoordinateEffect {
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }
        joint
    }

    fn add_custom(&mut self, parent_frame: Entity, child_frame: Entity, coordinates: Vec<Entity>, limits: Option<(f64, f64)>) -> Entity {
        let joint = self.spawn_entity();
        self.set_parent(joint, parent_frame);
        self.set_parent(child_frame, joint);
        for coord in &coordinates {
            self.set_parent(*coord, joint);
        }
        joint
    }
}
