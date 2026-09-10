use bevy::prelude::*;

use super::{
    attach_joint, validate_joint_endpoints, Body, Coordinate, CoordinateOf, CoordinateProperties,
    CoordinateState, DrivenBy, Frame, InitialConditions, Joint, KinematicError, Site, Twist,
    CouplingKind, InertialProperties,
};

/// Format-neutral coordinate data consumed by the live ECS model builder.
///
/// A `CoordinateSpec` with `driven_by` is a derived axis coordinate: it owns
/// its own `Twist`, but obtains its scalar value from another coordinate.
#[derive(Clone, Debug)]
pub struct CoordinateSpec {
    pub name: String,
    pub properties: CoordinateProperties,
    pub initial: InitialConditions,
    pub twist: Option<Twist>,
    pub driven_by: Option<(Entity, CouplingKind)>,
}

/// Entities created for one body connection.
#[derive(Clone, Copy, Debug)]
pub struct JointConnection {
    pub frame: Entity,
    pub joint: Entity,
    pub child: Entity,
}

/// Validated construction boundary for the canonical Bevy model.
pub struct ModelBuilder<'w> {
    world: &'w mut World,
}

// TODO: Maybe make this into Template functions
impl<'w> ModelBuilder<'w> {
    pub fn new(world: &'w mut World) -> Self {
        Self { world }
    }

    pub fn world_mut(&mut self) -> &mut World {
        self.world
    }

    /// Spawn a body, optionally attaching it to an existing transform parent.
    pub fn body(
        &mut self,
        name: impl Into<String>,
        inertial: InertialProperties,
        transform: Transform,
        parent: Option<Entity>,
    ) -> Entity {
        let mut body = self
            .world
            .spawn((Body, Name::new(name.into()), inertial, transform));
        if let Some(parent) = parent {
            body.insert(ChildOf(parent));
        }
        body.id()
    }

    /// Spawn a fixed frame under an existing body or frame.
    pub fn frame(
        &mut self,
        name: impl Into<String>,
        transform: Transform,
        parent: Entity,
    ) -> Entity {
        self.world
            .spawn((Frame, Name::new(name.into()), transform, ChildOf(parent)))
            .id()
    }

    /// Spawn a site under a body or frame.
    pub fn site(&mut self, name: impl Into<String>, transform: Transform, parent: Entity) -> Entity {
        self.world
            .spawn((Site, Name::new(name.into()), transform, ChildOf(parent)))
            .id()
    }

    /// Spawn a coordinate before its owning joint is known. This is useful for
    /// importers that must resolve cross-joint coordinate references first.
    pub fn coordinate(&mut self, spec: CoordinateSpec) -> Entity {
        let mut coordinate = self.world.spawn((
            Coordinate,
            Name::new(spec.name),
            spec.properties,
            spec.initial.clone(),
            CoordinateState {
                value: spec.initial.value,
                velocity: spec.initial.velocity,
            },
        ));
        if let Some(twist) = spec.twist {
            coordinate.insert(twist);
        }
        let coordinate = coordinate.id();
        if let Some((source, coupling)) = spec.driven_by {
            self.world.entity_mut(coordinate).insert(DrivenBy {
                source,
                coupling,
            });
        }
        coordinate
    }

    /// Add an existing coordinate to a joint's ordered coordinate target.
    pub fn add_coordinate_to_joint(
        &mut self,
        joint: Entity,
        coordinate: Entity,
    ) -> Result<(), KinematicError> {
        if self.world.get::<Joint>(joint).is_none() {
            return Err(KinematicError::NotAJoint(joint));
        }
        if self.world.get::<Coordinate>(coordinate).is_none() {
            return Err(KinematicError::NotACoordinate(coordinate));
        }
        self.world.entity_mut(coordinate).insert(CoordinateOf(joint));
        Ok(())
    }

    /// Spawn a fixed-frame → joint → child-body/frame connection and validate
    /// the two `ChildOf` links through the model construction API.
    pub fn joint_connection(
        &mut self,
        parent: Entity,
        frame_name: impl Into<String>,
        frame_transform: Transform,
        joint_name: impl Into<String>,
        child: Entity,
        child_transform: Transform,
        coordinates: impl IntoIterator<Item = CoordinateSpec>,
    ) -> Result<JointConnection, KinematicError> {
        validate_joint_endpoints(self.world, parent, child)?;
        self.world.entity_mut(child).insert(child_transform);

        let frame = self.frame(frame_name, frame_transform, parent);
        let joint = self
            .world
            .spawn((Joint, Name::new(joint_name.into()), Transform::IDENTITY))
            .id();

        for spec in coordinates {
            let coordinate = self.coordinate(spec);
            self.add_coordinate_to_joint(joint, coordinate)?;
        }

        attach_joint(self.world, frame, joint, child)?;
        Ok(JointConnection { frame, joint, child })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::validate_kinematic_hierarchy;

    #[test]
    fn builder_creates_valid_joint_connection() {
        let mut world = World::new();
        let parent = world
            .spawn((Body, Transform::IDENTITY, InertialProperties::default()))
            .id();
        let child = world
            .spawn((Body, Transform::IDENTITY, InertialProperties::default()))
            .id();

        let coordinate = CoordinateSpec {
            name: "hinge_q".into(),
            properties: CoordinateProperties::default(),
            initial: InitialConditions::default(),
            twist: Some(Twist::rotation(nalgebra::Vector3::z())),
            driven_by: None,
        };

        ModelBuilder::new(&mut world)
            .joint_connection(
                parent,
                "hinge_frame",
                Transform::from_xyz(1.0, 0.0, 0.0),
                "hinge",
                child,
                Transform::IDENTITY,
                [coordinate],
            )
            .unwrap();

        assert!(validate_kinematic_hierarchy(&mut world).is_empty());
    }
}
