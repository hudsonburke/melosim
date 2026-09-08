use std::collections::HashSet;
use std::fmt;

use bevy::prelude::*;

use super::{Body, Coordinate, CoordinateOf, Frame, Joint, JointCoordinates};

/// A structural problem in the model's body/frame/joint tree.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct KinematicIssue {
    pub entity: Entity,
    pub message: String,
}

impl fmt::Display for KinematicIssue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.entity, self.message)
    }
}

/// Failure while constructing one kinematic connection.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum KinematicError {
    MissingEntity(Entity),
    ParentMustBeBodyOrFrame(Entity),
    NotAJoint(Entity),
    ChildMustBeBodyOrFrame(Entity),
    AlreadyHasParent(Entity),
    WouldCreateCycle,
}

impl fmt::Display for KinematicError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::MissingEntity(entity) => write!(f, "entity {entity} does not exist"),
            Self::ParentMustBeBodyOrFrame(entity) => {
                write!(f, "joint parent {entity} is not a Body or Frame")
            }
            Self::NotAJoint(entity) => write!(f, "entity {entity} is not a Joint"),
            Self::ChildMustBeBodyOrFrame(entity) => {
                write!(f, "joint child {entity} is not a Body or Frame")
            }
            Self::AlreadyHasParent(entity) => write!(f, "entity {entity} already has a parent"),
            Self::WouldCreateCycle => write!(f, "kinematic connection would create a cycle"),
        }
    }
}

/// Attach an existing joint between an existing parent and child node.
///
/// This is the validated construction boundary for importers and editor
/// actions. `ChildOf` remains the source of truth for both transform and
/// kinematic topology; this function simply prevents malformed connections
/// from being inserted through those relationships.
pub fn attach_joint(
    world: &mut World,
    parent: Entity,
    joint: Entity,
    child: Entity,
) -> Result<(), KinematicError> {
    for entity in [parent, joint, child] {
        if world.get_entity(entity).is_err() {
            return Err(KinematicError::MissingEntity(entity));
        }
    }

    if world.get::<Body>(parent).is_none() && world.get::<Frame>(parent).is_none() {
        return Err(KinematicError::ParentMustBeBodyOrFrame(parent));
    }
    if world.get::<Joint>(joint).is_none() {
        return Err(KinematicError::NotAJoint(joint));
    }
    if world.get::<Body>(child).is_none() && world.get::<Frame>(child).is_none() {
        return Err(KinematicError::ChildMustBeBodyOrFrame(child));
    }
    if parent == joint || parent == child || joint == child {
        return Err(KinematicError::WouldCreateCycle);
    }
    if world.get::<ChildOf>(joint).is_some() {
        return Err(KinematicError::AlreadyHasParent(joint));
    }
    if world.get::<ChildOf>(child).is_some() {
        return Err(KinematicError::AlreadyHasParent(child));
    }
    if has_ancestor(world, parent, child) {
        return Err(KinematicError::WouldCreateCycle);
    }

    world.entity_mut(joint).insert(ChildOf(parent));
    world.entity_mut(child).insert(ChildOf(joint));
    Ok(())
}

/// Validate all joints and their coordinate ownership relationships.
pub fn validate_kinematic_hierarchy(world: &mut World) -> Vec<KinematicIssue> {
    let joints: Vec<Entity> = world
        .query_filtered::<Entity, With<Joint>>()
        .iter(world)
        .collect();
    let mut issues = Vec::new();

    for joint in joints {
        let Some(parent) = world.get::<ChildOf>(joint).map(ChildOf::parent) else {
            issues.push(issue(joint, "joint has no parent Body or Frame"));
            continue;
        };

        if world.get::<Body>(parent).is_none() && world.get::<Frame>(parent).is_none() {
            issues.push(issue(joint, "joint parent is not a Body or Frame"));
        }

        let children: Vec<Entity> = world
            .get::<Children>(joint)
            .map(|children| {
                children
                    .iter()
                    .filter(|entity| {
                        world.get::<Body>(*entity).is_some() || world.get::<Frame>(*entity).is_some()
                    })
                    .collect()
            })
            .unwrap_or_default();

        match children.as_slice() {
            [] => issues.push(issue(joint, "joint has no Body or Frame child")),
            [_] => {}
            _ => issues.push(issue(joint, "joint has more than one Body or Frame child")),
        }

        if has_ancestor(world, joint, joint) {
            issues.push(issue(joint, "joint is part of a ChildOf cycle"));
        }

        let coordinates = world
            .get::<JointCoordinates>(joint)
            .map(|coordinates| coordinates.iter().collect::<Vec<_>>())
            .unwrap_or_default();
        let mut seen = HashSet::new();
        for coordinate in coordinates {
            if !seen.insert(coordinate) {
                issues.push(issue(joint, format!("coordinate {coordinate} appears more than once")));
            }
            if world.get::<Coordinate>(coordinate).is_none() {
                issues.push(issue(joint, format!("coordinate {coordinate} is missing Coordinate")));
            }
            match world.get::<CoordinateOf>(coordinate) {
                None => issues.push(issue(
                    joint,
                    format!("coordinate {coordinate} has no CoordinateOf relationship"),
                )),
                Some(owner) if owner.0 != joint => issues.push(issue(
                    joint,
                    format!("coordinate {coordinate} is owned by another joint"),
                )),
                Some(_) => {}
            }
        }
    }

    issues
}

fn issue(entity: Entity, message: impl Into<String>) -> KinematicIssue {
    KinematicIssue {
        entity,
        message: message.into(),
    }
}

fn has_ancestor(world: &World, entity: Entity, target: Entity) -> bool {
    let mut current = entity;
    let mut visited = HashSet::new();
    while let Some(parent) = world.get::<ChildOf>(current).map(ChildOf::parent) {
        if parent == target {
            return true;
        }
        if !visited.insert(parent) {
            return true;
        }
        current = parent;
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{CoordinateProperties, InitialConditions};

    #[test]
    fn attach_joint_constructs_a_valid_connection() {
        let mut world = World::new();
        let parent = world.spawn((Body, Transform::IDENTITY)).id();
        let joint = world.spawn((Joint, Transform::IDENTITY)).id();
        let child = world.spawn((Body, Transform::IDENTITY)).id();

        attach_joint(&mut world, parent, joint, child).unwrap();

        assert_eq!(world.get::<ChildOf>(joint).unwrap().parent(), parent);
        assert_eq!(world.get::<ChildOf>(child).unwrap().parent(), joint);
        assert!(validate_kinematic_hierarchy(&mut world).is_empty());
    }

    #[test]
    fn attach_joint_rejects_a_joint_parent() {
        let mut world = World::new();
        let parent = world.spawn((Joint, Transform::IDENTITY)).id();
        let joint = world.spawn((Joint, Transform::IDENTITY)).id();
        let child = world.spawn((Body, Transform::IDENTITY)).id();

        assert_eq!(
            attach_joint(&mut world, parent, joint, child),
            Err(KinematicError::ParentMustBeBodyOrFrame(parent))
        );
    }

    #[test]
    fn validator_reports_missing_coordinate_owner() {
        let mut world = World::new();
        let parent = world.spawn((Body, Transform::IDENTITY)).id();
        let coordinate = world
            .spawn((Coordinate, CoordinateProperties::default(), InitialConditions::default()))
            .id();
        let joint = world
            .spawn((
                Joint,
                JointCoordinates::new(vec![coordinate]),
                Transform::IDENTITY,
            ))
            .id();
        let child = world.spawn((Body, Transform::IDENTITY)).id();
        attach_joint(&mut world, parent, joint, child).unwrap();

        let issues = validate_kinematic_hierarchy(&mut world);
        assert!(issues
            .iter()
            .any(|issue| issue.message.contains("no CoordinateOf relationship")));
    }
}
