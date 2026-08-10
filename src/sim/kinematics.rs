use bevy::prelude::*;
use nalgebra::Isometry3;

use crate::model::{Coordinate, CoordinateState, DrivesCoordinate, Function, Joint, Twist};

/// Walk the kinematic tree from root to leaves, computing each body's
/// world-frame transform from its parent joint's PoE evaluation.
///
/// Triggers on any change to Joint, Twist, Function, or CoordinateState
/// so the simulation stays in sync with editor edits.
pub fn forward_kinematics(
    joints: Query<
        (Entity, &Children),
        (With<Joint>, Or<(Changed<Joint>, Changed<CoordinateState>)>),
    >,
    twists: Query<(&Twist, &DrivesCoordinate, Option<&Function>)>,
    states: Query<&CoordinateState>,
) {
    for (joint_entity, _children) in &joints {
        let _transform = evaluate_joint(joint_entity, &twists, &states);
        // TODO: propagate to child bodies/frames, write to rendering sync
        debug!("Joint {:?}", joint_entity);
    }
}

/// Evaluate the product-of-exponentials for a single joint.
fn evaluate_joint(
    joint: Entity,
    twists: &Query<(&Twist, &DrivesCoordinate, Option<&Function>)>,
    states: &Query<&CoordinateState>,
) -> Isometry3<f64> {
    // TODO: walk Children, composing Twist::exp for each axis
    Isometry3::identity()
}
