use bevy::prelude::*;
use nalgebra::{Isometry3, Vector3};

use crate::model::{Coordinate, DrivesCoordinate, FixedFrame, Joint, Polynomial, Twist};

type Iso3 = Isometry3<f64>;

/// Walk the kinematic tree from root to leaves, computing each body's
/// world-frame transform from its parent joint's PoE evaluation.
///
/// Triggers on any change to Joint, Twist, Polynomial, or Coordinate
/// so the simulation stays in sync with editor edits.
pub fn forward_kinematics(
    joints: Query<
        (Entity, &Children),
        (With<Joint>, Or<(Changed<Joint>, Changed<Coordinate>)>),
    >,
    twists: Query<(&Twist, &DrivesCoordinate, Option<&Polynomial>)>,
    coordinates: Query<&Coordinate>,
    parents: Query<&Children>,
) {
    for (joint_entity, children) in &joints {
        let transform = evaluate_joint(joint_entity, &twists, &coordinates);

        // Propagate to child bodies/frames.
        // For now, just log — rendering sync will be in render::sync.
        debug!("Joint {:?}: {:?}", joint_entity, transform);
    }
}

/// Evaluate the product-of-exponentials for a single joint.
fn evaluate_joint(
    joint: Entity,
    twists: &Query<(&Twist, &DrivesCoordinate, Option<&Polynomial>)>,
    coordinates: &Query<&Coordinate>,
) -> Iso3 {
    // TODO: walk Children, composing Twist::exp for each axis
    Iso3::identity()
}
