use bevy::prelude::*;

use crate::model::{evaluate_joint, CoordinateState, DrivesCoordinate, Function, Joint, Twist};

/// Forward kinematics: evaluate all joints and propagate transforms.
///
/// Runs in FixedUpdate. Reads coordinate values, computes PoE transforms.
pub fn forward_kinematics(
    joints: Query<&Children, With<Joint>>,
    twists: Query<(&Twist, &DrivesCoordinate, Option<&Function>)>,
    states: Query<&CoordinateState>,
) {
    for children in &joints {
        let _transform = evaluate_joint(children, &twists, &states);
        // TODO: store world-frame transforms for dynamics/muscle path computation
    }
}
