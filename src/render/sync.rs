use bevy::prelude::*;

use crate::model::{
    evaluate_joint, to_bevy_transform, CoordinateState, DrivesCoordinate, FixedFrame, Function,
    Joint, Twist,
};

/// Copy fixed-frame offsets (sim f64) into render Transforms.
///
/// Frames never move relative to their parent, so this only runs when a
/// `FixedFrame` is added.
pub fn sync_fixed_frames(mut frames: Query<(&FixedFrame, &mut Transform), Added<FixedFrame>>) {
    for (frame, mut transform) in &mut frames {
        *transform = to_bevy_transform(&frame.0);
    }
}

/// Sync system: evaluate each joint's PoE transform (sim f64) and write it
/// to the joint entity's own Transform. Frames and bodies nested below the
/// joint follow via Bevy's transform propagation.
///
/// Runs in PostUpdate, before `TransformSystems::Propagate`.
pub fn sync_kinematics(
    mut joints: Query<(&Children, &mut Transform), With<Joint>>,
    twists: Query<(&Twist, &DrivesCoordinate, Option<&Function>)>,
    states: Query<&CoordinateState>,
) {
    for (children, mut transform) in &mut joints {
        *transform = to_bevy_transform(&evaluate_joint(children, &twists, &states));
    }
}
