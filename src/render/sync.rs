use nalgebra::Isometry3;

use crate::model::to_bevy_transform;

type Iso3 = Isometry3<f64>;

/// After simulation runs in FixedUpdate, write results to Bevy's
/// Transform components so the renderer can display them.
///
/// Runs in PostUpdate — after all simulation and editor systems,
/// before the frame is rendered.
pub fn sync_simulation_to_transform(
    // TODO: query bodies/joints that have simulation-computed world transforms
    // and write them to Bevy Transform components via to_bevy_transform
) {
    // Stub — will be populated when kinematics produces world-frame isometries
}
