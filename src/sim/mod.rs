mod kinematics;

use bevy::prelude::*;

pub struct SimPlugin;

impl Plugin for SimPlugin {
    fn build(&self, app: &mut App) {
        app.add_systems(FixedUpdate, kinematics::forward_kinematics);
    }
}
