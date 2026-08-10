use bevy::{
    camera_controller::free_camera::{FreeCamera, FreeCameraPlugin},
    dev_tools::infinite_grid::{InfiniteGrid, InfiniteGridPlugin, InfiniteGridSettings},
    prelude::*,
};

pub struct EditorPlugin;

impl Plugin for EditorPlugin {
    fn build(&self, app: &mut App) {
        app.add_plugins((InfiniteGridPlugin, FreeCameraPlugin))
            .add_systems(Startup, setup_system);
    }
}

fn setup_system(mut commands: Commands) {
    commands.spawn((InfiniteGrid, InfiniteGridSettings::default()));

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-12.5, 5.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
        FreeCamera::default(),
    ));

    commands.spawn((
        DirectionalLight { ..default() },
        Transform::from_translation(Vec3::X * 15. + Vec3::Y * 20.).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
