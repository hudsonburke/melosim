//! Import a GLTF/GLB scene as a new model body.
//!
//! Standalone visual scenes use Bevy's `AssetServer` and `WorldAssetRoot`.
//! MuJoCo model import separately renders compiled meshes directly and retains
//! STL/OBJ source provenance for physical geometry export.

use std::path::{Path, PathBuf};

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use bevy::window::FileDragAndDrop;
use bevy::world_serialization::{WorldAsset, WorldAssetRoot};

use crate::model::{Body, InertialProperties};

fn is_gltf_ext(path: &Path) -> bool {
    matches!(
        path.extension()
            .and_then(|e| e.to_str())
            .map(|s| s.to_ascii_lowercase())
            .as_deref(),
        Some("glb") | Some("gltf")
    )
}

/// Spawn a new root `Body` holding a GLTF scene.
fn spawn_gltf_body(
    commands: &mut Commands,
    asset_server: &AssetServer,
    asset_path: String,
    name: &str,
) {
    info!("spawn_gltf_body: asset_path={asset_path}");

    let body = commands
        .spawn((
            Name::new(name.to_owned()),
            Body,
            InertialProperties::default(),
        ))
        .id();

    // A GLTF scene contains its own meshes, materials, and node hierarchy.
    let scene: Handle<WorldAsset> = asset_server.load(format!("{asset_path}#Scene0"));
    let scene_root = commands
        .spawn((
            Name::new(format!("{name}_mesh")),
            WorldAssetRoot(scene),
            Transform::IDENTITY,
        ))
        .id();

    commands.entity(body).add_child(scene_root);
}

/// Copy a GLTF/GLB file into `assets/imported/` and spawn a new `Body`.
pub fn import_file(commands: &mut Commands, asset_server: &AssetServer, src: &Path) {
    if !is_gltf_ext(src) {
        error!(
            "mesh import: only GLTF/GLB is supported; refusing {}",
            src.display()
        );
        return;
    }

    let Some(file_name) = src.file_name().and_then(|s| s.to_str()) else {
        return;
    };

    let dest_dir = Path::new("assets").join("imported");
    if let Err(e) = std::fs::create_dir_all(&dest_dir) {
        error!("mesh import: cannot create {}: {e}", dest_dir.display());
        return;
    }

    let safe_name = file_name.replace(' ', "_");
    let dest = dest_dir.join(&safe_name);
    if let Err(e) = std::fs::copy(src, &dest) {
        error!(
            "mesh import: cannot copy {} → {}: {e}",
            src.display(),
            dest.display()
        );
        return;
    }

    let asset_path = format!("imported/{safe_name}");
    let stem = Path::new(file_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("part");
    spawn_gltf_body(commands, asset_server, asset_path, stem);
}

/// Handle GLTF/GLB files dropped onto the window.
pub fn import_dropped_mesh(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut events: MessageReader<FileDragAndDrop>,
) {
    for event in events.read() {
        let &FileDragAndDrop::DroppedFile { ref path_buf, .. } = event else {
            continue;
        };
        if is_gltf_ext(path_buf) {
            import_file(&mut commands, &asset_server, path_buf);
        }
    }
}

/// Open the native file dialog filtered to GLTF/GLB files.
pub fn pick_mesh_file() -> Option<PathBuf> {
    rfd::FileDialog::new()
        .add_filter("GLTF scenes", &["glb", "gltf"])
        .pick_file()
}
