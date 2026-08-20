//! Layer 1: import a mesh as a new Body/part.
//!
//! CAD-native files (STEP / IGES / Fusion F3D / SolidWorks SLDPRT) can't be read
//! directly — export as a mesh first: **STL** (universal) or **GLB/glTF**
//! (preferred; keeps material, Bevy's native mesh format).
//!
//! Import sources:
//! - **Drag & drop** a mesh file onto the window (copied into `assets/imported/`).
//! - **Type an asset path** in the Import Mesh panel (relative to `assets/`).
//!
//! Each import spawns a **new root Body** holding the mesh (a part you can then
//! place and joint in Layer 2), with the CAD Z-up → Bevy Y-up rotation baked in.

use std::path::Path;

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use bevy::window::FileDragAndDrop;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use crate::model::{Body, InertialProperties};

/// Import UI state: the typed asset-path fallback.
#[derive(Resource, Default)]
pub struct MeshImport {
    pub path: String,
}

fn is_mesh_ext(path: &Path) -> bool {
    matches!(
        path.extension()
            .and_then(|e| e.to_str())
            .map(|s| s.to_ascii_lowercase())
            .as_deref(),
        Some("glb") | Some("gltf") | Some("obj") | Some("stl")
    )
}

/// Spawn a new root `Body` with the given mesh as a Z-up→Y-up-rotated child.
/// `asset_path` is relative to the `assets/` root (e.g. `imported/part.stl`).
pub fn spawn_mesh_body(
    commands: &mut Commands,
    asset_server: &AssetServer,
    materials: &mut Assets<StandardMaterial>,
    asset_path: String,
    name: &str,
) {
    let mesh: Handle<Mesh> = asset_server.load(asset_path);
    let material = materials.add(StandardMaterial {
        base_color: Color::srgb(0.8, 0.7, 0.6),
        ..default()
    });

    let body_id = commands
        .spawn((Name::new(name.to_owned()), Body, InertialProperties::default()))
        .id();

    let mesh_child = commands
        .spawn((
            Name::new(format!("{name}_mesh")),
            Mesh3d(mesh),
            MeshMaterial3d(material),
            Transform::from_rotation(Quat::from_xyzw(-0.707107, 0.0, 0.0, 0.707107)),
        ))
        .insert(ChildOf(body_id))
        .id();
    commands.entity(body_id).add_children(&[mesh_child]);
}

/// Handle files dropped onto the window: copy mesh files into `assets/imported/`
/// and spawn a new Body for each.
pub fn import_dropped_mesh(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut events: MessageReader<FileDragAndDrop>,
) {
    for event in events.read() {
        let &FileDragAndDrop::DroppedFile { ref path_buf, .. } = event else {
            continue;
        };
        if !is_mesh_ext(path_buf) {
            continue;
        }
        let Some(file_name) = path_buf.file_name().and_then(|s| s.to_str()) else {
            continue;
        };
        let dest_dir = Path::new("assets").join("imported");
        if let Err(e) = std::fs::create_dir_all(&dest_dir) {
            error!("mesh import: cannot create {}: {e}", dest_dir.display());
            continue;
        }
        let dest = dest_dir.join(file_name);
        if let Err(e) = std::fs::copy(path_buf, &dest) {
            error!("mesh import: cannot copy {} → {}: {e}", path_buf.display(), dest.display());
            continue;
        }
        let asset_path = format!("imported/{}", file_name);
        let stem = Path::new(file_name)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("part");
        spawn_mesh_body(&mut commands, &asset_server, &mut materials, asset_path, stem);
        info!("imported mesh: {}", dest.display());
    }
}

/// Small panel to import a mesh by typed asset path (fallback to drag & drop).
/// Runs in the egui pass (separate system so `editor_ui` stays under Bevy's
/// 16-parameter limit).
pub fn mesh_import_ui(
    mut contexts: EguiContexts,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut import: ResMut<MeshImport>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    egui::Window::new("Import Mesh")
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -36.0))
        .default_width(380.0)
        .collapsible(false)
        .show(ctx, |ui| {
            ui.label("Drag & drop a mesh file, or enter an asset path (.glb/.obj/.stl):");
            let imported = ui
                .horizontal(|ui| {
                    ui.text_edit_singleline(&mut import.path);
                    ui.button("Import").clicked()
                })
                .inner;
            if imported {
                let asset_path = import.path.trim().to_owned();
                if !asset_path.is_empty() {
                    let stem = Path::new(&asset_path)
                        .file_stem()
                        .and_then(|s| s.to_str())
                        .unwrap_or("part")
                        .to_owned();
                    spawn_mesh_body(&mut commands, &asset_server, &mut materials, asset_path, &stem);
                    import.path.clear();
                }
            }
        });
}
