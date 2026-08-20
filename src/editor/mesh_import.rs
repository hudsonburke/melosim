//! Layer 1: import a mesh as a new Body/part.
//!
//! CAD-native files (STEP / IGES / Fusion F3D / SolidWorks SLDPRT) can't be read
//! directly — export as a mesh first: **STL** (universal) or **GLB/glTF**
//! (preferred; keeps material, Bevy's native mesh format).
//!
//! Two sources, both spawning a **new root Body** holding the mesh:
//! - **File picker** (Browse…) — native `rfd` GTK dialog.
//! - **Drag & drop** a mesh file onto the window.
//!
//! The file is copied into `assets/imported/` (so `AssetServer` can load it) and
//! the CAD Z-up → Bevy Y-up rotation is baked in.

use std::path::{Path, PathBuf};

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use bevy::window::FileDragAndDrop;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use crate::model::{Body, InertialProperties};

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
fn spawn_mesh_body(
    commands: &mut Commands,
    asset_server: &AssetServer,
    materials: &mut Assets<StandardMaterial>,
    asset_path: String,
    name: &str,
) {
    // glTF/GLB files aren't a bare `Mesh` asset — a mesh is addressed by a
    // `#MeshN/PrimitiveM` label (e.g. glTF "gltf/part.glb#Mesh0/Primitive0").
    // STL/OBJ load directly as a `Mesh`, no label needed.
    let mut load_path = asset_path;
    if !load_path.contains('#') {
        let ext = Path::new(&load_path)
            .extension()
            .and_then(|e| e.to_str())
            .map(|s| s.to_ascii_lowercase());
        if matches!(ext.as_deref(), Some("glb") | Some("gltf")) {
            load_path.push_str("#Mesh0/Primitive0");
        }
    }

    let mesh: Handle<Mesh> = asset_server.load(load_path);
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

/// Copy a mesh file into `assets/imported/` and spawn a new Body for it.
fn import_file(
    commands: &mut Commands,
    asset_server: &AssetServer,
    materials: &mut Assets<StandardMaterial>,
    src: &Path,
) {
    if !is_mesh_ext(src) {
        error!("mesh import: unsupported extension for {}", src.display());
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
    let dest = dest_dir.join(file_name);
    if let Err(e) = std::fs::copy(src, &dest) {
        error!("mesh import: cannot copy {} → {}: {e}", src.display(), dest.display());
        return;
    }
    let asset_path = format!("imported/{}", file_name);
    let stem = Path::new(file_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("part");
    spawn_mesh_body(commands, asset_server, materials, asset_path, stem);
    info!("imported mesh: {}", dest.display());
}

/// Handle files dropped onto the window.
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
        if is_mesh_ext(path_buf) {
            import_file(&mut commands, &asset_server, &mut materials, path_buf);
        }
    }
}

/// Small panel to import a mesh via the native file picker (or drag & drop).
/// Runs in the egui pass (separate system so `editor_ui` stays under Bevy's
/// 16-parameter limit).
pub fn mesh_import_ui(
    mut contexts: EguiContexts,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    egui::Window::new("Import Mesh")
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -36.0))
        .collapsible(false)
        .show(ctx, |ui| {
            ui.label("Drag & drop a mesh, or pick a file:");
            if ui.button("Browse…").clicked() {
                if let Some(path) = pick_mesh_file() {
                    import_file(&mut commands, &asset_server, &mut materials, &path);
                }
            }
        });
}

/// Open the native file dialog filtered to mesh formats.
fn pick_mesh_file() -> Option<PathBuf> {
    rfd::FileDialog::new()
        .add_filter("Meshes", &["stl", "glb", "gltf", "obj"])
        .pick_file()
}
