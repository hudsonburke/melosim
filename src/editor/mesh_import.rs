//! Layer 1: import a mesh as a new Body/part, normalized to glTF conventions.
//!
//! **Canonical mesh space = glTF conventions: Y-up, right-handed, meters.**
//! - **GLB / glTF** already conform — imported as-is (whole scene, all
//!   nodes/primitives).
//! - **STL / OBJ** (CAD) adapt to those conventions: rotated Z-up → Y-up and
//!   scaled to meters (unit picked on import).
//!
//! CAD-native files (STEP / IGES / Fusion F3D / SolidWorks SLDPRT) can't be read
//! directly — export as a mesh first (STL or GLB).
//!
//! Sources: file picker (Browse…, native `rfd`) + drag & drop. Each import
//! spawns a **new root Body** holding the mesh.

use std::path::{Path, PathBuf};

use bevy::ecs::message::MessageReader;
use bevy::prelude::*;
use bevy::window::FileDragAndDrop;
use bevy::world_serialization::{WorldAsset, WorldAssetRoot};
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use crate::model::{Body, InertialProperties, MeshSource};

/// Z-up (CAD/MuJoCo) → Y-up (glTF) rotation, matching the myoarm mesh nodes.
fn zup_to_yup() -> Quat {
    Quat::from_xyzw(-0.707107, 0.0, 0.0, 0.707107)
}

/// Source unit of the imported file (converted to meters on import).
#[derive(Clone, Copy, PartialEq, Eq, Default)]
pub enum ImportUnit {
    #[default]
    Mm,
    Cm,
    M,
    In,
    Ft,
}

impl ImportUnit {
    /// Scale factor to meters.
    fn to_m(self) -> f32 {
        match self {
            ImportUnit::Mm => 0.001,
            ImportUnit::Cm => 0.01,
            ImportUnit::M => 1.0,
            ImportUnit::In => 0.0254,
            ImportUnit::Ft => 0.3048,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            ImportUnit::Mm => "mm",
            ImportUnit::Cm => "cm",
            ImportUnit::M => "m",
            ImportUnit::In => "in",
            ImportUnit::Ft => "ft",
        }
    }
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

/// glTF is meters / Y-up; CAD mesh formats are assumed mm and Z-up.
pub fn default_unit(path: &Path) -> ImportUnit {
    let ext = path.extension().and_then(|e| e.to_str());
    if matches!(ext, Some("glb") | Some("gltf")) {
        ImportUnit::M
    } else {
        ImportUnit::Mm
    }
}

/// Spawn a new root `Body` holding the given mesh file, normalized to glTF
/// conventions (Y-up, meters): glTF as-is; STL/OBJ rotated Z-up→Y-up + scaled.
fn spawn_mesh_body(
    commands: &mut Commands,
    asset_server: &AssetServer,
    materials: &mut Assets<StandardMaterial>,
    mesh_assets: &mut Assets<Mesh>,
    asset_path: String,
    name: &str,
    unit: ImportUnit,
) {
    info!("spawn_mesh_body: asset_path={}", asset_path);
    let body_id = commands
        .spawn((Name::new(name.to_owned()), Body, InertialProperties::default()))
        .id();

    let ext = Path::new(&asset_path)
        .extension()
        .and_then(|e| e.to_str())
        .map(|s| s.to_ascii_lowercase());
    let is_gltf = matches!(ext.as_deref(), Some("glb") | Some("gltf"));

    // Normalize to glTF Y-up + meters.
    let transform = Transform::from_rotation(if is_gltf {
        Quat::IDENTITY
    } else {
        zup_to_yup()
    })
    .with_scale(Vec3::splat(unit.to_m()));

    let mesh_child = if is_gltf {
        // Whole glTF scene (`#Scene0`) via its WorldAsset — all nodes/primitives.
        let scene: Handle<WorldAsset> = asset_server.load(format!("{asset_path}#Scene0"));
        let mesh_name = Path::new(&asset_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("mesh")
            .to_string();
        commands
            .spawn((
                Name::new(format!("{name}_mesh")),
                WorldAssetRoot(scene),
                transform,
                MeshSource {
                    stl_path: std::path::PathBuf::from(format!("assets/{asset_path}")),
                    mesh_name,
                },
            ))
            .insert(ChildOf(body_id))
            .id()
    } else {
        // Use asset_server.load() — same approach as b412b0f that worked.
        let mesh: Handle<Mesh> = asset_server.load(&asset_path);
        let material = materials.add(StandardMaterial {
            base_color: Color::srgb(0.8, 0.7, 0.6),
            ..default()
        });
        let mesh_name = Path::new(&asset_path)
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("mesh")
            .to_string();
        commands
            .spawn((
                Name::new(format!("{name}_mesh")),
                Mesh3d(mesh),
                MeshMaterial3d(material),
                MeshSource {
                    stl_path: std::path::PathBuf::from(format!("assets/{asset_path}")),
                    mesh_name,
                },
            ))
            .insert(ChildOf(body_id))
            .id()
    };

    commands.entity(body_id).add_children(&[mesh_child]);
}

/// Copy a mesh file into `assets/imported/` and spawn a new Body for it.
pub fn import_file(
    commands: &mut Commands,
    asset_server: &AssetServer,
    materials: &mut Assets<StandardMaterial>,
    mesh_assets: &mut Assets<Mesh>,
    src: &Path,
    unit: ImportUnit,
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
    let safe_name = file_name.replace(' ', "_");
    let dest = dest_dir.join(&safe_name);
    if let Err(e) = std::fs::copy(src, &dest) {
        error!("mesh import: cannot copy {} → {}: {e}", src.display(), dest.display());
        return;
    }
    let asset_path = format!("imported/{}", safe_name);
    let stem = Path::new(file_name)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("part");
    spawn_mesh_body(commands, asset_server, materials, mesh_assets, asset_path, stem, unit);
}

/// Handle files dropped onto the window (uses the format's default unit).
pub fn import_dropped_mesh(
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut mesh_assets: ResMut<Assets<Mesh>>,
    mut events: MessageReader<FileDragAndDrop>,
) {
    for event in events.read() {
        let &FileDragAndDrop::DroppedFile { ref path_buf, .. } = event else {
            continue;
        };
        if is_mesh_ext(path_buf) {
            import_file(&mut commands, &asset_server, &mut materials, &mut mesh_assets, path_buf, default_unit(path_buf));
        }
    }
}

/// Small panel to import a mesh via the native file picker. Lets you choose the
/// source unit (STL/OBJ are assumed mm and Z-up; glTF is meters/Y-up).
/// Runs in the egui pass (separate system so `editor_ui` stays under 16 params).
///
/// **Note:** Import now also available via the toolbar Import menu. This panel
/// is retained for standalone access but is no longer toggled via Tools window.
pub fn mesh_import_ui(
    mut contexts: EguiContexts,
    mut pending: ResMut<super::PendingModelImport>,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut unit: Local<ImportUnit>,
    mut mesh_assets: ResMut<Assets<Mesh>>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    egui::Window::new("Import Mesh")
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -36.0))
        .collapsible(false)
        .show(ctx, |ui| {
            ui.label("Drag & drop a mesh, or pick a file.");
            ui.horizontal(|ui| {
                ui.label("Unit:");
                egui::ComboBox::from_id_salt("import_unit")
                    .selected_text(unit.label())
                    .show_ui(ui, |ui| {
                        for u in [
                            ImportUnit::Mm,
                            ImportUnit::Cm,
                            ImportUnit::M,
                            ImportUnit::In,
                            ImportUnit::Ft,
                        ] {
                            ui.selectable_value(&mut *unit, u, u.label());
                        }
                    });
                if ui.button("Browse…").clicked() {
                    if let Some(path) = pick_mesh_file() {
                        import_file(&mut commands, &asset_server, &mut materials, &mut mesh_assets, &path, *unit);
                    }
                }
            });
            ui.separator();
            if ui.button("Import Model (.xml)").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("MuJoCo model", &["xml"])
                    .pick_file()
                {
                    pending.0 = Some(path);
                }
            }
        });
}

/// Open the native file dialog filtered to mesh formats.
pub fn pick_mesh_file() -> Option<PathBuf> {
    rfd::FileDialog::new()
        .add_filter("Meshes", &["stl", "glb", "gltf", "obj"])
        .pick_file()
}
