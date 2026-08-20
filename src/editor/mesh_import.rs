//! Layer 1: import a mesh onto the selected Body from an asset path.
//!
//! CAD-native files (STEP / IGES / Fusion F3D / SolidWorks SLDPRT) can't be read
//! directly — export them as a mesh first: **STL** (universal) or **GLB/glTF**
//! (preferred; keeps material, Bevy's native mesh format). On import we bake the
//! CAD Z-up → Bevy Y-up rotation (same convention as the myoarm meshes) and treat
//! 1 Bevy unit = 1 metre (CAD parts are commonly mm — scale if needed).

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::selection::Selection;

/// Mesh-import UI state (the asset path to attach onto the selected Body).
#[derive(Resource, Default)]
pub struct MeshImport {
    pub path: String,
}

/// Small panel: type an asset path (.glb / .obj / .stl) and attach it as a mesh
/// child of the selected Body. Runs in the egui pass (separate system so
/// `editor_ui` stays under Bevy's 16-parameter limit).
pub fn mesh_import_ui(
    mut contexts: EguiContexts,
    mut commands: Commands,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    selection: Res<Selection>,
    mut import: ResMut<MeshImport>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    egui::Window::new("Import Mesh")
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -36.0))
        .default_width(380.0)
        .collapsible(false)
        .show(ctx, |ui| {
            let Some(body) = selection.primary() else {
                ui.label("Select a Body to add a mesh to first.");
                return;
            };

            ui.label("Mesh asset path (.glb / .obj / .stl):");
            let imported = ui
                .horizontal(|ui| {
                    ui.text_edit_singleline(&mut import.path);
                    ui.button("Import").clicked()
                })
                .inner;

            if imported {
                let path = import.path.trim().to_owned();
                if !path.is_empty() {
                    let mesh: Handle<Mesh> = asset_server.load(path);
                    let material = materials.add(StandardMaterial {
                        base_color: Color::srgb(0.8, 0.7, 0.6),
                        ..default()
                    });
                    // CAD/Z-up -> Bevy Y-up, matching the myoarm mesh convention.
                    let child = commands
                        .spawn((
                            Name::new("imported_mesh"),
                            Mesh3d(mesh),
                            MeshMaterial3d(material),
                            Transform::from_rotation(Quat::from_xyzw(
                                -0.707107, 0.0, 0.0, 0.707107,
                            )),
                        ))
                        .insert(ChildOf(body))
                        .id();
                    commands.entity(body).add_children(&[child]);
                    import.path.clear();
                }
            }
        });
}
