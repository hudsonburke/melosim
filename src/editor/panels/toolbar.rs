use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use crate::editor::mesh_import;
use crate::editor::models::{ModelRegistry, SelectedModel};
use crate::editor::popups::ActivePopup;
use crate::editor::selection::Selection;
use crate::editor::PendingMujocoExport;
use crate::render::RenderSettings;

/// Render the top toolbar content into the given `Ui`.
pub fn show(
    ui: &mut egui::Ui,
    selection: &Selection,
    registry: &ModelRegistry,
    selected_model: &mut SelectedModel,
    settings: &mut RenderSettings,
    names: &Query<&mut Name>,
    active_popup: &mut ActivePopup,
    pending_model_import: &mut crate::editor::PendingModelImport,
    pending_mesh_import: &mut crate::editor::PendingMeshImport,
    pending_export: &mut PendingMujocoExport,
) {
    ui.horizontal(|ui| {
        ui.heading("melosim");
        ui.separator();
        match selection.primary() {
            Some(e) => {
                let name = names
                    .get(e)
                    .map(|n| n.as_str().to_owned())
                    .unwrap_or_else(|_| format!("{:?}", e));
                ui.label(format!("Selected: {}", name));
            }
            None => {
                ui.label("Nothing selected");
            }
        }
        ui.separator();
        // Load a model from the registry (despawns the previous one).
        ui.menu_button("Model", |ui| {
            for (i, def) in registry.0.iter().enumerate() {
                if ui.button(def.name).clicked() {
                    selected_model.0 = Some(i);
                    ui.close();
                }
            }
        });
        ui.separator();
        // Import menu: mesh files and MuJoCo models
        ui.menu_button("Import", |ui| {
            if ui.button("Mesh (STL/OBJ/gltf)…").clicked() {
                if let Some(path) = mesh_import::pick_mesh_file() {
                    pending_mesh_import.0 = Some(path);
                    *active_popup = ActivePopup::ImportMeshUnit;
                }
                ui.close();
            }
            if ui.button("Model (.xml)…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("MuJoCo model", &["xml"])
                    .pick_file()
                {
                    pending_model_import.0 = Some(path);
                }
                ui.close();
            }
        });
        ui.separator();
        // Export menu
        ui.menu_button("Export", |ui| {
            if ui.button("MuJoCo (.xml)…").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("MuJoCo model", &["xml"])
                    .set_file_name("export.xml")
                    .save_file()
                {
                    pending_export.0 = Some(path);
                }
                ui.close();
            }
        });
        ui.separator();
        // Create menu for adding new components
        ui.menu_button("Create", |ui| {
            if ui.button("Body").clicked() {
                *active_popup = ActivePopup::AddBody;
                ui.close();
            }
            if ui.button("Joint").clicked() {
                *active_popup = ActivePopup::AddJoint;
                ui.close();
            }
            if ui.button("Muscle").clicked() {
                *active_popup = ActivePopup::AddMuscle;
                ui.close();
            }
            if ui.button("Site").clicked() {
                *active_popup = ActivePopup::AddSite;
                ui.close();
            }
            if ui.button("Frame").clicked() {
                *active_popup = ActivePopup::AddFrame;
                ui.close();
            }
            ui.separator();
            if ui.button("Attach Body…").clicked() {
                *active_popup = ActivePopup::AttachBody;
                ui.close();
            }
        });
        ui.separator();
        ui.menu_button("View", |ui| {
            ui.checkbox(&mut settings.meshes, "Meshes");
            ui.checkbox(&mut settings.bodies, "Bodies");
            ui.checkbox(&mut settings.frames, "Frames");
            ui.checkbox(&mut settings.sites, "Sites");
            ui.checkbox(&mut settings.muscles, "Muscles");
            ui.checkbox(&mut settings.joint_axes, "Joint Axes");
        });
        ui.separator();
    });
}
