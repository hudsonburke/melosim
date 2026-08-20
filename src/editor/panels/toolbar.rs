use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use crate::editor::models::{ModelRegistry, SelectedModel};
use crate::editor::selection::Selection;
use crate::render::RenderSettings;

/// Render the top toolbar content into the given `Ui`.
pub fn show(
    ui: &mut egui::Ui,
    selection: &Selection,
    registry: &ModelRegistry,
    selected_model: &mut SelectedModel,
    settings: &mut RenderSettings,
    names: &Query<&mut Name>,
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
                    ui.close_menu();
                }
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
