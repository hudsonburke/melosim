//! "Add Joint" popup dialog.

use bevy_inspector_egui::bevy_egui::egui;

use super::{AddJointPopup, AddJointRequest, JointType, PartCounter, PendingModelActions};
use crate::editor::selection::Selection;

/// Show the "Add Joint" popup window.
///
/// Returns `true` if the popup should be closed.
pub fn show(
    ctx: &egui::Context,
    popup: &mut AddJointPopup,
    counter: &mut PartCounter,
    pending_actions: &mut PendingModelActions,
    selection: &Selection,
) -> bool {
    let mut close = false;
    let parent = selection.primary();

    egui::Window::new("Add Joint")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Create a new joint.");

            ui.horizontal(|ui| {
                ui.label("Name:");
                if popup.name.is_empty() {
                    popup.name = format!("joint_{}", counter.0);
                }
                ui.text_edit_singleline(&mut popup.name);
            });

            // Joint type dropdown
            ui.horizontal(|ui| {
                ui.label("Type:");
                egui::ComboBox::from_id_salt("joint_type_selector")
                    .selected_text(popup.joint_type.as_str())
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut popup.joint_type, JointType::Weld, "Weld");
                        ui.selectable_value(&mut popup.joint_type, JointType::Hinge, "Hinge");
                        ui.selectable_value(&mut popup.joint_type, JointType::Ball, "Ball");
                    });
            });

            // Parent info
            if let Some(parent) = parent {
                ui.label(format!("Parent: {:?}", parent));
            } else {
                ui.label(egui::RichText::new("No parent selected (select a body first)").weak());
            }

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    close = true;
                }
                let can_create = parent.is_some();
                ui.add_enabled_ui(can_create, |ui| {
                    if ui.button("Create").clicked() {
                        let name = popup.name.clone();
                        let joint_type = popup.joint_type.clone();
                        counter.0 += 1;

                        if let Some(parent) = parent {
                            pending_actions.add_joints.push(AddJointRequest {
                                parent,
                                name,
                                joint_type,
                            });
                        }

                        popup.name.clear();
                        close = true;
                    }
                });
            });
        });

    close
}
