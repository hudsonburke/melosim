//! "Add Body" popup dialog.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::{AddBodyPopup, PartCounter};
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::model::Body;

/// Show the "Add Body" popup window.
///
/// Returns `true` if the popup should be closed.
pub fn show(
    ctx: &egui::Context,
    popup: &mut AddBodyPopup,
    counter: &mut PartCounter,
    commands: &mut Commands,
    events: &mut EditorEvents,
) -> bool {
    let mut close = false;

    egui::Window::new("Add Body")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Create a new rigid body.");

            ui.horizontal(|ui| {
                ui.label("Name:");
                if popup.name.is_empty() {
                    popup.name = format!("body_{}", counter.0);
                }
                ui.text_edit_singleline(&mut popup.name);
            });

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    close = true;
                }
                if ui.button("Create").clicked() {
                    let name = popup.name.clone();
                    counter.0 += 1;

                    let entity = commands
                        .spawn((Name::new(name), Body))
                        .id();

                    events.push(EditorEvent::ModelMutation {
                        kind: MutationKind::AddChild {
                            parent: Entity::PLACEHOLDER,
                            child: entity,
                            marker: "Body",
                        },
                        entity,
                    });

                    popup.name.clear();
                    close = true;
                }
            });
        });

    close
}
