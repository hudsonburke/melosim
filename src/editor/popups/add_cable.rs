use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::{AddCablePopup, PartCounter};
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::model::{Cable, CableParameters, PathEntities};

pub fn show(
    ctx: &egui::Context,
    popup: &mut AddCablePopup,
    counter: &mut PartCounter,
    commands: &mut Commands,
    events: &mut EditorEvents,
) -> bool {
    let mut close = false;

    egui::Window::new("Add Cable")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Create an empty cable, then edit its ordered path in the inspector.");
            ui.horizontal(|ui| {
                ui.label("Name:");
                if popup.name.is_empty() {
                    popup.name = format!("cable_{}", counter.0);
                }
                ui.text_edit_singleline(&mut popup.name);
            });
            ui.horizontal(|ui| {
                ui.label("Max tension (N):");
                ui.add(egui::DragValue::new(&mut popup.max_tension).speed(10.0));
            });
            ui.horizontal(|ui| {
                ui.label("Actuator force (N):");
                ui.add(egui::DragValue::new(&mut popup.actuator_force).speed(10.0));
            });

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    close = true;
                }
                if ui.button("Create").clicked() {
                    let name = if popup.name.is_empty() {
                        format!("cable_{}", counter.0)
                    } else {
                        popup.name.clone()
                    };
                    counter.0 += 1;
                    let cable = commands
                        .spawn((
                            Name::new(name.clone()),
                            Cable,
                            PathEntities::default(),
                            CableParameters {
                                max_tension: popup.max_tension,
                                actuator_force: popup.actuator_force,
                                ..default()
                            },
                        ))
                        .id();
                    events.push(EditorEvent::ModelMutation {
                        kind: MutationKind::AddChild {
                            parent: Entity::PLACEHOLDER,
                            child: cable,
                            marker: "Cable",
                        },
                        entity: cable,
                    });
                    popup.name.clear();
                    close = true;
                }
            });
        });

    close
}
