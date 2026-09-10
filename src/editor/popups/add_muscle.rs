//! "Add Muscle" popup dialog.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::{AddMusclePopup, PartCounter};
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::model::{HillTypeMuscleParams, Muscle};

/// Show the "Add Muscle" popup window.
///
/// Returns `true` if the popup should be closed.
pub fn show(
    ctx: &egui::Context,
    popup: &mut AddMusclePopup,
    counter: &mut PartCounter,
    commands: &mut Commands,
    events: &mut EditorEvents,
    bodies: &[(Entity, String)],
) -> bool {
    let mut close = false;

    egui::Window::new("Add Muscle")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Create a new muscle.");

            ui.horizontal(|ui| {
                ui.label("Name:");
                if popup.name.is_empty() {
                    popup.name = format!("muscle_{}", counter.0);
                }
                ui.text_edit_singleline(&mut popup.name);
            });

            // Parent body selector
            ui.horizontal(|ui| {
                ui.label("Parent Body:");
                let selected_label = if popup.parent_body.is_empty() {
                    "Select...".to_string()
                } else {
                    popup.parent_body.clone()
                };

                egui::ComboBox::from_id_salt("muscle_parent_selector")
                    .selected_text(&selected_label)
                    .show_ui(ui, |ui| {
                        for (entity, name) in bodies {
                            let label = format!("{} ({:?})", name, entity);
                            if ui.selectable_label(popup.parent_body == *name, &label).clicked() {
                                popup.parent_body = name.clone();
                            }
                        }
                    });
            });

            ui.separator();
            ui.label("Muscle Parameters:");

            ui.horizontal(|ui| {
                ui.label("Max Force (N):");
                ui.add(egui::DragValue::new(&mut popup.max_force).speed(10.0));
            });

            ui.horizontal(|ui| {
                ui.label("Fiber Length (m):");
                ui.add(egui::DragValue::new(&mut popup.fiber_length).speed(0.01));
            });

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    close = true;
                }
                if ui.button("Create").clicked() {
                    let name = popup.name.clone();
                    let max_force = popup.max_force;
                    let fiber_length = popup.fiber_length;
                    counter.0 += 1;

                    // Find parent body entity
                    let parent_entity = bodies
                        .iter()
                        .find(|(_, name)| name == &popup.parent_body)
                        .map(|(entity, _)| *entity);

                    let entity = commands
                        .spawn((
                            Name::new(name),
                            Muscle,
                            HillTypeMuscleParams {
                                max_isometric_force: max_force,
                                optimal_fiber_length: fiber_length,
                                tendon_slack_length: fiber_length * 0.25,
                                pennation_angle_at_optimal: 0.0,
                                minimum_activation: 0.01,
                                fiber_damping: 0.1,
                            },
                        ))
                        .id();

                    // Attach to parent body if specified
                    if let Some(parent) = parent_entity {
                        commands.entity(entity).insert(ChildOf(parent));
                        commands.entity(parent).add_children(&[entity]);
                    }

                    events.push(EditorEvent::ModelMutation {
                        kind: MutationKind::AddChild {
                            parent: parent_entity.unwrap_or(Entity::PLACEHOLDER),
                            child: entity,
                            marker: "Muscle",
                        },
                        entity,
                    });

                    popup.name.clear();
                    popup.parent_body.clear();
                    close = true;
                }
            });
        });

    close
}
