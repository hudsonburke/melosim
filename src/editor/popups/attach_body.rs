//! "Attach Body" popup dialog.
//!
//! Rigidly attaches an exo part (Body A) to a model body (Body B) by creating
//! a Weld Joint as a child of Body B, then reparenting Body A under that joint.
//!
//! Result: Body B → Joint → Body A

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::AttachBodyPopup;
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::editor::selection::Selection;
use crate::model::{Body, Coordinate, CoordinateOf, Joint, JointCoordinates, Twist};

/// Show the "Attach Body" popup window.
///
/// Returns `true` if the popup should be closed.
pub fn show(
    ctx: &egui::Context,
    popup: &mut AttachBodyPopup,
    commands: &mut Commands,
    events: &mut EditorEvents,
    selection: &Selection,
    bodies: &Query<(Entity, &Name), With<Body>>,
) -> bool {
    let mut close = false;

    // Capture the selected body when the popup opens
    let selected = selection.primary();
    if popup.exo_body.is_none() {
        popup.exo_body = selected;
    }

    let exo_body = popup.exo_body;
    let exo_name = exo_body
        .and_then(|e| bodies.get(e).ok())
        .map(|(_, n)| n.as_str().to_owned())
        .unwrap_or_else(|| "(none)".to_string());

    // Collect all body entities except the exo body
    let other_bodies: Vec<(Entity, String)> = bodies
        .iter()
        .filter(|(e, _)| Some(*e) != exo_body)
        .map(|(e, n)| (e, n.as_str().to_owned()))
        .collect();

    egui::Window::new("Attach Body")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Rigidly attach an exo part to a model body.");

            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Exo Body:");
                ui.strong(&exo_name);
            });

            // Target body dropdown
            ui.horizontal(|ui| {
                ui.label("Attach to:");

                let selected_label = popup
                    .target_body
                    .and_then(|e| bodies.get(e).ok())
                    .map(|(_, n)| n.as_str().to_owned())
                    .unwrap_or_else(|| "Select...".to_string());

                egui::ComboBox::from_id_salt("attach_body_target")
                    .selected_text(&selected_label)
                    .show_ui(ui, |ui| {
                        for (entity, name) in &other_bodies {
                            let label = format!("{}", name);
                            let is_selected = popup.target_body == Some(*entity);
                            if ui.selectable_label(is_selected, &label).clicked() {
                                popup.target_body = Some(*entity);
                            }
                        }
                    });
            });

            // Show preview of what will be created
            if let Some(target) = popup.target_body {
                let target_name = bodies
                    .get(target)
                    .map(|(_, n)| n.as_str().to_owned())
                    .unwrap_or_else(|_| "Unknown".to_string());
                ui.add_space(4.0);
                ui.label(format!(
                    "Will create: {} → Weld Joint → {}",
                    target_name, exo_name
                ));
            }

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    popup.exo_body = None;
                    popup.target_body = None;
                    close = true;
                }

                let can_attach = exo_body.is_some() && popup.target_body.is_some();
                ui.add_enabled_ui(can_attach, |ui| {
                    if ui.button("Attach").clicked() {
                        let exo = exo_body.unwrap();
                        let target = popup.target_body.unwrap();

                        // Create a Weld Joint as a child of the target body
                        let joint_name = format!(
                            "weld_{}_{}",
                            bodies
                                .get(target)
                                .map(|(_, n)| n.as_str().to_owned())
                                .unwrap_or_else(|_| "unknown".to_string()),
                            bodies
                                .get(exo)
                                .map(|(_, n)| n.as_str().to_owned())
                                .unwrap_or_else(|_| "unknown".to_string()),
                        );

                        // Create a coordinate for the weld (identity twist = no DOF)
                        let coord = commands
                            .spawn((
                                Name::new(format!("{}_coord", joint_name)),
                                Coordinate::default(),
                                crate::model::CoordinateProperties::default(),
                            ))
                            .id();

                        // Weld twist: zero angular + zero linear = rigid
                        let twist = Twist::default();
                        commands.entity(coord).insert(twist);

                        // Create the joint entity as child of target body
                        let joint = commands
                            .spawn((
                                Name::new(joint_name),
                                Joint,
                                JointCoordinates::new(vec![coord]),
                                Transform::default(),
                            ))
                            .insert(ChildOf(target))
                            .id();

                        // Link coordinate to joint
                        commands.entity(coord).insert(CoordinateOf(joint));

                        // Add joint as child of target body
                        commands.entity(target).add_children(&[joint]);

                        // Reparent the exo body under the joint
                        commands.entity(exo).insert(ChildOf(joint));
                        commands.entity(joint).add_children(&[exo]);

                        // Emit events for the joint creation
                        events.push(EditorEvent::ModelMutation {
                            kind: MutationKind::AddChild {
                                parent: target,
                                child: joint,
                                marker: "Joint",
                            },
                            entity: joint,
                        });

                        // Emit event for the reparent
                        events.push(EditorEvent::ModelMutation {
                            kind: MutationKind::RemoveChild {
                                parent: Entity::PLACEHOLDER, // was root
                                child: exo,
                            },
                            entity: exo,
                        });

                        events.push(EditorEvent::ModelMutation {
                            kind: MutationKind::AddChild {
                                parent: joint,
                                child: exo,
                                marker: "Body",
                            },
                            entity: exo,
                        });

                        // Reset popup state
                        popup.exo_body = None;
                        popup.target_body = None;
                        close = true;
                    }
                });
            });
        });

    close
}
