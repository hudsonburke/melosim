//! "Add Joint" popup dialog.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::{AddJointPopup, JointType, PartCounter};
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::editor::selection::Selection;
use crate::model::{Coordinate, CoordinateProperties, CoordinateOf, Joint, JointCoordinates, Twist};

/// Show the "Add Joint" popup window.
///
/// Returns `true` if the popup should be closed.
pub fn show(
    ctx: &egui::Context,
    popup: &mut AddJointPopup,
    counter: &mut PartCounter,
    commands: &mut Commands,
    events: &mut EditorEvents,
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
                            // Create a coordinate entity for the joint
                            let coord = commands
                                .spawn((
                                    Name::new(format!("{}_coord", name)),
                                    Coordinate::default(),
                                    CoordinateProperties::default(),
                                ))
                                .id();

                            // Create the joint entity
                            let joint = commands
                                .spawn((
                                    Name::new(name),
                                    Joint,
                                    JointCoordinates::new(vec![coord]),
                                    Transform::default(),
                                ))
                                .insert(ChildOf(parent))
                                .id();

                            // Link coordinate to joint
                            commands.entity(coord).insert(CoordinateOf(joint));

                            // Link coordinate to parent body
                            commands.entity(parent).add_children(&[joint]);

                            // Set up twist based on joint type
                            let twist = match joint_type {
                                JointType::Weld => Twist::default(),
                                JointType::Hinge => Twist {
                                    angular: nalgebra::Vector3::new(0.0, 0.0, 1.0),
                                    linear: nalgebra::Vector3::zeros(),
                                },
                                JointType::Ball => Twist {
                                    angular: nalgebra::Vector3::new(1.0, 0.0, 0.0),
                                    linear: nalgebra::Vector3::zeros(),
                                },
                            };
                            commands.entity(coord).insert(twist);

                            events.push(EditorEvent::ModelMutation {
                                kind: MutationKind::AddChild {
                                    parent,
                                    child: joint,
                                    marker: "Joint",
                                },
                                entity: joint,
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
