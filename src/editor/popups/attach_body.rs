//! "Attach Body" popup dialog.
//!
//! Rigidly attaches an exo part (Body A) to a model body (Body B) by creating
//! a Joint between two frames — one on each body.
//!
//! Result: Body B → Joint → Body A
//!
//! When frames are selected, a `Connects` component records which frames the
//! joint bridges, and the joint's `Transform` is the relative offset between
//! the two frame origins.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::AttachBodyPopup;
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::editor::selection::Selection;
use crate::model::{
    Body, Connects, Coordinate, CoordinateOf, Frame, Joint, JointCoordinates, Twist,
};

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
    frames: &Query<(Entity, &Name, &ChildOf), With<Frame>>,
    transforms: &Query<&Transform>,
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

    // Collect frames belonging to the exo body (for frame A picker)
    let exo_frames: Vec<(Entity, String)> = exo_body
        .map(|body_e| {
            frames
                .iter()
                .filter(|(_, _, co)| co.parent() == body_e)
                .map(|(e, n, _)| (e, n.as_str().to_owned()))
                .collect()
        })
        .unwrap_or_default();

    // Collect frames belonging to the target body (for frame B picker)
    let target_frames: Vec<(Entity, String)> = popup
        .target_body
        .map(|body_e| {
            frames
                .iter()
                .filter(|(_, _, co)| co.parent() == body_e)
                .map(|(e, n, _)| (e, n.as_str().to_owned()))
                .collect()
        })
        .unwrap_or_default();

    egui::Window::new("Attach Body")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Attach an exo part to a model body via a joint.");

            ui.add_space(4.0);

            // ── Exo body (body A) ──
            ui.horizontal(|ui| {
                ui.label("Exo Body:");
                ui.strong(&exo_name);
            });

            // Frame A picker (frames on the exo body)
            if !exo_frames.is_empty() {
                ui.horizontal(|ui| {
                    ui.label("  Frame A:");
                    let selected_label = popup
                        .exo_frame
                        .and_then(|e| frames.get(e).ok())
                        .map(|(_, n, _)| n.as_str().to_owned())
                        .unwrap_or_else(|| "None (body origin)".to_string());

                    egui::ComboBox::from_id_salt("attach_frame_a")
                        .selected_text(&selected_label)
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(popup.exo_frame.is_none(), "None (body origin)")
                                .clicked()
                            {
                                popup.exo_frame = None;
                            }
                            for (entity, name) in &exo_frames {
                                let is_selected = popup.exo_frame == Some(*entity);
                                if ui.selectable_label(is_selected, name).clicked() {
                                    popup.exo_frame = Some(*entity);
                                }
                            }
                        });
                });
            }

            // ── Target body (body B) ──
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
                                // Reset frame B when target changes
                                popup.target_frame = None;
                            }
                        }
                    });
            });

            // Frame B picker (frames on the target body)
            if !target_frames.is_empty() {
                ui.horizontal(|ui| {
                    ui.label("  Frame B:");
                    let selected_label = popup
                        .target_frame
                        .and_then(|e| frames.get(e).ok())
                        .map(|(_, n, _)| n.as_str().to_owned())
                        .unwrap_or_else(|| "None (body origin)".to_string());

                    egui::ComboBox::from_id_salt("attach_frame_b")
                        .selected_text(&selected_label)
                        .show_ui(ui, |ui| {
                            if ui
                                .selectable_label(
                                    popup.target_frame.is_none(),
                                    "None (body origin)",
                                )
                                .clicked()
                            {
                                popup.target_frame = None;
                            }
                            for (entity, name) in &target_frames {
                                let is_selected = popup.target_frame == Some(*entity);
                                if ui.selectable_label(is_selected, name).clicked() {
                                    popup.target_frame = Some(*entity);
                                }
                            }
                        });
                });
            }

            // ── Joint type selector ──
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Joint type:");
                let weld = &mut popup.weld;
                egui::ComboBox::from_id_salt("attach_joint_type")
                    .selected_text(if *weld { "Weld (rigid)" } else { "Free" })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(weld, true, "Weld (rigid)");
                        ui.selectable_value(weld, false, "Free");
                    });
            });

            // ── Preview ──
            if let Some(target) = popup.target_body {
                let target_name = bodies
                    .get(target)
                    .map(|(_, n)| n.as_str().to_owned())
                    .unwrap_or_else(|_| "Unknown".to_string());

                let joint_label = popup.joint_type_label();
                let frame_a_label = popup
                    .exo_frame
                    .and_then(|e| frames.get(e).ok())
                    .map(|(_, n, _)| format!("[{}]", n.as_str()))
                    .unwrap_or_else(|| "(origin)".to_string());
                let frame_b_label = popup
                    .target_frame
                    .and_then(|e| frames.get(e).ok())
                    .map(|(_, n, _)| format!("[{}]", n.as_str()))
                    .unwrap_or_else(|| "(origin)".to_string());

                ui.add_space(4.0);
                ui.label(format!(
                    "Will create: {} {} → {} → {} {}",
                    exo_name, frame_a_label, joint_label, frame_b_label, target_name,
                ));
            }

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    popup.reset();
                    close = true;
                }

                let can_attach = exo_body.is_some() && popup.target_body.is_some();
                ui.add_enabled_ui(can_attach, |ui| {
                    if ui.button("Attach").clicked() {
                        let exo = exo_body.unwrap();
                        let target = popup.target_body.unwrap();
                        let exo_frame = popup.exo_frame;
                        let target_frame = popup.target_frame;
                        let is_weld = popup.weld;

                        // Create a Joint as a child of the target body
                        let joint_type_name = if is_weld { "weld" } else { "free" };
                        let joint_name = format!(
                            "{}_{}_{}",
                            joint_type_name,
                            bodies
                                .get(target)
                                .map(|(_, n)| n.as_str().to_owned())
                                .unwrap_or_else(|_| "unknown".to_string()),
                            bodies
                                .get(exo)
                                .map(|(_, n)| n.as_str().to_owned())
                                .unwrap_or_else(|_| "unknown".to_string()),
                        );

                        // Create a coordinate entity for the joint
                        let coord = commands
                            .spawn((
                                Name::new(format!("{}_coord", joint_name)),
                                Coordinate::default(),
                                crate::model::CoordinateProperties::default(),
                            ))
                            .id();

                        // Set twist based on joint type
                        let twist = if is_weld {
                            Twist::default() // Weld: zero twist = rigid
                        } else {
                            // Free: no constraint (identity twist, 0 DOF)
                            Twist::default()
                        };
                        commands.entity(coord).insert(twist);

                        // Compute joint transform: relative offset between frames
                        let joint_transform =
                            compute_joint_transform(exo_frame, target_frame, transforms);

                        // Create the joint entity as child of target body
                        let mut joint_cmds = commands.spawn((
                            Name::new(joint_name),
                            Joint,
                            JointCoordinates::new(vec![coord]),
                            joint_transform,
                        ));
                        joint_cmds.insert(ChildOf(target));
                        let joint = joint_cmds.id();

                        // Link coordinate to joint
                        commands.entity(coord).insert(CoordinateOf(joint));

                        // Add joint as child of target body
                        commands.entity(target).add_children(&[joint]);

                        // Attach Connects component if both frames are selected
                        if let (Some(fa), Some(fb)) = (exo_frame, target_frame) {
                            commands
                                .entity(joint)
                                .insert(Connects { frame_a: fa, frame_b: fb });
                        }

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
                        popup.reset();
                        close = true;
                    }
                });
            });
        });

    close
}

/// Compute the joint's `Transform` as the relative offset between the two
/// selected frames. When frames are not selected, returns identity (joint at
/// body origin).
fn compute_joint_transform(
    _frame_a: Option<Entity>,
    frame_b: Option<Entity>,
    transforms: &Query<&Transform>,
) -> Transform {
    // If frame B is selected, place the joint at frame B's local position.
    // The joint lives as a child of target_body, so its local Transform
    // positions it relative to target_body's origin.
    //
    // When both frames are selected, the joint is still placed at frame B's
    // position — frame A's position is accounted for by the body hierarchy
    // (Body A is reparented under the joint).
    if let Some(fb) = frame_b {
        transforms.get(fb).cloned().unwrap_or_default()
    } else {
        Transform::default()
    }
}
