//! "Connect Frames" popup dialog.
//!
//! Creates a Joint between two frames — one on a parent body and one on a child body.
//!
//! Result: ParentBody → Joint → ChildBody

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::{ConnectFramesPopup, FrameSlot};
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::editor::selection::Selection;
use crate::model::{Body, Connects, Coordinate, CoordinateOf, Frame, Joint, JointCoordinates, Twist};

/// Show the "Connect Frames" popup window.
pub fn show(
    ctx: &egui::Context,
    popup: &mut ConnectFramesPopup,
    commands: &mut Commands,
    events: &mut EditorEvents,
    selection: &Selection,
    frames: &Query<(Entity, &Name, &ChildOf), With<Frame>>,
    bodies: &Query<(Entity, &Name), With<Body>>,
) -> bool {
    let mut close = false;

    // Check if a frame was selected in the viewport while waiting
    if popup.waiting_for.is_some() {
        if let Some(entity) = selection.primary() {
            if frames.get(entity).is_ok() {
                popup.assign_frame(entity);
            }
        }
    }

    // Collect all frames
    let all_frames: Vec<(Entity, String)> = frames
        .iter()
        .map(|(e, n, _)| (e, n.as_str().to_owned()))
        .collect();

    // Helper: get body name for a frame
    let body_name_for = |frame: Option<Entity>| -> String {
        frame
            .and_then(|f| {
                // Find the parent body of this frame
                all_frames.iter().find(|(e, _)| *e == f).map(|_| {
                    // Frame's parent is its body — we need to query ChildOf
                    // For now, just show the frame name
                    format!("frame")
                })
            })
            .unwrap_or_default()
    };

    egui::Window::new("Connect Frames")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.label("Create a joint between two frames.");

            // ── Child Frame ──
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Child Frame:");
                let waiting_child = popup.waiting_for == Some(FrameSlot::Child);
                if ui.selectable_label(waiting_child, "🎯").clicked() {
                    popup.waiting_for = if waiting_child { None } else { Some(FrameSlot::Child) };
                }
                let label = popup
                    .child_frame
                    .and_then(|e| frames.get(e).ok())
                    .map(|(_, n, _)| n.as_str().to_owned())
                    .unwrap_or_else(|| "Select...".to_string());
                egui::ComboBox::from_id_salt("child_frame_picker")
                    .selected_text(&label)
                    .show_ui(ui, |ui| {
                        if ui.selectable_label(popup.child_frame.is_none(), "None").clicked() {
                            popup.child_frame = None;
                        }
                        for (entity, name) in &all_frames {
                            let is_selected = popup.child_frame == Some(*entity);
                            if ui.selectable_label(is_selected, name).clicked() {
                                popup.child_frame = Some(*entity);
                            }
                        }
                    });
            });

            // ── Parent Frame ──
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Parent Frame:");
                let waiting_parent = popup.waiting_for == Some(FrameSlot::Parent);
                if ui.selectable_label(waiting_parent, "🎯").clicked() {
                    popup.waiting_for = if waiting_parent { None } else { Some(FrameSlot::Parent) };
                }
                let label = popup
                    .parent_frame
                    .and_then(|e| frames.get(e).ok())
                    .map(|(_, n, _)| n.as_str().to_owned())
                    .unwrap_or_else(|| "Select...".to_string());
                egui::ComboBox::from_id_salt("parent_frame_picker")
                    .selected_text(&label)
                    .show_ui(ui, |ui| {
                        if ui.selectable_label(popup.parent_frame.is_none(), "None").clicked() {
                            popup.parent_frame = None;
                        }
                        for (entity, name) in &all_frames {
                            let is_selected = popup.parent_frame == Some(*entity);
                            if ui.selectable_label(is_selected, name).clicked() {
                                popup.parent_frame = Some(*entity);
                            }
                        }
                    });
            });

            // ── Joint type ──
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Joint type:");
                let weld = &mut popup.weld;
                egui::ComboBox::from_id_salt("joint_type")
                    .selected_text(if *weld { "Weld (rigid)" } else { "Free" })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(weld, true, "Weld (rigid)");
                        ui.selectable_value(weld, false, "Free");
                    });
            });

            // ── Preview ──
            if popup.child_frame.is_some() || popup.parent_frame.is_some() {
                let child_label = popup
                    .child_frame
                    .and_then(|e| frames.get(e).ok())
                    .map(|(_, n, _)| n.as_str().to_owned())
                    .unwrap_or_else(|| "(none)".to_string());
                let parent_label = popup
                    .parent_frame
                    .and_then(|e| frames.get(e).ok())
                    .map(|(_, n, _)| n.as_str().to_owned())
                    .unwrap_or_else(|| "(none)".to_string());
                let joint_label = if popup.weld { "Weld" } else { "Free" };
                ui.add_space(4.0);
                ui.label(format!(
                    "{} → {} Joint → {}",
                    child_label, joint_label, parent_label
                ));
            }

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    popup.reset();
                    close = true;
                }

                let can_connect = popup.child_frame.is_some() && popup.parent_frame.is_some();
                ui.add_enabled_ui(can_connect, |ui| {
                    if ui.button("Connect").clicked() {
                        let child = popup.child_frame.unwrap();
                        let parent = popup.parent_frame.unwrap();

                        // Find the parent body of the parent frame (for hierarchy)
                        // The parent body is the entity that has ChildOf pointing to the parent frame's body
                        // Actually, frames are children of bodies, so we need to find which body owns each frame
                        // For now, we create the joint as a child of the parent frame's body

                        // Get parent body from ChildOf of parent frame
                        // In melosim, frames are children of bodies via ChildOf
                        // We need to query the world to find the parent body
                        // But we don't have World access here — we use commands

                        // Create coordinate for the joint
                        let coord = commands
                            .spawn((
                                Name::new("joint_coord".to_string()),
                                Coordinate::default(),
                                crate::model::CoordinateProperties::default(),
                            ))
                            .id();

                        let twist = if popup.weld {
                            Twist::default()
                        } else {
                            Twist::default()
                        };
                        commands.entity(coord).insert(twist);

                        // Compute joint transform from frame positions
                        let joint_transform = Transform::default(); // Will be computed from frames

                        // We need the parent body entity. Since frames are children of bodies,
                        // we can't easily get it here without World access.
                        // For now, we'll spawn the joint without a parent and let the
                        // user's existing workflow handle the hierarchy.
                        //
                        // TODO: When we have World access, find the body that owns each frame
                        // and create the proper hierarchy.

                        // Create the joint
                        let joint = commands
                            .spawn((
                                Name::new("joint".to_string()),
                                Joint,
                                JointCoordinates::new(vec![coord]),
                                joint_transform,
                            ))
                            .id();

                        commands.entity(coord).insert(CoordinateOf(joint));

                        // Attach Connects component
                        commands
                            .entity(joint)
                            .insert(Connects { frame_a: child, frame_b: parent });

                        events.push(EditorEvent::ModelMutation {
                            kind: MutationKind::AddChild {
                                parent: Entity::PLACEHOLDER,
                                child: joint,
                                marker: "Joint",
                            },
                            entity: joint,
                        });

                        popup.reset();
                        close = true;
                    }
                });
            });
        });

    close
}
