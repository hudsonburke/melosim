//! Popup/dialog system for model component creation.
//!
//! This module provides the `PopupManager` resource and individual popup
//! dialogs for creating Body, Joint, Muscle, Site, and Frame components.
//! Popups emit `EditorEvent::ModelMutation` events when confirmed.

pub mod add_body;
pub mod add_cable;
pub mod add_joint;
pub mod add_muscle;
pub mod add_path_site;

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::events::{EditorEvent, EditorEvents, MutationKind};
use crate::model::{Body, Frame, Site};

/// Which popup dialog is currently open, if any.
#[derive(Default, Resource, Debug, Clone, PartialEq)]
pub enum ActivePopup {
    #[default]
    None,
    AddBody,
    AddCable,
    AddJoint,
    AddMuscle,
    AddPathSite,
    AddSite,
    AddFrame,
}

/// State for the "Add Body" dialog.
#[derive(Resource, Default)]
pub struct AddBodyPopup {
    pub name: String,
}

/// State for the "Add Cable" dialog.
#[derive(Resource)]
pub struct AddCablePopup {
    pub name: String,
    pub max_tension: f64,
    pub actuator_force: f64,
}

impl Default for AddCablePopup {
    fn default() -> Self {
        Self {
            name: String::new(),
            max_tension: 1000.0,
            actuator_force: 100.0,
        }
    }
}

/// State for the "Add Joint" dialog.
#[derive(Resource, Default)]
pub struct AddJointPopup {
    pub name: String,
    pub joint_type: JointType,
}

#[derive(Default, Debug, Clone, PartialEq)]
pub enum JointType {
    #[default]
    Weld,
    Hinge,
    Ball,
}

impl JointType {
    pub fn as_str(&self) -> &'static str {
        match self {
            JointType::Weld => "Weld",
            JointType::Hinge => "Hinge",
            JointType::Ball => "Ball",
        }
    }
}

/// State for the "Add Muscle" dialog.
#[derive(Resource)]
pub struct AddMusclePopup {
    pub name: String,
    pub parent_body: String,
    pub max_force: f64,
    pub fiber_length: f64,
}

impl Default for AddMusclePopup {
    fn default() -> Self {
        Self {
            name: String::new(),
            parent_body: String::new(),
            max_force: 1000.0,
            fiber_length: 0.1,
        }
    }
}

/// State for the "Add Cable Path Site" dialog.
#[derive(Resource)]
pub struct AddPathSitePopup {
    pub name: String,
    pub parent: Option<Entity>,
    pub snap_to_parent: bool,
}

impl Default for AddPathSitePopup {
    fn default() -> Self {
        Self {
            name: String::new(),
            parent: None,
            snap_to_parent: true,
        }
    }
}

/// State for the "Add Site" dialog.
#[derive(Resource, Default)]
pub struct AddSitePopup {
    pub name: String,
}

/// State for the "Add Frame" dialog.
#[derive(Resource, Default)]
pub struct AddFramePopup {
    pub name: String,
}

/// Shared part counter for auto-naming components.
#[derive(Resource, Default)]
pub struct PartCounter(pub u64);

/// Structural model requests emitted by popup UI and applied by an exclusive
/// world system after the egui pass.
#[derive(Resource, Default)]
pub struct PendingModelActions {
    pub add_joints: Vec<AddJointRequest>,
}

#[derive(Clone, Debug)]
pub struct AddJointRequest {
    pub parent: Entity,
    pub name: String,
    pub joint_type: JointType,
}

/// System that shows the active popup window.
///
/// This runs in `EguiPrimaryContextPass` after the panels.
pub fn show_popups(
    mut contexts: bevy_inspector_egui::bevy_egui::EguiContexts,
    mut active_popup: ResMut<ActivePopup>,
    mut add_body: ResMut<AddBodyPopup>,
    mut add_cable: ResMut<AddCablePopup>,
    mut add_joint: ResMut<AddJointPopup>,
    mut add_muscle: ResMut<AddMusclePopup>,
    mut add_path_site: ResMut<AddPathSitePopup>,
    mut add_site: ResMut<AddSitePopup>,
    mut add_frame: ResMut<AddFramePopup>,
    mut counter: ResMut<PartCounter>,
    mut commands: Commands,
    mut events: ResMut<EditorEvents>,
    parents: Query<(
        Entity,
        &Name,
        Option<&Body>,
        Option<&Frame>,
    ), Or<(With<Body>, With<Frame>)>>,
    cables: Query<(), With<crate::model::Cable>>,
    selection: Res<super::selection::Selection>,
    mut pending_actions: ResMut<PendingModelActions>,
) {
    let ctx = match contexts.ctx_mut() {
        Ok(ctx) => ctx,
        Err(_) => return,
    };
    let parent_items: Vec<(Entity, String)> = parents
        .iter()
        .map(|(entity, name, _, _)| (entity, name.as_str().to_owned()))
        .collect();
    let body_items: Vec<(Entity, String)> = parents
        .iter()
        .filter_map(|(entity, name, body, _)| body.map(|_| (entity, name.as_str().to_owned())))
        .collect();

    match &*active_popup {
        ActivePopup::None => return,
        ActivePopup::AddBody => {
            let close = add_body::show(ctx, &mut add_body, &mut counter, &mut commands, &mut events);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddCable => {
            let close = add_cable::show(ctx, &mut add_cable, &mut counter, &mut commands, &mut events);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddJoint => {
            let close = add_joint::show(ctx, &mut add_joint, &mut counter, &mut pending_actions, &selection);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddMuscle => {
            let close = add_muscle::show(ctx, &mut add_muscle, &mut counter, &mut commands, &mut events, &body_items);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddPathSite => {
            let close = add_path_site::show(
                ctx,
                &mut add_path_site,
                &mut counter,
                &mut commands,
                &mut events,
                &selection,
                &parent_items,
                &cables,
            );
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddSite => {
            let close = add_site_popup(ctx, &mut add_site, &mut counter, &mut commands, &mut events, &selection);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddFrame => {
            let close = add_frame_popup(ctx, &mut add_frame, &mut counter, &mut commands, &mut events, &selection);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
    }
}

/// Show the "Add Site" popup.
fn add_site_popup(
    ctx: &egui::Context,
    popup: &mut AddSitePopup,
    counter: &mut PartCounter,
    commands: &mut Commands,
    events: &mut EditorEvents,
    selection: &super::selection::Selection,
) -> bool {
    let mut close = false;
    let parent = selection.primary();

    egui::Window::new("Add Site")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Name:");
                let _name = if popup.name.is_empty() {
                    format!("site_{}", counter.0)
                } else {
                    popup.name.clone()
                };
                ui.text_edit_singleline(&mut popup.name);
            });

            if let Some(parent) = parent {
                ui.label(format!("Parent: {:?}", parent));
            }

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    close = true;
                }
                if ui.button("Create").clicked() {
                    let name = if popup.name.is_empty() {
                        format!("site_{}", counter.0)
                    } else {
                        popup.name.clone()
                    };
                    counter.0 += 1;

                    if let Some(parent) = parent {
                        let child = commands
                            .spawn((Name::new(name.clone()), Site, Transform::from_xyz(0.03, 0.0, 0.0)))
                            .insert(ChildOf(parent))
                            .id();
                        commands.entity(parent).add_children(&[child]);

                        events.push(EditorEvent::ModelMutation {
                            kind: MutationKind::AddChild {
                                parent,
                                child,
                                marker: "Site",
                            },
                            entity: child,
                        });
                    }

                    popup.name.clear();
                    close = true;
                }
            });
        });
    close
}

/// Show the "Add Frame" popup.
fn add_frame_popup(
    ctx: &egui::Context,
    popup: &mut AddFramePopup,
    counter: &mut PartCounter,
    commands: &mut Commands,
    events: &mut EditorEvents,
    selection: &super::selection::Selection,
) -> bool {
    let mut close = false;
    let parent = selection.primary();

    egui::Window::new("Add Frame")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label("Name:");
                let _name = if popup.name.is_empty() {
                    format!("frame_{}", counter.0)
                } else {
                    popup.name.clone()
                };
                ui.text_edit_singleline(&mut popup.name);
            });

            if let Some(parent) = parent {
                ui.label(format!("Parent: {:?}", parent));
            }

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    close = true;
                }
                if ui.button("Create").clicked() {
                    let name = if popup.name.is_empty() {
                        format!("frame_{}", counter.0)
                    } else {
                        popup.name.clone()
                    };
                    counter.0 += 1;

                    if let Some(parent) = parent {
                        let child = commands
                            .spawn((Name::new(name.clone()), Frame, Transform::from_xyz(0.03, 0.0, 0.0)))
                            .insert(ChildOf(parent))
                            .id();
                        commands.entity(parent).add_children(&[child]);

                        events.push(EditorEvent::ModelMutation {
                            kind: MutationKind::AddChild {
                                parent,
                                child,
                                marker: "Frame",
                            },
                            entity: child,
                        });
                    }

                    popup.name.clear();
                    close = true;
                }
            });
        });
    close
}
