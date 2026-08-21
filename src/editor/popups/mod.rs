//! Popup/dialog system for model component creation.
//!
//! This module provides the `PopupManager` resource and individual popup
//! dialogs for creating Body, Joint, Muscle, Site, and Frame components.
//! Popups emit `EditorEvent::ModelMutation` events when confirmed.

pub mod add_body;
pub mod add_joint;
pub mod add_muscle;

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
    AddJoint,
    AddMuscle,
    AddSite,
    AddFrame,
    ImportMeshUnit,
}

/// State for the "Add Body" dialog.
#[derive(Resource, Default)]
pub struct AddBodyPopup {
    pub name: String,
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

/// System that shows the active popup window.
///
/// This runs in `EguiPrimaryContextPass` after the panels.
pub fn show_popups(
    mut contexts: bevy_inspector_egui::bevy_egui::EguiContexts,
    mut active_popup: ResMut<ActivePopup>,
    mut add_body: ResMut<AddBodyPopup>,
    mut add_joint: ResMut<AddJointPopup>,
    mut add_muscle: ResMut<AddMusclePopup>,
    mut add_site: ResMut<AddSitePopup>,
    mut add_frame: ResMut<AddFramePopup>,
    mut counter: ResMut<PartCounter>,
    mut commands: Commands,
    mut events: ResMut<EditorEvents>,
    bodies: Query<(Entity, &Name), With<Body>>,
    selection: Res<super::selection::Selection>,
    mut pending_mesh: ResMut<super::PendingMeshImport>,
    asset_server: Res<AssetServer>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut mesh_assets: ResMut<Assets<Mesh>>,
) {
    let ctx = match contexts.ctx_mut() {
        Ok(ctx) => ctx,
        Err(_) => return,
    };

    match &*active_popup {
        ActivePopup::None => return,
        ActivePopup::AddBody => {
            let close = add_body::show(ctx, &mut add_body, &mut counter, &mut commands, &mut events);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddJoint => {
            let close = add_joint::show(ctx, &mut add_joint, &mut counter, &mut commands, &mut events, &selection);
            if close {
                *active_popup = ActivePopup::None;
            }
        }
        ActivePopup::AddMuscle => {
            let close = add_muscle::show(ctx, &mut add_muscle, &mut counter, &mut commands, &mut events, &bodies);
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
        ActivePopup::ImportMeshUnit => {
            let close = show_import_mesh_unit(ctx, &mut pending_mesh, &mut commands, &asset_server, &mut materials, &mut mesh_assets);
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

/// Show the "Import Mesh Unit" popup — asks user to specify units before importing.
fn show_import_mesh_unit(
    ctx: &egui::Context,
    pending_mesh: &mut super::PendingMeshImport,
    commands: &mut Commands,
    asset_server: &AssetServer,
    materials: &mut Assets<StandardMaterial>,
    mesh_assets: &mut Assets<Mesh>,
) -> bool {
    // Only show if there's a pending mesh import
    let Some(path) = pending_mesh.0.clone() else {
        return true;
    };

    let mut close = false;
    let mut unit = crate::editor::mesh_import::ImportUnit::Mm;

    egui::Window::new("Import Mesh")
        .collapsible(false)
        .resizable(false)
        .anchor(egui::Align2::CENTER_CENTER, egui::vec2(0.0, 0.0))
        .show(ctx, |ui| {
            ui.label(format!("File: {}", path.file_name().unwrap_or_default().to_string_lossy()));
            ui.add_space(4.0);
            ui.horizontal(|ui| {
                ui.label("Unit:");
                egui::ComboBox::from_id_salt("import_mesh_unit")
                    .selected_text(unit.label())
                    .show_ui(ui, |ui| {
                        for u in [
                            crate::editor::mesh_import::ImportUnit::Mm,
                            crate::editor::mesh_import::ImportUnit::Cm,
                            crate::editor::mesh_import::ImportUnit::M,
                            crate::editor::mesh_import::ImportUnit::In,
                            crate::editor::mesh_import::ImportUnit::Ft,
                        ] {
                            ui.selectable_value(&mut unit, u, u.label());
                        }
                    });
            });
            ui.add_space(8.0);
            ui.horizontal(|ui| {
                if ui.button("Import").clicked() {
                    pending_mesh.0.take();
                    crate::editor::mesh_import::import_file(
                        commands,
                        asset_server,
                        materials,
                        mesh_assets,
                        &path,
                        unit,
                    );
                    close = true;
                }
                if ui.button("Cancel").clicked() {
                    close = true;
                }
            });
        });
    close
}
