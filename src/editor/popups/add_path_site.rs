use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use super::{AddPathSitePopup, PartCounter};
use crate::editor::events::{EditorEvent, EditorEvents, MutationKind};
use crate::editor::selection::Selection;
use crate::model::{Cable, PathElement, Site};

pub fn show(
    ctx: &egui::Context,
    popup: &mut AddPathSitePopup,
    counter: &mut PartCounter,
    commands: &mut Commands,
    events: &mut EditorEvents,
    selection: &Selection,
    parents: &[(Entity, String)],
    cables: &Query<(), With<Cable>>,
) -> bool {
    let mut close = false;
    let cable = selection.primary().filter(|entity| cables.get(*entity).is_ok());
    let parent_items = parents;
    if popup.parent.is_none() {
        popup.parent = parent_items.first().map(|(entity, _)| *entity);
    }

    egui::Window::new("Add Cable Path Site")
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            if cable.is_none() {
                ui.colored_label(egui::Color32::YELLOW, "Select a Cable first.");
            }
            ui.horizontal(|ui| {
                ui.label("Name:");
                if popup.name.is_empty() {
                    popup.name = format!("site_{}", counter.0);
                }
                ui.text_edit_singleline(&mut popup.name);
            });
            ui.horizontal(|ui| {
                ui.label("Parent:");
                let selected_name = popup
                    .parent
                    .and_then(|entity| parent_items.iter().find(|(candidate, _)| *candidate == entity))
                    .map(|(_, name)| name.clone())
                    .unwrap_or_else(|| "Select parent…".to_string());
                egui::ComboBox::from_id_salt("path_site_parent")
                    .selected_text(selected_name)
                    .show_ui(ui, |ui| {
                        for (entity, name) in parent_items {
                            ui.selectable_value(&mut popup.parent, Some(*entity), name);
                        }
                    });
            });
            ui.checkbox(&mut popup.snap_to_parent, "Snap to parent frame");
            ui.label("The site can be moved later with the gizmo or translation fields.");

            ui.separator();
            ui.horizontal(|ui| {
                if ui.button("Cancel").clicked() {
                    close = true;
                }
                let can_create = cable.is_some() && popup.parent.is_some();
                ui.add_enabled_ui(can_create, |ui| {
                    if ui.button("Add to path").clicked() {
                        let cable = cable.unwrap();
                        let parent = popup.parent.unwrap();
                        let name = if popup.name.is_empty() {
                            format!("site_{}", counter.0)
                        } else {
                            popup.name.clone()
                        };
                        let transform = if popup.snap_to_parent {
                            Transform::IDENTITY
                        } else {
                            Transform::from_xyz(0.03, 0.0, 0.0)
                        };
                        let site = commands
                            .spawn((Name::new(name), Site, transform, ChildOf(parent), PathElement(cable)))
                            .id();
                        events.push(EditorEvent::ModelMutation {
                            kind: MutationKind::AddChild {
                                parent,
                                child: site,
                                marker: "Site",
                            },
                            entity: site,
                        });
                        counter.0 += 1;
                        popup.name.clear();
                        close = true;
                    }
                });
            });
        });

    close
}
