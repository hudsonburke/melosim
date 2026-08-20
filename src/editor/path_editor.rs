//! Path editor — author a cable/muscle path from existing or newly created sites.
//!
//! Lets you spawn a new **Muscle** (with Hill-type params) or **Cable** (path
//! only), then build its ordered `PathEntities` list by appending existing
//! `Site`s or creating a brand-new Site on a Body. Mutating `PathEntities`
//! auto-syncs the `PathElement` inverse via Bevy's relationship machinery.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use crate::model::{Body, HillTypeMuscleParams, Muscle, PathEntities, Site};

use super::selection::Selection;

/// Editor state: the muscle/cable currently being authored + its point list.
#[derive(Default, Resource)]
pub struct PathEditor {
    pub current: Option<Entity>,
    pub path: Vec<Entity>,
}

/// Write the current point list onto the authored entity's `PathEntities`.
fn apply(editor: &PathEditor, paths: &mut Query<&mut PathEntities>) {
    if let Some(m) = editor.current {
        if let Ok(mut pe) = paths.get_mut(m) {
            pe.set(editor.path.clone());
        }
    }
}

/// Small panel to author cable/muscle paths. Separate system so `editor_ui`
/// stays under Bevy's 16-parameter limit.
#[allow(clippy::too_many_arguments)]
pub fn path_editor_ui(
    mut contexts: EguiContexts,
    panels: Res<super::ToolPanels>,
    mut commands: Commands,
    mut editor: ResMut<PathEditor>,
    mut paths: Query<&mut PathEntities>,
    names: Query<&Name>,
    sites: Query<Entity, With<Site>>,
    bodies: Query<Entity, With<Body>>,
    selection: Res<Selection>,
    mut counter: Local<u64>,
) {
    if !panels.path_editor {
        return;
    }
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    egui::Window::new("Path Editor")
        .anchor(egui::Align2::CENTER_BOTTOM, egui::vec2(0.0, -150.0))
        .collapsible(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("＋ New Muscle").clicked() {
                    *counter += 1;
                    let e = commands
                        .spawn((
                            Name::new(format!("muscle_{}", *counter)),
                            Muscle,
                            HillTypeMuscleParams::default(),
                            PathEntities::new(Vec::new()),
                        ))
                        .id();
                    editor.current = Some(e);
                    editor.path.clear();
                }
                if ui.button("＋ New Cable").clicked() {
                    *counter += 1;
                    let e = commands
                        .spawn((Name::new(format!("cable_{}", *counter)), PathEntities::new(Vec::new())))
                        .id();
                    editor.current = Some(e);
                    editor.path.clear();
                }
            });

            apply(&editor, &mut paths);
            ui.separator();

            let title = editor
                .current
                .and_then(|e| names.get(e).ok())
                .map(|n| n.as_str().to_owned())
                .unwrap_or_else(|| "no path selected".to_owned());
            ui.label(egui::RichText::new(format!("Path: {title}")).strong());

            // Ordered point list (with remove).
            let mut remove: Option<usize> = None;
            for (i, &p) in editor.path.iter().enumerate() {
                let nm = names
                    .get(p)
                    .map(|n| n.as_str().to_owned())
                    .unwrap_or_else(|_| format!("{p:?}"));
                ui.horizontal(|ui| {
                    ui.label(format!("{i}: {nm}"));
                    if ui.button("−").clicked() {
                        remove = Some(i);
                    }
                });
            }
            if let Some(i) = remove {
                editor.path.remove(i);
                apply(&editor, &mut paths);
            }

            ui.separator();
            ui.label("Add existing site:");
            let site_list: Vec<Entity> = sites.iter().collect();
            for s in site_list {
                let nm = names
                    .get(s)
                    .map(|n| n.as_str().to_owned())
                    .unwrap_or_else(|_| format!("{s:?}"));
                if ui.button(nm).clicked() {
                    editor.path.push(s);
                    apply(&editor, &mut paths);
                }
            }

            ui.separator();
            if ui.button("＋ New Site (on Body)").clicked() {
                let parent = selection
                    .primary()
                    .filter(|e| bodies.get(*e).is_ok())
                    .or_else(|| bodies.iter().next());
                if let Some(body) = parent {
                    *counter += 1;
                    let site = commands
                        .spawn((Name::new(format!("site_{}", *counter)), Site, Transform::from_xyz(0.0, 0.0, 0.0)))
                        .insert(ChildOf(body))
                        .id();
                    commands.entity(body).add_children(&[site]);
                    editor.path.push(site);
                    apply(&editor, &mut paths);
                } else {
                    ui.label("Select a Body first.");
                }
            }
        });
}
