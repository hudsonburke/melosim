//! egui-based editor shell, overlaid on the native Bevy 3D viewport.
//!
//! Runs in `EguiPrimaryContextPass` — bevy_egui begins egui's frame there, so
//! fonts/`available_rect` are ready. The 3D viewport is the central Bevy
//! camera; the egui shell docks panels on top.
//!
//! Phase 1 (panel extraction): toolbar, hierarchy, and inspector live in
//! `panels/`. Phase 2 (egui_dock): `dock.rs` defines `Tab` enum and
//! `EditorDockState` resource for future dock-area integration. Currently
//! panels render via manual Panel layout.
//!
//! The former monolithic `editor_ui` (16 params, at Bevy's cap) is split into
//! narrow-scoped systems that each query only what they need.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::models::{ModelRegistry, SelectedModel};
use super::panels;
use super::popups::ActivePopup;
use super::selection::Selection;
use crate::model::{
    Body, Coordinate, CoordinateProperties, CoordinateState, Frame, HillTypeMuscleParams,
    InitialConditions, InertialProperties, Joint, JointCoordinates, Muscle, Site, Twist,
};
use crate::render::RenderSettings;

// ── Shared state for inter-system communication ─────────────

/// Stores the entity clicked in the hierarchy panel during this frame.
/// Written by `hierarchy_panel`, consumed by `apply_hierarchy_selection`.
#[derive(Default, Resource)]
pub struct HierarchyClick(pub Option<Entity>);

/// Stores right-click info from the hierarchy panel for context menu handling.
#[derive(Default, Resource)]
pub struct HierarchyRightClickState {
    pub entity: Option<Entity>,
    pub screen_pos: egui::Pos2,
}

// ── Narrow-scoped panel systems ─────────────────────────────

/// Top toolbar: model name, model menu, view toggles.
pub fn toolbar_panel(
    mut contexts: EguiContexts,
    selection: Res<Selection>,
    registry: Res<ModelRegistry>,
    mut selected_model: ResMut<SelectedModel>,
    mut settings: ResMut<RenderSettings>,
    names: Query<&mut Name>,
    mut active_popup: ResMut<ActivePopup>,
    mut pending_model_import: ResMut<super::PendingModelImport>,
    mut pending_mesh_import: ResMut<super::PendingMeshImport>,
    mut pending_export: ResMut<super::PendingMujocoExport>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    #[expect(deprecated, reason = "top-level panels require show(ctx), not show_inside")]
    egui::Panel::top("melosim_toolbar")
        .default_size(36.0)
        .show(ctx, |ui| {
            panels::toolbar::show(
                ui,
                &selection,
                &registry,
                &mut selected_model,
                &mut settings,
                &names,
                &mut active_popup,
                &mut pending_model_import,
                &mut pending_mesh_import,
                &mut pending_export,
            );
        });
}

/// Left hierarchy panel: tree of model entities.
pub fn hierarchy_panel(
    mut contexts: EguiContexts,
    root_entities: Query<
        Entity,
        (
            Without<ChildOf>,
            Or<(
                With<Body>,
                With<Frame>,
                With<Joint>,
                With<Muscle>,
                With<Site>,
                With<Coordinate>,
            )>,
        ),
    >,
    names: Query<&mut Name>,
    children_query: Query<&Children>,
    joint_coords: Query<&JointCoordinates>,
    model_markers: Query<(
        Option<&Body>,
        Option<&Joint>,
        Option<&Coordinate>,
        Option<&Site>,
        Option<&Muscle>,
        Option<&Frame>,
    )>,
    selection: Res<Selection>,
    mut click: ResMut<HierarchyClick>,
    mut right_click: ResMut<HierarchyRightClickState>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    click.0 = None;
    right_click.entity = None;

    #[expect(deprecated, reason = "top-level panels require show(ctx), not show_inside")]
    egui::Panel::left("melosim_hierarchy")
        .resizable(true)
        .default_size(260.0)
        .show(ctx, |ui| {
            let (clicked, rc) = panels::hierarchy::show(
                ui,
                &root_entities,
                &names,
                &children_query,
                &joint_coords,
                &model_markers,
                &selection,
            );
            click.0 = clicked;
            if let Some(rc) = rc {
                right_click.entity = Some(rc.entity);
                right_click.screen_pos = rc.screen_pos;
            }
        });

    // Show context menu if right-click happened
    if let Some(_entity) = right_click.entity {
        let popup_id = egui::Id::new("hierarchy_context_menu");
        egui::Area::new(popup_id)
            .fixed_pos(right_click.screen_pos)
            .order(egui::Order::Foreground)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.menu_button("Actions", |ui| {
                        if ui.button("Add Site").clicked() {
                            // TODO: spawn site as child of entity
                            ui.close();
                        }
                        if ui.button("Add Frame").clicked() {
                            // TODO: spawn frame as child of entity
                            ui.close();
                        }
                        ui.separator();
                        if ui.button("Rename…").clicked() {
                            // TODO: open rename dialog
                            ui.close();
                        }
                        if ui.button("Delete").clicked() {
                            // TODO: delete entity
                            ui.close();
                        }
                    });
                });
            });
        // Reset after showing menu
        right_click.entity = None;
    }
}

/// Right inspector panel: component editing for the selected entity.
pub fn inspector_panel(
    mut contexts: EguiContexts,
    mut selection: ResMut<Selection>,
    mut names: Query<&mut Name>,
    children_query: Query<&Children>,
    model_markers: Query<(
        Option<&Body>,
        Option<&Joint>,
        Option<&Coordinate>,
        Option<&Site>,
        Option<&Muscle>,
        Option<&Frame>,
    )>,
    mut inertial: Query<&mut InertialProperties>,
    mut twists: Query<&mut Twist>,
    mut coord_editor: Query<(&mut CoordinateProperties, &mut InitialConditions, &mut CoordinateState)>,
    mut hill: Query<&mut HillTypeMuscleParams>,
    mut commands: Commands,
    mut part_counter: Local<u64>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    #[expect(deprecated, reason = "top-level panels require show(ctx), not show_inside")]
    egui::Panel::right("melosim_inspector")
        .resizable(true)
        .default_size(320.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.add_space(4.0);
                ui.heading("Inspector");

                let Some(entity) = panels::inspector::show_name(ui, &selection, &mut names) else {
                    ui.label("Nothing selected");
                    return;
                };

                panels::inspector::show_inertial(ui, entity, &mut inertial);
                panels::inspector::show_twist(ui, entity, &mut twists);
                panels::inspector::show_coordinate(ui, entity, &mut coord_editor);
                panels::inspector::show_muscle(ui, entity, &mut hill);
                panels::inspector::show_children(
                    ui,
                    entity,
                    &mut selection,
                    &mut names,
                    &children_query,
                    &model_markers,
                    &mut commands,
                    &mut part_counter,
                );
            });
        });
}

/// Apply the hierarchy-click selection after the egui pass.
pub fn apply_hierarchy_selection(
    click: Res<HierarchyClick>,
    mut selection: ResMut<Selection>,
) {
    if let Some(entity) = click.0 {
        selection.select_single(entity);
    }
}

