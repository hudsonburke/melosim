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
use super::PendingMujocoExport;
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
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    click.0 = None;
    #[expect(deprecated, reason = "top-level panels require show(ctx), not show_inside")]
    egui::Panel::left("melosim_hierarchy")
        .resizable(true)
        .default_size(260.0)
        .show(ctx, |ui| {
            click.0 = panels::hierarchy::show(
                ui,
                &root_entities,
                &names,
                &children_query,
                &joint_coords,
                &model_markers,
                &selection,
            );
        });
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

/// Tiny top-left control for showing/hiding the tool windows and the export
/// button. Separate system so it doesn't need extra params on `editor_ui`
/// (which is at Bevy's 16-param system cap).
pub fn tool_windows_toggle(
    mut contexts: EguiContexts,
    mut panels: ResMut<super::ToolPanels>,
    mut pending_export: ResMut<PendingMujocoExport>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    egui::Window::new("Tools")
        .anchor(egui::Align2::LEFT_TOP, egui::vec2(4.0, 30.0))
        .collapsible(false)
        .resizable(false)
        .show(ctx, |ui| {
            ui.checkbox(&mut panels.import_mesh, "Import Mesh tool");
            ui.checkbox(&mut panels.path_editor, "Path Editor tool");
            ui.separator();
            if ui.button("Export → MuJoCo (.xml)").clicked() {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("MuJoCo model", &["xml"])
                    .set_file_name("export.xml")
                    .save_file()
                {
                    pending_export.0 = Some(path);
                }
            }
        });
}
