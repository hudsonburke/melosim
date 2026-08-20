//! egui-based editor shell, overlaid on the native Bevy 3D viewport.
//!
//! Runs in `EguiPrimaryContextPass` — bevy_egui begins egui's frame there, so
//! fonts/`available_rect` are ready. The 3D viewport is the central Bevy
//! camera; the egui shell docks panels on top.
//!
//! Phase 1 (panel extraction): toolbar, hierarchy, and inspector live in
//! `panels/`. Phase 2 (egui_dock): `dock.rs` defines `Tab` enum and
//! `EditorDockState` resource for future dock-area integration. Currently
//! panels render via manual SidePanel/TopBottomPanel layout.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::models::{ModelRegistry, SelectedModel};
use super::panels;
use super::selection::Selection;
use super::PendingMujocoExport;
use crate::model::{
    Body, Coordinate, CoordinateProperties, CoordinateState, Frame, HillTypeMuscleParams,
    InitialConditions, InertialProperties, Joint, JointCoordinates, Muscle, Site, Twist,
};
use crate::render::RenderSettings;

/// Editor shell UI: toolbar (top) + left hierarchy, right inspector.
#[allow(clippy::too_many_arguments)]
pub fn editor_ui(
    mut contexts: EguiContexts,
    mut commands: Commands,
    mut selection: ResMut<Selection>,
    registry: Res<ModelRegistry>,
    mut selected_model: ResMut<SelectedModel>,
    mut settings: ResMut<RenderSettings>,
    mut part_counter: Local<u64>,
    mut names: Query<&mut Name>,
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
    mut inertial: Query<&mut InertialProperties>,
    mut twists: Query<&mut Twist>,
    mut coord_editor: Query<(&mut CoordinateProperties, &mut InitialConditions, &mut CoordinateState)>,
    mut hill: Query<&mut HillTypeMuscleParams>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");

    // ── Toolbar (top — fixed, not docked) ───────────────────────────
    egui::TopBottomPanel::top("melosim_toolbar").show(ctx, |ui| {
        panels::toolbar::show(
            ui,
            &selection,
            &registry,
            &mut selected_model,
            &mut settings,
            &names,
        );
    });

    // ── Hierarchy (left) ─────────────────────────────────────────────
    let mut hierarchy_clicked: Option<Entity> = None;
    egui::SidePanel::left("melosim_hierarchy")
        .resizable(true)
        .default_width(260.0)
        .show(ctx, |ui| {
            hierarchy_clicked = panels::hierarchy::show(
                ui,
                &root_entities,
                &names,
                &children_query,
                &joint_coords,
                &model_markers,
                &selection,
            );
        });

    // ── Inspector (right) ────────────────────────────────────────────
    egui::SidePanel::right("melosim_inspector")
        .resizable(true)
        .default_width(320.0)
        .show(ctx, |ui| {
            panels::inspector::show(
                ui,
                &mut selection,
                &mut names,
                &children_query,
                &model_markers,
                &mut inertial,
                &mut twists,
                &mut coord_editor,
                &mut hill,
                &mut commands,
                &mut part_counter,
            );
        });

    // Apply a hierarchy-click selection after the egui pass.
    if let Some(entity) = hierarchy_clicked {
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
