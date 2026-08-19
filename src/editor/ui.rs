//! egui-based editor shell, overlaid on the native Bevy 3D viewport.
//!
//! Runs in `EguiPrimaryContextPass` — bevy_egui begins egui's frame there, so
//! fonts/`available_rect` are ready. The 3D viewport is the central Bevy
//! camera; the egui shell docks panels on top.
//!
//! NOTE: `Panel`/`CentralPanel::show(&Context)` are deprecated in egui 0.34
//! (→ `show_inside(&mut Ui)`), but the root-`Ui` replacement isn't surfaced
//! through bevy_egui yet and they work once the pass is running. Revisit when
//! egui 0.34 panels become cleanly usable.

use bevy::log::debug;
use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::selection::Selection;
use crate::model::{
    Body, Coordinate, CoordinateProperties, CoordinateState, Frame, InitialConditions,
    InertialProperties, Joint, JointCoordinates, Muscle, Site,
};

/// Editor shell UI: toolbar + left hierarchy of model entities, right inspector.
#[allow(deprecated, clippy::too_many_arguments)]
pub fn editor_ui(
    mut contexts: EguiContexts,
    mut selection: ResMut<Selection>,
    names: Query<&Name>,
    root_entities: Query<Entity, (Without<ChildOf>, Or<(With<Body>, With<Frame>, With<Joint>, With<Muscle>, With<Site>, With<Coordinate>)>)>,
    children_query: Query<&Children>,
    joint_coords: Query<&JointCoordinates>,
    bodies: Query<&Body>,
    joints: Query<&Joint>,
    sites: Query<&Site>,
    frames: Query<&Frame>,
    muscles: Query<&Muscle>,
    coord_marker: Query<&Coordinate>,
    inertial: Query<&InertialProperties>,
    mut coord_states: Query<(&CoordinateProperties, &InitialConditions, &mut CoordinateState)>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");
    let mut clicked: Option<Entity> = None;

    // ── Toolbar (top) ────────────────────────────────────────────────
    egui::TopBottomPanel::top("melosim_toolbar").show(ctx, |ui| {
        ui.horizontal(|ui| {
            ui.heading("melosim");
            ui.separator();
            match selection.primary() {
                Some(e) => {
                    let name = names
                        .get(e)
                        .map(|n| n.as_str().to_owned())
                        .unwrap_or_else(|_| format!("{:?}", e));
                    ui.label(format!("Selected: {}", name));
                }
                None => {
                    ui.label("Nothing selected");
                }
            }
        });
    });

    // ── Hierarchy (left) ─────────────────────────────────────────────
    egui::SidePanel::left("melosim_hierarchy")
        .resizable(true)
        .default_width(260.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.add_space(4.0);
                ui.heading("Hierarchy");

                let mut roots: Vec<Entity> = root_entities.iter().collect();
                roots.sort_by(|a, b| {
                    names.get(*a).map(|n| n.as_str().to_owned()).unwrap_or_default().cmp(
                        &names.get(*b).map(|n| n.as_str().to_owned()).unwrap_or_default(),
                    )
                });

                for root in roots {
                    hierarchy_node(
                        ui,
                        root,
                        0,
                        &names,
                        &children_query,
                        &joint_coords,
                        &bodies,
                        &joints,
                        &sites,
                        &frames,
                        &muscles,
                        &coord_marker,
                        &selection,
                        &mut clicked,
                    );
                }
            });
        });

    // ── Inspector (right): entity info + coordinate articulation ─────
    egui::SidePanel::right("melosim_inspector")
        .resizable(true)
        .default_width(300.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.add_space(4.0);
                ui.heading("Inspector");

                let Some(entity) = selection.primary() else {
                    ui.label("Nothing selected");
                    return;
                };
                let name = names
                    .get(entity)
                    .map(|n| n.as_str().to_owned())
                    .unwrap_or_else(|_| format!("{:?}", entity));
                ui.label(egui::RichText::new(name).strong());

                // Coordinate articulation: editing CoordinateState → FK re-poses.
                if let Ok((props, _init, mut state)) = coord_states.get_mut(entity) {
                    ui.separator();
                    ui.label(egui::RichText::new("Coordinate").underline());
                    ui.label(format!("range: [{:.4}, {:.4}]", props.range.0, props.range.1));
                    let val_resp = ui.add(
                        egui::Slider::new(&mut state.value, props.range.0..=props.range.1)
                            .text("value"),
                    );
                    // Diagnostic: confirm the selected coordinate's value edit
                    // actually reaches the CoordinateState the FK reads.
                    #[cfg(debug_assertions)]
                    if val_resp.changed() {
                        debug!("inspector: set CoordinateState.value = {}", state.value);
                    }
                    ui.add(egui::Slider::new(&mut state.velocity, -10.0..=10.0).text("velocity"));
                }

                if let Ok(ip) = inertial.get(entity) {
                    ui.separator();
                    ui.label(format!("mass: {:.4} kg", ip.mass));
                }
            });
        });

    // Apply a hierarchy-click selection after the egui pass.
    if let Some(entity) = clicked {
        selection.select_single(entity);
    }
}

/// Recursively render a model entity as an expandable/selectable tree row.
/// Children come from Bevy's `ChildOf` hierarchy plus, for joints, the
/// `JointCoordinates` relationship so generalized coordinates are selectable.
#[allow(clippy::too_many_arguments)]
fn hierarchy_node(
    ui: &mut egui::Ui,
    entity: Entity,
    depth: usize,
    names: &Query<&Name>,
    children_query: &Query<&Children>,
    joint_coords: &Query<&JointCoordinates>,
    bodies: &Query<&Body>,
    joints: &Query<&Joint>,
    sites: &Query<&Site>,
    frames: &Query<&Frame>,
    muscles: &Query<&Muscle>,
    coord_marker: &Query<&Coordinate>,
    selection: &Selection,
    clicked: &mut Option<Entity>,
) {
    let name = names
        .get(entity)
        .map(|n| n.as_str().to_owned())
        .unwrap_or_else(|_| format!("{:?}", entity));

    let badge = if bodies.get(entity).is_ok() {
        "B"
    } else if joints.get(entity).is_ok() {
        "J"
    } else if coord_marker.get(entity).is_ok() {
        "C"
    } else if sites.get(entity).is_ok() {
        "S"
    } else if muscles.get(entity).is_ok() {
        "M"
    } else if frames.get(entity).is_ok() {
        "F"
    } else {
        ""
    };

    let label = if badge.is_empty() {
        name.clone()
    } else {
        format!("[{}] {}", badge, name)
    };
    let is_selected = selection.is_selected(entity);
    let text_color = if is_selected {
        egui::Color32::LIGHT_BLUE
    } else {
        egui::Color32::WHITE
    };

    // Children = ChildOf hierarchy + (for joints) generalized coordinates.
    let has_children = children_query
        .get(entity)
        .map(|c| !c.is_empty())
        .unwrap_or(false)
        || joint_coords.get(entity).map(|c| !c.is_empty()).unwrap_or(false);

    if has_children {
        let response = egui::CollapsingHeader::new(egui::RichText::new(label).color(text_color))
            .default_open(depth < 1)
            .id_salt(entity.index())
            .show(ui, |ui| {
                if let Ok(children) = children_query.get(entity) {
                    for child in children.iter() {
                        hierarchy_node(
                            ui, child, depth + 1, names, children_query, joint_coords, bodies,
                            joints, sites, frames, muscles, coord_marker, selection, clicked,
                        );
                    }
                }
                if let Ok(coords) = joint_coords.get(entity) {
                    for coord in coords.iter() {
                        hierarchy_node(
                            ui, coord, depth + 1, names, children_query, joint_coords, bodies,
                            joints, sites, frames, muscles, coord_marker, selection, clicked,
                        );
                    }
                }
            });
        if response.header_response.clicked() {
            *clicked = Some(entity);
        }
    } else if ui.selectable_label(is_selected, label).clicked() {
        *clicked = Some(entity);
    }
}
