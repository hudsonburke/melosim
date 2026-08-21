use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use crate::editor::selection::Selection;
use crate::model::{Body, Coordinate, Joint, JointCoordinates, Muscle, Site, Frame};
/// Render the hierarchy tree into the given `Ui`.
///
/// Returns the entity that was clicked (if any), so the caller can update
/// the selection after the egui pass.
#[allow(clippy::too_many_arguments)]
pub fn show(
    ui: &mut egui::Ui,
    root_entities: &Query<Entity, (Without<ChildOf>, Or<(With<Body>, With<Frame>, With<Joint>, With<Muscle>, With<Site>, With<Coordinate>)>)>,
    names: &Query<&mut Name>,
    children_query: &Query<&Children>,
    joint_coords: &Query<&JointCoordinates>,
    model_markers: &Query<(
        Option<&Body>,
        Option<&Joint>,
        Option<&Coordinate>,
        Option<&Site>,
        Option<&Muscle>,
        Option<&Frame>,
    )>,
    selection: &Selection,
) -> Option<Entity> {
    let mut clicked: Option<Entity> = None;

    egui::ScrollArea::vertical().show(ui, |ui| {
        ui.add_space(4.0);
        ui.heading("Hierarchy");

        let mut roots: Vec<Entity> = root_entities.iter().collect();
        roots.sort_by(|a, b| {
            names
                .get(*a)
                .map(|n| n.as_str().to_owned())
                .unwrap_or_default()
                .cmp(
                    &names
                        .get(*b)
                        .map(|n| n.as_str().to_owned())
                        .unwrap_or_default(),
                )
        });

        for root in roots {
            hierarchy_node(
                ui,
                root,
                0,
                names,
                children_query,
                joint_coords,
                model_markers,
                selection,
                &mut clicked,
            );
        }
    });

    clicked
}

/// Recursively render a model entity as an expandable/selectable tree row.
/// Children come from Bevy's `ChildOf` hierarchy plus, for joints, the
/// `JointCoordinates` relationship so generalized coordinates are selectable.
#[allow(clippy::too_many_arguments)]
fn hierarchy_node(
    ui: &mut egui::Ui,
    entity: Entity,
    depth: usize,
    names: &Query<&mut Name>,
    children_query: &Query<&Children>,
    joint_coords: &Query<&JointCoordinates>,
    model_markers: &Query<(
        Option<&Body>,
        Option<&Joint>,
        Option<&Coordinate>,
        Option<&Site>,
        Option<&Muscle>,
        Option<&Frame>,
    )>,
    selection: &Selection,
    clicked: &mut Option<Entity>,
) {
    let name = names
        .get(entity)
        .map(|n| n.as_str().to_owned())
        .unwrap_or_else(|_| format!("{:?}", entity));

    let badge = match model_markers.get(entity) {
        Ok((Some(_), _, _, _, _, _)) => "B",
        Ok((_, Some(_), _, _, _, _)) => "J",
        Ok((_, _, Some(_), _, _, _)) => "C",
        Ok((_, _, _, Some(_), _, _)) => "S",
        Ok((_, _, _, _, Some(_), _)) => "M",
        Ok((_, _, _, _, _, Some(_))) => "F",
        _ => "",
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
        || joint_coords
            .get(entity)
            .map(|c| !c.is_empty())
            .unwrap_or(false);

    if has_children {
        let response = egui::CollapsingHeader::new(egui::RichText::new(label).color(text_color))
            .default_open(depth < 1)
            .id_salt(entity.index())
            .show(ui, |ui| {
                if let Ok(children) = children_query.get(entity) {
                    for child in children.iter() {
                        hierarchy_node(
                            ui,
                            child,
                            depth + 1,
                            names,
                            children_query,
                            joint_coords,
                            model_markers,
                            selection,
                            clicked,
                        );
                    }
                }
                if let Ok(coords) = joint_coords.get(entity) {
                    for coord in coords.iter() {
                        hierarchy_node(
                            ui,
                            coord,
                            depth + 1,
                            names,
                            children_query,
                            joint_coords,
                            model_markers,
                            selection,
                            clicked,
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
