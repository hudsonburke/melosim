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

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::models::{ModelRegistry, SelectedModel};
use super::selection::Selection;
use super::PendingMujocoExport;
use crate::model::{
    Body, Coordinate, CoordinateProperties, CoordinateState, Frame, HillTypeMuscleParams,
    InitialConditions, InertialProperties, Joint, JointCoordinates, Muscle, Site, Twist,
};
use crate::render::RenderSettings;

/// Editor shell UI: toolbar + left hierarchy of model entities, right inspector.
#[allow(deprecated, clippy::too_many_arguments)]
pub fn editor_ui(
    mut contexts: EguiContexts,
    mut commands: Commands,
    mut selection: ResMut<Selection>,
    registry: Res<ModelRegistry>,
    mut selected_model: ResMut<SelectedModel>,
    mut settings: ResMut<RenderSettings>,
    mut part_counter: Local<u64>,
    mut names: Query<&mut Name>,
    root_entities: Query<Entity, (Without<ChildOf>, Or<(With<Body>, With<Frame>, With<Joint>, With<Muscle>, With<Site>, With<Coordinate>)>)>,
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
            ui.separator();
            // Load a model from the registry (despawns the previous one).
            ui.menu_button("Model", |ui| {
                for (i, def) in registry.0.iter().enumerate() {
                    if ui.button(def.name).clicked() {
                        selected_model.0 = Some(i);
                        ui.close_menu();
                    }
                }
            });
            ui.separator();
            ui.menu_button("View", |ui| {
                ui.checkbox(&mut settings.meshes, "Meshes");
                ui.checkbox(&mut settings.bodies, "Bodies");
                ui.checkbox(&mut settings.frames, "Frames");
                ui.checkbox(&mut settings.sites, "Sites");
                ui.checkbox(&mut settings.muscles, "Muscles");
                ui.checkbox(&mut settings.joint_axes, "Joint Axes");
            });
            ui.separator();

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
                        &model_markers,
                        &selection,
                        &mut clicked,
                    );
                }
            });
        });

    // ── Inspector (right): per-component editors ─────────────────────
    egui::SidePanel::right("melosim_inspector")
        .resizable(true)
        .default_width(320.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical().show(ui, |ui| {
                ui.add_space(4.0);
                ui.heading("Inspector");

                let Some(entity) = selection.primary() else {
                    ui.label("Nothing selected");
                    return;
                };
                // Editable name (rename any named model entity).
                let mut name = names
                    .get(entity)
                    .map(|n| n.as_str().to_owned())
                    .unwrap_or_else(|_| format!("{:?}", entity));
                if names.contains(entity) {
                    ui.horizontal(|ui| {
                        ui.label("Name");
                        if ui.text_edit_singleline(&mut name).changed() {
                            if let Ok(mut n) = names.get_mut(entity) {
                                *n = Name::new(name);
                            }
                        }
                    });
                } else {
                    ui.label(egui::RichText::new(name).strong());
                }

                edit_inertial(ui, entity, &mut inertial);
                edit_twist(ui, entity, &mut twists);
                edit_coordinate(ui, entity, &mut coord_editor);
                edit_muscle(ui, entity, &mut hill);

                // ── Children (Layer 1 part authoring): immediate Sites & Frames ──
                ui.separator();
                ui.label(egui::RichText::new("Children").strong());

                let mut sites: Vec<Entity> = Vec::new();
                let mut frames: Vec<Entity> = Vec::new();
                if let Ok(children) = children_query.get(entity) {
                    for child in children.iter() {
                        match model_markers.get(child) {
                            Ok((_, _, _, Some(_), _, _)) => sites.push(child),
                            Ok((_, _, _, _, _, Some(_))) => frames.push(child),
                            _ => {}
                        }
                    }
                }

                ui.label("Sites");
                if ui.button("＋ Add Site").clicked() {
                    let name = format!("site_{}", *part_counter);
                    *part_counter += 1;
                    let child = commands
                        .spawn((Name::new(name), Site, Transform::from_xyz(0.03, 0.0, 0.0)))
                        .insert(ChildOf(entity))
                        .id();
                    commands.entity(entity).add_children(&[child]);
                }
                for child in &sites {
                    let cname = names
                        .get(*child)
                        .map(|n| n.as_str().to_owned())
                        .unwrap_or_default();
                    if ui.selectable_label(selection.is_selected(*child), cname).clicked() {
                        selection.select_single(*child);
                    }
                }

                ui.add_space(4.0);
                ui.label("Frames");
                if ui.button("＋ Add Frame").clicked() {
                    let name = format!("frame_{}", *part_counter);
                    *part_counter += 1;
                    let child = commands
                        .spawn((Name::new(name), Frame, Transform::from_xyz(0.03, 0.0, 0.0)))
                        .insert(ChildOf(entity))
                        .id();
                    commands.entity(entity).add_children(&[child]);
                }
                for child in &frames {
                    let cname = names
                        .get(*child)
                        .map(|n| n.as_str().to_owned())
                        .unwrap_or_default();
                    if ui.selectable_label(selection.is_selected(*child), cname).clicked() {
                        selection.select_single(*child);
                    }
                }
            });
        });

    // Apply a hierarchy-click selection after the egui pass.
    if let Some(entity) = clicked {
        selection.select_single(entity);
    }
}

/// Editable inspector for a Body's `InertialProperties`.
fn edit_inertial(ui: &mut egui::Ui, entity: Entity, q: &mut Query<&mut InertialProperties>) {
    let Ok(mut ip) = q.get_mut(entity) else { return; };

    ui.separator();
    ui.label(egui::RichText::new("Inertial properties").strong());
    ui.add(egui::DragValue::new(&mut ip.mass).speed(0.01).suffix("mass (kg)"));

    ui.label("mass_center (m)");
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut ip.mass_center.x).speed(0.001).suffix("x"));
        ui.add(egui::DragValue::new(&mut ip.mass_center.y).speed(0.001).suffix("y"));
        ui.add(egui::DragValue::new(&mut ip.mass_center.z).speed(0.001).suffix("z"));
    });

    ui.label("inertia (kg·m²)");
    {
        let i = &mut ip.inertia.0;
        ui.horizontal(|ui| {
            ui.add(egui::DragValue::new(&mut i[0]).speed(0.0001).suffix("Ixx"));
            ui.add(egui::DragValue::new(&mut i[1]).speed(0.0001).suffix("Iyy"));
            ui.add(egui::DragValue::new(&mut i[2]).speed(0.0001).suffix("Izz"));
        });
        ui.horizontal(|ui| {
            ui.add(egui::DragValue::new(&mut i[3]).speed(0.0001).suffix("Ixy"));
            ui.add(egui::DragValue::new(&mut i[4]).speed(0.0001).suffix("Ixz"));
            ui.add(egui::DragValue::new(&mut i[5]).speed(0.0001).suffix("Iyz"));
        });
    }
}

/// Editable inspector for a joint's `Twist` (se(3) screw axis).
fn edit_twist(ui: &mut egui::Ui, entity: Entity, q: &mut Query<&mut Twist>) {
    let Ok(mut tw) = q.get_mut(entity) else { return; };

    ui.separator();
    ui.label(egui::RichText::new("Twist / screw axis").strong());
    ui.label("angular (rotation)");
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut tw.angular.x).speed(0.01).suffix("ax"));
        ui.add(egui::DragValue::new(&mut tw.angular.y).speed(0.01).suffix("ay"));
        ui.add(egui::DragValue::new(&mut tw.angular.z).speed(0.01).suffix("az"));
    });
    ui.label("linear");
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut tw.linear.x).speed(0.001).suffix("lx"));
        ui.add(egui::DragValue::new(&mut tw.linear.y).speed(0.001).suffix("ly"));
        ui.add(egui::DragValue::new(&mut tw.linear.z).speed(0.001).suffix("lz"));
    });
}

/// Editable inspector for a generalized coordinate: properties + initial + state.
fn edit_coordinate(
    ui: &mut egui::Ui,
    entity: Entity,
    q: &mut Query<(&mut CoordinateProperties, &mut InitialConditions, &mut CoordinateState)>,
) {
    let Ok((mut props, mut init, mut state)) = q.get_mut(entity) else { return; };

    ui.separator();
    ui.label(egui::RichText::new("Coordinate").strong());
    // Articulation: editing CoordinateState → FK re-poses (sync_kinematics).
    ui.add(egui::Slider::new(&mut state.value, props.range.0..=props.range.1).text("value"));
    ui.add(egui::Slider::new(&mut state.velocity, -5.0..=5.0).text("velocity"));

    ui.add(egui::DragValue::new(&mut props.range.0).speed(0.01).suffix("min"));
    ui.add(egui::DragValue::new(&mut props.range.1).speed(0.01).suffix("max"));
    ui.checkbox(&mut props.clamped, "clamped");
    ui.checkbox(&mut props.locked, "locked");
    ui.add(egui::DragValue::new(&mut props.stiffness).speed(0.1).suffix("stiffness"));
    ui.add(egui::DragValue::new(&mut props.damping).speed(0.1).suffix("damping"));
    ui.add(egui::DragValue::new(&mut init.value).speed(0.01).suffix("default"));
}

/// Editable inspector for `HillTypeMuscleParams`.
fn edit_muscle(ui: &mut egui::Ui, entity: Entity, q: &mut Query<&mut HillTypeMuscleParams>) {
    let Ok(mut m) = q.get_mut(entity) else { return; };

    ui.separator();
    ui.label(egui::RichText::new("Hill-type muscle").strong());
    ui.add(egui::DragValue::new(&mut m.max_isometric_force).speed(1.0).suffix("max force (N)"));
    ui.add(egui::DragValue::new(&mut m.optimal_fiber_length).speed(0.001).suffix("fiber length (m)"));
    ui.add(egui::DragValue::new(&mut m.tendon_slack_length).speed(0.001).suffix("tendon slack (m)"));
    ui.add(egui::DragValue::new(&mut m.pennation_angle_at_optimal).speed(0.01).suffix("pennation (rad)"));
    ui.add(egui::DragValue::new(&mut m.minimum_activation).speed(0.01).suffix("min activation"));
    ui.add(egui::DragValue::new(&mut m.fiber_damping).speed(0.01).suffix("fiber damping"));
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
        || joint_coords.get(entity).map(|c| !c.is_empty()).unwrap_or(false);

    if has_children {
        let response = egui::CollapsingHeader::new(egui::RichText::new(label).color(text_color))
            .default_open(depth < 1)
            .id_salt(entity.index())
            .show(ui, |ui| {
                if let Ok(children) = children_query.get(entity) {
                    for child in children.iter() {
                        hierarchy_node(
                            ui, child, depth + 1, names, children_query, joint_coords,
                            model_markers, selection, clicked,
                        );
                    }
                }
                if let Ok(coords) = joint_coords.get(entity) {
                    for coord in coords.iter() {
                        hierarchy_node(
                            ui, coord, depth + 1, names, children_query, joint_coords,
                            model_markers, selection, clicked,
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
