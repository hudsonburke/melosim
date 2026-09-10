use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;

use crate::editor::popups::ActivePopup;
use crate::editor::selection::Selection;
use crate::model::{
    Body, CableParameters, Coordinate, CoordinateProperties, CoordinateState, Frame,
    HillTypeMuscleParams, InitialConditions, InertialProperties, Joint, Muscle, PathEntities, Site,
    Twist,
};

// ── Sub-panel functions ─────────────────────────────────────
//
// Each sub-panel is a plain function (not a system) called from the
// inspector panel system. This keeps concerns separated while
// avoiding Bevy's system-parameter limit.

/// Show the name sub-panel. Returns the selected entity (if any).
pub fn show_name(ui: &mut egui::Ui, selection: &Selection, names: &mut Query<&mut Name>) -> Option<Entity> {
    let entity = selection.primary()?;

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

    Some(entity)
}

/// Editable local transform inspector for frames and sites.
pub fn show_transform(ui: &mut egui::Ui, entity: Entity, q: &mut Query<&mut Transform>) {
    let Ok(mut transform) = q.get_mut(entity) else { return };

    ui.separator();
    ui.label(egui::RichText::new("Local transform").strong());
    ui.label("Translation (m)");
    ui.horizontal(|ui| {
        ui.add(egui::DragValue::new(&mut transform.translation.x).speed(0.001).suffix("x"));
        ui.add(egui::DragValue::new(&mut transform.translation.y).speed(0.001).suffix("y"));
        ui.add(egui::DragValue::new(&mut transform.translation.z).speed(0.001).suffix("z"));
    });
}

/// Editable inspector for a Body's `InertialProperties`.
pub fn show_inertial(ui: &mut egui::Ui, entity: Entity, q: &mut Query<&mut InertialProperties>) {
    let Ok(mut ip) = q.get_mut(entity) else { return };

    ui.separator();
    ui.label(egui::RichText::new("Inertial properties").strong());
    ui.add(
        egui::DragValue::new(&mut ip.mass)
            .speed(0.01)
            .suffix("mass (kg)"),
    );

    ui.label("mass_center (m)");
    ui.horizontal(|ui| {
        ui.add(
            egui::DragValue::new(&mut ip.mass_center.x)
                .speed(0.001)
                .suffix("x"),
        );
        ui.add(
            egui::DragValue::new(&mut ip.mass_center.y)
                .speed(0.001)
                .suffix("y"),
        );
        ui.add(
            egui::DragValue::new(&mut ip.mass_center.z)
                .speed(0.001)
                .suffix("z"),
        );
    });

    ui.label("inertia (kg·m²)");
    {
        let i = &mut ip.inertia.0;
        ui.horizontal(|ui| {
            ui.add(
                egui::DragValue::new(&mut i[0])
                    .speed(0.0001)
                    .suffix("Ixx"),
            );
            ui.add(
                egui::DragValue::new(&mut i[1])
                    .speed(0.0001)
                    .suffix("Iyy"),
            );
            ui.add(
                egui::DragValue::new(&mut i[2])
                    .speed(0.0001)
                    .suffix("Izz"),
            );
        });
        ui.horizontal(|ui| {
            ui.add(
                egui::DragValue::new(&mut i[3])
                    .speed(0.0001)
                    .suffix("Ixy"),
            );
            ui.add(
                egui::DragValue::new(&mut i[4])
                    .speed(0.0001)
                    .suffix("Ixz"),
            );
            ui.add(
                egui::DragValue::new(&mut i[5])
                    .speed(0.0001)
                    .suffix("Iyz"),
            );
        });
    }
}

/// Editable inspector for a joint's `Twist` (se(3) screw axis).
pub fn show_twist(ui: &mut egui::Ui, entity: Entity, q: &mut Query<&mut Twist>) {
    let Ok(mut tw) = q.get_mut(entity) else { return };

    ui.separator();
    ui.label(egui::RichText::new("Twist / screw axis").strong());
    ui.label("angular (rotation)");
    ui.horizontal(|ui| {
        ui.add(
            egui::DragValue::new(&mut tw.angular.x)
                .speed(0.01)
                .suffix("ax"),
        );
        ui.add(
            egui::DragValue::new(&mut tw.angular.y)
                .speed(0.01)
                .suffix("ay"),
        );
        ui.add(
            egui::DragValue::new(&mut tw.angular.z)
                .speed(0.01)
                .suffix("az"),
        );
    });
    ui.label("linear");
    ui.horizontal(|ui| {
        ui.add(
            egui::DragValue::new(&mut tw.linear.x)
                .speed(0.001)
                .suffix("lx"),
        );
        ui.add(
            egui::DragValue::new(&mut tw.linear.y)
                .speed(0.001)
                .suffix("ly"),
        );
        ui.add(
            egui::DragValue::new(&mut tw.linear.z)
                .speed(0.001)
                .suffix("lz"),
        );
    });
}

/// Editable inspector for a generalized coordinate: properties + initial + state.
pub fn show_coordinate(
    ui: &mut egui::Ui,
    entity: Entity,
    q: &mut Query<(
        &mut CoordinateProperties,
        &mut InitialConditions,
        &mut CoordinateState,
    )>,
) {
    let Ok((mut props, mut init, mut state)) = q.get_mut(entity) else {
        return;
    };

    ui.separator();
    ui.label(egui::RichText::new("Coordinate").strong());
    // Articulation: editing CoordinateState → FK re-poses (sync_kinematics).
    ui.add(
        egui::Slider::new(&mut state.value, props.range.0..=props.range.1).text("value"),
    );
    ui.add(egui::Slider::new(&mut state.velocity, -5.0..=5.0).text("velocity"));

    ui.add(
        egui::DragValue::new(&mut props.range.0)
            .speed(0.01)
            .suffix("min"),
    );
    ui.add(
        egui::DragValue::new(&mut props.range.1)
            .speed(0.01)
            .suffix("max"),
    );
    ui.checkbox(&mut props.clamped, "clamped");
    ui.checkbox(&mut props.locked, "locked");
    ui.add(
        egui::DragValue::new(&mut props.stiffness)
            .speed(0.1)
            .suffix("stiffness"),
    );
    ui.add(
        egui::DragValue::new(&mut props.damping)
            .speed(0.1)
            .suffix("damping"),
    );
    ui.add(
        egui::DragValue::new(&mut init.value)
            .speed(0.01)
            .suffix("default"),
    );
}

/// Editable inspector for a routed cable and its ordered path.
pub fn show_cable(
    ui: &mut egui::Ui,
    entity: Entity,
    selection: &Selection,
    active_popup: &mut ActivePopup,
    parameters: &mut Query<&mut CableParameters>,
    paths: &Query<&PathEntities>,
    names: &Query<&mut Name>,
) {
    let Ok(mut params) = parameters.get_mut(entity) else { return };
    let Ok(path) = paths.get(entity) else { return };

    ui.separator();
    ui.label(egui::RichText::new("Cable").strong());
    ui.add(egui::DragValue::new(&mut params.rest_length).speed(0.001).suffix("rest length (m)"));
    ui.add(egui::DragValue::new(&mut params.stiffness).speed(10.0).suffix("stiffness"));
    ui.add(egui::DragValue::new(&mut params.damping).speed(0.1).suffix("damping"));
    ui.add(egui::DragValue::new(&mut params.max_tension).speed(10.0).suffix("max tension (N)"));
    ui.add(egui::DragValue::new(&mut params.actuator_force).speed(10.0).suffix("actuator force (N)"));

    ui.label(format!("Path sites ({})", path.len()));
    for (index, site) in path.iter().enumerate() {
        let name = names
            .get(site)
            .map(|name| name.as_str().to_owned())
            .unwrap_or_else(|_| format!("{:?}", site));
        ui.label(format!("{}. {name}", index + 1));
    }
    if ui.button("＋ Add Path Site").clicked() && selection.primary() == Some(entity) {
        *active_popup = ActivePopup::AddPathSite;
    }
}

/// Editable inspector for `HillTypeMuscleParams`.
pub fn show_muscle(ui: &mut egui::Ui, entity: Entity, q: &mut Query<&mut HillTypeMuscleParams>) {
    let Ok(mut m) = q.get_mut(entity) else { return };

    ui.separator();
    ui.label(egui::RichText::new("Hill-type muscle").strong());
    ui.add(
        egui::DragValue::new(&mut m.max_isometric_force)
            .speed(1.0)
            .suffix("max force (N)"),
    );
    ui.add(
        egui::DragValue::new(&mut m.optimal_fiber_length)
            .speed(0.001)
            .suffix("fiber length (m)"),
    );
    ui.add(
        egui::DragValue::new(&mut m.tendon_slack_length)
            .speed(0.001)
            .suffix("tendon slack (m)"),
    );
    ui.add(
        egui::DragValue::new(&mut m.pennation_angle_at_optimal)
            .speed(0.01)
            .suffix("pennation (rad)"),
    );
    ui.add(
        egui::DragValue::new(&mut m.minimum_activation)
            .speed(0.01)
            .suffix("min activation"),
    );
    ui.add(
        egui::DragValue::new(&mut m.fiber_damping)
            .speed(0.01)
            .suffix("fiber damping"),
    );
}

/// Children sub-panel: list Sites & Frames under the selected entity, with
/// buttons to add new ones.
pub fn show_children(
    ui: &mut egui::Ui,
    entity: Entity,
    selection: &mut Selection,
    names: &mut Query<&mut Name>,
    children_query: &Query<&Children>,
    model_markers: &Query<(
        Option<&Body>,
        Option<&Joint>,
        Option<&Coordinate>,
        Option<&Site>,
        Option<&Muscle>,
        Option<&crate::model::Cable>,
        Option<&Frame>,
    )>,
    commands: &mut Commands,
    part_counter: &mut u64,
) {
    ui.separator();
    ui.label(egui::RichText::new("Children").strong());

    let mut sites: Vec<Entity> = Vec::new();
    let mut frames: Vec<Entity> = Vec::new();
    if let Ok(children) = children_query.get(entity) {
        for child in children.iter() {
            match model_markers.get(child) {
                Ok((_, _, _, Some(_), _, _, _)) => sites.push(child),
                Ok((_, _, _, _, _, _, Some(_))) => frames.push(child),
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
}
