use bevy::prelude::*;
use bevy::feathers::theme::ThemedText;

use super::selection::Selection;
use super::toolbar::InspectorVisible;

use crate::model::*;

/// Marker for the inspector panel root.
#[derive(Component)]
pub struct InspectorRoot;

/// Marker for inspector content that gets rebuilt.
#[derive(Component)]
pub struct InspectorContent;

/// Resource that holds the data to display in the inspector.
#[derive(Resource, Default)]
pub struct InspectorData {
    pub entity: Option<Entity>,
    pub name: Option<String>,
    pub badges: Vec<&'static str>,
    pub transform: Option<(Vec3, Quat, Vec3)>,
    pub mass: Option<f64>,
    pub com: Option<(f64, f64, f64)>,
    pub coord_range: Option<(f64, f64)>,
    pub stiffness: Option<f64>,
    pub damping: Option<f64>,
    pub twist_angular: Option<(f64, f64, f64)>,
    pub twist_linear: Option<(f64, f64, f64)>,
    pub hill_force: Option<f64>,
    pub hill_fiber: Option<f64>,
    pub path_count: Option<usize>,
}

/// Setup the inspector panel UI.
pub fn setup_inspector(mut commands: Commands) {
    commands.insert_resource(InspectorData::default());
    commands.spawn((
        InspectorRoot,
        Node {
            width: px(300),
            height: percent(100),
            flex_direction: FlexDirection::Column,
            border: UiRect::left(px(1)),
            ..default()
        },
        BackgroundColor(bevy::color::palettes::css::DARK_GRAY.into()),
    ));
}

/// System 1: Gather data from ECS into InspectorData resource.
pub fn gather_inspector_data(
    selection: Res<Selection>,
    mut data: ResMut<InspectorData>,
    names: Query<&Name>,
    transforms: Query<&Transform>,
    bodies: Query<&Body>,
    joints: Query<&Joint>,
    sites: Query<&Site>,
    frames: Query<&Frame>,
    muscles: Query<&Muscle>,
    inertial: Query<&InertialProperties>,
    coord_props: Query<&CoordinateProperties>,
    twists: Query<&Twist>,
    hill_params: Query<&HillTypeMuscleParams>,
    path_entities: Query<&PathEntities>,
) {
    if !selection.is_changed() {
        return;
    }

    let Some(entity) = selection.primary() else {
        *data = InspectorData::default();
        return;
    };

    data.entity = Some(entity);
    data.name = names.get(entity).ok().map(|n| n.as_str().to_owned());

    data.badges.clear();
    if bodies.get(entity).is_ok() { data.badges.push("Body"); }
    if joints.get(entity).is_ok() { data.badges.push("Joint"); }
    if sites.get(entity).is_ok() { data.badges.push("Site"); }
    if frames.get(entity).is_ok() { data.badges.push("Frame"); }
    if muscles.get(entity).is_ok() { data.badges.push("Muscle"); }

    data.transform = transforms.get(entity).ok().map(|t| (t.translation, t.rotation, t.scale));
    data.mass = inertial.get(entity).ok().map(|p| p.mass);
    data.com = inertial.get(entity).ok().map(|p| (p.mass_center.x, p.mass_center.y, p.mass_center.z));
    data.coord_range = coord_props.get(entity).ok().map(|cp| (cp.range.0, cp.range.1));
    data.stiffness = coord_props.get(entity).ok().map(|cp| cp.stiffness);
    data.damping = coord_props.get(entity).ok().map(|cp| cp.damping);
    data.twist_angular = twists.get(entity).ok().map(|tw| (tw.angular.x, tw.angular.y, tw.angular.z));
    data.twist_linear = twists.get(entity).ok().map(|tw| (tw.linear.x, tw.linear.y, tw.linear.z));
    data.hill_force = hill_params.get(entity).ok().map(|hp| hp.max_isometric_force);
    data.hill_fiber = hill_params.get(entity).ok().map(|hp| hp.optimal_fiber_length);
    data.path_count = path_entities.get(entity).ok().map(|pe| pe.len());
}

/// System 2: Render inspector UI from InspectorData resource.
pub fn render_inspector(
    mut commands: Commands,
    data: Res<InspectorData>,
    vis: Res<InspectorVisible>,
    inspector_root: Query<Entity, With<InspectorRoot>>,
    existing_content: Query<Entity, With<InspectorContent>>,
) {
    if !data.is_changed() && !vis.is_changed() {
        return;
    }

    let Ok(root) = inspector_root.single() else {
        return;
    };

    // Remove old content
    for entity in &existing_content {
        commands.entity(entity).despawn();
    }

    if !vis.0 || data.entity.is_none() {
        return;
    }

    commands.entity(root).with_children(|parent| {
        parent.spawn((InspectorContent, Node {
            flex_direction: FlexDirection::Column,
            ..default()
        })).with_children(|content| {
            if let Some(ref name) = data.name {
                spawn_row(content, format!("Entity: {}", name));
            }

            if !data.badges.is_empty() {
                spawn_row(content, format!("Type: {}", data.badges.join(", ")));
            }

            if let Some((t, r, _s)) = data.transform {
                spawn_row(content, "Transform".to_string());
                spawn_indent(content, format!("T: ({:.4}, {:.4}, {:.4})", t.x, t.y, t.z));
                spawn_indent(content, format!("R: ({:.4}, {:.4}, {:.4}, {:.4})", r.x, r.y, r.z, r.w));
            }

            if let Some(mass) = data.mass {
                spawn_row(content, "Inertial Properties".to_string());
                spawn_indent(content, format!("Mass: {:.6} kg", mass));
            }
            if let Some((x, y, z)) = data.com {
                spawn_indent(content, format!("CoM: ({:.4}, {:.4}, {:.4})", x, y, z));
            }

            if let Some((min, max)) = data.coord_range {
                spawn_row(content, "Coordinate Properties".to_string());
                spawn_indent(content, format!("Range: ({:.4}, {:.4})", min, max));
            }
            if let Some(s) = data.stiffness {
                spawn_indent(content, format!("Stiffness: {:.4}", s));
            }
            if let Some(d) = data.damping {
                spawn_indent(content, format!("Damping: {:.4}", d));
            }

            if let Some((x, y, z)) = data.twist_angular {
                spawn_row(content, "Twist".to_string());
                spawn_indent(content, format!("Angular: ({:.4}, {:.4}, {:.4})", x, y, z));
            }
            if let Some((x, y, z)) = data.twist_linear {
                spawn_indent(content, format!("Linear: ({:.4}, {:.4}, {:.4})", x, y, z));
            }

            if let Some(force) = data.hill_force {
                spawn_row(content, "Hill-Type Muscle Params".to_string());
                spawn_indent(content, format!("Max force: {:.2} N", force));
            }
            if let Some(fiber) = data.hill_fiber {
                spawn_indent(content, format!("Fiber length: {:.6} m", fiber));
            }

            if let Some(count) = data.path_count {
                spawn_row(content, format!("Path: {} entities", count));
            }
        });
    });
}

fn spawn_row(parent: &mut ChildSpawnerCommands, text: String) {
    parent.spawn(Node { padding: UiRect::horizontal(px(8)), ..default() })
        .with_children(|row| {
            row.spawn((Text::new(text), ThemedText));
        });
}

fn spawn_indent(parent: &mut ChildSpawnerCommands, text: String) {
    parent.spawn(Node { padding: UiRect::left(px(16)), ..default() })
        .with_children(|row| {
            row.spawn((Text::new(text), ThemedText));
        });
}
