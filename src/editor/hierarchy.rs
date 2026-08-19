use bevy::prelude::*;
use bevy::feathers::theme::ThemedText;

use super::selection::Selection;
use super::toolbar::HierarchyVisible;

use crate::model::{Body, Frame, Joint, Muscle, Site};

/// Marker for the hierarchy panel root.
#[derive(Component)]
pub struct HierarchyRoot;

/// Marker for hierarchy list rows so we can find which entity was clicked.
#[derive(Component)]
pub struct HierarchyRow(pub Entity);

/// Set true to force a full rebuild of the tree rows. This fires only when the
/// model's entity *structure* changes (e.g. a part is instantiated, a joint
/// added) — NOT on selection, which is handled reactively by
/// `update_hierarchy_selection` so picking never tears down the list.
#[derive(Resource, Default)]
pub struct HierarchyDirty(pub bool);

const ROW_BG: Color = Color::srgba(0.0, 0.0, 0.0, 0.35);
const SELECTED_BG: Color = Color::srgb(0.18, 0.28, 0.45);

/// Setup the hierarchy panel UI.
pub fn setup_hierarchy(mut commands: Commands) {
    commands.insert_resource(HierarchyDirty::default());
    commands.spawn((
        HierarchyRoot,
        Node {
            width: px(250),
            height: percent(100),
            flex_direction: FlexDirection::Column,
            border: UiRect::right(px(1)),
            ..default()
        },
        BackgroundColor(bevy::color::palettes::css::DARK_GRAY.into()),
    ));
}

/// Rebuild the row list only when the structure is dirty or visibility toggled.
/// Selection changes do NOT reach this system, so clicking a row never rebuilds.
pub fn rebuild_hierarchy(
    mut commands: Commands,
    mut dirty: ResMut<HierarchyDirty>,
    vis: Res<HierarchyVisible>,
    hierarchy_root: Query<Entity, With<HierarchyRoot>>,
    existing_rows: Query<Entity, With<HierarchyRow>>,
    names: Query<&Name>,
    root_entities: Query<Entity, (Without<ChildOf>, Or<(With<Body>, With<Frame>, With<Joint>, With<Muscle>, With<Site>)>)>,
    children_query: Query<&Children>,
    bodies: Query<&Body>,
    joints: Query<&Joint>,
    sites: Query<&Site>,
    frames: Query<&Frame>,
    muscles: Query<&Muscle>,
) {
    if !dirty.0 && !vis.is_changed() {
        return;
    }
    dirty.0 = false;

    let Ok(root) = hierarchy_root.single() else {
        return;
    };

    // Tear down the current list (structure change only).
    for entity in &existing_rows {
        commands.entity(entity).despawn();
    }

    // Rebuild children of the root.
    commands.entity(root).with_children(|parent| {
        // Header
        parent.spawn(Node {
            padding: UiRect::all(px(6)),
            ..default()
        }).with_children(|header| {
            header.spawn((Text::new("Hierarchy"), ThemedText));
        });

        if !vis.0 {
            return;
        }

        // Collect model entities in tree order.
        let mut entities: Vec<(Entity, usize, String)> = Vec::new();
        let mut roots: Vec<Entity> = root_entities.iter().collect();
        roots.sort_by(|a, b| {
            let na = names.get(*a).map(|n| n.as_str().to_owned()).unwrap_or_default();
            let nb = names.get(*b).map(|n| n.as_str().to_owned()).unwrap_or_default();
            na.cmp(&nb)
        });
        for &root_entity in &roots {
            collect_tree(
                root_entity,
                0,
                &mut entities,
                &names,
                &children_query,
                &bodies,
                &joints,
                &sites,
                &frames,
                &muscles,
            );
        }

        // Spawn one row per model entity. Highlight is applied in-place by
        // `update_hierarchy_selection`; it is NOT baked in here.
        for (entity, indent, label) in &entities {
            parent.spawn((
                HierarchyRow(*entity),
                Node {
                    padding: UiRect::horizontal(px(8.0 + *indent as f32 * 14.0)),
                    ..default()
                },
                BackgroundColor(ROW_BG),
                bevy::picking::Pickable::default(),
            )).with_children(|row| {
                row.spawn((Text::new(label.clone()), ThemedText));
            });
        }
    });
}

/// Reactively tint selected rows — no spawning or despawning.
pub fn update_hierarchy_selection(
    selection: Res<Selection>,
    mut rows: Query<(&HierarchyRow, &mut BackgroundColor)>,
) {
    for (row, mut color) in &mut rows {
        *color = BackgroundColor(if selection.is_selected(row.0) { SELECTED_BG } else { ROW_BG });
    }
}

/// Recursively collect entities for the hierarchy tree.
fn collect_tree(
    entity: Entity,
    depth: usize,
    out: &mut Vec<(Entity, usize, String)>,
    names: &Query<&Name>,
    children_query: &Query<&Children>,
    bodies: &Query<&Body>,
    joints: &Query<&Joint>,
    sites: &Query<&Site>,
    frames: &Query<&Frame>,
    muscles: &Query<&Muscle>,
) {
    let name = names.get(entity).map(|n| n.as_str().to_owned()).unwrap_or_else(|_| format!("{:?}", entity));
    let badge = if bodies.get(entity).is_ok() { "B" }
        else if joints.get(entity).is_ok() { "J" }
        else if sites.get(entity).is_ok() { "S" }
        else if muscles.get(entity).is_ok() { "M" }
        else if frames.get(entity).is_ok() { "F" }
        else { "" };

    let label = if badge.is_empty() { name } else { format!("[{}] {}", badge, name) };
    out.push((entity, depth, label));

    if let Ok(children) = children_query.get(entity) {
        for child in children.iter() {
            collect_tree(
                child,
                depth + 1,
                out,
                names,
                children_query,
                bodies,
                joints,
                sites,
                frames,
                muscles,
            );
        }
    }
}

/// Handle clicks on hierarchy rows. Registered as an observer so clicking a row
/// selects its model entity.
pub fn on_hierarchy_row_click(
    trigger: On<Pointer<Click>>,
    hierarchy_rows: Query<&HierarchyRow>,
    mut selection: ResMut<Selection>,
) {
    let target = trigger.original_event_target();
    if let Ok(row) = hierarchy_rows.get(target) {
        selection.select_single(row.0);
    }
}
