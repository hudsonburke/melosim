use bevy::prelude::*;
use bevy::feathers::theme::ThemedText;
use bevy::ui::{Interaction, Selected};

use super::selection::Selection;
use super::toolbar::HierarchyVisible;

use crate::model::{Body, Frame, Joint, ModelEntity, Muscle, Site};

/// Marker for the hierarchy panel root.
#[derive(Component)]
pub struct HierarchyRoot;

/// Marker for hierarchy list rows so we can find which entity was clicked.
#[derive(Component)]
pub struct HierarchyRow(pub Entity);

/// Setup the hierarchy panel UI.
pub fn setup_hierarchy(mut commands: Commands) {
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

/// Update the hierarchy list when selection or visibility changes.
pub fn update_hierarchy(
    mut commands: Commands,
    selection: Res<Selection>,
    vis: Res<HierarchyVisible>,
    hierarchy_root: Query<Entity, With<HierarchyRoot>>,
    existing_rows: Query<Entity, With<HierarchyRow>>,
    names: Query<&Name>,
    root_entities: Query<Entity, (Without<ChildOf>, With<ModelEntity>)>,
    children_query: Query<&Children>,
    bodies: Query<&Body>,
    joints: Query<&Joint>,
    sites: Query<&Site>,
    frames: Query<&Frame>,
    muscles: Query<&Muscle>,
) {
    if !selection.is_changed() && !vis.is_changed() {
        return;
    }

    let Ok(root) = hierarchy_root.single() else {
        return;
    };

    // Remove old list content
    for entity in &existing_rows {
        commands.entity(entity).despawn();
    }

    // If hidden, just clear and return
    if !vis.0 {
        return;
    }

    // Collect all entities in tree order
    let mut entities = Vec::new();
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

    // Spawn list rows
    commands.entity(root).with_children(|parent| {
        // Header
        parent.spawn(Node {
            padding: UiRect::all(px(8)),
            ..default()
        }).with_children(|header| {
            header.spawn((Text::new("Hierarchy"), ThemedText));
        });

        // List rows
        for (entity, indent, label) in &entities {
            let is_selected = selection.is_selected(*entity);
            let mut row = parent.spawn((
                HierarchyRow(*entity),
                Node {
                    padding: UiRect::horizontal(px(8.0 + *indent as f32 * 16.0)),
                    ..default()
                },
                Interaction::None,
                bevy::picking::Pickable::default(),
            ));

            if is_selected {
                row.insert(Selected);
            }

            row.with_children(|row_child| {
                row_child.spawn((Text::new(label.clone()), ThemedText));
            });
        }
    });
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

    let label = if badge.is_empty() {
        name
    } else {
        format!("[{}] {}", badge, name)
    };

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

/// Handle clicks on hierarchy rows.
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
