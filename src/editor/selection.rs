use bevy::prelude::*;

/// Tracks which entities are currently selected in the editor.
#[derive(Resource, Debug, Default)]
pub struct Selection {
    pub entities: Vec<Entity>,
}

/// Marker component placed on entities that are currently selected.
#[derive(Component, Debug)]
pub struct Selected;

impl Selection {
    pub fn select_single(&mut self, entity: Entity) {
        self.entities.clear();
        self.entities.push(entity);
    }

    pub fn toggle(&mut self, entity: Entity) {
        if let Some(pos) = self.entities.iter().position(|&e| e == entity) {
            self.entities.remove(pos);
        } else {
            self.entities.push(entity);
        }
    }

    pub fn clear(&mut self) {
        self.entities.clear();
    }

    pub fn primary(&self) -> Option<Entity> {
        self.entities.first().copied()
    }

    pub fn is_selected(&self, entity: Entity) -> bool {
        self.entities.contains(&entity)
    }
}

/// Syncs the `Selected` marker component with the `Selection` resource.
pub fn sync_selection_markers(
    selection: Res<Selection>,
    mut commands: Commands,
    selected_query: Query<Entity, With<Selected>>,
    all_entities: Query<Entity>,
) {
    if !selection.is_changed() {
        return;
    }

    // Remove Selected from entities that are no longer selected
    for entity in &selected_query {
        if !selection.is_selected(entity) {
            commands.entity(entity).remove::<Selected>();
        }
    }

    // Add Selected to newly selected entities
    for &entity in &selection.entities {
        if all_entities.get(entity).is_ok() {
            commands.entity(entity).insert(Selected);
        }
    }
}

/// Clears selection when Escape is pressed.
pub fn clear_selection_on_escape(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut selection: ResMut<Selection>,
) {
    if keyboard.just_pressed(KeyCode::Escape) {
        selection.clear();
    }
}
