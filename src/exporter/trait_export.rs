use crate::components::*;
use crate::world::World;
use crate::world::WorldExt;
use bevy_ecs::prelude::Entity;
use std::collections::HashMap;

/// Marker types for export target formats.
pub struct Mjcf;
pub struct OsIm;

/// Context passed to component exporters during a format-specific export.
/// Pre-computes all data needed by exporters — no world reference held.
pub struct ExportCtx {
    names: HashMap<Entity, String>,
    /// Pre-computed muscle params indexed by muscle entity.
    muscle_params: HashMap<Entity, Millard2012Params>,
    /// Pre-computed muscle paths indexed by muscle entity.
    muscle_paths: HashMap<Entity, MusclePath>,
}

impl ExportCtx {
    pub fn new(world: &mut World) -> Self {
        let mut names = HashMap::new();
        for (entity, _) in world.iter::<Name>() {
            if let Some(name) = world.get::<Name>(entity) {
                names.insert(entity, name.value.clone());
            }
        }
        let mut muscle_params = HashMap::new();
        for (entity, params) in world.iter::<Millard2012Params>() {
            muscle_params.insert(entity, params);
        }
        let mut muscle_paths = HashMap::new();
        for (entity, path) in world.iter::<MusclePath>() {
            muscle_paths.insert(entity, path);
        }
        Self { names, muscle_params, muscle_paths }
    }

    pub fn name(&self, entity: Entity) -> Option<&str> {
        self.names.get(&entity).map(|s| s.as_str())
    }

    pub fn name_or_unnamed(&self, entity: Entity) -> &str {
        self.name(entity).unwrap_or("unnamed")
    }

    pub fn muscle_params(&self, entity: Entity) -> Option<&Millard2012Params> {
        self.muscle_params.get(&entity)
    }

    pub fn muscle_path(&self, entity: Entity) -> Option<&MusclePath> {
        self.muscle_paths.get(&entity)
    }
}

/// The export contract. No world reference — use ctx for pre-computed data.
pub trait ExportAs<Format> {
    fn export_as(&self, entity: Entity, ctx: &ExportCtx) -> Option<String>;
}

pub fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
