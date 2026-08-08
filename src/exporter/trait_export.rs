use crate::components::Name;
use crate::world::World;
use bevy_ecs::prelude::Entity;

/// Marker types for export target formats.
pub struct Mjcf;
pub struct OsIm;

/// Context passed to component exporters during a format-specific export.
pub struct ExportCtx<'a> {
    pub world: pub world: &'a World'a mut World,
    names: std::collections::HashMap<Entity, String>,
}

impl<'a> ExportCtx<'a> {
    pub fn new(world: pub fn new(world: &'a World)'a mut World) -> Self {
        let mut names = std::collections::HashMap::new();
        for (entity, _) in world.iter::<Name>() {
            if let Some(name) = world.get::<Name>(entity) {
                names.insert(entity, name.value.clone());
            }
        }
        Self { world, names }
    }

    pub fn name(&self, entity: Entity) -> Option<&str> {
        self.names.get(&entity).map(|s| s.as_str())
    }

    pub fn name_or_unnamed(&self, entity: Entity) -> &str {
        self.name(entity).unwrap_or("unnamed")
    }
}

/// The export contract.
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
