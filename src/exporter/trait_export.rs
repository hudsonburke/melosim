use crate::components::Name;
use crate::id::EntityID;
use crate::world::World;

/// Marker types for export target formats.
pub struct Mjcf;
pub struct OsIm;

/// Context passed to component exporters during a format-specific export.
pub struct ExportCtx<'a> {
    pub world: &'a World,
    names: std::collections::HashMap<EntityID, String>,
}

impl<'a> ExportCtx<'a> {
    pub fn new(world: &'a World) -> Self {
        let mut names = std::collections::HashMap::new();
        for (entity, _) in world.iter::<Name>() {
            if let Some(name) = world.get::<Name>(entity) {
                names.insert(entity, name.value.clone());
            }
        }
        Self { world, names }
    }

    pub fn name(&self, entity: EntityID) -> Option<&str> {
        self.names.get(&entity).map(|s| s.as_str())
    }

    pub fn name_or_unnamed(&self, entity: EntityID) -> &str {
        self.name(entity).unwrap_or("unnamed")
    }
}

/// The export contract. Implement on component types to define how they
/// render into a specific target format.
///
/// Returns `Some(xml_snippet)` if the component has a representation,
/// or `None` if it should be skipped.
pub trait ExportAs<Format> {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String>;
}

/// Escape special characters for XML attribute values.
pub fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
