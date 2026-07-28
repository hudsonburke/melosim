use crate::components::Name;
use crate::id::EntityID;
use crate::world::World;

/// Marker types for export target formats.
///
/// Each format gets a unit struct that acts as a type-level key.
/// Implement `ExportAs<Mjcf>` or `ExportAs<OsIm>` on your components.
pub struct Mjcf;
pub struct OsIm;

/// Context passed to component exporters during a format-specific export.
///
/// Holds shared state (name lookups, hierarchy info) so individual
/// component impls don't need to re-query the World.
pub struct ExportCtx<'a> {
    pub world: &'a World,
    /// Pre-built entity → name map (avoids repeated lookups)
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

/// The export contract. Implement this on component types to define
/// how they render into a specific target format.
///
/// `Output` is `Option<String>` for most components — `None` means
/// this component has no representation in the target format (skip it).
///
/// For components that need to emit into an existing XML context
/// (like joints that go inside a `<body>`), the string is an XML
/// snippet. For section-level components (like muscles in `<actuator>`),
/// the string is a complete element.
///
/// # Example
///
/// ```ignore
/// impl ExportAs<Mjcf> for HingeJoint {
///     type Output = String;
///
///     fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
///         let name = ctx.name_or_unnamed(entity);
///         let mut xml = format!(r#"<joint name="{}" type="hinge" axis="{} {} {}""#,
///             name, self.axis[0], self.axis[1], self.axis[2]);
///         if let Some(ref lim) = self.limits {
///             xml.push_str(&format!(r#" limited="true" range="{} {}""#, lim.lower, lim.upper));
///         }
///         xml.push_str("/>");
///         Some(xml)
///     }
/// }
/// ```
pub trait ExportAs<Format> {
    type Output;

    /// Render this component for the given format.
    ///
    /// `entity` is the entity this component is attached to — used for
    /// name lookups and cross-references.
    ///
    /// `ctx` provides access to the World and pre-built lookups.
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Self::Output;
}

/// Convenience: export a component if it exists on an entity.
///
/// ```ignore
/// if let Some(xml) = export_component::<HingeJoint, Mjcf>(world, entity, &ctx) {
///     elements.push(xml);
/// }
/// ```
pub fn export_component<C, F>(world: &World, entity: EntityID, ctx: &ExportCtx) -> Option<F::Output>
where
    C: ExportAs<F>,
    F: 'static,
{
    world.get::<C>(entity).and_then(|c| c.export_as(entity, ctx))
}

/// Escape special characters for XML attribute values.
pub fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
