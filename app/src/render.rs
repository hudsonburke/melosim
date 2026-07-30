// ── Render infrastructure ──────────────────────────────
//
// Components implement the `Render` trait to define how they
// draw themselves. Render systems register via `inventory::submit!`.
//
// To add rendering for a component:
//   1. impl Render for YourComponent { ... }
//   2. inventory::submit! { RenderSystem::new("render_your", render_all::<YourComponent>) }

use melosim::components::*;
use melosim::id::EntityID;
use melosim::world::World;

/// Per-frame rendering state passed to all render systems.
pub struct FrameState<'a> {
    pub context: &'a three_d::Context,
    pub camera: &'a three_d::Camera,
    pub screen: (u32, u32),
}

/// A system that renders components to the screen.
pub struct RenderSystem {
    pub name: &'static str,
    pub render: fn(&World, &FrameState),
}

impl RenderSystem {
    pub const fn new(name: &'static str, render: fn(&World, &FrameState)) -> Self {
        Self { name, render }
    }
}

inventory::collect!(RenderSystem);

/// Run all registered render systems.
pub fn render_world(world: &World, state: &FrameState) {
    for system in inventory::iter::<RenderSystem> {
        (system.render)(world, state);
    }
}

// ── Render trait ───────────────────────────────────────

/// Components implement this trait to define how they render.
/// This is optional — render systems can iterate components directly
/// without the trait, but the trait enables generic `render_all::<T>()`.
pub trait Render {
    fn render(&self, entity: EntityID, world: &World, state: &FrameState);
}

/// Generic render helper: iterate all instances of T, call render on each.
/// Use this as the system function for a component type:
///
/// ```ignore
/// inventory::submit! {
///     RenderSystem::new("render_meshes", render_all::<DisplayGeometry>)
/// }
/// ```
pub fn render_all<T: Render + 'static>(world: &World, state: &FrameState) {
    for (entity, component) in world.iter::<T>() {
        component.render(entity, world, state);
    }
}
