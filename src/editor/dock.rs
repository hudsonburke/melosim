//! Editor dock infrastructure.
//!
//! egui_dock is a dependency for future dockable-panel support, but cannot
//! be wired into the render loop yet because Bevy's query lifetimes
//! (`'w, 's`) are incompatible with `egui_dock::TabViewer` (single `'a`).
//!
//! Panels currently render via `Panel::top/left/right` in separate systems.
//! When egui_dock supports a two-lifetime `TabViewer` (or Bevy adds a
//! bridge), this module provides the `Tab` enum and layout defaults to
//! wire in.

// egui_dock is in Cargo.toml but not imported here yet —
// the Tab enum and DockState will be added when the bridge is built.
