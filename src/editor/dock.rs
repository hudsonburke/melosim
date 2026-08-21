//! Editor dock infrastructure.
//!
//! `EditorDockState` stores the dock layout configuration. Panels currently
//! render via `Panel::top/left/right` in separate systems. DockArea wiring
//! is deferred — Bevy's exclusive system API provides `QueryState` (not
//! `&Query`), which is incompatible with the panel functions' signatures.
//!
//! When bevy_egui surfaces `show_inside()` or the panel functions are
//! refactored to accept `&mut World`, DockArea can be wired in.

use bevy::prelude::*;
use egui_dock::{DockState, NodeIndex};

/// All panel types in the editor.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Tab {
    Hierarchy,
    Inspector,
}

/// Dock state stored as a Bevy resource.
#[derive(Resource)]
pub struct EditorDockState {
    pub state: DockState<Tab>,
}

impl Default for EditorDockState {
    fn default() -> Self {
        let mut state = DockState::new(vec![Tab::Hierarchy]);
        let [_hierarchy, inspector] = state
            .main_surface_mut()
            .split_right(NodeIndex::root(), 0.4, vec![Tab::Inspector]);
        Self { state }
    }
}
