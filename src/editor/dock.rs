use bevy::prelude::*;
use egui_dock::{DockState, NodeIndex};

/// Identifies the panels that can appear as docked tabs.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Tab {
    Hierarchy,
    Inspector,
}

/// Persistent dock layout state, stored as a Bevy [`Resource`].
///
/// Currently unused for rendering (panels use manual SidePanel layout),
/// but provides the infrastructure for future egui_dock integration.
#[derive(Resource)]
pub struct EditorDockState {
    pub state: DockState<Tab>,
}

impl Default for EditorDockState {
    fn default() -> Self {
        let mut state = DockState::new(vec![Tab::Inspector]);
        let [_hierarchy_node, _inspector_node] =
            state.main_surface_mut().split_left(
                NodeIndex::root(),
                0.3,
                vec![Tab::Hierarchy],
            );
        Self { state }
    }
}
