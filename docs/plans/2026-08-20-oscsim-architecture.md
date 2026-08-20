# melosim UI Architecture — OpenSimCreator-inspired Refactor

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Reshape melosim's editor UI to match OpenSimCreator's clean panel architecture while keeping Bevy ECS as the model layer.

**Architecture:** Extract the monolithic `editor/ui.rs` into individual panel modules using `egui_dock` for dockable panel management (following Chiron's proven pattern), and add event-driven cross-panel communication. The model layer (`src/model/`) stays as-is — it's already strong.

**Tech Stack:** Bevy 0.19, egui (via bevy_egui), egui_dock, bevy_picking, bevy_gizmos

---

## Overview

OpenSimCreator's key UI patterns we're adopting, with Chiron's egui_dock integration:

1. **One panel = one module** — each panel is a self-contained file implementing the `egui_dock::TabViewer` trait
2. **egui_dock DockState** — replaces hand-rolled PanelManager; gives drag-and-drop docking, split views, tab management for free
3. **Event-based communication** — panels post events, don't call each other
4. **Popup/dialog system** — modals for add-body, add-joint, import, etc.
5. **Document/editor separation** — model actions live apart from UI code

### Why egui_dock over hand-rolled PanelManager

Chiron (biomechanics-foundation/chiron) already proved this pattern works for biomechanics UIs. egui_dock provides:
- Drag-and-drop panel rearrangement
- Split views (side-by-side hierarchy + inspector)
- Tab management (close, rename, reorder)
- Persistent layout serialization
- Multiple viewport support (spawnable viewers)

The `DockState<Tab>` resource replaces the custom `PanelManager`. Each panel type is a variant of a `Tab` enum, and a `TabViewer` bridges Bevy World into egui rendering.

## Phase 1: Extract Panels from ui.rs

Split the 415-line `editor/ui.rs` into separate panel files. Each panel becomes a standalone function in its own module.

### Task 1.1: Create panel module structure

**Objective:** Set up the `editor/panels/` directory with empty module files.

**Files:**
- Create: `src/editor/panels/mod.rs`
- Create: `src/editor/panels/toolbar.rs`
- Create: `src/editor/panels/hierarchy.rs`
- Create: `src/editor/panels/inspector.rs`

**Step 1:** Create `src/editor/panels/mod.rs`:
```rust
pub mod toolbar;
pub mod hierarchy;
pub mod inspector;
```

**Step 2:** Update `src/editor/mod.rs` to include the panels module:
```rust
pub mod panels;
```

**Step 3:** Verify it compiles:
```bash
nix develop --command cargo check
```

**Step 4:** Commit:
```bash
git add src/editor/panels/ src/editor/mod.rs
git commit -m "chore: add panels module structure"
```

### Task 1.2: Extract toolbar panel

**Objective:** Move the toolbar code from `ui.rs` into `panels/toolbar.rs`.

**Files:**
- Create: `src/editor/panels/toolbar.rs`
- Modify: `src/editor/ui.rs` (remove toolbar code, call toolbar::show)

**Step 1:** Create `src/editor/panels/toolbar.rs` with the toolbar function extracted from `ui.rs` lines 54-91.

**Step 2:** In `ui.rs`, replace the toolbar block with `toolbar::show(ctx, &selection, &registry, &mut selected_model, &mut settings, &names);`

**Step 3:** Verify compile + visual check.

**Step 4:** Commit.

### Task 1.3: Extract hierarchy panel

**Objective:** Move hierarchy code into `panels/hierarchy.rs`.

**Files:**
- Create: `src/editor/panels/hierarchy.rs`
- Modify: `src/editor/ui.rs`

**Step 1:** Create `src/editor/panels/hierarchy.rs` with `pub fn show(...)` containing the hierarchy SidePanel code + `hierarchy_node` helper.

**Step 2:** Update `ui.rs` to call `hierarchy::show(...)`.

**Step 3:** Verify compile + visual check.

**Step 4:** Commit.

### Task 1.4: Extract inspector panel

**Objective:** Move inspector code into `panels/inspector.rs`.

**Files:**
- Create: `src/editor/panels/inspector.rs`
- Modify: `src/editor/ui.rs`

**Step 1:** Create `src/editor/panels/inspector.rs` with `pub fn show(...)` containing the inspector SidePanel code + `edit_inertial`, `edit_twist`, `edit_coordinate`, `edit_muscle` helpers.

**Step 2:** Update `ui.rs` to call `inspector::show(...)`.

**Step 3:** Verify compile + visual check.

**Step 4:** Commit.

### Task 1.5: Slim down ui.rs to orchestrator

**Objective:** `ui.rs` should only wire panels together, not contain panel logic.

**Files:**
- Modify: `src/editor/ui.rs`

**Step 1:** Verify `ui.rs` is now ~30-50 lines: just imports, the `editor_ui` function signature, and calls to `toolbar::show`, `hierarchy::show`, `inspector::show`.

**Step 2:** Verify compile + visual check.

**Step 3:** Commit.

---

## Phase 2: egui_dock Integration

Replace the hand-rolled panel visibility tracking with `egui_dock::DockState`. Each panel becomes a `Tab` variant with a `TabViewer` implementation.

### Task 2.1: Add egui_dock dependency

**Objective:** Add `egui_dock` to Cargo.toml.

**Files:**
- Modify: `Cargo.toml`

**Step 1:** Add to dependencies:
```toml
egui_dock = "0.15"
```

**Step 2:** Verify it compiles:
```bash
nix develop --command cargo check
```

**Step 3:** Commit.

### Task 2.2: Create Tab enum and DockState resource

**Objective:** Define the panel types and initial dock layout.

**Files:**
- Create: `src/editor/dock.rs`
- Modify: `src/editor/mod.rs`

**Step 1:** Create `src/editor/dock.rs`:
```rust
use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use egui_dock::{DockState, TabViewer};

/// All panel types in the editor. Each variant maps to a panel module.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Tab {
    Hierarchy,
    Inspector,
    Viewport,
    MusclePlot,
    Log,
    OutputWatches,
    CoordinateEditor,
}

/// Resource wrapping the dock state. Initialized with a default layout.
#[derive(Resource)]
pub struct EditorDockState {
    pub state: DockState<Tab>,
}

impl Default for EditorDockState {
    fn default() -> Self {
        let mut state = DockState::new(Tab::Viewport);
        // Left: Hierarchy tab
        let tree = state.main_surface_mut();
        let [viewport, left] = tree.split_left(
            egui_dock::NodeIndex::root(),
            0.25,
            vec![Tab::Hierarchy],
        );
        // Right: Inspector tab
        let [viewport, right] = tree.split_right(
            viewport,
            0.75,
            vec![Tab::Inspector],
        );
        // Bottom: Log tab
        let [_viewport, bottom] = tree.split_bottom(
            viewport,
            0.3,
            vec![Tab::Log],
        );
        Self { state }
    }
}

/// Bridges Bevy World into egui_dock tab rendering.
pub struct MelosimTabViewer<'a> {
    pub world: &'a mut World,
    pub egui_ctx: &'a egui::Context,
}

impl<'a> TabViewer for MelosimTabViewer<'a> {
    type Tab = Tab;

    fn ui(&mut self, tab: &mut Self::Tab, ui: &mut egui::Ui) {
        match tab {
            Tab::Hierarchy => crate::editor::panels::hierarchy::show(ui, self.world),
            Tab::Inspector => crate::editor::panels::inspector::show(ui, self.world),
            Tab::Viewport => {} // Viewport is rendered by Bevy, not egui
            Tab::MusclePlot => {} // TODO: muscle plot panel
            Tab::Log => {} // TODO: log panel
            Tab::OutputWatches => {} // TODO: output watches
            Tab::CoordinateEditor => {} // TODO: coordinate editor
        }
    }

    fn title(&mut self, tab: &mut Self::Tab) -> egui::WidgetText {
        match tab {
            Tab::Hierarchy => "Hierarchy".into(),
            Tab::Inspector => "Inspector".into(),
            Tab::Viewport => "Viewport".into(),
            Tab::MusclePlot => "Muscle Plot".into(),
            Tab::Log => "Log".into(),
            Tab::OutputWatches => "Output Watches".into(),
            Tab::CoordinateEditor => "Coordinates".into(),
        }
    }
}
```

**Step 2:** Register in `mod.rs`:
```rust
pub mod dock;
app.init_resource::<dock::EditorDockState>();
```

**Step 3:** Commit.

### Task 2.3: Wire dock into editor_ui

**Objective:** Replace the manual SidePanel/TopBottomPanel calls with DockArea rendering.

**Files:**
- Modify: `src/editor/ui.rs`

**Step 1:** Replace the panel rendering code with:
```rust
use egui_dock::DockArea;

pub fn editor_ui(
    mut contexts: EguiContexts,
    mut dock_state: ResMut<dock::EditorDockState>,
    // ... other params for toolbar only
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");

    // Toolbar stays outside the dock (top panel)
    toolbar::show(ctx, /* ... */);

    // Dock area fills the remaining space
    let mut tab_viewer = dock::MelosimTabViewer { world: /* ... */, egui_ctx: ctx };
    DockArea::new(&mut dock_state.state)
        .show(ctx, &mut tab_viewer);
}
```

**Step 2:** Update each panel's `show()` to take `&mut egui::Ui` instead of `egui::Context` (egui_dock gives you a `&mut Ui`).

**Step 3:** Verify compile + visual check — panels should now be dockable.

**Step 4:** Commit.

### Task 2.4: Add View menu panel toggles

**Objective:** The View menu can show/hide panels by adding/removing tabs from the dock.

**Files:**
- Modify: `src/editor/panels/toolbar.rs`

**Step 1:** Add a "Windows" submenu that toggles tabs in the dock state:
```rust
ui.menu_button("Windows", |ui| {
    for tab in [Tab::Hierarchy, Tab::Inspector, Tab::Log, Tab::MusclePlot] {
        let is_open = dock_state.state.has_tab(&tab);
        if ui.selectable_label(is_open, format!("{:?}", tab)).clicked() {
            if is_open {
                dock_state.state.remove_tab(&tab);
            } else {
                // Add to appropriate location
            }
        }
    }
});
```

**Step 2:** Commit.

---

## Phase 3: Event-Based Communication

Replace direct resource reads between panels with Bevy Events.

### Task 3.1: Define editor events

**Objective:** Create an events module with all cross-panel signals.

**Files:**
- Create: `src/editor/events.rs`
- Modify: `src/editor/mod.rs`

**Step 1:** Create `src/editor/events.rs`:
```rust
use bevy::prelude::*;

/// Posted when the user selects an entity (from hierarchy, viewport, or inspector).
#[derive(Event)]
pub struct SelectionChanged {
    pub entity: Option<Entity>,
}

/// Posted when the user requests a context menu on an entity.
#[derive(Event)]
pub struct ContextMenuRequest {
    pub entity: Entity,
    pub screen_pos: egui::Pos2,
}

/// Posted when a model mutation occurs (for undo/redo tracking).
#[derive(Event)]
pub struct ModelAction {
    pub description: String,
}
```

**Step 2:** Register events in `mod.rs`: `app.add_event::<SelectionChanged>()` etc.

**Step 3:** Commit.

### Task 3.2: Replace Selection resource with events

**Objective:** Selection changes go through events, not direct ResMut.

**Files:**
- Modify: `src/editor/panels/hierarchy.rs`
- Modify: `src/editor/viewport.rs`
- Modify: `src/editor/selection.rs`

**Step 1:** Hierarchy posts `SelectionChanged` instead of writing `selection.select_single()`.

**Step 2:** A central system reads `SelectionChanged` and updates the `Selection` resource.

**Step 3:** Commit.

---

## Phase 4: Popup/Dialog System

Add reusable popup dialogs following OSC's pattern.

### Task 4.1: Create popup infrastructure

**Objective:** A PopupManager resource and base popup trait.

**Files:**
- Create: `src/editor/popups/mod.rs`
- Create: `src/editor/popups/add_body.rs`
- Modify: `src/editor/mod.rs`

**Step 1:** Create `src/editor/popups/mod.rs` with a `PopupManager` resource that holds a `Vec<Box<dyn Popup>>`.

**Step 2:** Create `src/editor/popups/add_body.rs` — a simple "Add Body" dialog with name input and parent selector.

**Step 3:** Register in `mod.rs`.

**Step 4:** Add "Add Body" to the Model menu in the toolbar.

**Step 5:** Commit.

### Task 4.2: Add body/scene/skeleton creation popup

**Objective:** Dialog for creating new model components (bodies, joints, muscles).

**Files:**
- Create: `src/editor/popups/add_joint.rs`
- Create: `src/editor/popups/add_muscle.rs`

**Step 1:** Create add_joint popup — parent body selector, joint type dropdown, coordinate setup.

**Step 2:** Create add_muscle popup — name, parent body, max force, fiber length.

**Step 3:** Wire to Model menu.

**Step 4:** Commit.

---

## Phase 5: Document/Editor Separation

### Task 5.1: Create actions module

**Objective:** Move model mutation logic out of UI code into reusable action functions.

**Files:**
- Create: `src/editor/actions.rs`
- Modify: `src/editor/panels/inspector.rs`

**Step 1:** Create `src/editor/actions.rs` with functions like:
```rust
pub fn add_site_to_body(commands: &mut Commands, body: Entity, counter: &mut u64) -> Entity { ... }
pub fn add_frame_to_body(commands: &mut Commands, body: Entity, counter: &mut u64) -> Entity { ... }
pub fn rename_entity(names: &mut Query<&mut Name>, entity: Entity, new_name: &str) { ... }
```

**Step 2:** Update inspector to call these instead of inline logic.

**Step 3:** Commit.

---

## Phase 6: Advanced Features

### Task 6.1: File change polling (live reload)

**Objective:** Watch imported model files and reload on change.

**Files:**
- Create: `src/editor/file_watcher.rs`
- Modify: `src/editor/mod.rs`

**Step 1:** Create a `FileWatcher` resource that tracks imported file paths and modification times.

**Step 2:** Add a system that polls file mtimes and triggers re-import when changed.

**Step 3:** Commit.

### Task 6.2: Navigator search/filter

**Objective:** Add search filtering to the hierarchy panel (like OSC's NavigatorPanel).

**Files:**
- Modify: `src/editor/panels/hierarchy.rs`

**Step 1:** Add a text input at the top of the hierarchy panel.

**Step 2:** Filter tree nodes by name match.

**Step 3:** Commit.

### Task 6.3: Component context menu

**Objective:** Right-click on hierarchy nodes or 3D entities to get a context menu.

**Files:**
- Create: `src/editor/popups/context_menu.rs`
- Modify: `src/editor/panels/hierarchy.rs`

**Step 1:** Create context menu with: Rename, Delete, Add Child (Site/Frame), Properties.

**Step 2:** Wire to hierarchy right-click and viewport right-click.

**Step 3:** Commit.

---

## Verification Checklist

After each phase:
- [ ] `nix develop --command cargo check` passes
- [ ] `nix develop --command cargo run` launches editor
- [ ] Model loads from registry menu
- [ ] Hierarchy shows correct tree
- [ ] Inspector edits propagate (coordinate slider → FK update)
- [ ] 3D selection works (click in viewport)
- [ ] Transform gizmo appears on selection
- [ ] View menu toggles work
- [ ] No visual regressions
