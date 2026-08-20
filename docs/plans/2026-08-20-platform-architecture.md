# melosim Platform Architecture Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Build melosim into a platform for interacting with subject-specific musculoskeletal models — featuring an OSC-inspired editor UI, SOMA-X integration for markerless motion capture input, and native model scaling for exoskeleton design.

**Architecture:** Three pillars — (1) egui_dock-based editor with clean panel separation, (2) SOMA-X as the subject data layer (mesh visualization + motion input + anthropometric measurements), (3) native scaling as pure Rust functions on World.

**Tech Stack:** Bevy 0.19, egui (via bevy_egui), egui_dock, bevy_picking, bevy_gizmos, nalgebra

---

## Overview

This plan covers the full platform vision discussed in the 2026-08-20 thread:

1. **UI Architecture** — OSC-inspired editor with egui_dock (Phases 1-6)
2. **SOMA-X Integration** — parametric body mesh + markerless mocap input (Phase 7)
3. **Native Scaling** — melosim-owned model scaling replacing OpenSim's Scale Tool (Phase 8)

### Design Decisions (from this thread)

- **egui_dock over hand-rolled PanelManager** — Chiron proved this works for biomechanics UIs
- **SOMA is the I/O format, not the kinematic model** — biomechanical models define kinematics; SOMA provides visualization and motion input
- **Scaling is a plain function** — `scale_model(world, profile)` operates on World directly, no Bevy scheduling needed
- **Skinned mesh via Bevy's native support** — Bevy handles LBS in the vertex shader; we provide joint hierarchy + skinning weights + inverse bind poses
- **mannequin/Chiron as reference only** — both projects are abandoned; useful for patterns but not dependencies

---

## Phase 1: Extract Panels from ui.rs

Split the 415-line `editor/ui.rs` into separate panel files.

### Task 1.1: Create panel module structure
- Create: `src/editor/panels/mod.rs`, `toolbar.rs`, `hierarchy.rs`, `inspector.rs`
- Update `src/editor/mod.rs` to include panels module
- Verify: `nix develop --command cargo check`

### Task 1.2: Extract toolbar panel
- Move toolbar code (ui.rs lines 54-91) into `panels/toolbar.rs`
- `toolbar::show(ctx, &selection, &registry, &mut selected_model, &mut settings, &names)`

### Task 1.3: Extract hierarchy panel
- Move hierarchy SidePanel + `hierarchy_node` helper into `panels/hierarchy.rs`

### Task 1.4: Extract inspector panel
- Move inspector SidePanel + `edit_inertial`, `edit_twist`, `edit_coordinate`, `edit_muscle` into `panels/inspector.rs`

### Task 1.5: Slim down ui.rs to orchestrator
- `ui.rs` becomes ~30-50 lines wiring panels together

---

## Phase 2: egui_dock Integration

Replace manual SidePanel/TopBottomPanel with `egui_dock::DockState`.

### Task 2.1: Add egui_dock dependency
- `egui_dock = "0.15"` in Cargo.toml

### Task 2.2: Create Tab enum and DockState resource
- Create `src/editor/dock.rs` with `Tab` enum (Hierarchy, Inspector, Viewport, MusclePlot, Log, etc.)
- `EditorDockState` resource with default layout (left: hierarchy, right: inspector, center: viewport, bottom: log)
- `MelosimTabViewer` implements `TabViewer` trait, dispatches to panel modules

### Task 2.3: Wire dock into editor_ui
- Replace manual panel calls with `DockArea::new(&mut dock_state.state).show(ctx, &mut tab_viewer)`
- Update panel `show()` to take `&mut egui::Ui`

### Task 2.4: Add View menu panel toggles
- Windows submenu toggles tabs in dock state

---

## Phase 3: Event-Based Communication

### Task 3.1: Define editor events
- `SelectionChanged { entity: Option<Entity> }`
- `ContextMenuRequest { entity, screen_pos }`
- `ModelAction { description }`

### Task 3.2: Replace Selection resource with events
- Hierarchy posts `SelectionChanged` instead of writing `selection.select_single()`
- Central system reads events and updates `Selection` resource

---

## Phase 4: Popup/Dialog System

### Task 4.1: Create popup infrastructure
- `PopupManager` resource + base popup trait
- `add_body.rs` — dialog with name input and parent selector

### Task 4.2: Add body/joint/muscle creation popups
- `add_joint.rs` — parent body selector, joint type dropdown
- `add_muscle.rs` — name, parent body, max force, fiber length

---

## Phase 5: Document/Editor Separation

### Task 5.1: Create actions module
- `src/editor/actions.rs` with `add_site_to_body()`, `add_frame_to_body()`, `rename_entity()`
- Inspector calls actions instead of inline logic

---

## Phase 6: Advanced Features

### Task 6.1: File change polling (live reload)
- `FileWatcher` resource + polling system

### Task 6.2: Navigator search/filter
- Text input at top of hierarchy panel, filter tree nodes by name

### Task 6.3: Component context menu
- Right-click on hierarchy/viewport → Rename, Delete, Add Child, Properties

---

## Phase 7: SOMA-X Integration

Import SOMA's canonical body mesh, skeleton, and skinning weights. Use as subject data layer for markerless motion capture input and visualization.

### Task 7.1: Add SOMA-X dependencies
- `usd-rs` or custom USDA parser for rig file
- `ndarray` or `numpy` (via PyO3) for NPZ reading

### Task 7.2: Parse SOMA template rig (USDA)
- Create `src/importer/soma.rs`
- Parse `SOMA_template_rig.usda` → extract:
  - Joint hierarchy (78 joints, parent indices)
  - Bind pose transforms (J, 4, 4)
  - Skinning weights (V × J CSC sparse → dense)
  - Rest mesh vertices (V, 3) in centimeters

### Task 7.3: Parse SOMA neutral shape (NPZ)
- Parse `SOMA_neutral.npz` → extract:
  - Mean vertices (V, 3)
  - PCA shape directions (128, V*3)
  - Eigenvalues (128,)
  - Triangle faces (T, 3)

### Task 7.4: Create SOMA components
```rust
/// Skinned mesh with skeleton reference.
#[derive(Component)]
pub struct SkinnedMesh {
    pub skeleton: Entity,
    pub inverse_bindposes: Vec<Mat4>,
}

/// Marker for a SOMA joint entity.
#[derive(Component)]
pub struct SomaJoint {
    pub index: usize,
    pub name: String,
}
```

### Task 7.5: SOMA importer function
```rust
pub fn import_soma(
    world: &mut World,
    rig_path: &Path,
    neutral_path: &Path,
    identity_coeffs: Option<&[f32]>,  // optional: shape subject
) -> Entity  // returns skeleton root entity
```
Spawns: skeleton hierarchy (ChildOf chain), joint entities with Transform, SkinnedMesh entity with Bevy Mesh + skinning weights.

### Task 7.6: Subject profile extraction
```rust
pub fn soma_to_profile(
    identity_coeffs: &[f32],
    scale_params: Option<&[f32]>,
) -> SubjectProfile
```
Extracts anthropometric measurements from SOMA identity parameters.

### Task 7.7: SOMA editor panel
- New `Tab::SomaViewer` in the dock
- Displays SOMA mesh with current identity
- Slider for identity coefficients (body shape control)
- Button to extract subject profile for scaling

### Task 7.8: Retargeting interface (stub)
- Define `RetargetMap` struct: SOMA joint → biomech joint correspondences
- Implement basic retargeting: given SOMA joint angles, find biomech joint angles that match body positions
- This is a future full implementation; stub shows the interface

---

## Phase 8: Native Scaling

Replace OpenSim's Scale Tool with melosim-owned scaling as pure Rust functions.

### Task 8.1: SubjectProfile component
```rust
#[derive(Component, Clone, Debug)]
pub struct SubjectProfile {
    pub mass: f32,
    pub height: f32,
    pub segment_scales: HashMap<Name, SegmentScale>,
}

#[derive(Clone, Debug)]
pub struct SegmentScale {
    pub length: f32,   // subject / generic
    pub mass: f32,     // subject / generic
}
```

### Task 8.2: Geometric scaling function
```rust
pub fn scale_geometry(world: &mut World, profile: &SubjectProfile)
```
- Scale joint translations (child offset from parent)
- Scale mesh vertex positions
- Scale marker/site positions
- Scale wrap surface positions and radii

### Task 8.3: Inertial property regression
```rust
pub fn hicks_regression(
    segment_length: f32,
    segment_mass: f32,
    body_mass: f32,
    segment_name: &str,
) -> InertialProperties
```
- COM location (% of segment length from proximal end)
- Radius of gyration (longitudinal + transverse axes)
- Mass, moments of inertia from regression coefficients

### Task 8.4: Muscle parameter scaling
```rust
pub fn scale_muscles(world: &mut World, profile: &SubjectProfile)
```
- Optimal fiber length ∝ segment length
- Tendon slack length ∝ segment length
- Maximum isometric force ∝ PCSA (length²)
- Pennation angle: unchanged (subject-specific if available)

### Task 8.5: Path and wrap surface scaling
```rust
pub fn scale_paths(world: &mut World, profile: &SubjectProfile)
```
- Scale PathPoint positions by segment factor
- Scale WrappingSurface radii by segment factor

### Task 8.6: Top-level scale function
```rust
pub fn scale_model(world: &mut World, profile: &SubjectProfile) {
    scale_geometry(world, profile);
    // Apply Hicks regression to all bodies
    for (entity, mut inertia) in world.query::<&mut InertialProperties>() { ... }
    scale_muscles(world, profile);
    scale_paths(world, profile);
}
```
Called from editor button or CLI — plain function, no Bevy scheduling.

### Task 8.7: Scaling UI panel
- New `Tab::Scaling` in the dock
- Load subject profile from file or SOMA
- Show before/after measurements
- Apply button triggers `scale_model()`

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

After Phase 7:
- [ ] SOMA mesh loads and displays in viewport
- [ ] Skeleton hierarchy shows in hierarchy panel
- [ ] Identity coefficient sliders change body shape
- [ ] Subject profile extraction produces reasonable measurements

After Phase 8:
- [ ] Scale Rajagopal model to subject measurements
- [ ] Verify segment lengths match target
- [ ] Verify inertial properties updated via regression
- [ ] Verify muscle parameters scaled correctly
- [ ] Export scaled model to OpenSim format
