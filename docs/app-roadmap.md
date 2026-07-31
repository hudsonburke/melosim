# melosim-app Roadmap

## Architecture

```
melosim-app/
├── src/
│   ├── main.rs           # eframe + three-d + egui setup, render loop
│   ├── render.rs         # Render trait, RenderSystem, render_all()
│   ├── state.rs          # Application state (World, selection, mode)
│   ├── camera.rs         # Orbit camera controls
│   ├── viewport.rs       # 3D scene rendering (meshes, lines, points)
│   ├── interaction.rs    # Click-to-select, hover, drag
│   ├── panels/
│   │   ├── mod.rs
│   │   ├── body.rs       # Body properties panel
│   │   ├── joint.rs      # Joint parameters panel
│   │   ├── muscle.rs     # Muscle editor panel
│   │   ├── import.rs     # Import dialog
│   │   └── export.rs     # Export dialog
│   └── tools/
│       ├── mod.rs
│       ├── attach.rs     # Exoskeleton attachment tool
│       ├── route.rs      # Cable routing tool
│       └── select.rs     # Selection tool
```

## Phase 1: Basic viewport ✅ Done

- [x] eframe + three-d + egui integration
- [x] Window with egui side panel
- [x] Dark background
- [x] Render loop structure

## Phase 2: Import and render from World

**Goal:** Load a model and see it in 3D.

### 2.1 Application state
- [ ] `State` struct holding World, camera, selection, mode
- [ ] Load MJCF on startup (from args or file dialog)
- [ ] Store World in State

### 2.2 Body world poses
- [ ] FK solver (positions only, no rotations yet)
- [ ] BFS from root bodies, accumulating positions
- [ ] Cache world poses in State (recompute on import/edit)

### 2.3 Mesh rendering
- [ ] Load STL files via `stl_io` crate
- [ ] Create three-d `CpuMesh` from vertices
- [ ] Apply body world position to mesh group
- [ ] Apply geom offset/rotation/scale
- [ ] Render with `PhysicalMaterial` (color from DisplayGeometry)

### 2.4 Line rendering
- [ ] Joint lines (between body_a and body_b origins)
- [ ] Muscle path lines (body-local points rotated to world)

### 2.5 Point rendering
- [ ] Site points (body-local offset rotated to world)

### 2.6 Camera
- [ ] Orbit camera with mouse controls
- [ ] Auto-fit to model bounds on import

**Estimated time:** 2-3 days

## Phase 3: Click-to-select

**Goal:** Click on a mesh to select the parent body.

### 3.1 Ray casting
- [ ] Three-d pick utility for mesh intersection
- [ ] Map hit mesh → parent body EntityID

### 3.2 Selection state
- [ ] `selected: Option<EntityID>` in State
- [ ] Highlight selected body (re-color mesh)
- [ ] Clear selection on empty click

### 3.3 Hover feedback
- [ ] Hover highlight (subtle color change)
- [ ] Cursor change on hover

**Estimated time:** 1 day

## Phase 4: Body properties panel

**Goal:** Show and edit properties of the selected body.

### 4.1 Display
- [ ] Body name
- [ ] Mass
- [ ] Center of mass
- [ ] Inertia tensor
- [ ] Parent body name
- [ ] Joint type and parameters

### 4.2 Editing
- [ ] Mass slider
- [ ] Name text field
- [ ] Joint limits (if selected body has a joint)

**Estimated time:** 1 day

## Phase 5: Import dialog

**Goal:** Import models via file dialog.

### 5.1 MJCF import
- [ ] File dialog (rfd crate) for .xml files
- [ ] Import via mujoco-rs (in-process)
- [ ] Update World, recompute poses
- [ ] Auto-fit camera

### 5.2 OSIM import
- [ ] File dialog for .json (extracted OpenSim)
- [ ] Import via serde_json + import_opensim_model()
- [ ] Same flow as MJCF

### 5.3 Mesh import
- [ ] File dialog for .stl/.obj files
- [ ] Create MeshGeometry entity
- [ ] Prompt for parent body (or attach to selected)

**Estimated time:** 1 day

## Phase 6: Attachment tool

**Goal:** Attach exoskeleton parts to bodies using anatomical landmarks.

### Design: Station-defined frames

Instead of manually entering offset + orientation, the user defines an attachment frame using stations (anatomical landmarks) on the body. The frame is computed from these points and updates automatically when the body scales.

**Why this approach:**
- Anatomically grounded — landmarks are meaningful, offsets are arbitrary
- Self-updating — scale the body, attachment follows automatically
- Composable — frames can reference other frames, building hierarchies
- Compatible with mesh warping — stations are points, warping operates on points

**Data model:**

```rust
// Station (already exists as Site component)
let asis = world.spawn();
world.attach(asis, Site { parent: pelvis, offset: Vec3::new(0.01, 0.02, 0.13) });
world.attach(asis, Name { value: "ASIS".into() });

// Station-defined frame computes transform from stations
#[derive(Clone)]
pub struct StationDefinedFrame {
    pub origin: EntityID,  // station for origin
    pub axis_x: EntityID,  // station defining X axis
    pub axis_y: EntityID,  // station defining Y axis
}

// FK system computes frame pose from stations
fn compute_station_frames(world: &mut World) {
    for (eid, sdf) in world.iter::<StationDefinedFrame>() {
        let origin = get_station_world_pos(world, sdf.origin);
        let px = get_station_world_pos(world, sdf.axis_x);
        let py = get_station_world_pos(world, sdf.axis_y);
        let transform = compute_frame_from_points(origin, px, py);
        // Store computed transform
    }
}
```

**Workflow:**
1. User clicks "Attach exoskeleton"
2. System prompts: "Define attachment frame using landmarks"
3. User clicks 3-4 points on the body (stations)
4. System computes frame from stations
5. Exoskeleton part attaches to this frame
6. When body scales → stations move → frame recomputes → attachment follows

### 6.1 Tool mode
- [ ] Tool selector in side panel (Select, Attach, Route)
- [ ] Attach mode: click parent body → define landmarks → place part

### 6.2 Landmark placement
- [ ] Click to add stations on body surfaces
- [ ] Show station markers (colored dots)
- [ ] Minimum 3 stations for frame computation
- [ ] Preview frame axes (RGB = XYZ)

### 6.3 Frame computation
- [ ] Compute origin from first station
- [ ] Compute X axis from origin → second station
- [ ] Compute Y axis from cross product (X × up vector)
- [ ] Orthogonalize to get Z axis
- [ ] Display frame at attachment point

### 6.4 Part placement
- [ ] Load STL/OBJ for exoskeleton part
- [ ] Align part to computed frame
- [ ] Preview ghost mesh at placement position
- [ ] Fine-tune offset within frame (optional)

### 6.5 Commit
- [ ] Create StationDefinedFrame entity
- [ ] Create Frame entity with computed transform
- [ ] Attach part entity to frame
- [ ] Undo support

**Estimated time:** 2-3 days

## Phase 7: Cable routing

**Goal:** Route cables through wrapping surfaces.

### 7.1 Waypoint placement
- [ ] Click to add waypoints on bodies
- [ ] Show path as line segments
- [ ] Snap to body surfaces

### 7.2 Wrapping computation
- [ ] Wrap around spheres (closest point on sphere surface)
- [ ] Wrap around cylinders (geodesic path)
- [ ] Wrap around ellipsoids (approximation)

### 7.3 Path visualization
- [ ] Render computed path as colored line
- [ ] Show wrap points on surfaces
- [ ] Animate path when editing

**Estimated time:** 3-5 days (core algorithm is complex)

## Phase 8: Export

**Goal:** Export the edited model.

### 8.1 MJCF export
- [ ] File dialog for save location
- [ ] Export via world_to_mjcf()
- [ ] Include all entities (bodies, joints, muscles, display)

### 8.2 OSIM export
- [ ] File dialog for save location
- [ ] Export via opensim exporter

### 8.3 Mesh export
- [ ] Save attached STL/OBJ files alongside model
- [ ] Update mesh paths in exported model

**Estimated time:** 1-2 days

## Phase 9: Muscle visualization

**Goal:** Show muscle paths and parameters.

### 9.1 Path rendering
- [ ] Render MusclePath as colored lines
- [ ] Show muscle attachment points
- [ ] Highlight muscle on hover

### 9.2 Muscle panel
- [ ] Show muscle parameters (force, fiber length, etc.)
- [ ] Edit parameters via sliders
- [ ] Visualize force-length curve

**Estimated time:** 1-2 days

## Phase 10: Refinements

### 10.1 Undo/redo
- [ ] Command stack for edits
- [ ] Ctrl+Z / Ctrl+Shift+Z

### 10.2 Grid and snapping
- [ ] Reference grid (already in three-d examples)
- [ ] Snap to grid option
- [ ] Snap to body surfaces

### 10.3 Keyboard shortcuts
- [ ] Delete selected
- [ ] Escape to deselect
- [ ] Tab to cycle selection

### 10.4 Labels
- [ ] Body name labels (three-d text rendering)
- [ ] Joint axis arrows
- [ ] Muscle names

### 10.5 Performance
- [ ] LOD for large models
- [ ] Frustum culling
- [ ] Instanced rendering for repeated meshes

**Estimated time:** 2-3 days

---

## Dependency graph

```
Phase 1 (basic viewport) ✅
    │
    ├── Phase 2 (import + render)
    │       │
    │       ├── Phase 3 (click-to-select)
    │       │       │
    │       │       ├── Phase 4 (body properties)
    │       │       │
    │       │       ├── Phase 6 (attachment tool)
    │       │       │
    │       │       └── Phase 7 (cable routing)
    │       │
    │       ├── Phase 5 (import dialog)
    │       │
    │       ├── Phase 8 (export)
    │       │
    │       └── Phase 9 (muscle visualization)
    │
    └── Phase 10 (refinements) — can happen anytime
```

## Estimated total: 12-18 days

- Phase 2-3: 3-4 days (core rendering + interaction)
- Phase 4-5: 2 days (UI panels + import)
- Phase 6-7: 5-7 days (tools — most complex)
- Phase 8-9: 2-3 days (export + muscles)
- Phase 10: 2-3 days (polish)

## Priority

**MVP (useful for editing):** Phases 1-6
- Import MJCF → see model → click bodies → attach parts

**Full workflow:** Phases 1-8
- Add cable routing and export

**Polish:** Phases 9-10
- Muscle visualization, undo/redo, shortcuts
