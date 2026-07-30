# melosim — Design Document

## Guiding Philosophy

**A neutral data model for neuromusculoskeletal simulation.** melosim is not a simulator — it's a common language that can translate between OpenSim, MuJoCo, and other tools. The value is in the data model, not in reimplementing physics.

**Data-model-first architecture.** The type system is the foundation. Rust enforces invariants at compile time. The ECS pattern separates data from behavior. Systems are functions that read and write components.

**Composition over inheritance.** Entities are composed of components. A muscle entity has MusclePath + Millard2012Params. An exo part entity has Frame + InertialProperties + Joint. Add components to give entities new capabilities, remove components to take them away.

**Runtime validation, not just compile-time.** The type system prevents most invalid states. But entity references (does body 42 exist?) are runtime checks. The `Validate` trait and `run_systems()` catch what the compiler can't.

**No monolithic components.** Decompose by access pattern. A body is not a single component — it's an entity with InertialProperties + Frame + maybe MeshGeometry + maybe DisplayGeometry. A muscle is not a single component — it's an entity with MusclePath + Millard2012Params. Each component serves one system.

**Open for extension.** Plugins register systems via `inventory::submit!` at link time. Adding a new component type, validation rule, or system requires no changes to melosim core.

## ECS Architecture

### Entity IDs

- `EntityID(pub u32)` — dense integer, direct index into `Vec<Option<T>>` storage
- Created by `World::spawn()` (monotonic counter)
- Used as foreign keys in component fields (e.g., `HingeJoint.body_a: EntityID`)
- No generational safety — entities are never deleted during the build phase. Validation catches stale references.

### Storage

Component storage uses `AnyMap<Vec<Option<T>>>` — each component type T lives in its own `Vec<Option<T>>` indexed by EntityID, type-erased behind AnyMap:

```rust
pub struct World {
    pub components: AnyMap,  // stores Vec<Option<T>> for each T
    pub resources: AnyMap,
    pub next_id: u32,
}
```

This is a variant of the Catherine West pattern (RustConf 2018): type-erased storage with typed access. Adding a new component type does NOT require modifying the World struct — downstream crates just call `world.attach::<MyType>(entity, val)`.

**Why Vec<Option<T>> instead of SlotMap:** SlotMap gave opaque keys with generation counters, but the dense Vec pattern allows direct indexing (`vec[id.0 as usize]`) without translation. The cost is no generational safety, but entities are never deleted during the build phase.

### Spawn vs Attach

**Entities and components are separate operations.** `spawn()` allocates an EntityID. `attach()` places a component on that entity.

```rust
let body = world.spawn();  // returns EntityID
world.attach(body, InertialProperties { mass: 11.78, com: [0.0; 3], inertia: [0.18, 0.22, 0.20, 0.0, 0.0, 0.0] });

let joint = world.spawn();
world.attach(joint, HingeJoint { body_a: body, body_b: other_body, axis: [1.0, 0.0, 0.0] });
```

Component references between entities use explicit EntityID fields (HingeJoint.body_a, Frame.parent, etc.). Each entity has exactly one spawn and components are attached by reference — not by index alignment.

### Core Components

| Component | Fields | Read by |
|---|---|---|
| `Name` | value (String) | Import/export, logging, debugging |
| `InertialProperties` | mass, com, inertia | Rigid body solver |
| `Frame` | parent: EntityID, transform | All systems (parent-relative transforms) |
| `Site` | parent: EntityID, offset | Marker/anatomical landmarks |
| `Material` | density, youngs_modulus, poissons_ratio | FEM solver |
| `DisplayGeometry` | body, mesh_file, scale, color, opacity, transform | Visualization, export |
| `MeshGeometry` | mesh (path) | Visualization, export |

### Primitive Geometries

| Component | Fields | Purpose |
|---|---|---|
| `Sphere` | radius | Collision/display sphere |
| `Cylinder` | radius, half_height | Collision/display cylinder |
| `Capsule` | radius, half_height | Collision/display capsule |
| `BoxGeom` | half_extents [f64; 3] | Collision/display box |
| `Plane` | (unit struct) | Ground plane |

### Joint Types

Each joint type is a standalone component. Adding a new joint type: define the component struct, write a system function, register via `inventory::submit!`. No changes to existing code.

| Component | Fields | Purpose |
|---|---|---|
| `HingeJoint` | body_a, body_b, limits, axis | Single-axis rotation |
| `SlideJoint` | body_a, body_b, limits, axis | Single-axis translation |
| `BallJoint` | body_a, body_b, limits | 3-DOF rotation |
| `FreeJoint` | body_a, body_b, limits | 6-DOF (free body) |
| `FixedJoint` | body_a, body_b, limits | No relative motion |
| `UniversalJoint` | body_a, body_b, limits, axis1, axis2 | 2-DOF rotation |
| `CustomJoint` | body_a, body_b, limits, coordinates: Vec\<EntityID\> | N-DOF via SpatialTransform |

**Why separate components instead of an enum?** An enum is a closed set — adding a variant requires modifying the enum definition and every match statement. Separate component types are an open set — a downstream crate can define a `PrismaticJoint` without touching melosim's source code.

### Coordinate System

The coordinate system models generalized coordinates and their effect on joint transforms. This is the core of OpenSim's `CustomJoint` — without it, coupled joint motion (like knee flexion driving tibial translation) cannot be represented.

Coordinates are **separate entities** (not inlined into joints). This allows independent iteration — a system can find all locked coordinates without touching every joint.

| Component | Fields | Purpose |
|---|---|---|
| `JointCoordinate` | range_min, range_max, default_value, stiffness, damping, clamped, locked, prescribed_function | A single DOF definition |
| `CoordinateEffect` | coordinate, joint, component, function | Maps one coordinate → one spatial transform axis |
| `SpatialTransform` | joint, effects: Vec\<EntityID\> | Groups all CoordinateEffects for a CustomJoint |

#### TransformComponent enum

```rust
enum TransformComponent { RotationX, RotationY, RotationZ, TranslationX, TranslationY, TranslationZ }
```

#### JointFunction enum

```rust
enum JointFunction {
    Constant(f64),
    Linear { slope: f64, intercept: f64 },
    Polynomial { coefficients: Vec<f64> },
}
```

#### Entity relationship diagram

```
CustomJoint ──coordinates──→ [JointCoordinate, JointCoordinate, ...]
                │
                └──→ SpatialTransform
                          ├── CoordinateEffect ──→ JointCoordinate (drives RotationY)
                          ├── CoordinateEffect ──→ JointCoordinate (drives TranslationX)
                          └── CoordinateEffect ──→ JointCoordinate (drives TranslationZ)
```

### Muscle Decomposition

| Component | Fields | Purpose |
|---|---|---|
| `Muscle` | (identity — name from Name component) | Marker entity |
| `MusclePath` | muscle: EntityID, points: Vec\<PathPoint\> | Wrapping path for visualization/export |
| `Millard2012Params` | muscle, max_isometric_force, optimal_fiber_length, tendon_slack_length, pennation_angle_at_optimal, ... | Millard force model parameters |
| `HillTypeMuscleParams` | max_force, optimal_fiber_length, tendon_slack_length, pcsa, pennation_angle | Hill-type force model parameters |
| `MuscleState` | fiber_length, fiber_velocity, activation | Runtime state |
| `TendonParams` | spring_length, width | Tendon properties |

### Actuator Types

| Component | Fields | Purpose |
|---|---|---|
| `CoordinateActuator` | coordinate: EntityID, optimal_force, min_control, max_control | Generalized force on a single DOF |

### Wrap Objects

| Component | Fields | Purpose |
|---|---|---|
| `WrapGeom` | body, wrap_type, dimensions, location, orientation | Wrapping surface for muscle paths |

## Systems and Plugins

melosim uses a decentralized plugin system based on the `inventory` crate. Systems are named functions that operate on the World. Plugins register systems at link time — no central list, no registration code in core.

### The System struct

```rust
// src/systems.rs
pub struct System {
    pub name: &'static str,
    pub run: fn(&mut World),
}

inventory::collect!(System);

pub fn run_systems(world: &mut World) {
    for system in inventory::iter::<System> {
        (system.run)(world);
    }
}
```

### Registering a system

Any crate in the workspace (or a downstream dependency) can register a system:

```rust
use melosim::systems::{System, validate_all};
use melosim::components::*;

fn my_system(world: &mut World) {
    // ... operate on components
}

inventory::submit! {
    System::new("my_system", my_system)
}
```

The `inventory::submit!` macro runs at link time. Importing the crate is enough — no `registry.add(...)` call needed.

### Running systems

```rust
world.run_systems();  // runs all registered systems
// or
melosim::systems::run_systems(&mut world);
```

### Current systems

All current systems are validation systems. Each component module registers its own validator via `inventory::submit!`:

| System | Registered in | Purpose |
|---|---|---|
| `validate_hinge` | `components/joint.rs` | Check HingeJoint body references |
| `validate_slide` | `components/joint.rs` | Check SlideJoint body references |
| `validate_ball` | `components/joint.rs` | Check BallJoint body references |
| `validate_free` | `components/joint.rs` | Check FreeJoint body references |
| `validate_fixed` | `components/joint.rs` | Check FixedJoint body references |
| `validate_universal` | `components/joint.rs` | Check UniversalJoint body references |
| `validate_custom` | `components/joint.rs` | Check CustomJoint body + coordinate references |
| `validate_coordinate` | `components/coordinate.rs` | Check JointCoordinate range validity |
| `validate_coordinate_effect` | `components/coordinate.rs` | Check CoordinateEffect references |
| `validate_spatial_transform` | `components/coordinate.rs` | Check SpatialTransform references |
| `validate_frame` | `components/body.rs` | Check Frame parent references |
| `validate_site` | `components/body.rs` | Check Site parent references |
| `validate_coordinate_actuator` | `components/actuator.rs` | Check CoordinateActuator references |

All validation systems use the generic `validate_all::<T>()` helper, which iterates all instances of a component type and calls its `Validate` impl.

### Adding a plugin

To add a new system (e.g., deformation, rendering, FK):

1. Define your component types (if any)
2. Implement `Validate` on them (optional)
3. Write your system function
4. Register with `inventory::submit!`
5. Import the crate in your app

```rust
// In melosim-deformation/src/lib.rs
use melosim::prelude::*;

#[derive(Clone, Debug)]
pub struct Deformable {
    pub youngs_modulus: f64,
    pub poissons_ratio: f64,
}

fn deformation_system(world: &mut World) {
    for (entity, deformable) in world.iter::<Deformable>() {
        // ... compute deformation
    }
}

inventory::submit! {
    System::new("deformation", deformation_system)
}
```

```rust
// In the app
use melosim_deformation;  // import triggers registration
world.run_systems();       // runs deformation + validation + everything else
```

### Validation

Validation is a specific kind of system. Components implement the `Validate` trait to define their invariants:

```rust
pub trait Validate {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String>;
}
```

The generic `validate_all::<T>()` function iterates all instances of T and collects errors into the world's error resource. Each component module registers its validator via `inventory::submit!`.

`world.validate()` calls `run_systems()` and returns accumulated errors. This is the primary API for tests and the CLI.

## Importers

### MuJoCo (MJCF)

Uses `mujoco-rs` to load MJCF files. The C library handles includes, compiler directives, inheritance, and computed quantities. We walk the compiled `MjModel` arrays to populate the ECS world.

Feature-gated: `#[cfg(feature = "mujoco")]`. Enable via `features = ["mujoco"]` in Cargo.toml.

Key components:
- `src/importer/mujoco.rs` — walks MjModel arrays, creates entities
- `src/importer/mujoco_spec.rs` — alternative MjSpec-based import

### OpenSim

Two-stage architecture:
1. **Python extraction** (requires OpenSim installed) — `scripts/extract_opensim.py` extracts model data to JSON
2. **Rust import** (runs anywhere) — reads JSON, spawns entities, resolves references

Key components:
- `src/importer/opensim.rs` — `OpenSimModelData` types + `import_opensim_model()`
- `scripts/extract_opensim.py` — Python extraction script

## Exporters

### MuJoCo (MJCF)

String-based XML generation. No `mujoco-rs` dependency — writes XML directly from component data.

Key components:
- `src/exporter/mujoco.rs` — `world_to_mjcf()` function
- `src/exporter/mjcf_components.rs` — per-component XML rendering
- `src/exporter/mujoco_spec.rs` — MjSpec-based export (feature-gated)

### OpenSim

Generates .osim XML from ECS components.

Key components:
- `src/exporter/opensim.rs` — export functions

## Frontend Architecture

### Current state (prototype)

React + R3F (React Three Fiber) + TypeScript frontend served by the axum server. Renders meshes, joint lines, muscle paths, and site points with orbit controls and click-to-select.

This prototype validated the data pipeline (import → JSON → display) and exposed transform chain bugs that are now fixed. It is not the target architecture.

### Target architecture: egui + three-d + eframe (pure Rust)

The application is a biomechanics workbench where behavior is fully domain-specific. The 3D viewport is a thin display layer; the hard parts are all custom domain logic.

**Stack:**
- **eframe** — cross-platform window shell (native GL on desktop, WebGL2/WASM in browser)
- **egui** — immediate-mode UI panels (body properties, joint parameters, import/export controls)
- **three-d** — 3D viewport rendered to an offscreen texture, displayed in egui via `egui::Image`

```
┌─────────────────────────────────┐
│          eframe window          │
│  ┌──────────┬─────────────────┐ │
│  │  egui    │    three-d      │ │
│  │  panels  │    viewport     │ │
│  └──────────┴─────────────────┘ │
└─────────────────────────────────┘
```

**Why not Dioxus:** Dioxus renders DOM via webview — no canvas primitive for 3D. eframe provides native GL integration for the viewport.

**Why not Bevy:** Game engine with its own ECS. melosim already has its own ECS — two ECS worlds adds complexity without solving the problem.

**Why not R3F (current prototype):** R3F + drei has the best 3D editor ecosystem, but our 3D features are thin (orbit camera, click-select, mesh/line/point rendering). The heavy lifting is custom domain code. The ecosystem advantage doesn't offset the cost of maintaining a Rust core + TypeScript frontend.

### What three-d provides

| Need | three-d | You write |
|---|---|---|
| Render STL meshes | `stl_io` + `CpuMesh` (~25 lines) | — |
| Render OBJ meshes | `three-d-io` built-in | — |
| Line segments (joints, muscles, cables) | `CpuMesh::line_segments` | — |
| Points (sites) | `Points` marker | — |
| Orbit camera | `OrbitControl` | — |
| Click to select | ray pick utility (~30 lines) | — |
| Render to egui texture | `RenderTarget::from_color_and_depth` (~20 lines glue) | — |

### Desktop and web

- **Desktop (primary):** eframe compiles to a native window with GL context. File dialogs via `rfd` crate. Real file paths on drag-drop — no upload endpoint needed.
- **Web (embed):** eframe compiles to WASM. Embeddable via `<iframe>` or mounted WASM module.

### Migration path

1. **Now:** egui panels reading from the melosim World directly (no JSON, no HTTP)
2. **Then:** three-d viewport rendering meshes/lines/points from ECS components
3. **Then:** domain tools (attach, route, export) built on egui interaction + custom 3D picking
4. **Finally:** remove the axum server and R3F frontend (or keep server for remote/shared use)

## Workspace Structure

```
melosim/              (workspace root + core library)
├── src/              (lib: ECS, components, importers, exporters, systems)
├── server/           (binary: axum web server)
├── app/              (binary: eframe + egui + three-d desktop app)
├── python/           (PyO3 bindings)
├── frontend/         (React/R3F prototype — deprecated, kept as reference)
├── tests/
├── scripts/
└── docs/
    ├── roundtrip-opensim.md  (OpenSim round-trip plan — completed)
    └── future.md             (Shipyard, SparseSet, simulation — deferred)
```

## Related Documents

- [OpenSim Round-Trip](docs/roundtrip-opensim.md) — Rajagopal 2015 import/export plan and status
- [Future Considerations](docs/future.md) — Shipyard ECS, SparseSet, simulation plans
