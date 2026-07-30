# melosim — Design Document

## Guiding Philosophy

**A neutral data model for neuromusculoskeletal simulation.** melosim is not a simulator — it's a common language that can translate between OpenSim, MuJoCo, and other tools. The value is in the data model, not in reimplementing physics.

**Data-model-first architecture.** The type system is the foundation. Rust enforces invariants at compile time. The ECS pattern separates data from behavior. Systems are functions that read and write components.

**Import the model, don't parse the format.** Use each simulator's own API to load models (OpenSim Python API, MuJoCo C API). Don't parse XML yourself. The API handles includes, compiler directives, inheritance, and computed quantities.

**Separate concerns by access pattern.** Components are organized by which system reads them, not by what they represent. A body's mass is read by the rigid body solver. A muscle's path is read by the wrapping solver. A material's Young's modulus is read by the FEM solver. Different systems, different components.

**Composition over inheritance.** Entities are composed of components. A muscle entity has Muscle + MusclePath + HillTypeParams. An exo part entity has Frame + InertialProperties + Joint + CableGuide. Add components to give entities new capabilities, remove components to take them away.

**One-to-one → enum field. One-to-many → separate components.** A joint can only be one type (Hinge, Slide, Ball) — use an enum. A site can serve multiple purposes (muscle attachment, cable guide, landmark) — use separate components linked by entity ID.

**Runtime validation, not just compile-time.** The type system prevents most invalid states. But entity references (does body 42 exist?) are runtime checks. The `validate()` method catches what the compiler can't.

**No monolithic components.** Decompose by access pattern. A body is not a single component — it's an entity with InertialProperties + Frame + maybe MeshGeometry + maybe PrimitiveGeometry. A muscle is not a single component — it's an entity with Muscle + MusclePath + HillTypeParams + maybe FEMMuscleMesh. Each component serves one system.

## ECS Architecture

### Entity IDs

- `EntityID(pub u32)` — dense integer, direct index into `Vec<Option<T>>` storage
- Created by `World::spawn()` (monotonic counter)
- Used as foreign keys in component fields (e.g., `HingeJoint.body_a: EntityID`)
- No generational safety — entities are never deleted during the build phase. Validation catches stale references before freeze.

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

**Why Vec<Option<T>> instead of SlotMap:** SlotMap gave opaque keys with generation counters, but every freeze had to extract slot indices into dense Vecs anyway — the `collect_dense` translation pass. Dense Vecs eliminate that pass entirely. The cost is no generational safety, but entities are never deleted during the build phase.

### Spawn vs Attach

**Entities and components are separate operations.** `spawn()` allocates an EntityID. `attach()` places a component on that entity.

```rust
let body = world.spawn();  // returns EntityID
world.attach(body, InertialProperties { name: "pelvis".into(), mass: 11.78, ... });

let joint = world.spawn();
world.attach(joint, HingeJoint { body_a: body, body_b: other_body, axis: [1.0, 0.0, 0.0] });
```

This differs from `world.insert::<T>(component)` which both allocates and stores in one call, returning a key. The two-step pattern makes it explicit that the entity exists before any component is attached, and multiple components can be attached to the same entity.

Component references between entities use explicit EntityID fields (HingeJoint.body_a, Frame.parent, etc.). Unlike shared-key-insertion ECS patterns, each entity has exactly one spawn and components are attached by reference — not by index alignment.

### Systems

Systems are standalone functions that read/write specific component types:
- MJCF parser: XML → World (populates components)
- MJCF compiler: World → XML (reads components, emits MJCF)
- OpenSim importer: Python API → World (populates components)
- OpenSim exporter: World → Python API (reads components, emits .osim)
- Rigid body solver: reads Frame, InertialProperties → writes ForceOutput
- Muscle force solver: reads MusclePath, HillTypeParams → writes ForceOutput
- Wrapping solver: reads MusclePath, WrapGeom → updates MusclePath points

### System Registry

Systems are registered in a `SystemRegistry` at startup:

```rust
pub struct SystemRegistry {
    systems: Vec<Box<dyn Fn(&mut World)>>,
}
```

```rust
let mut registry = SystemRegistry::new();
registry.add("hinge_joints", hinge_system);
registry.add("ball_joints", ball_system);
// Custom joint type — no existing code changed
registry.add("prismatic_joint", prismatic_system);
registry.run(&mut world);
```

Each system reads ONLY the concrete types it needs. The registry iterates systems in order. Adding a new component type = add a struct + write a system + register it. No changes to World, no changes to existing systems, no trait objects.

### Core Components

| Component | Fields | Read by |
|---|---|---|
| `Name` | value (String) | Import/export, logging, debugging (metadata only — not needed for simulation) |
| `InertialProperties` | mass, com, inertia | Rigid body solver |
| `Frame` | parent, transform | All systems (parent-relative transforms) |
| `Site` | parent, offset | Marker/anatomical landmarks (name from Name component) |
| `Material` | density, youngs_modulus, poissons_ratio | FEM solver |
| `MeshGeometry` | mesh | Visualization, export |
| `DisplayGeometry` | body, mesh_file, scale, color, opacity, transform | Visualization, export |

### Muscle Decomposition
A muscle is an entity that can have multiple components:

| Component | Fields | Read by |
|---|---|---|
| `Muscle` | (none — identity from Name component) | Identity only |
| `MusclePath` | muscle, points | Wrapping solver, visualization, export |
| `Millard2012Params` | muscle, max_isometric_force, optimal_fiber_length, tendon_slack_length, pennation_angle_at_optimal, ... | Millard force solver |
| `HillTypeMuscleParams` | max_force, optimal_fiber_length, tendon_slack_length, pcsa, pennation_angle | Hill-type force solver |
| `MuscleState` | fiber_length, fiber_velocity, activation | Runtime state |

**Why decompose:** The Hill-type solver reads physiology params. The FEM solver reads mesh + material. The wrapping solver reads the path. Different systems, different components.

### Actuator Types

Same pattern as joints — each actuator type is its own component. Adding a new actuator type (e.g., `TorqueActuator`, `PointActuator`) means defining a struct and wiring it into the importer/exporter. No changes to existing code.

| Component | Fields | Purpose |
|---|---|---|
| `CoordinateActuator` | coordinate: EntityID, optimal_force, min_control, max_control | Generalized force on a single DOF |

### Joint Types

Each joint type is a standalone component carrying both the common fields (body_a, body_b, limits) and type-specific data. Every component is its own entity — joints are not inlined into bodies.

| Component | Fields | FK Solver |
|---|---|---|
| `HingeJoint` | body_a, body_b, limits, axis | Hinge system |
| `SlideJoint` | body_a, body_b, limits, axis | Slide system |
| `BallJoint` | body_a, body_b, limits | Ball system |
| `FreeJoint` | body_a, body_b, limits | Free system |
| `FixedJoint` | body_a, body_b, limits | Fixed system |
| `UniversalJoint` | body_a, body_b, limits, axis1, axis2 | Universal system |
| `CustomJoint` | body_a, body_b, limits, coordinates: Vec<EntityID> | Custom system |

Adding a new joint type: define the component struct, write a FK system function, register the system. No changes to any existing code.

**Why separate components instead of an enum?** An enum is a closed set — adding a variant requires modifying the enum definition and every match statement. Separate component types are an open set — a downstream crate can define a `PrismaticJoint` without touching melosim's source code. The system registry handles iteration. Each joint type lives in its own Vec in the AnyMap, so there's no wasted space for unused variants.

### Coordinate System

The coordinate system models generalized coordinates and their effect on joint transforms. This is the core of OpenSim's `CustomJoint` — without it, coupled joint motion (like knee flexion driving tibial translation) cannot be represented.

Coordinates are **separate entities** (not inlined into joints). This allows independent iteration — a system can find all locked coordinates without touching every joint — and avoids duplicating coordinate data when multiple effects reference the same coordinate.

#### Components

| Component | Fields | Purpose |
|---|---|---|
| `JointCoordinate` | range_min, range_max, default_value, stiffness, damping, clamped, locked, prescribed_function | A single DOF definition (name from Name component) |
| `CoordinateEffect` | coordinate, joint, component (TransformComponent), function (JointFunction) | Maps one coordinate → one spatial transform axis |
| `SpatialTransform` | joint, effects: Vec<EntityID> | Groups all CoordinateEffects for a CustomJoint |

#### TransformComponent enum

Identifies which of the 6 spatial DOFs a CoordinateEffect drives:

```rust
enum TransformComponent { RotationX, RotationY, RotationZ, TranslationX, TranslationY, TranslationZ }
```

#### JointFunction enum

Functions that map coordinate values (q) to transform components:

```rust
enum JointFunction {
    Constant(f64),                       // f(q) = c
    Linear { slope, intercept },         // f(q) = slope * q + intercept
    Polynomial { coefficients: Vec<f64> }, // f(q) = c0 + c1*q + c2*q^2 + ...
}
```

OpenSim's CustomJoint uses PolynomialFunction extensively for coupled motion. A knee joint might have:
- Coordinate `knee_flexion` drives `RotationY` via `Linear(-1.0, 0.0)`
- Same coordinate drives `TranslationX` via `Polynomial([0.002, -0.015, 0.0])`
- Same coordinate drives `TranslationZ` via `Polynomial([-0.42, 0.01, 0.0])`

Each of these is a separate `CoordinateEffect` entity referencing the same `JointCoordinate` entity.

#### Entity relationship diagram

```
CustomJoint ──coordinates──→ [JointCoordinate, JointCoordinate, ...]
                │
                └──→ SpatialTransform
                          ├── CoordinateEffect ──→ JointCoordinate (drives RotationY)
                          ├── CoordinateEffect ──→ JointCoordinate (drives TranslationX)
                          └── CoordinateEffect ──→ JointCoordinate (drives TranslationZ)
```

Components reference other entities by EntityID, and systems iterate the components they need independently.

---

## Round-Trip Plan: OpenSim (Rajagopal 2015)

### What's in the model
- 23 bodies (pelvis, femur_r/l, tibia_r/l, talus_r/l, etc.)
- 22 joints (hip, knee, ankle, etc.)
- 80 muscles (Millard2012EquilibriumMuscle with wrapping surfaces)
- 66 markers
- 40 wrap objects
- 103 display geometries

### Import pipeline
1. Load model via OpenSim Python API: `model = osim.Model('Rajagopal2015.osim')`
2. Walk `model.getBodySet()` → spawn + attach InertialProperties + Frame entities
3. Walk `model.getJointSet()` → spawn + attach Joint entities (detect type via `getConcreteClassName()`)
4. Walk `model.getMuscleSet()` → spawn + attach Muscle + MusclePath + Millard2012Params entities
5. Walk `model.getMarkerSet()` → spawn + attach Site + Name entities
6. Walk `model.getWrapObjectSet()` → spawn + attach WrapGeom entities
7. Walk body frames for display geometries → spawn + attach DisplayGeometry entities
8. Validate all references (bodies exist, muscles reference valid bodies, etc.)

### Export pipeline
1. Walk all InertialProperties → emit `<Body>` elements with real names
2. Walk all joints → emit `<Joint>` elements (detect type, emit appropriate XML)
3. Walk all muscles → emit `<Millard2012EquilibriumMuscle>` elements with GeometryPath
4. Walk all Sites → emit `<Marker>` elements (name from Name component)
5. Walk all WrapGeom → emit `<WrapObject>` elements
6. Walk all DisplayGeometry → emit <DisplayGeometry> elements
7. Write the .osim XML file

### Current status (2026-07-28)

✅ Full round-trip of Rajagopal 2015 model: 23 bodies, 22 joints (CustomJoint + PinJoint), 80 muscles, 66 markers, 40 wrap objects, 103 display geometries. ~149K structurally valid .osim XML output.

### Key challenges
- OpenSim's inheritance hierarchy — Python API flattens this
- Muscle wrapping surfaces — multiple algorithms (sphere, cylinder, ellipsoid)
- Custom joints — SpatialTransform with polynomial functions
- OpenSim 4.x API differences: markers use ComponentList API, wrap objects use per-body getWrapObjectSet, display geometry uses frame/geometry API

### Validation
- Parse Rajagopal 2015 → ECS World
- Export ECS World → .osim file
- Verify body count, joint count, muscle count, marker count match
- Verify muscle paths, joint axes, body masses match

## Importer Architecture

The OpenSim importer follows a two-stage architecture:

1. **Python extraction** (runs on machine with OpenSim installed) — loads the .osim model via the OpenSim Python API, extracts raw data to JSON
2. **Rust import** (runs anywhere) — reads JSON, spawns entities, attaches components, resolves body name references, validates

### Round-trip binary

`cargo run --bin roundtrip -- Rajagopal2015.osim [output.osim]` — imports via PyO3, validates, exports. The `--from-json` flag skips PyO3 and reads a pre-extracted JSON fixture (used on architectures without OpenSim).

### Module structure

```
src/importer/
├── mod.rs          # Re-exports
└── opensim.rs      # OpenSimModelData types + import functions

tests/
├── import_test.rs  # Tests for each fixture
├── export_test.rs  # Tests for export XML
└── fixtures/
    ├── simple_hip.json      # ground → pelvis → femur (PinJoint)
    ├── simple_knee.json     # ground → femur → tibia (CustomJoint)
    └── simple_muscle.json   # Includes muscle, wrap, display geometry

scripts/
└── extract_opensim.py      # Python extraction script
```

### Whole-model vs individual functions

The importer operates at the whole-model level (resolves names to entity IDs). Internal functions that process individual components are separated for testing and composability:

```
Import pipeline:
├── import_opensim_model()    ← whole-model: resolves names, spawns entities
│   ├── import_opensim_body()       ← individual: spawns InertialProperties + Frame entities
│   ├── import_opensim_joint()      ← individual: spawns Joint entity, resolves body refs
│   ├── import_opensim_muscle()     ← individual: spawns Muscle entity, resolves body refs
│   ├── import_opensim_marker()     ← individual: spawns Site + Name entity
│   └── import_opensim_wrap()       ← individual: spawns WrapGeom entity
```

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
// In the app crate
world.run_systems();  // runs all registered systems
// or
crate::systems::run_systems(&mut world);
```

### Current systems

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
crate::melosim_deformation;  // import triggers registration
world.run_systems();          // runs deformation + validation + everything else
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

## Frontend Architecture

### Current state (prototype)

The current frontend is a React + R3F (React Three Fiber) + TypeScript prototype served by the axum server. It renders meshes, joint lines, muscle paths, and site points with orbit controls and click-to-select. The `server/src/main.rs` JSON API serves a `Scene` snapshot; the frontend consumes it and renders via three.js.

This prototype validated the data pipeline (import → JSON → display) and exposed several issues in the transform chain (FK math, MuJoCo mesh frame corrections, color passthrough) that are now fixed. It is not the target architecture.

### Target architecture: egui + three-d + eframe (pure Rust)

The application is becoming a biomechanics workbench where behavior is fully domain-specific — importing models, attaching exoskeleton parts, routing cables through wrapping surfaces, editing parameters, exporting for simulation. The 3D viewport is a thin display layer; the hard parts are all custom domain logic that any framework would require you to write.

**Stack:**
- **eframe** — cross-platform window shell (native GL on desktop, WebGL2/WASM in browser)
- **egui** — immediate-mode UI panels (body properties, joint parameters, muscle editors, import/export controls). De facto standard for Rust engineering tools.
- **three-d** — 3D viewport rendered to an offscreen texture, displayed in egui via `egui::Image`. Provides mesh rendering, line/point primitives, orbit camera, and ray picking.

```
┌─────────────────────────────────┐
│          eframe window          │
│  ┌──────────┬─────────────────┐ │
│  │  egui    │    three-d      │ │
│  │  panels  │    viewport     │ │
│  │          │                 │ │
│  │  body    │  ┌─gizmo────┐  │ │
│  │  props   │  │  mesh    │  │ │
│  │  joints  │  │  lines   │  │ │
│  │  import  │  │  points  │  │ │
│  │  export  │  └──────────┘  │ │
│  └──────────┴─────────────────┘ │
└─────────────────────────────────┘
```

**Why not Dioxus:** Dioxus is a webview-based UI framework — it renders DOM. It does not provide a canvas primitive for 3D rendering. Embedding a native GL viewport alongside a Dioxus webview means fighting both frameworks. eframe already solves the same problem (cross-platform UI + 3D viewport in one window) with native GL integration.

**Why not Bevy:** Bevy is a game engine with its own ECS, scene graph, and rendering pipeline. melosim already has its own ECS and data model — running two ECS worlds adds complexity without solving the actual problem. The viewport is ~200 lines of rendering code, not a game.

**Why not R3F (current prototype):** R3F + drei has the best 3D editor ecosystem (TransformControls, loaders, orbit controls). But our workflow's 3D features are thin — orbit camera, click-select, mesh/line/point rendering. The heavy lifting (exoskeleton attachment, cable routing, parameter scaling, MJCF export) is custom domain code that any framework requires you to write. The ecosystem advantage doesn't offset the cost of maintaining a Rust core + TypeScript frontend with duplicate types.

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

### What you write

| Feature | Complexity |
|---|---|
| Exoskeleton attachment tool | Custom interaction (~80 lines) + domain math |
| Cable/tendon routing through wrapping surfaces | Core algorithm — viewport just shows it |
| Body property editing (mass, inertia, com) | egui form fields (~20 lines) |
| Joint parameter editing | egui sliders (~30 lines) |
| MJCF/OSIM export | Pure Rust, no 3D involvement |
| Undo/redo | Custom (~100 lines) |
| Grid | Simple quad (~20 lines) |
| Selection highlight | Traverse mesh, set material (~10 lines) |
| Transform gizmo (if needed) | Write custom — domain-constrained, not generic |

### Desktop and web

- **Desktop (primary):** eframe compiles to a native window with GL context. File dialogs via `rfd` crate. Real file paths on drag-drop — no upload endpoint needed. The entire `/upload` endpoint and folder-traversal JS from the prototype disappears.
- **Web (embed):** eframe compiles to WASM. The result renders in a `<canvas>` element with egui panels and three-d viewport together. Embeddable via `<iframe>` or mounted WASM module.

### Migration path

The current R3F prototype serves as the interactive reference — it validated the data pipeline, exposed transform bugs, and proved the Scene JSON format. The migration to Rust-native UI follows the natural progression:

1. **Now:** egui panels reading from the melosim World directly (no JSON, no HTTP)
2. **Then:** three-d viewport rendering meshes/lines/points from ECS components
3. **Then:** domain tools (attach, route, export) built on egui interaction + custom 3D picking
4. **Finally:** remove the axum server and R3F frontend (or keep server for remote/shared use)

The server (`server/src/main.rs`) remains useful for remote access or multi-user scenarios even after the desktop app ships. The Scene JSON format is the wire protocol between them.

## Next Steps

1. ✅ Write MuJoCo importer (using mujoco-rs)
2. Write MuJoCo exporter
3. Write OpenSim exporter (structural output works, dead code cleanup pending)
4. Begin egui + three-d desktop app (see Frontend Architecture)
5. FK solver (when simulation work begins)

## Future Considerations

### Shipyard ECS

Shipyard (by Catherine West / kyren) provides component queries, generational entity IDs, sparse set iteration, and system scheduling. We evaluated it against melosim's current architecture (AnyMap of `Vec<Option<T>>`, dense `EntityID(u32)`, explicit cross-entity references).

**What Shipyard would add:** Combined component iteration (`View<A>` + `View<B>` filter to entities with both), generational safety on entity deletion, better iteration (sparse sets skip Nones), and workloads for system scheduling.

**What it would cost:** Opaque EntityId (not u32) breaks direct Vec indexing. Derive macros on all components. Closure-based `run()` API changes how systems are written. Double indirection for lookup (sparse array → dense array).

**Why we're not adopting it now:** At 200-entity biomechanics models with static entity sets and explicit cross-entity references (HingeJoint.body_a, Frame.parent), manual iteration with `world.get::<T>(entity)` lookups is simple, fast, and debuggable. Shipyard's query power shines at 10K+ entities with dynamic component addition/removal during simulation.

**When to reconsider:** If the FK solver or muscle force solver develops complex multi-component iteration patterns that manual iteration can't express cleanly, or if parallel system execution becomes necessary for solver performance, revisit Shipyard. Keep explicit cross-entity references (components store EntityID fields) — this makes any future migration cleaner since you're replacing storage and iteration, not rearchitecting entity relationships.

### SparseSet storage

Shipyard uses SparseSet internally — a dense packed array of components plus a sparse index array. This provides O(1) lookup, O(min(n,m)) combined iteration, and skip-Nones iteration. We evaluated replacing `Vec<Option<T>>` with a custom SparseSet. Not adopted because: the performance gain is negligible at 200 entities, direct array indexing (`vec[id.0 as usize]`) is simpler and GPU-mappable, and the double indirection (sparse → dense → component) is unnecessary overhead for our scale. Revisit if entity counts grow to thousands or if solver iteration becomes a bottleneck.
