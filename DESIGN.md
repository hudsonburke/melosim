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

## Two-Phase Architecture: Build → Freeze → Simulate

melosim operates in two distinct phases that solve opposite requirements:

### Phase 1: Build World (extensible, dynamic)

The Build World is the authoring environment. Importers, validators, editors, and downstream plugins all operate here. New component types can be added at any time without modifying melosim core.

```rust
let mut world = World::new();
let entity = world.spawn();
world.attach(entity, HingeJoint { ... });
// Downstream crate adds a custom model:
let neuron = world.spawn();
world.attach(neuron, MyCustomNeuron { ... });
```

- Storage: AnyMap of `Vec<Option<T>>` (dense arrays, type-erased)
- Entity IDs: `EntityID(u32)` — direct index into Vecs
- Extensibility: any `'static` type, zero World changes
- Mutation: full (spawn, attach, remove, update)
- Systems: validation, import, export, editing

### Phase 2: FlatWorld (dense, GPU-ready)

The FlatWorld is the simulation snapshot. After building and validating the model, `freeze()` copies each known component type's Vec from the World's AnyMap into named fields for zero-hash access during simulation.

```rust
let flat = world.freeze();
// flat.inertials[id.0 as usize] — single load, zero hash lookups
// &flat.custom_joints — direct slice, no indirection
```

- Storage: Named `Vec<Option<T>>` fields on FlatWorld struct
- Entity IDs: `EntityID(u32)` — direct index into parallel arrays
- Extensibility: custom types in `extensions: AnyMap<Vec<Option<T>>>`
- Cross-type join: `flat.frames[hinge.body_a.0 as usize]` — single load
- GPU extraction: `&flat.inertials` is `&[Option<InertialProperties>]`
- Mutation: immutable after freeze (copy-on-write for state updates)

### Freeze contract

The `freeze()` method clones each known component type's `Vec<Option<T>>` from the World's AnyMap into FlatWorld's named fields. Because the World already stores at dense EntityID indices, no index translation is needed (no more `collect_dense` slot-index extraction).

Known component types are extracted explicitly by freeze. Custom types are not collected automatically — add them to `flat.extensions.insert::<Vec<Option<MyType>>>(...)` after freeze if needed.

```rust
pub fn freeze(&self) -> FlatWorld {
    FlatWorld {
        inertials: extract::<InertialProperties>(self),
        frames: extract::<Frame>(self),
        hinge_joints: extract::<HingeJoint>(self),
        // ... one per known type
        num_entities: self.next_id,
    }
}

fn extract<T: Clone + 'static>(world: &World) -> Vec<Option<T>> {
    world.components.get::<ComponentStorage<T>>()
        .cloned().unwrap_or_default()
}
```

### When to use which

| Operation | Use |
|---|---|
| Importing a model (OpenSim, MJCF) | Build World |
| Editing (add/remove bodies, muscles) | Build World |
| Validation | Build World |
| Forward kinematics | FlatWorld |
| Muscle force computation | FlatWorld |
| Warp/GPU integration | FlatWorld (zero-copy slices) |
| Serialization/save | Build World (serde-native) |
| Export to OpenSim/MJCF | Build World |

### Core Components

| Component | Fields | Read by |
|---|---|---|
| `InertialProperties` | name, mass, com, inertia | Rigid body solver |
| `Frame` | parent, transform | All systems (parent-relative transforms) |
| `Site` | parent, offset | Cable routing, landmarks, muscle paths |
| `Material` | density, youngs_modulus, poissons_ratio | FEM solver |
| `MeshGeometry` | mesh | Visualization, export |
| `DisplayGeometry` | body, mesh_file, scale, color, opacity, transform | Visualization, export |
| `Landmark` | site, name | Marker export |

### Muscle Decomposition
A muscle is an entity that can have multiple components:

| Component | Fields | Read by |
|---|---|---|
| `Muscle` | name | Identity only |
| `MusclePath` | muscle, points | Wrapping solver, visualization, export |
| `Millard2012Params` | muscle, max_isometric_force, optimal_fiber_length, tendon_slack_length, pennation_angle_at_optimal, ... | Millard force solver |
| `HillTypeMuscleParams` | max_force, optimal_fiber_length, tendon_slack_length, pcsa, pennation_angle | Hill-type force solver |
| `MuscleState` | fiber_length, fiber_velocity, activation | Runtime state |

**Why decompose:** The Hill-type solver reads physiology params. The FEM solver reads mesh + material. The wrapping solver reads the path. Different systems, different components.

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
| `JointCoordinate` | name, range_min, range_max, default_value, stiffness, damping, clamped, locked, prescribed_function | A single DOF definition |
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
5. Walk `model.getMarkerSet()` → spawn + attach Site + Landmark entities
6. Walk `model.getWrapObjectSet()` → spawn + attach WrapGeom entities
7. Walk body frames for display geometries → spawn + attach DisplayGeometry entities
8. Validate all references (bodies exist, muscles reference valid bodies, etc.)

### Export pipeline
1. Walk all InertialProperties → emit `<Body>` elements with real names
2. Walk all joints → emit `<Joint>` elements (detect type, emit appropriate XML)
3. Walk all muscles → emit `<Millard2012EquilibriumMuscle>` elements with GeometryPath
4. Walk all Landmarks → emit `<Marker>` elements
5. Walk all WrapGeom → emit `<WrapObject>` elements
6. Walk all DisplayGeometry → emit <DisplayGeometry> elements
7. Write the .osim XML file

### Current status (2026-07-28)

✅ Full round-trip of Rajagopal 2015 model: 23 bodies, 22 joints (CustomJoint + PinJoint), 80 muscles, 66 markers, 40 wrap objects, 103 display geometries. ~149K structurally valid .osim XML output.

### Key challenges
- OpenSim's inheritance hierarchy — Python API flattens this
- Muscle wrapping surfaces — multiple algorithms (sphere, cylinder, ellipsoid)
- Custom joints — SpatialTransform with polynomial functions
- OpenSim 4.x API differences: markers use ComponentList/Landmark API, wrap objects use per-body getWrapObjectSet, display geometry uses frame/geometry API

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
│   ├── import_opensim_marker()     ← individual: spawns Site + Landmark entities
│   └── import_opensim_wrap()       ← individual: spawns WrapGeom entity
```

## What's Next

1. Write FK solver on FlatWorld
2. Write MuJoCo importer (using mujoco-rs)
3. Write MuJoCo exporter
4. Write OpenSim exporter (in progress — structural output works, dead code cleanup pending)

## Future Considerations

### Shipyard ECS

Shipyard (by Catherine West / kyren) provides component queries, generational entity IDs, sparse set iteration, and system scheduling. We evaluated it against melosim's current architecture (AnyMap of `Vec<Option<T>>`, dense `EntityID(u32)`, explicit cross-entity references).

**What Shipyard would add:** Combined component iteration (`View<A>` + `View<B>` filter to entities with both), generational safety on entity deletion, better iteration (sparse sets skip Nones), and workloads for system scheduling.

**What it would cost:** Opaque EntityId (not u32) breaks direct FlatWorld indexing — a freeze extraction step becomes required. Derive macros on all components. Closure-based `run()` API changes how systems are written. Double indirection for lookup (sparse array → dense array). The freeze pattern goes from trivial Vec clone to custom extraction from Shipyard's internal sparse sets.

**Why we're not adopting it now:** At 200-entity biomechanics models with static entity sets and explicit cross-entity references (HingeJoint.body_a, Frame.parent), manual iteration with `world.get::<T>(entity)` lookups is simple, fast, and debuggable. Shipyard's query power shines at 10K+ entities with dynamic component addition/removal during simulation.

**When to reconsider:** If the FK solver or muscle force solver develops complex multi-component iteration patterns that manual iteration can't express cleanly, or if parallel system execution becomes necessary for solver performance, revisit Shipyard. The migration scope is comparable to the SlotMap → Vec<Option<T>> refactor (~20 source files, ~2-3 focused sessions). Keep explicit cross-entity references (components store EntityID fields) — this makes the migration cleaner since you're replacing storage and iteration, not rearchitecting entity relationships.

### SparseSet storage

Shipyard uses SparseSet internally — a dense packed array of components plus a sparse index array. This provides O(1) lookup, O(min(n,m)) combined iteration, and skip-Nones iteration. We evaluated replacing `Vec<Option<T>>` with a custom SparseSet. Not adopted because: the performance gain is negligible at 200 entities, direct array indexing (`vec[id.0 as usize]`) is simpler and GPU-mappable, and the double indirection (sparse → dense → component) is unnecessary overhead for our scale. Revisit if entity counts grow to thousands or if solver iteration becomes a bottleneck.
