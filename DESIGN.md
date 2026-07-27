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
- `u32` dense integers (not UUIDs)
- Created by `World::spawn()`
- Used as foreign keys in component fields

| Storage (AnyMap of SlotMaps)
|Each component type is stored in its own `SlotMap` inside an `AnyMap`:
|```rust
|pub struct World {
|    pub components: AnyMap,  // stores SlotMap<EntityKey, T> for each T
|}
|```
|This is the Catherine West pattern (RustConf 2018): type-erased storage with typed access.
|Adding a new component type does NOT require modifying the World struct — just insert
|a new SlotMap into the AnyMap at runtime.
|
|### Systems
|Systems are standalone functions that read/write specific component types:
|- MJCF parser: XML → World (populates components)
|- MJCF compiler: World → XML (reads components, emits MJCF)
|- OpenSim importer: Python API → World (populates components)
|- OpenSim exporter: World → Python API (reads components, emits .osim)
|- Rigid body solver: reads Frame, InertialProperties → writes ForceOutput
|- Muscle force solver: reads MusclePath, HillTypeParams → writes ForceOutput
|- Wrapping solver: reads MusclePath, WrapGeom → updates MusclePath points
|
|### System Registry
|Systems are registered in a `SystemRegistry` at startup:
|```rust
|pub struct SystemRegistry {
|    systems: Vec<Box<dyn Fn(&mut World)>>,
|}
|```
|```rust
|let mut registry = SystemRegistry::new();
|registry.add("hinge_joints", hinge_system);
|registry.add("ball_joints", ball_system);
|// Custom joint type — no existing code changed
|registry.add("prismatic_joint", prismatic_system);
|registry.run(&mut world);
|```
|Each system reads ONLY the concrete types it needs. The registry iterates systems in
|order. Adding a new component type = add a struct + write a system + register it.
|No changes to World, no changes to existing systems, no trait objects.

## Two-Phase Architecture: Build → Freeze → Simulate

melosim operates in two distinct phases that solve opposite requirements:

### Phase 1: Build World (extensible, dynamic)

The Build World is the authoring environment. Importers, validators, editors, and downstream plugins
all operate here. New component types can be added at any time without modifying melosim core.

```rust
let mut world = World::new();
world.insert::<HingeJoint>(...);
// Downstream crate adds a custom model:
world.insert::<MyCustomNeuron>(...);
```

- Storage: AnyMap of SlotMaps (Catherine West pattern)
- Entity IDs: `EntityKey` (opaque slotmap key with generational safety)
- Extensibility: any `'static` type, zero World changes
- Mutation: full (insert, remove, update)
- Systems: validation, import, export, editing

### Phase 2: FlatWorld (dense, GPU-ready)

The FlatWorld is the simulation snapshot. After building and validating the model, `freeze()`
produces a dense, indexable copy optimized for solver iteration and GPU extraction.

```rust
let flat = world.freeze::<SolverComponents>();
// flat.inertials[id] — single load, zero hash lookups
// &flat.muscle_force — &[f64] for cudaMemcpy
```

- Storage: `Vec<Option<T>>` indexed by dense `EntityID(u32)`
- Entity IDs: `EntityID(u32)` — direct index into parallel arrays
- Extensibility: custom types in `extensions: AnyMap<Vec<Option<T>>>`
- Cross-type join: `flat.frames[hinge.body_a]` — single load, no indirection
- GPU extraction: `&flat.inertials` is `&[InertialProperties]`
- Mutation: immutable after freeze (copy-on-write for state updates)

### Freeze contract

The `freeze()` method iterates every registered component SlotMap and copies each entity's
data into the corresponding dense-indexed Vec. The mapping from SlotMap key → dense ID is
determined by the key's internal index, which is stable because entities are never deleted
during the build phase (validation catches stale references before freeze).

```rust
fn collect_dense<T: Clone + 'static>(world: &World) -> Vec<Option<T>> {
    let count = world.next_id() as usize;
    let mut vec = vec![None; count];
    if let Some(slotmap) = world.components.get::<ComponentMap<T>>() {
        for (key, component) in slotmap.iter() {
            let id = key.data().as_ffi() as usize;
            if id < count {
                vec[id] = Some(component.clone());
            }
        }
    }
    vec
}
```

Custom types from downstream crates are collected into `extensions: AnyMap<Vec<Option<T>>>`.
Their solvers access them via the extension API — no core changes needed.

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
| `InertialProperties` | entity, mass, com, inertia | Rigid body solver |
| `Frame` | entity, body, transform | All systems (parent-relative transforms) |
| `Joint` | entity, body_a, body_b, joint_type, limits | Rigid body solver |
| `Site` | entity, body, offset | Cable routing, landmarks, muscle paths |
| `Material` | entity, body, density, youngs_modulus, poissons_ratio | FEM solver |
| `MeshGeometry` | entity, body, mesh | Visualization, export |
| `PrimitiveGeometry` | entity, body, shape (Sphere/Cylinder/etc.) | Collision, wrapping |

### Muscle Decomposition
A muscle is an entity that can have multiple components:

| Component | Fields | Read by |
|---|---|---|
| `Muscle` | entity, name | Identity only |
| `MusclePath` | muscle, points, wrap_geoms | Wrapping solver, visualization, export |
| `HillTypeParams` | muscle, max_force, fiber_length, pcsa, pennation, curves | Hill-type force solver |
| `FEMMuscleMesh` | muscle, mesh, material, fiber_directions | FEM force solver |
| `MuscleActivation` | muscle, activation, time_constant | Control system |

**Why decompose:** The Hill-type solver reads physiology params. The FEM solver reads mesh + material. The wrapping solver reads the path. Different systems, different components. The muscle entity doesn't know which solver is used.

### Cable Routing
| Component | Fields | Read by |
|---|---|---|
| `CableGuide` | entity, site, diameter | Cable routing solver |
| `Cable` | entity, name, path, tendon | Cable routing solver |
| `CableSegment` | ViaPoint / Port / Wrap (enum) | Cable routing solver |
| `CablePort` | entity, port_type, offset | Cable routing solver |
| `Tendon` | entity, name, spring_length, width, via_points | Tendon force solver |

### Wrapping
| Component | Fields | Read by |
|---|---|---|
| `WrapGeom` | entity, body, geom_type (Sphere/Cylinder) | Wrapping solver |
| `WrapPoint` | site, wrap_geom | Wrapping solver |

Wrapping is defined by path points referencing geometry entities. The geometry entity has a `PrimitiveGeometry` component. The wrapping solver reads path points that reference wrap geometries and computes the wrapping behavior.

### Actuators
| Component | Fields | Read by |
|---|---|---|
| `Actuator` | entity, name, actuator_type (Motor/Position/CableMotor/MuscleActuator) | Control system |

|### Joint Types
|Each joint type is a standalone component carrying both the common fields
|(body_a, body_b, limits) and type-specific data. Every component is its own
|entity — there is no secondary join needed.
|
|| Component | Fields | FK Solver |
||---|---|---|
|| `HingeJoint` | body_a, body_b, limits, axis | Hinge system |
|| `SlideJoint` | body_a, body_b, limits, axis | Slide system |
|| `BallJoint` | body_a, body_b, limits | Ball system |
|| `FreeJoint` | body_a, body_b, limits | Free system |
|| `FixedJoint` | body_a, body_b, limits | Fixed system |
| `UniversalJoint` | body_a, body_b, limits, axis1, axis2 | Universal system |
| `CustomJoint` | body_a, body_b, limits, coordinates: Vec<EntityKey> | Custom system |
|
|Adding a new joint type: define the component struct, write a FK system
|function, register the system. No changes to any existing code.
|
||**Why separate components instead of an enum?** An enum is a closed set —
||adding a variant requires modifying the enum definition and every match
||statement. Separate component types are an open set — a downstream crate
||can define a `PrismaticJoint` without touching melosim's source code.
||The system registry handles iteration. Each joint type lives in its own
||SlotMap in the AnyMap, so there's no wasted space for unused variants.

|## Coordinate System

|The coordinate system is a family of components that model generalized
|coordinates and their effect on joint transforms. This is the core of
|OpenSim's `CustomJoint` — without it, coupled joint motion (like knee
|flexion driving tibial translation) cannot be represented.

|Coordinates are **separate entities** (not inlined into joints). This
|allows independent iteration — a system can find all locked coordinates
|without touching every joint — and avoids duplicating coordinate data
|when multiple effects reference the same coordinate.

|### Components

|| Component | Fields | Purpose |
||---|---|---|
|| `JointCoordinate` | name, range_min, range_max, default_value, stiffness, damping, clamped, locked, prescribed_function | A single DOF definition |
|| `CoordinateEffect` | coordinate, joint, component (TransformComponent), function (JointFunction) | Maps one coordinate → one spatial transform axis |
|| `SpatialTransform` | joint, effects: Vec<EntityKey> | Groups all CoordinateEffects for a CustomJoint |

|### TransformComponent enum

|Identifies which of the 6 spatial DOFs a CoordinateEffect drives:

|\```
|enum TransformComponent { RotationX, RotationY, RotationZ, TranslationX, TranslationY, TranslationZ }
|\```

|### JointFunction enum

|Functions that map coordinate values (q) to transform components:

|\```
|enum JointFunction {
|    Constant(f64),                       // f(q) = c
|    Linear { slope, intercept },         // f(q) = slope * q + intercept
|    Polynomial { coefficients: Vec<f64> }, // f(q) = c0 + c1*q + c2*q^2 + ...
|}
|\```

|OpenSim's CustomJoint uses PolynomialFunction extensively for coupled
|motion. A knee joint might have:
|- Coordinate `knee_flexion` drives `RotationY` via `Linear(-1.0, 0.0)`
|- Same coordinate drives `TranslationX` via `Polynomial([0.002, -0.015, 0.0])`
|- Same coordinate drives `TranslationZ` via `Polynomial([-0.42, 0.01, 0.0])`

|Each of these is a separate `CoordinateEffect` entity referencing the same
|`JointCoordinate` entity.

|### Entity relationship diagram

|\```
|CustomJoint ──coordinates──→ [JointCoordinate, JointCoordinate, ...]
|                │
|                └──→ SpatialTransform
|                          ├── CoordinateEffect ──→ JointCoordinate (drives RotationY)
|                          ├── CoordinateEffect ──→ JointCoordinate (drives TranslationX)
|                          └── CoordinateEffect ──→ JointCoordinate (drives TranslationZ)
|\```

|This is a pure ECS pattern — components reference other entities by
|EntityKey, and systems iterate the components they need independently.

## Round-Trip Plan: OpenSim (Rajagopal 2015)

### What's in the model
- 22 bodies (pelvis, femur_r/l, tibia_r/l, talus_r/l, etc.)
- 22 joints (hip, knee, ankle, etc.)
- 80+ muscles (with wrapping surfaces, via points)
- Markers (anatomical landmarks)
- Tendons
- Actuators

### Import pipeline
1. Load model via OpenSim Python API: `model = osim.Model('Rajagopal2015.osim')`
2. Walk `model.getBodySet()` → create InertialProperties + Frame entities
3. Walk `model.getJointSet()` → create Joint entities (detect type via `getConcreteClassName()`)
4. Walk `model.getMuscleSet()` → create Muscle + MusclePath + HillTypeParams entities
5. Walk `model.getMarkerSet()` → create Site + Landmark entities
6. Walk `model.getWrapObjectSet()` → create WrapGeom entities
7. Walk `model.getTendonSet()` → create Tendon entities
8. Validate all references (bodies exist, muscles reference valid bodies, etc.)

### Export pipeline
1. Walk all inertials → emit `<Body>` elements
2. Walk all joints → emit `<Joint>` elements (detect type, emit appropriate XML)
3. Walk all muscles → emit `<Muscle>` elements with `<GeometryPath>` and `<PathPoint>` elements
4. Walk all markers → emit `<Marker>` elements
5. Walk all wrap objects → emit `<WrapObject>` elements
6. Walk all tendons → emit `<Tendon>` elements
7. Write the .osim XML file

### Key challenges
- OpenSim's inheritance hierarchy (Body → BodySet → Model) — Python API flattens this
- Muscle wrapping surfaces — multiple algorithms (ball, ellipsoid, cylinder, spindle)
- Custom joints — SpatialTransform with polynomial functions
- Coordinate systems — OpenSim uses Z-up, right-handed

### Validation
- Parse Rajagopal 2015 → ECS World
- Export ECS World → .osim file
- Parse the exported .osim file → compare with original
- Verify body count, joint count, muscle count, marker count match
- Verify muscle paths, joint axes, body masses match

## Round-Trip Plan: MuJoCo

### What's in a MuJoCo model
- Bodies (mass, inertia, position, geometry)
- Joints (hinge, slide, ball, free, fixed)
- Geoms (mesh, sphere, cylinder, capsule, plane)
- Sites (attachment points)
- Actuators (motor, position, muscle)
- Tendons (spatial routing through sites/wraps)
- Equality constraints (weld, connect, joint, tendon)
- Contact settings

### Import pipeline
1. Load via MuJoCo C API: `mj_loadXML()` (handles includes, compiler directives)
2. Walk `mjModel.body_*` → create InertialProperties + Frame entities
3. Walk `mjModel.joint_*` → create Joint entities
4. Walk `mjModel.geom_*` → create MeshGeometry or PrimitiveGeometry entities
5. Walk `mjModel.site_*` → create Site entities
6. Walk `mjModel.actuator_*` → create Actuator entities
7. Walk `mjModel.tendon_*` → create Tendon entities
8. Validate all references

### Export pipeline
1. Walk all inertials → emit `<body>` elements with `<inertial>`, `<joint>`, `<geom>`, `<site>`
2. Walk all actuators → emit `<actuator>` elements
3. Walk all tendons → emit `<tendon><spatial>` elements
4. Walk all equality constraints → emit `<equality>` elements
5. Write the MJCF XML file

### Key differences from OpenSim
- MuJoCo uses Y-up (vs OpenSim Z-up) — convert at adapter boundary
- Muscle model is built-in Hill-type (vs OpenSim's multiple implementations)
- Sites are first-class (vs OpenSim's markers)
- Tendons route through sites and wrap geoms

### Coordinate system conversion
```
Import (MuJoCo → ECS): Y-up → Z-up: swap Y and Z axes
Export (ECS → MuJoCo): Z-up → Y-up: swap Z and Y axes
```

## Importer Architecture

The OpenSim importer follows a two-stage architecture:

1. **Python extraction** (runs on machine with OpenSim installed) — loads the .osim model via the OpenSim Python API, extracts raw data to JSON
2. **Rust import** (runs anywhere) — reads JSON, creates ECS entities, resolves body name references, validates

```
Your machine (OpenSim) ──JSON──→ Any machine (Rust importer)
```

### Why not PyO3 directly?

A two-stage pipeline avoids the OpenSim runtime dependency on every machine. The JSON intermediate format is portable and debuggable. The Python script is a simple translator — it doesn't need to understand melosim's data model.

### Incremental development

The importer is built incrementally, one joint type at a time:

| Step | What | Test fixture | Status |
|---|---|---|---|
| 1 | Bodies + PinJoint | `simple_hip.json` (ground → pelvis → femur) | ✅ Done |
| 2 | FreeJoint + CustomJoint | `simple_knee.json` (ground → femur → tibia) | ✅ Done |
| 3 | UniversalJoint + BallJoint | TBD | ⬜ |
| 4 | Muscles (identity + path + params) | TBD | ⬜ |
| 5 | Markers + WrapGeoms + full Rajagopal | Rajagopal2015.osim | ⬜ |

Each step adds import functions for one component type and a test fixture.

### Module structure

```
src/importer/
├── mod.rs          # Re-exports
└── opensim.rs      # OpenSimModelData types + import functions

tests/
├── import_test.rs  # Tests for each fixture
└── fixtures/
    ├── simple_hip.json     # ground → pelvis → femur (PinJoint)
    └── simple_knee.json    # ground → femur → tibia (CustomJoint)

scripts/
└── extract_opensim.py      # Python extraction script
```

### Adding a new joint type

1. Define the joint's intermediate data in `OpenSimJointData` (optional fields)
2. Add a `match` arm in `import_opensim_joint()` dispatching to the type
3. Write the type-specific import function
4. Create a test fixture JSON
5. Add a test

No changes to the World struct, component types, or existing import functions.

### Whole-model vs individual functions
The importer operates at the whole-model level (resolves names to entity IDs). But internal functions that process individual components are separated for testing and composability.

```
Import pipeline:
├── import_model()          ← whole-model: resolves names, creates entities
│   ├── import_body()       ← individual: creates InertialProperties + Frame entities
│   ├── import_joint()      ← individual: creates Joint entity, resolves body refs
│   ├── import_muscle()     ← individual: creates Muscle entity, resolves body refs
│   └── import_wrap()       ← individual: creates WrapGeom entity

Export pipeline:
├── export_model()          ← whole-model: walks World, emits XML
│   ├── export_body()       ← individual: emits <body> element
│   ├── export_joint()      ← individual: emits <joint> element
│   └── export_muscle()     ← individual: emits <muscle> element
```

The individual functions are pure — they take inputs and produce outputs without side effects. The whole-model functions handle name resolution and entity ID mapping.

## What's Next

1. Add more joint importers (UniversalJoint, BallJoint)
2. Add muscle importer (Muscle + MusclePath + Millard2012Params)
3. Run Python extraction script on Rajagopal2015.osim
4. Validate full round-trip
5. Write MuJoCo importer (using mujoco-rs)
6. Write MuJoCo exporter
7. Write OpenSim exporter
8. Validate round-trip with Rajagopal 2015
