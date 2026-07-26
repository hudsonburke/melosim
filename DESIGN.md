# melosim — Design Document

## Guiding Philosophy

**A neutral data model for neuromusculoskeletal simulation.** melosim is not a simulator — it's a common language that can translate between OpenSim, MuJoCo, and other tools. The value is in the data model, not in reimplementing physics.

**Data-model-first architecture.** The type system is the foundation. Rust enforces invariants at compile time. The ECS pattern separates data from behavior. Systems are functions that read and write components.

**Import the model, don't parse the format.** Use each simulator's own API to load models (OpenSim Python API, MuJoCo C API). Don't parse XML yourself. The API handles includes, compiler directives, inheritance, and computed quantities.

**Separate concerns by access pattern.** Components are organized by which system reads them, not by what they represent. A body's mass is read by the rigid body solver. A muscle's path is read by the wrapping solver. A material's Young's modulus is read by the FEM solver. Different systems, different components.

**Composition over inheritance.** Entities are composed of components. A muscle entity has Muscle + MusclePath + HillTypeParams. An exo part entity has Body + Joint + CableGuide. Add components to give entities new capabilities, remove components to take them away.

**One-to-one → enum field. One-to-many → separate components.** A joint can only be one type (Hinge, Slide, Ball) — use an enum. A site can serve multiple purposes (muscle attachment, cable guide, landmark) — use separate components linked by entity ID.

**Runtime validation, not just compile-time.** The type system prevents most invalid states. But entity references (does body 42 exist?) are runtime checks. The `validate()` method catches what the compiler can't.

## ECS Architecture

### Entity IDs
- `u32` dense integers (not UUIDs)
- Created by `World::spawn()`
- Used as foreign keys in component fields

### Components (Struct of Arrays)
Each component type is a separate `Vec` in the World:
```rust
pub struct World {
    bodies: Vec<Body>,
    joints: Vec<Joint>,
    muscles: Vec<Muscle>,
    // ...
}
```
SoA layout for cache-friendly iteration. The rigid body solver iterates over all bodies reading mass and inertia — contiguous memory.

### Systems
Systems are functions that read/write components:
- MJCF parser: XML → World (populates components)
- MJCF compiler: World → XML (reads components, emits MJCF)
- OpenSim importer: Python API → World (populates components)
- OpenSim exporter: World → Python API (reads components, emits .osim)
- Rigid body solver: reads Body, Joint → writes ForceOutput
- Muscle force solver: reads Muscle, MusclePath, HillTypeParams → writes ForceOutput
- Wrapping solver: reads MusclePath, WrapGeom → updates MusclePath points

## Component Decomposition

### Core Components
| Component | Fields | Read by |
|---|---|---|
| `Body` | id, mass, com, inertia | Rigid body solver |
| `Joint` | id, body_a, body_b, joint_type, limits | Rigid body solver |
| `Site` | id, body, offset | Cable routing, landmarks, muscle paths |
| `Material` | id, body, density, youngs_modulus, poissons_ratio | FEM solver |
| `Geometry` | id, body, mesh, role | Visualization, collision |
| `Frame` | id, body, transform | All systems (parent-relative transforms) |

### Muscle Decomposition
A muscle is an entity that can have multiple components:

| Component | Fields | Read by |
|---|---|---|
| `Muscle` | id, name | Identity only |
| `MusclePath` | muscle, points, wrap_geoms | Wrapping solver, visualization, export |
| `HillTypeParams` | muscle, max_force, fiber_length, pcsa, pennation, curves | Hill-type force solver |
| `FEMMuscleMesh` | muscle, mesh, material, fiber_directions | FEM force solver |
| `MuscleActivation` | muscle, activation, time_constant | Control system |

**Why decompose:** The Hill-type solver reads physiology params. The FEM solver reads mesh + material. The wrapping solver reads the path. Different systems, different components. The muscle entity doesn't know which solver is used.

### Cable Routing
| Component | Fields | Read by |
|---|---|---|
| `CableGuide` | id, site, diameter | Cable routing solver |
| `Cable` | id, name, path, tendon | Cable routing solver |
| `CableSegment` | ViaPoint / Port / Wrap (enum) | Cable routing solver |
| `CablePort` | id, port_type, offset | Cable routing solver |
| `Tendon` | id, name, spring_length, width, via_points | Tendon force solver |

### Wrapping
| Component | Fields | Read by |
|---|---|---|
| `WrapGeom` | id, body, geom_type (Sphere/Cylinder) | Wrapping solver |
| `WrapPoint` | site, wrap_geom | Wrapping solver |

### Actuators
| Component | Fields | Read by |
|---|---|---|
| `Actuator` | id, target, actuator_type (Motor/Position/CableMotor/MuscleActuator) | Control system |

### Joint Types
| Variant | Data | Use case |
|---|---|---|
| `Hinge` | axis | Pin joint (hip flexion, knee flexion) |
| `Slide` | axis | Prismatic joint (knee translation) |
| `Ball` | (none) | Ball-and-socket (hip rotation) |
| `Free` | (none) | Free body (pelvis in space) |
| `Fixed` | (none) | Rigid connection |
| `CustomJoint` | coordinates, base_transform, coordinate_effects | OpenSim custom joints (knee with coupled motion) |

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
2. Walk `model.getBodySet()` → create Body + Frame entities
3. Walk `model.getJointSet()` → create Joint entities (detect type via `getConcreteClassName()`)
4. Walk `model.getMuscleSet()` → create Muscle + MusclePath + HillTypeParams entities
5. Walk `model.getMarkerSet()` → create Site + Landmark entities
6. Walk `model.getWrapObjectSet()` → create WrapGeom entities
7. Walk `model.getTendonSet()` → create Tendon entities
8. Validate all references (bodies exist, muscles reference valid bodies, etc.)

### Export pipeline
1. Walk all bodies → emit `<Body>` elements
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
2. Walk `mjModel.body_*` → create Body + Frame entities
3. Walk `mjModel.joint_*` → create Joint entities
4. Walk `mjModel.geom_*` → create Geometry entities
5. Walk `mjModel.site_*` → create Site entities
6. Walk `mjModel.actuator_*` → create Actuator entities
7. Walk `mjModel.tendon_*` → create Tendon entities
8. Validate all references

### Export pipeline
1. Walk all bodies → emit `<body>` elements with `<inertial>`, `<joint>`, `<geom>`, `<site>`
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

## Import/Export Strategy

### Whole-model vs individual functions
The importer operates at the whole-model level (resolves names to entity IDs). But internal functions that process individual components are separated for testing and composability.

```
Import pipeline:
├── import_model()          ← whole-model: resolves names, creates entities
│   ├── import_body()       ← individual: creates Body entity
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

1. Add missing components (Actuator, MuscleTendonUnit, EqualityConstraint)
2. Write OpenSim importer (using Python API via PyO3)
3. Write OpenSim exporter
4. Write MuJoCo importer (using mujoco-rs)
5. Write MuJoCo exporter
6. Validate round-trip with Rajagopal 2015
