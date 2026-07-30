# OpenSim Round-Trip: Rajagopal 2015

Status: ✅ Complete (2026-07-28)

## Model

- 23 bodies (pelvis, femur_r/l, tibia_r/l, talus_r/l, etc.)
- 22 joints (hip, knee, ankle, etc.)
- 80 muscles (Millard2012EquilibriumMuscle with wrapping surfaces)
- 66 markers
- 40 wrap objects
- 103 display geometries

## Import pipeline

1. Load model via OpenSim Python API: `model = osim.Model('Rajagopal2015.osim')`
2. Walk `model.getBodySet()` → spawn + attach InertialProperties + Frame entities
3. Walk `model.getJointSet()` → spawn + attach Joint entities (detect type via `getConcreteClassName()`)
4. Walk `model.getMuscleSet()` → spawn + attach Muscle + MusclePath + Millard2012Params entities
5. Walk `model.getMarkerSet()` → spawn + attach Site + Name entities
6. Walk `model.getWrapObjectSet()` → spawn + attach WrapGeom entities
7. Walk body frames for display geometries → spawn + attach DisplayGeometry entities
8. Validate all references (bodies exist, muscles reference valid bodies, etc.)

## Export pipeline

1. Walk all InertialProperties → emit `<Body>` elements with real names
2. Walk all joints → emit `<Joint>` elements (detect type, emit appropriate XML)
3. Walk all muscles → emit `<Millard2012EquilibriumMuscle>` elements with GeometryPath
4. Walk all Sites → emit `<Marker>` elements (name from Name component)
5. Walk all WrapGeom → emit `<WrapObject>` elements
6. Walk all DisplayGeometry → emit `<DisplayGeometry>` elements
7. Write the .osim XML file

## Result

~149K structurally valid .osim XML output with all counts matching.

## Key challenges

- OpenSim's inheritance hierarchy — Python API flattens this
- Muscle wrapping surfaces — multiple algorithms (sphere, cylinder, ellipsoid)
- Custom joints — SpatialTransform with polynomial functions
- OpenSim 4.x API differences: markers use ComponentList API, wrap objects use per-body getWrapObjectSet, display geometry uses frame/geometry API

## Importer architecture

The OpenSim importer follows a two-stage architecture:

1. **Python extraction** (runs on machine with OpenSim installed) — loads the .osim model via the OpenSim Python API, extracts raw data to JSON
2. **Rust import** (runs anywhere) — reads JSON, spawns entities, attaches components, resolves body name references, validates

### Round-trip binary

`cargo run --bin roundtrip -- Rajagopal2015.osim [output.osim]` — imports via PyO3, validates, exports. The `--from-json` flag skips PyO3 and reads a pre-extracted JSON fixture (used on architectures without OpenSim).

### Module structure

```
src/importer/
├── mod.rs          # Re-exports
├── opensim.rs      # OpenSimModelData types + import functions
├── mujoco.rs       # MJCF import (via mujoco-rs)
└── mujoco_spec.rs  # MJCF import (via MjSpec)

src/exporter/
├── mod.rs
├── opensim.rs      # OpenSim export
├── mujoco.rs       # MJCF export (string generation)
├── mujoco_spec.rs  # MJCF export (via MjSpec)
├── mujoco_trait.rs # Trait-based MJCF export
├── mjcf_components.rs  # Component-level MJCF export
└── trait_export.rs     # Export trait infrastructure
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

## Tests

```
tests/
├── import_test.rs       # JSON → World import tests
├── export_test.rs       # World → XML export tests
├── mujoco_import_test.rs    # MJCF import tests (requires mujoco feature)
├── mujoco_roundtrip_test.rs # MJCF round-trip tests
├── mjspec_complex_models_test.rs  # MjSpec-based tests
├── mjspec_roundtrip_test.rs       # MjSpec round-trip tests
└── fixtures/
    ├── simple_hip.json      # ground → pelvis → femur (PinJoint)
    ├── simple_knee.json     # ground → femur → tibia (CustomJoint)
    └── simple_muscle.json   # Includes muscle, wrap, display geometry
```
