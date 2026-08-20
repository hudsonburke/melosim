# MuJoCo Exporter — melosim → MJCF

**Date:** 2026-08-20
**Branch:** `bevy-migration`
**Status:** Plan (not yet implemented)

## Goal

Export a live melosim model (the Bevy `World`) as a MuJoCo MJCF model, so edited
models — skeleton, joints, muscles/cables, exo parts — can be simulated in
MuJoCo.

Bidirectional MuJoCo I/O is done entirely through **`mujoco_rs`'s MjSpec** — the
same reference library the importer uses. We **never hand-write or hand-parse
MJCF XML**:

- **Importer:** `MjSpec::load` parses `.xml` → melosim model.
- **Exporter:** we *build* an `MjSpec` from the melosim `World`, then let
  MuJoCo's own compiler/renderer serialize it to `.xml` (and/or `mj_model`).

## Why MjSpec for the exporter

- No XML string assembly (no escaping/ordering bugs); output is structurally
  valid by construction.
- **Roundtrip validation is free**: the same parse library consumes our output.
  `to_mjcf()` is verified by loading its result back and asserting entity counts
  / attributes — mirroring the importer's `extract_mjcf` tests.
- Defaults, units, angle conventions, and mesh-dir handling are MuJoCo's problem,
  not ours.

## Dependency setup

`mujoco_rs` is **not yet a dependency** (the old importer's `mujoco` feature was
never wired into this Cargo.toml). Add:

```toml
[features]
mujoco = ["dep:mujoco_rs", "dep:anyhow"]

[dependencies]
mujoco_rs = { version = "5", features = ["auto-download-mujoco"], optional = true }
```

- `auto-download-mujoco` keeps the flake light; it downloads MuJoCo into
  `.mujoco/` at build (already gitignored) and we set `LD_LIBRARY_PATH` in the
  dev shell (as the old server did).
- Everything roundtrip/`mujoco`-dependent is `#[cfg(feature = "mujoco")]` so the
  core still builds without MuJoCo.

## Architecture

Exporter walks the **live Bevy `World`** (component queries), building an
`MjSpec` — it does **not** round-trip through the importer's IR (that's old and
stale). Functions are pure (take `&World`), so they're unit-testable.

```
src/exporter/
  mod.rs      ExportError; pub fn to_mjcf(world: &World, root: Entity) -> Result<MjSpec, ExportError>
              (then serialize: spec.save(path) or spec.to_xml())
  graph.rs    Query-based traversal: worldbody tree, joints, sites, tendons.
  mujoco.rs   melosim component  →  MjSpec builder calls  (the mapping table below)
  verify.rs   #[cfg(feature="mujoco")] roundtrip checks + myoarm sanity test
```

A thin `ExporterPlugin` registers an **“Export → MuJoCo (.xml)”** action in the
editor (toolbar menu → `rfd` save dialog writes the file).

## Mapping table (current model → MjSpec / MJCF)

Model components (all `crate::model`, hand-authored BSN + editor): `Body`,
`Frame`, `Site`, `Joint`, `Coordinate`, `Twist`, `CoordinateState`,
`CoordinateProperties`, `InitialConditions`, `JointCoordinates`, `CoordinateOf`,
`Muscle`, `HillTypeMuscleParams`, `PathEntities`, `PathElement`,
`InertialProperties`, `Inertia`.

| melosim | MjSpec / MJCF |
|---|---|
| root body (world) | `MjSpec` → `worldbody` |
| `Body` + `InertialProperties` | `MjsBody` (parent-aligned) + `<inertial pos mass fullinertia/>` via `body.add_from_/set_inertial` |
| `Frame`/`Site` child | `body.add_site(name, pos)` (site = attachment/routing point) |
| `Joint` + `JointCoordinates` | for each `Coordinate`: `body.add_joint(name, type, pos, axis)` on the **child** body |
| `Coordinate` + `CoordinateProperties` | joint `range`, `limited`(clamped), `stiffness`, `damping`, `armature`; locked ⇒ stiff spring |
| `Twist` (angular/linear) | hinge (angular axis ≠ 0), slide (linear axis ≠ 0); 1-DOF per joint |
| `InitialConditions` / `CoordinateState` | keyframe + `qpos` (default pose) |
| `Muscle` + `PathEntities` (ordered Sites) | `<tendon><spatial>` with `<site site=…/>` per point; `<pair/>` for wrap geoms later |
| `HillTypeMuscleParams` | `<actuator><muscle name tendon=… lmax range force …/>` (map Hill params) |
| mesh child (`Mesh3d`/`SceneRoot`) | `<asset><mesh name file=…/>` + `<geom type=mesh mesh=… pos quat/>`; copy/convert mesh to STL |
| joint/body placement | transforms resolved in **parent** body frame (like the importer's parent/child offsets) |

## Conventions

- **Coordinate systems:** melosim is glTF Y-up, meters; MuJoCo default world is
  Z-up. Apply a root Y-up→Z-up rotation at the model root the first phase ships.
- **Units:** meters (mass kg, inertia kg·m²); degrees for angles (`compiler angle=degree`).
- **One `<joint>` per `Coordinate`.** A joint with multiple coordinates
  (e.g. ball/universal) is emitted as multiple on-axis joints on the same child
  body; true composite dofs are a later phase.
- **Names:** use the melosim `Name` (unique) for bodies/sites/joints/tendons.

## Verification (per phase, `nix develop --command cargo test --features mujoco`)

1. Build myoarm (`myoarm_skeleton` → spawn) → `to_mjcf` → `MjSpec::load` the
   emitted XML → assert body count (5), joint dofs (5 coords), no error.
2. `mujoco` roundtrip: load emitted file, walk `MjsBody`/joints/sites and compare
   counts + names to the source melosim `World`.
3. Manual: open the exported `.xml` in MuJoCo / `simulate` and confirm it
   articulates like the editor.

## Phases

- **P0:** add `mujoco_rs` dep + `mujoco` feature; scaffold `src/exporter/`; flake
  `LD_LIBRARY_PATH` for auto-downloaded MuJoCo.
- **P1:** worldbody tree — bodies + inertial + hierarchy (Y-up→Z-up root).
- **P2:** joints — hinge/slide from `Joint`+`Coordinate`+`Twist`, limits/stiffness/damping.
- **P3:** tendons — muscle/cable paths (`PathEntities` → `<tendon><spatial>`).
- **P4:** actuators — Hill-type muscle params → `<actuator><muscle>`.
- **P5:** meshes/geoms — copy/convert mesh assets → `<asset><mesh>` + `<geom>`.
- **P6:** editor “Export → MuJoCo” action (rfd save dialog); myoarm roundtrip shows in MuJoCo.

## Risks / notes

- **`mujoco_rs` builder API** (`MjsBody::add_joint`, tendon/actuator sinks,
  `MjSpec::save`/serialize) — confirm exact signatures against the installed
  crate before coding; the old importer shows the wrapper names (`MjSpec`,
  `MjsBody`, `MjsJoint`, …).
- **Lossy areas:** composite joints, wrapping (`<pair>`), complex OpenSim
  functions — flat to MuJoCo-native forms first; document what’s approximate.
- **Meshes:** MuJoCo consumes STL; glTF/GLB assets need conversion (or accepted
  later). Defer to P5.
- Keep the exporter **headless-workable**: `to_mjcf(&World, root)` is pure — only
  the editor save action and verification need the window/feature.
