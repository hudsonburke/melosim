# Bevy Migration Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Migrate melosim's custom ECS to Bevy, eliminating ~500 lines of relationship boilerplate and gaining component hooks, queries, and a mature ECS ecosystem.

**Architecture:** Replace `World` (AnyMap of `Vec<Option<T>>`) with `bevy_ecs::World`. Components become plain structs with `#[derive(Component)]`. Relationships become `#[derive(Relationship)]` / `#[derive(RelationshipTarget)]` pairs with auto-sync. The serialization boundary stays at the importer/exporter level (`OpenSimData` structs keep serde).

**Tech Stack:** Rust, `bevy_ecs` (not full Bevy — just the ECS), serde (for importer/exporter structs only), mujoco-rs

---

## Why Bevy

### The Case For

1. **We're rebuilding Bevy's relationship system.** We wrote ~250 lines of World boilerplate (`set_parent`, `set_in_frame`, `set_connects`, `set_has_dof`, `set_drives`, plus 8 reverse-list helpers) to replicate what Bevy's `#[relationship]` derive does in one macro.

2. **Component hooks solve the sync problem.** Bevy's `on_add`/`on_remove` hooks automatically maintain reverse relationship lists. No manual `set_*` methods needed — `attach(InFrame(frame))` auto-syncs.

3. **The query system is mature.** `Query<(&Name, &Position), With<InertialProperties>>` replaces manual iteration + filtering. We were going to build this — Bevy has it.

4. **Ecosystem.** Change detection, event systems, observers, hierarchy traversal (`iter_ancestors`, `iter_descendants`) — all built-in.

5. **No serde on ECS layer.** The World never serializes. Serialization is at the importer/exporter boundary (`OpenSimData` → JSON → World). This was the main blocker — it's gone.

### The Case Against (and why it's manageable)

1. **f32 vs f64.** Bevy uses `f32` for `Vec3`, `Quat`, `Transform`. melosim uses `f64`.
   - **Mitigation:** Use our own `Vec3`, `Quaternion` types (f64) as components. Bevy doesn't require its math types. The ECS layer is type-agnostic.

2. **App loop vs batch pipeline.** Bevy is designed for frame-based game loops with `Update`, `Startup` schedules.
   - **Mitigation:** Use `bevy_ecs` standalone (no `bevy_app`). Create `World` directly, call `world.query()` / `world.run_system()`. No app loop needed.

3. **Heavy dependency tree.** Full Bevy pulls in wgpu, winit, audio, etc.
   - **Mitigation:** Use `bevy_ecs` crate only (~10 deps, no rendering). This is what Bevy does internally — `bevy_ecs` is the ECS core.

4. **Archetype storage vs flat Vecs.** Bevy groups entities by component combination.
   - **Mitigation:** For melosim's 200-entity static models, archetype overhead is negligible. The freeze step (extracting to flat Vecs for GPU) still works — we just read from Bevy's archetypes instead of our AnyMap.

5. **Entity deletion.** Bevy's generational indices mean entity IDs change on reuse.
   - **Mitigation:** melosim models are static after import. No runtime deletion. For model editing, we can use `bevy_ecs::Entity` as opaque handles and map to stable indices at export time.

---

## Migration Roadmap

### Phase 1: Add bevy_ecs as dependency (30 min)

Add `bevy_ecs = "0.15"` to Cargo.toml. Remove `anymap2` (replaced by Bevy's storage).

### Phase 2: Replace World with bevy_ecs::World (2-3 hours)

**Current:**
```rust
pub struct World {
    pub components: AnyMap,
    pub resources: AnyMap,
    pub next_id: u32,
}
```

**New:**
```rust
pub struct World {
    pub ecs: bevy_ecs::World,
}
```

Replace methods:
- `spawn()` → `ecs.spawn()`
- `attach::<T>(entity, component)` → `ecs.entity_mut(entity).insert(component)`
- `get::<T>(entity)` → `ecs.get::<T>(entity)`
- `iter::<T>()` → `ecs.query::<&T>().iter(&ecs)`

### Phase 3: Convert components to Bevy Components (1 hour)

Replace `#[derive(Clone, Debug, Serialize, Deserialize)]` with `#[derive(Component, Clone, Debug)]` on all component types. Remove serde from ECS components.

**Files to update:**
- `src/components/body.rs` — InertialProperties, StationDefinedFrame
- `src/components/coordinate.rs` — JointCoordinate, CoordinateEffect
- `src/components/geometry.rs` — MeshGeometry, PrimitiveGeometry, DisplayGeometry
- `src/components/muscle.rs` — Muscle, Millard2012Params, HillTypeMuscleParams, etc.
- `src/components/relationship.rs` — ChildOf, Children, InFrame, Connects, HasDOF, Drives
- `src/components/spatial.rs` — Position, Rotation
- `src/components/actuator.rs` — CoordinateActuator
- `src/components/wrap.rs` — WrapGeom
- `src/components/material.rs` — Material
- `src/components/name.rs` — Name
- `src/components/path.rs` — MusclePath, PathPoint
- `src/components/tendon.rs` — TendonPath

### Phase 4: Replace relationship system with Bevy Relationships (2 hours)

Replace our typed relationship boilerplate with Bevy's derive macros.

**Current:**
```rust
pub struct InFrame(pub EntityID);
pub struct FrameContents { pub entities: Vec<EntityID> }

pub fn set_in_frame(&mut self, entity: EntityID, frame: EntityID) {
    // 20 lines of sync logic
}
```

**New:**
```rust
#[derive(Component, Clone, Debug)]
#[relationship(relationship_target = FrameContents)]
pub struct InFrame(pub Entity);

#[derive(Component, Clone, Debug, Default)]
#[relationship_target(relationship = InFrame)]
pub struct FrameContents(Vec<Entity>);
```

Same for Connects/ConnectedJoints, HasDOF/JointDOFs, Drives/CoordinateEffects.

Delete the `set_*` methods from World. Use `world.spawn((InFrame(parent), Position::new(...)))` or `commands.entity(entity).insert(InFrame(parent))`.

### Phase 5: Update World builders (1 hour)

Rewrite `add_hinge`, `add_ball`, `add_free`, etc. to use Bevy's API.

**Current:**
```rust
pub fn add_hinge(&mut self, parent_frame: EntityID, child_frame: EntityID, ...) -> EntityID {
    let joint_entity = self.spawn();
    self.set_in_frame(joint_entity, parent_frame);
    self.set_connects(joint_entity, child_frame);
    // ...
}
```

**New:**
```rust
pub fn add_hinge(&mut self, parent_frame: Entity, child_frame: Entity, ...) -> Entity {
    let joint = self.ecs.spawn((
        InFrame(parent_frame),
        Connects(child_frame),
        Name::new("hip_joint"),
    )).id();

    let coord = self.ecs.spawn((
        HasDOF(joint),
        JointCoordinate { ... },
        Name::new("hip_flexion"),
    )).id();

    self.ecs.spawn((
        Drives(coord),
        CoordinateEffect { ... },
    ));

    joint
}
```

### Phase 6: Update importers (2-3 hours)

Replace `world.attach()` / `world.set_parent()` calls with Bevy API.

**OpenSim importer:**
- `import_opensim_body()` → `ecs.spawn((InertialProperties, Name, InFrame(parent)))`
- `import_*_joint()` → `ecs.spawn((InFrame(parent), Connects(child)))` + coordinate/effect entities

**MuJoCo importer:**
- Same pattern — replace World methods with Bevy spawn/insert

**MuJoCo Spec importer:**
- Same pattern

### Phase 7: Update exporters (2-3 hours)

Replace manual World iteration with Bevy queries.

**Current:**
```rust
for (entity, name) in world.iter::<Name>() {
    if world.get::<InertialProperties>(entity).is_some() {
        // body
    }
}
```

**New:**
```rust
let mut query = world.query::<(&Name, &InertialProperties)>();
for (name, inertial) in query.iter(&world) {
    // body
}
```

Update `infer_joint_kind` to use Bevy queries for relationship traversal.

### Phase 8: Update main.rs example (30 min)

Replace World API calls with Bevy API.

### Phase 9: Update tests (1 hour)

Update test assertions to use Bevy queries.

**Current:**
```rust
assert_eq!(world.count::<JointCoordinate>(), 7);
```

**New:**
```rust
let mut query = world.query::<&JointCoordinate>();
assert_eq!(query.iter(&world).count(), 7);
```

### Phase 10: Remove deprecated code (30 min)

- Delete `src/world.rs` (replaced by Bevy World)
- Delete `src/id.rs` (Bevy Entity replaces EntityID)
- Delete relationship boilerplate from World
- Clean up unused imports

---

## Verification Checklist

After migration:
- [ ] `cargo check` compiles with no errors
- [ ] `cargo test` passes all tests (same assertions, Bevy queries)
- [ ] OpenSim roundtrip works (import .osim → World → export .osim)
- [ ] MuJoCo roundtrip works (import .mjcf → World → export .mjcf)
- [ ] No `anymap2` dependency
- [ ] No custom World struct (uses bevy_ecs::World)
- [ ] No serde on ECS components
- [ ] Relationship sync is automatic (no set_* methods)

---

## Estimated Effort

| Phase | Time | Files Changed |
|-------|------|---------------|
| 1. Add dependency | 30 min | 1 (Cargo.toml) |
| 2. Replace World | 2-3 hrs | 2-3 (world.rs, lib.rs, main.rs) |
| 3. Convert components | 1 hr | 11 (all component files) |
| 4. Replace relationships | 2 hrs | 2 (relationship.rs, world.rs) |
| 5. Update builders | 1 hr | 1 (world.rs) |
| 6. Update importers | 2-3 hrs | 3 (opensim.rs, mujoco.rs, mujoco_spec.rs) |
| 7. Update exporters | 2-3 hrs | 4 (opensim.rs, mujoco.rs, mujoco_trait.rs, mjcf_components.rs) |
| 8. Update main.rs | 30 min | 1 |
| 9. Update tests | 1 hr | 7 (all test files) |
| 10. Cleanup | 30 min | 2 (delete world.rs, id.rs) |
| **Total** | **~15 hrs** | **~25 files** |

---

## Risk Mitigation

1. **f64 precision:** Use our own `Vec3`/`Quaternion` types (f64) as Bevy components. Don't use Bevy's `Vec3`/`Quat` (f32).

2. **Static model assumption:** melosim models are static after import. Bevy's generational entity IDs are fine — we don't delete entities at runtime.

3. **Rollback plan:** Keep the current `ecs-relationships` branch. If Bevy migration hits blockers, we can fall back to the custom ECS with typed relationships (current state of PR #2).

4. **Incremental migration:** Migrate one importer/exporter pair at a time (OpenSim first, then MuJoCo). Keep tests passing at each step.

---

## Success Criteria

The migration is successful if:
1. All existing tests pass with Bevy backend
2. Relationship sync is automatic (no manual set_* calls)
3. Importer/exporter code is shorter or same length
4. No performance regression on 200-entity models (measure before/after)
5. The codebase is easier to extend (new component types don't require World changes)
