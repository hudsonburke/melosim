# ECS Relationships Refactor — Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Replace `Frame`/`Site` components with primitive `Position`/`Rotation`/`ChildOf` components, refactor joints to connect frames via relationships instead of storing `body_a`/`body_b` fields, and remove explicit joint type strings.

**Architecture:** Entities become bundles of composable primitive components. A "frame" is an entity with `ChildOf` + `Position` + `Rotation`. A "site" is `ChildOf` + `Position` (no rotation). Joints connect frame entities via `ParentFrame`/`ChildFrame` relationship components. Joint types emerge from the `SpatialTransform`/`CoordinateEffect` configuration rather than a type string.

**Tech Stack:** Rust, serde, melosim custom ECS (AnyMap of `Vec<Option<T>>`)

---

## Phase 1: New Primitive Components

### Task 1: Create `ChildOf` component

**Objective:** Add a `ChildOf` relationship component that replaces `Frame.parent` and `Site.parent`.

**Files:**
- Create: `src/components/relationship.rs`
- Modify: `src/components/mod.rs`

**Step 1: Write the component**

```rust
// src/components/relationship.rs
use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// A relationship: this entity is a child of `parent`.
/// Replaces `Frame.parent` and `Site.parent` fields.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChildOf {
    pub parent: EntityID,
}
```

**Step 2: Add to mod.rs**

```rust
// Add to src/components/mod.rs
mod relationship;
pub use relationship::*;
```

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check`

**Step 4: Commit**

```bash
git add src/components/relationship.rs src/components/mod.rs
git commit -m "feat: add ChildOf relationship component"
```

---

### Task 2: Create `Position` and `Rotation` components

**Objective:** Add standalone `Position` and `Rotation` components that replace `Transform` in entity storage.

**Files:**
- Create: `src/components/spatial.rs`
- Modify: `src/components/mod.rs`

**Step 1: Write the components**

```rust
// src/components/spatial.rs
use serde::{Deserialize, Serialize};
use crate::math::{Vec3, Quaternion};

/// A 3D position in the parent's coordinate system.
/// Used alone for sites/landmarks (no rotation).
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Position {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn zero() -> Self {
        Self { x: 0.0, y: 0.0, z: 0.0 }
    }

    pub fn to_vec3(&self) -> Vec3 {
        Vec3 { x: self.x, y: self.y, z: self.z }
    }
}

impl From<Vec3> for Position {
    fn from(v: Vec3) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }
}

/// A rotation in the parent's coordinate system.
/// Combined with Position, this defines a full 6-DOF frame.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct Rotation {
    pub quaternion: Quaternion,
}

impl Rotation {
    pub fn identity() -> Self {
        Self { quaternion: Quaternion::identity() }
    }
}

impl From<Quaternion> for Rotation {
    fn from(q: Quaternion) -> Self {
        Self { quaternion: q }
    }
}
```

**Step 2: Add to mod.rs**

```rust
// Add to src/components/mod.rs
mod spatial;
pub use spatial::*;
```

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check`

**Step 4: Commit**

```bash
git add src/components/spatial.rs src/components/mod.rs
git commit -m "feat: add Position and Rotation components"
```

---

### Task 3: Create `ParentFrame` and `ChildFrame` relationship components for joints

**Objective:** Add relationship components that joints use to connect frame entities.

**Files:**
- Modify: `src/components/relationship.rs`

**Step 1: Add to relationship.rs**

```rust
/// A joint connects two frames.
/// ParentFrame points to the frame that serves as the joint's parent.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ParentFrame {
    pub frame: EntityID,
}

/// A joint connects two frames.
/// ChildFrame points to the frame that moves relative to the parent.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ChildFrame {
    pub frame: EntityID,
}
```

**Step 2: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check`

**Step 3: Commit**

```bash
git add src/components/relationship.rs
git commit -m "feat: add ParentFrame and ChildFrame relationship components for joints"
```

---

## Phase 2: Refactor Joint Component

### Task 4: Remove `body_a`, `body_b`, `joint_type` from Joint

**Objective:** Simplify `Joint` to only carry joint-specific data (limits, coordinates). The frame connections are now `ParentFrame`/`ChildFrame` components on the joint entity.

**Files:**
- Modify: `src/components/joint.rs`

**Step 1: Simplify Joint**

Replace the current Joint struct:

```rust
/// A joint entity connects two frames and defines a degree of freedom.
///
/// Frame connections are stored as `ParentFrame` and `ChildFrame`
/// components on the same entity. Joint type is emergent from the
/// `SpatialTransform`/`CoordinateEffect` configuration.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint {
    pub limits: Option<JointLimits>,
    pub coordinates: Vec<EntityID>,
}
```

**Step 2: Update validation**

Update `Validate for Joint` to check `ParentFrame`/`ChildFrame` components instead of `body_a`/`body_b`:

```rust
impl Validate for Joint {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        let mut e: Vec<String> = Vec::new();

        if let Some(err) = check_has::<super::ParentFrame>(world, entity, "parent_frame", entity) {
            e.push(err);
        }
        if let Some(err) = check_has::<super::ChildFrame>(world, entity, "child_frame", entity) {
            e.push(err);
        }

        for (i, coord_key) in self.coordinates.iter().enumerate() {
            if let Some(err) = check_has::<JointCoordinate>(world, entity, &format!("coordinates[{i}]"), *coord_key) {
                e.push(err);
            }
        }
        e
    }
}
```

**Step 3: Verify it compiles (will have errors in importers/exporters — expected)**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check 2>&1 | head -30`

Note: compile errors in importers/exporters are expected and will be fixed in later tasks.

**Step 4: Commit**

```bash
git add src/components/joint.rs
git commit -m "refactor: remove body_a/body_b/joint_type from Joint"
```

---

### Task 5: Update convenience builders to use new components

**Objective:** Update `add_hinge`, `add_ball`, `add_free`, `add_fixed`, `add_slide`, `add_universal`, `add_custom` to attach `ParentFrame`/`ChildFrame` components instead of storing body references in Joint.

**Files:**
- Modify: `src/world.rs`

**Step 1: Update add_hinge**

The builder now takes frame entities instead of body entities:

```rust
pub fn add_hinge(
    &mut self,
    parent_frame: EntityID,
    child_frame: EntityID,
    axis: [f64; 3],
    limits: Option<JointLimits>,
) -> EntityID {
    let joint_entity = self.spawn();

    // Relationship components
    self.attach(joint_entity, ParentFrame { frame: parent_frame });
    self.attach(joint_entity, ChildFrame { frame: child_frame });

    // Create coordinate
    let coord_entity = self.spawn();
    self.attach(coord_entity, JointCoordinate {
        range_min: limits.as_ref().map_or(-1e10, |l| l.lower),
        range_max: limits.as_ref().map_or(1e10, |l| l.upper),
        default_value: 0.0,
        stiffness: 0.0,
        damping: 0.0,
        clamped: limits.is_some(),
        locked: false,
        prescribed_function: None,
    });

    // Create CoordinateEffect
    let effect_entity = self.spawn();
    self.attach(effect_entity, CoordinateEffect {
        coordinate: coord_entity,
        joint: joint_entity,
        component: TransformComponent::RotationAboutAxis(axis),
        function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
    });

    // Create SpatialTransform
    let st_entity = self.spawn();
    self.attach(st_entity, SpatialTransform {
        joint: joint_entity,
        effects: vec![effect_entity],
    });

    // Create the joint (no body_a/body_b/joint_type)
    self.attach(joint_entity, Joint {
        limits,
        coordinates: vec![coord_entity],
    });

    joint_entity
}
```

**Step 2: Update all other builders** (add_ball, add_free, add_fixed, add_slide, add_universal, add_custom) with the same pattern — attach `ParentFrame`/`ChildFrame` instead of storing in Joint.

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check 2>&1 | head -30`

**Step 4: Commit**

```bash
git add src/world.rs
git commit -m "refactor: update convenience builders to use ParentFrame/ChildFrame"
```

---

## Phase 3: Update Importers

### Task 6: Update OpenSim importer to use new components

**Objective:** Replace all `Frame { parent, transform }` with `ChildOf` + `Position` + `Rotation`. Replace all `Site { parent, offset }` with `ChildOf` + `Position`. Update joint creation to use `ParentFrame`/`ChildFrame`.

**Files:**
- Modify: `src/importer/opensim.rs`

**Step 1: Find all Frame/Site creation sites**

Search for `Frame {` and `Site {` in the importer and replace each:

```rust
// Before:
world.attach(frame_entity, Frame {
    parent: parent_key,
    transform: Transform { translation, rotation },
});

// After:
world.attach(frame_entity, ChildOf { parent: parent_key });
world.attach(frame_entity, Position::new(translation.x, translation.y, translation.z));
world.attach(frame_entity, Rotation { quaternion: rotation });
```

```rust
// Before:
world.attach(site_entity, Site {
    parent: parent_key,
    offset: Vec3 { x, y, z },
});

// After:
world.attach(site_entity, ChildOf { parent: parent_key });
world.attach(site_entity, Position::new(x, y, z));
```

**Step 2: Update joint creation in importer**

Replace `body_a`/`body_b` with `ParentFrame`/`ChildFrame`:

```rust
// Before:
world.attach(joint_entity, Joint {
    body_a: parent_key,
    body_b: child_key,
    limits: ...,
    joint_type: "PinJoint",
    coordinates: coords,
});

// After:
world.attach(joint_entity, ParentFrame { frame: parent_frame_key });
world.attach(joint_entity, ChildFrame { frame: child_frame_key });
world.attach(joint_entity, Joint {
    limits: ...,
    coordinates: coords,
});
```

Note: The importer currently resolves body names to entity IDs. With the new model, it should resolve frame entity IDs (the entities that have `ChildOf` + `Position` + `Rotation`).

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check 2>&1 | head -30`

**Step 4: Commit**

```bash
git add src/importer/opensim.rs
git commit -m "refactor: update OpenSim importer to use relationship components"
```

---

### Task 7: Update MJCF importer to use new components

**Objective:** Same changes as Task 6 but for the MJCF importer.

**Files:**
- Modify: `src/importer/mujoco.rs`
- Modify: `src/importer/mujoco_spec.rs`

**Step 1: Replace Frame/Site creation with ChildOf + Position + Rotation**

Same pattern as Task 6.

**Step 2: Update joint creation**

Same pattern as Task 6 — use `ParentFrame`/`ChildFrame` instead of `body_a`/`body_b`.

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check 2>&1 | head -30`

**Step 4: Commit**

```bash
git add src/importer/mujoco.rs src/importer/mujoco_spec.rs
git commit -m "refactor: update MJCF importer to use relationship components"
```

---

## Phase 4: Update Exporters

### Task 8: Update OpenSim exporter to resolve relationships

**Objective:** Update the exporter to walk `ChildOf` relationships instead of reading `Frame.parent`. Resolve `ParentFrame`/`ChildFrame` on joints instead of reading `body_a`/`body_b`.

**Files:**
- Modify: `src/exporter/opensim.rs`

**Step 1: Add helper to find parent frame**

```rust
fn find_parent(world: &World, entity: EntityID) -> Option<EntityID> {
    world.get::<ChildOf>(entity).map(|c| c.parent)
}
```

**Step 2: Update `build_parent_set`**

Currently builds a set of `body_a` entities. Replace with:

```rust
fn build_parent_set(world: &World) -> std::collections::HashSet<EntityID> {
    let mut set = std::collections::HashSet::new();
    for (_, parent_frame) in world.iter::<ParentFrame>() {
        set.insert(parent_frame.frame);
    }
    set
}
```

**Step 3: Update body/frame traversal**

Replace `frame.parent` reads with `world.get::<ChildOf>(entity)`.

**Step 4: Update joint export**

Replace `joint.body_a`/`joint.body_b` reads with `world.get::<ParentFrame>(joint_entity)` / `world.get::<ChildFrame>(joint_entity)`.

**Step 5: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check 2>&1 | head -30`

**Step 6: Commit**

```bash
git add src/exporter/opensim.rs
git commit -m "refactor: update OpenSim exporter to resolve relationships"
```

---

### Task 9: Update MJCF exporter to resolve relationships

**Objective:** Same as Task 8 but for MJCF exporter.

**Files:**
- Modify: `src/exporter/mujoco.rs`
- Modify: `src/exporter/mujoco_trait.rs`
- Modify: `src/exporter/mjcf_components.rs`

**Step 1: Replace `frame.parent` reads with `ChildOf` resolution**

**Step 2: Update joint export to use `ParentFrame`/`ChildFrame`**

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check 2>&1 | head -30`

**Step 4: Commit**

```bash
git add src/exporter/mujoco.rs src/exporter/mujoco_trait.rs src/exporter/mjcf_components.rs
git commit -m "refactor: update MJCF exporter to resolve relationships"
```

---

### Task 10: Update `ExportAs<Mjcf>` for Site and Frame

**Objective:** Update the `ExportAs` implementations in `mjcf_components.rs` to use new components.

**Files:**
- Modify: `src/exporter/mjcf_components.rs`

**Step 1: Update `ExportAs<Mjcf> for Site`**

Replace `site.parent` with `ChildOf` resolution and `site.offset` with `Position`.

**Step 2: Update `ExportAs<Mjcf> for Frame`**

Replace `frame.parent`/`frame.transform` with `ChildOf`/`Position`/`Rotation`.

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check`

**Step 4: Commit**

```bash
git add src/exporter/mjcf_components.rs
git commit -m "refactor: update ExportAs implementations for new components"
```

---

## Phase 5: Update Muscle Paths and Tests

### Task 11: Update PathPoint to reference frames instead of bodies

**Objective:** Change `PathPoint::BodyFixed { body }` to `PathPoint::BodyFixed { frame }` so muscle paths reference frame entities.

**Files:**
- Modify: `src/components/path.rs`
- Modify: `src/importer/opensim.rs` (path import)
- Modify: `src/importer/mujoco.rs` (path import)
- Modify: `src/exporter/opensim.rs` (path export)

**Step 1: Update PathPoint enum**

```rust
pub enum PathPoint {
    BodyFixed {
        frame: EntityID,  // was: body
        location: [f64; 3],
    },
    Moving {
        frame: EntityID,  // was: body
        coordinate: EntityID,
        location_functions: [Vec<f64>; 3],
    },
}
```

**Step 2: Update all usages of `body` field to `frame`**

Search for `.body` in path-related code and update.

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check`

**Step 4: Commit**

```bash
git add src/components/path.rs src/importer/opensim.rs src/importer/mujoco.rs src/exporter/opensim.rs
git commit -m "refactor: update PathPoint to reference frames instead of bodies"
```

---

### Task 12: Update main.rs example

**Objective:** Update the main.rs example to use the new component model.

**Files:**
- Modify: `src/main.rs`

**Step 1: Replace Frame/Site usage**

```rust
// Before:
let ground_frame = world.spawn();
world.attach(ground_frame, Frame {
    parent: ground,
    transform: Transform::default(),
});

// After:
let ground_frame = world.spawn();
world.attach(ground_frame, ChildOf { parent: ground });
world.attach(ground_frame, Position::zero());
world.attach(ground_frame, Rotation::identity());
```

**Step 2: Update joint creation calls**

Change from body entities to frame entities:

```rust
// Before:
let _pelvis_free = world.add_free(ground, pelvis, None);
let _hip = world.add_hinge(pelvis, femur, [1.0, 0.0, 0.0], Some(...));

// After:
let _pelvis_free = world.add_free(ground_frame, pelvis_frame, None);
let _hip = world.add_hinge(pelvis_frame, femur_frame, [1.0, 0.0, 0.0], Some(...));
```

**Step 3: Verify it compiles and runs**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo run`

**Step 4: Commit**

```bash
git add src/main.rs
git commit -m "refactor: update main.rs example to use relationship components"
```

---

### Task 13: Update all tests

**Objective:** Update tests to use new components and verify the refactor works end-to-end.

**Files:**
- Modify: `tests/export_test.rs`
- Modify: `tests/import_test.rs`
- Modify: `tests/cross_format_test.rs`
- Modify: `tests/mjspec_roundtrip_test.rs`
- Modify: `tests/mjspec_complex_models_test.rs`
- Modify: `tests/mujoco_import_test.rs`
- Modify: `tests/mujoco_roundtrip_test.rs`

**Step 1: Update test helpers and assertions**

Replace any `Frame { parent, transform }` with `ChildOf` + `Position` + `Rotation`.
Replace any `joint.body_a`/`joint.body_b` reads with `ParentFrame`/`ChildFrame`.

**Step 2: Run all tests**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo test`

**Step 3: Fix any failures**

**Step 4: Commit**

```bash
git add tests/
git commit -m "refactor: update tests for relationship components"
```

---

## Phase 6: Cleanup

### Task 14: Remove deprecated Frame and Site types

**Objective:** Remove the old `Frame` and `Site` structs from `body.rs`. Move `StationDefinedFrame` to use `ChildOf` + `Position` + `Rotation`.

**Files:**
- Modify: `src/components/body.rs`

**Step 1: Remove Frame and Site structs**

**Step 2: Update StationDefinedFrame**

```rust
pub struct StationDefinedFrame {
    pub origin: EntityID,
    pub axis_x: EntityID,
    pub axis_y: EntityID,
}
```

**Step 3: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check`

**Step 4: Run all tests**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo test`

**Step 5: Commit**

```bash
git add src/components/body.rs
git commit -m "refactor: remove deprecated Frame and Site types"
```

---

### Task 15: Add World query helpers for relationships

**Objective:** Add convenience methods to World for common relationship queries.

**Files:**
- Modify: `src/world.rs`

**Step 1: Add parent_of helper**

```rust
/// Get the parent entity via ChildOf relationship.
pub fn parent_of(&self, entity: EntityID) -> Option<EntityID> {
    self.get::<ChildOf>(entity).map(|c| c.parent)
}

/// Get all children of an entity.
pub fn children_of(&self, entity: EntityID) -> Vec<EntityID> {
    self.iter::<ChildOf>()
        .filter(|(_, c)| c.parent == entity)
        .map(|(eid, _)| eid)
        .collect()
}

/// Get the parent frame of a joint.
pub fn joint_parent_frame(&self, joint: EntityID) -> Option<EntityID> {
    self.get::<ParentFrame>(joint).map(|p| p.frame)
}

/// Get the child frame of a joint.
pub fn joint_child_frame(&self, joint: EntityID) -> Option<EntityID> {
    self.get::<ChildFrame>(joint).map(|c| c.frame)
}
```

**Step 2: Verify it compiles**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo check`

**Step 3: Run all tests**

Run: `cd /var/lib/hermes/melosim && nix develop --command cargo test`

**Step 4: Commit**

```bash
git add src/world.rs
git commit -m "feat: add World query helpers for relationship traversal"
```

---

## Verification Checklist

After all tasks:

- [ ] `cargo check` passes with no errors
- [ ] `cargo test` passes all tests
- [ ] `cargo run` produces expected output
- [ ] No references to `Frame { parent` or `Site { parent` remain in codebase
- [ ] No references to `body_a` or `body_b` in Joint-related code
- [ ] All importers produce valid Worlds with new components
- [ ] All exporters correctly resolve relationships
- [ ] Roundtrip tests (OpenSim → World → OpenSim, MJCF → World → MJCF) pass
