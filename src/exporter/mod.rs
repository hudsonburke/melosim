//! Export a melosim model (the live Bevy `World`) as a MuJoCo MJCF model.
//!
//! Built programmatically through `mujoco-rs`'s **MjSpec** editing API — we
//! never hand-write MJCF XML; MuJoCo's own compiler serializes it. This is the
//! inverse of the importer, which *parses* MJCF via the same MjSpec.
//!
//! P1: worldbody tree — bodies (+ hierarchy, relative positions). Joints,
//! sites, tendons, actuators land in later phases.
//! See `docs/plans/2026-08-20-mujoco-exporter.md`.

use std::collections::{HashMap, HashSet};

use bevy::ecs::world::World;
use bevy::prelude::*;
use mujoco_rs::wrappers::mj_editing::{MjSpec, MjsBody, SpecItem};

use crate::model::Body;

/// Export failure.
#[derive(Debug)]
pub enum ExportError {
    /// No model root Body found in the world.
    NoRoot,
    /// An MjSpec mutation was rejected.
    MjSpec(String),
}

/// Build an `MjSpec` representing the model under `root`.
///
/// Pure read of the world's components (no structural mutation), so it is
/// unit-testable. `&mut World` is required only because Bevy `query()` needs it.
pub fn to_mjcf(world: &mut World, _root: Entity) -> Result<MjSpec, ExportError> {
    let mut spec = MjSpec::new();
    build_body_tree(world, &mut spec)?;
    Ok(spec)
}

/// melosim (Y-up) → MuJoCo (Z-up): rotate +90° about X (maps +Y → +Z).
fn zup(v: Vec3) -> Vec3 {
    Vec3::new(v.x, -v.z, v.y)
}

/// Emit every `Body` as an `MjsBody`, arranged by their `ChildOf` hierarchy
/// (the kinematic parent of a body is its nearest `Body` ancestor, found by
/// walking up through intermediate `Frame`/`Joint` entities).
fn build_body_tree(world: &mut World, spec: &mut MjSpec) -> Result<(), ExportError> {
    let mut bodies = world.query_filtered::<(Entity, &Name, &GlobalTransform), With<Body>>();
    let mut child_of = world.query::<&ChildOf>();
    let mut body_marker = world.query::<&Body>();

    let mut gpos: HashMap<Entity, Vec3> = HashMap::new();
    let mut names: HashMap<Entity, String> = HashMap::new();
    let mut nodes: Vec<Entity> = Vec::new();
    for (e, name, gt) in bodies.iter(world) {
        gpos.insert(e, gt.translation());
        names.insert(e, name.as_str().to_owned());
        nodes.push(e);
    }
    if nodes.is_empty() {
        return Err(ExportError::NoRoot);
    }

    // Nearest Body ancestor (the kinematic parent in the export tree).
    let mut parent: HashMap<Entity, Entity> = HashMap::new();
    for &e in &nodes {
        let mut cur = child_of.get(world, e).ok().map(|c| c.parent());
        while let Some(p) = cur {
            if p != e && body_marker.get(world, p).is_ok() {
                parent.insert(e, p);
                break;
            }
            cur = child_of.get(world, p).ok().map(|c| c.parent());
        }
    }

    let parent_set: HashSet<Entity> = parent.values().copied().collect();
    let mut roots: Vec<Entity> = nodes.iter().copied().filter(|e| !parent_set.contains(e)).collect();
    roots.sort_by_key(|e| e.index());

    let mut children: HashMap<Entity, Vec<Entity>> = HashMap::new();
    for (e, p) in &parent {
        children.entry(*p).or_default().push(*e);
    }
    for v in children.values_mut() {
        v.sort_by_key(|e| e.index());
    }

    let world_body = spec.world_body_mut();
    for root in roots {
        write_body_recursive(world_body, root, None, &gpos, &names, &children)?;
    }
    Ok(())
}

/// Recursively create an `MjsBody` (child of `parent_mjs`) for `e`, then its
/// `Body` children. `parent_world_pos` is the kinematic parent's global
/// position (None for a root body, which sits at the origin).
fn write_body_recursive(
    parent_mjs: &mut MjsBody,
    e: Entity,
    parent_world_pos: Option<Vec3>,
    gpos: &HashMap<Entity, Vec3>,
    names: &HashMap<Entity, String>,
    children: &HashMap<Entity, Vec<Entity>>,
) -> Result<(), ExportError> {
    let pos = match parent_world_pos {
        Some(p) => zup(gpos[&e] - p),
        None => Vec3::ZERO,
    };
    let name = names[&e].clone();

    let child_mjs = parent_mjs
        .add_body()
        .with_name(&name)
        .with_pos([pos.x as f64, pos.y as f64, pos.z as f64]);

    let kids = children.get(&e).map(|v| v.clone()).unwrap_or_default();
    for k in kids {
        write_body_recursive(&mut *child_mjs, k, Some(gpos[&e]), gpos, names, children)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Body;

    /// P1: body tree export → MuJoCo XML with the right body names.
    #[test]
    fn exports_body_tree() {
        let mut world = World::new();

        // root body at origin + one child body at [1,2,3] in its parent frame.
        let anchor = world
            .spawn((Name::new("anchor"), Body, Transform::IDENTITY, GlobalTransform::IDENTITY))
            .id();
        let seg1 = world
            .spawn((
                Name::new("seg1"),
                Body,
                Transform::from_xyz(1.0, 2.0, 3.0),
                GlobalTransform::from(Transform::from_xyz(1.0, 2.0, 3.0)),
            ))
            .id();
        world.entity_mut(seg1).insert(ChildOf(anchor));

        let spec = to_mjcf(&mut world, anchor).expect("export should succeed");
        let xml = spec.save_xml_string(1 << 16).expect("serialize should succeed");

        assert!(xml.contains("anchor"), "missing root body name: {xml}");
        assert!(xml.contains("seg1"), "missing child body name: {xml}");
        assert!(xml.contains("<body"), "expected at least one <body>: {xml}");
    }
}
