//! Export a melosim model (the live Bevy `World`) as a MuJoCo MJCF model.
//!
//! Built programmatically through `mujoco-rs`'s **MjSpec** editing API — we
//! never hand-write MJCF XML; MuJoCo's own compiler serializes it. This is the
//! inverse of the importer, which *parses* MJCF via the same MjSpec.
//!
//! P1: worldbody tree. P2: joints (hinge/slide from `Twist`). P3 tendons,
//! P4 actuators, P5 meshes, P6 editor action.
//! See `docs/plans/2026-08-20-mujoco-exporter.md`.

use std::collections::{HashMap, HashSet};

use bevy::ecs::world::World;
use bevy::prelude::*;
use mujoco_rs::wrappers::mj_editing::{MjSpec, SpecItem};
use mujoco_rs::wrappers::mj_model::MjtJoint;

use crate::model::{Body, Coordinate, Joint, JointCoordinates, Twist};

/// Export failure.
#[derive(Debug)]
pub enum ExportError {
    NoRoot,
    MjSpec(String),
}

/// Build an `MjSpec` representing the model under `root`.
pub fn to_mjcf(world: &mut World, _root: Entity) -> Result<MjSpec, ExportError> {
    let mut spec = MjSpec::new();
    build_model(world, &mut spec)?;
    Ok(spec)
}

/// A coordinate to emit as a joint on the child body.
#[derive(Clone)]
struct CoordSpec {
    name: String,
    twist: Twist,
}

/// A body in the export tree plus the joints that drive it.
struct BodySpec {
    name: String,
    gpos: Vec3,
    parent_body: Option<Entity>,
    coords: Vec<CoordSpec>,
}

fn build_model(world: &mut World, spec: &mut MjSpec) -> Result<(), ExportError> {
    // ── Phase A: gather (borrows the world) ──
    let mut body_q = world.query_filtered::<(Entity, &Name, &GlobalTransform), With<Body>>();
    let mut child_of = world.query::<&ChildOf>();
    let mut body_marker = world.query::<&Body>();
    let mut joint_marker = world.query::<&Joint>();
    let mut joint_coords = world.query::<&JointCoordinates>();
    let mut coord_q =
        world.query_filtered::<(Entity, Option<&Name>, &Twist), With<Coordinate>>();

    let mut coord_map: HashMap<Entity, CoordSpec> = HashMap::new();
    for (e, name, twist) in coord_q.iter(world) {
        coord_map.insert(
            e,
            CoordSpec {
                name: name
                    .map(|n| n.as_str().to_owned())
                    .unwrap_or_else(|| format!("coord_{e:?}")),
                twist: twist.clone(),
            },
        );
    }

    let mut spec_map: HashMap<Entity, BodySpec> = HashMap::new();
    let mut nodes: Vec<Entity> = Vec::new();
    for (e, name, gt) in body_q.iter(world) {
        let mut probe = child_of.get(world, e).ok().map(|c| c.parent());
        let mut parent_body = None;
        let mut coords: Vec<CoordSpec> = Vec::new();
        while let Some(p) = probe {
            if p != e {
                if joint_marker.get(world, p).is_ok() {
                    coords = joint_coords
                        .get(world, p)
                        .map(|jc| jc.iter().collect::<Vec<_>>())
                        .unwrap_or_default()
                        .into_iter()
                        .filter_map(|c| coord_map.get(&c).cloned())
                        .collect();
                }
                if body_marker.get(world, p).is_ok() {
                    parent_body = Some(p);
                    break;
                }
            }
            probe = child_of.get(world, p).ok().map(|c| c.parent());
        }
        spec_map.insert(
            e,
            BodySpec {
                name: name.as_str().to_owned(),
                gpos: gt.translation(),
                parent_body,
                coords,
            },
        );
        nodes.push(e);
    }
    if nodes.is_empty() {
        return Err(ExportError::NoRoot);
    }

    let parent_set: HashSet<Entity> = spec_map.values().filter_map(|b| b.parent_body).collect();
    let mut roots: Vec<Entity> = nodes.iter().copied().filter(|e| !parent_set.contains(e)).collect();
    roots.sort_by_key(|e| e.index());
    let mut children: HashMap<Entity, Vec<Entity>> = HashMap::new();
    for (e, b) in &spec_map {
        if let Some(p) = b.parent_body {
            children.entry(p).or_default().push(*e);
        }
    }
    for v in children.values_mut() {
        v.sort_by_key(|e| e.index());
    }

    // ── Phase B: emit (owned data only) ──
    let world_body = spec.world_body_mut();
    for root in roots {
        write_body_recursive(world_body, root, None, &spec_map, &children)?;
    }
    Ok(())
}

fn write_body_recursive(
    parent_mjs: &mut mujoco_rs::wrappers::mj_editing::MjsBody,
    e: Entity,
    parent_world_pos: Option<Vec3>,
    bodies: &HashMap<Entity, BodySpec>,
    children: &HashMap<Entity, Vec<Entity>>,
) -> Result<(), ExportError> {
    let spec = &bodies[&e];
    // Position relative to the nearest Body ancestor, melosim Y-up → Z-up.
    let pos = match parent_world_pos {
        Some(p) => Vec3::new(spec.gpos.x, -spec.gpos.z, spec.gpos.y) - Vec3::new(p.x, -p.z, p.y),
        None => Vec3::ZERO,
    };

    let child_mjs = parent_mjs
        .add_body()
        .with_name(&spec.name)
        .with_pos([pos.x as f64, pos.y as f64, pos.z as f64]);

    for c in &spec.coords {
        add_joint(&mut *child_mjs, c);
    }

    let gpos = spec.gpos;
    let kids = children.get(&e).map(|v| v.clone()).unwrap_or_default();
    for k in kids {
        write_body_recursive(&mut *child_mjs, k, Some(gpos), bodies, children)?;
    }
    Ok(())
}

/// Emit one `<joint>` per coordinate: hinge if `Twist.angular` dominates,
/// slide if `Twist.linear` dominates.
fn add_joint(body_mjs: &mut mujoco_rs::wrappers::mj_editing::MjsBody, c: &CoordSpec) {
    let ang = c.twist.angular.norm();
    let lin = c.twist.linear.norm();
    let is_hinge = ang > lin && ang > 1e-6;

    let kind = if is_hinge { MjtJoint::mjJNT_HINGE } else { MjtJoint::mjJNT_SLIDE };
    let v = if is_hinge { c.twist.angular } else { c.twist.linear };
    let v = if v.norm() > 1e-9 {
        v.normalize()
    } else {
        nalgebra::Vector3::new(1.0, 0.0, 0.0)
    };
    // melosim Y-up → MuJoCo Z-up axis.
    let axis = [v.x, -v.z, v.y];

    body_mjs.add_joint().with_name(&c.name).with_type(kind).with_axis(axis);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Coordinate, Joint, JointCoordinates, Twist};

    /// P1+P2: body tree + a hinge joint export → MuJoCo XML.
    #[test]
    fn exports_body_tree_with_joint() {
        let mut world = World::new();

        // anchor (root) body.
        let anchor = world
            .spawn((Name::new("anchor"), Body, Transform::IDENTITY, GlobalTransform::IDENTITY))
            .id();

        // one coordinate driving a hinge (axis = +Z in the twist/local frame).
        let coord = world
            .spawn((
                Name::new("knee_flex"),
                Coordinate,
                Twist {
                    angular: nalgebra::Vector3::new(0.0, 0.0, 1.0),
                    linear: nalgebra::Vector3::new(0.0, 0.0, 0.0),
                },
            ))
            .id();

        // a 1-dof joint owning that coordinate, parented to anchor.
        let joint = world
            .spawn((
                Name::new("knee"),
                Joint,
                JointCoordinates::new(vec![coord]),
                GlobalTransform::IDENTITY,
            ))
            .id();
        world.entity_mut(joint).insert(ChildOf(anchor));

        // child body driven by the joint, offset from anchor.
        let seg1 = world
            .spawn((
                Name::new("seg1"),
                Body,
                Transform::from_xyz(0.0, 1.0, 0.0),
                GlobalTransform::from(Transform::from_xyz(0.0, 1.0, 0.0)),
            ))
            .id();
        world.entity_mut(seg1).insert(ChildOf(joint));

        let spec = to_mjcf(&mut world, anchor).expect("export should succeed");
        let xml = spec.save_xml_string(1 << 16).expect("serialize should succeed");

        // Bodies.
        assert!(xml.contains("anchor"), "missing anchor: {xml}");
        assert!(xml.contains("seg1"), "missing seg1: {xml}");
        assert!(xml.contains("<body"), "no <body>: {xml}");
        // Joint (coordinate name) emitted.
        assert!(xml.contains("knee_flex"), "missing joint (knee_flex): {xml}");
        assert!(xml.contains("<joint"), "no <joint>: {xml}");
    }
}
