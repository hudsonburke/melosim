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
use mujoco_rs::wrappers::mj_editing::{MjSpec, MjtLimited, SpecItem};
use mujoco_rs::wrappers::mj_model::MjtJoint;

use crate::model::{
    Body, Coordinate, CoordinateProperties, InertialProperties, Joint, JointCoordinates, Twist,
};

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
    props: Option<CoordinateProperties>,
}

/// A body in the export tree plus the joints that drive it.
struct BodySpec {
    name: String,
    gpos: Vec3,
    parent_body: Option<Entity>,
    inertial: Option<InertialProperties>,
    joint_world_pos: Option<Vec3>,
    coords: Vec<CoordSpec>,
}

/// melosim (Y-up) → MuJoCo (Z-up): rotate +90° about X (maps +Y → +Z).
fn zup(v: Vec3) -> Vec3 {
    Vec3::new(v.x, -v.z, v.y)
}

fn build_model(world: &mut World, spec: &mut MjSpec) -> Result<(), ExportError> {
    // ── Phase A: gather (borrows the world) ──
    let mut body_q =
        world.query_filtered::<(Entity, &Name, &GlobalTransform, Option<&InertialProperties>), With<Body>>();
    let mut child_of = world.query::<&ChildOf>();
    let mut body_marker = world.query::<&Body>();
    let mut joint_marker = world.query::<&Joint>();
    let mut joint_coords = world.query::<&JointCoordinates>();
    let mut coord_q = world.query_filtered::<(
        Entity,
        Option<&Name>,
        &Twist,
        Option<&CoordinateProperties>,
    ), With<Coordinate>>();
    let mut gt_q = world.query::<&GlobalTransform>();

    let mut coord_map: HashMap<Entity, CoordSpec> = HashMap::new();
    for (e, name, twist, props) in coord_q.iter(world) {
        coord_map.insert(
            e,
            CoordSpec {
                name: name
                    .map(|n| n.as_str().to_owned())
                    .unwrap_or_else(|| format!("coord_{e:?}")),
                twist: twist.clone(),
                props: props.cloned(),
            },
        );
    }

    let mut spec_map: HashMap<Entity, BodySpec> = HashMap::new();
    let mut nodes: Vec<Entity> = Vec::new();
    for (e, name, gt, inertial) in body_q.iter(world) {
        let mut probe = child_of.get(world, e).ok().map(|c| c.parent());
        let mut parent_body = None;
        let mut coords: Vec<CoordSpec> = Vec::new();
        let mut joint_world_pos = None;
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
                    joint_world_pos = gt_q.get(world, p).ok().map(|g| g.translation());
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
                inertial: inertial.cloned(),
                joint_world_pos,
                coords,
            },
        );
        nodes.push(e);
    }
    if nodes.is_empty() {
        return Err(ExportError::NoRoot);
    }

    // Child → nearest Body ancestor map.
    let parent: HashMap<Entity, Entity> = spec_map
        .iter()
        .filter_map(|(e, b)| b.parent_body.map(|p| (*e, p)))
        .collect();

    // Roots = bodies that are nobody's child. `parent` maps child → parent, so
    // the child set is its KEYS (not values — a value is a *parent*).
    let child_set: HashSet<Entity> = parent.keys().copied().collect();
    let mut roots: Vec<Entity> = nodes.iter().copied().filter(|e| !child_set.contains(e)).collect();
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

    // Emit inertial from InertialProperties (mass, COM, full inertia matrix).
    if let Some(inert) = &spec.inertial {
        let com = inert.mass_center;
        child_mjs
            .with_mass(inert.mass)
            .with_fullinertia(inert.inertia.0) // (Ixx,Iyy,Izz,Ixy,Ixz,Iyz) == MuJoCo fullinertia
            .with_ipos([com.x, com.y, com.z])
            .with_explicitinertial(true);
    }

    // Joint location relative to the nearest Body ancestor (or the body-local pos).
    let joint_pos = match (spec.joint_world_pos, parent_world_pos) {
        (Some(jp), Some(pp)) => zup(jp - pp),
        _ => pos,
    };
    for c in &spec.coords {
        add_joint(&mut *child_mjs, c, [joint_pos.x as f64, joint_pos.y as f64, joint_pos.z as f64]);
    }

    let gpos = spec.gpos;
    let kids = children.get(&e).map(|v| v.clone()).unwrap_or_default();
    for k in kids {
        write_body_recursive(&mut *child_mjs, k, Some(gpos), bodies, children)?;
    }
    Ok(())
}

/// Emit one `<joint>` per coordinate: hinge if `Twist.angular` dominates,
/// slide if `Twist.linear` dominates. Carries limit (`range`/`limited`) and
/// `damping` from `CoordinateProperties`; `stiffness` isn't a MuJoCo joint
/// attribute in this crate (it's modeled via actuators/springs instead).
fn add_joint(
    body_mjs: &mut mujoco_rs::wrappers::mj_editing::MjsBody,
    c: &CoordSpec,
    pos: [f64; 3],
) {
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

    let mut j = body_mjs
        .add_joint()
        .with_name(&c.name)
        .with_type(kind)
        .with_pos(pos)
        .with_axis(axis);

    if let Some(p) = &c.props {
        if p.clamped {
            j = j.with_range([p.range.0, p.range.1]).with_limited(MjtLimited::mjLIMITED_TRUE);
        }
        if p.damping > 0.0 {
            // mjNPOLY = 2 → damping coefficient array has 3 entries.
            j = j.with_damping([p.damping, 0.0, 0.0]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{
        Coordinate, CoordinateProperties, Inertia, InertialProperties, Joint, JointCoordinates,
        Twist,
    };

    /// P1+P2: body tree + a hinge joint export → MuJoCo XML.
    #[test]
    fn exports_body_tree_with_joint() {
        let mut world = World::new();

        let inertial = || InertialProperties {
            mass: 1.0,
            mass_center: nalgebra::Vector3::new(0.0, 0.0, 0.0),
            inertia: Inertia::new(0.01, 0.01, 0.01, 0.0, 0.0, 0.0),
        };

        // anchor (root) body.
        let anchor = world
            .spawn((Name::new("anchor"), Body, inertial(), Transform::IDENTITY, GlobalTransform::IDENTITY))
            .id();

        // one coordinate driving a hinge (axis = +Z in the twist/local frame),
        // with a limit range + damping.
        let coord = world
            .spawn((
                Name::new("knee_flex"),
                Coordinate,
                CoordinateProperties {
                    range: (-1.5, 1.5),
                    clamped: true,
                    locked: false,
                    stiffness: 0.0,
                    damping: 1.0,
                },
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
                inertial(),
                Transform::from_xyz(0.0, 1.0, 0.0),
                GlobalTransform::from(Transform::from_xyz(0.0, 1.0, 0.0)),
            ))
            .id();
        world.entity_mut(seg1).insert(ChildOf(joint));

        let mut spec = to_mjcf(&mut world, anchor).expect("export should succeed");
        let _model = spec.compile().expect("compile should succeed");
        let xml = spec.save_xml_string(1 << 16).expect("serialize should succeed");

        // Bodies.
        assert!(xml.contains("anchor"), "missing anchor: {xml}");
        assert!(xml.contains("seg1"), "missing seg1: {xml}");
        assert!(xml.contains("<body"), "no <body>: {xml}");
        // Joint (coordinate name) emitted, with limit range + damping.
        assert!(xml.contains("knee_flex"), "missing joint (knee_flex): {xml}");
        assert!(xml.contains("<joint"), "no <joint>: {xml}");
        assert!(xml.contains("range"), "missing joint range: {xml}");
        assert!(xml.contains("damping"), "missing joint damping: {xml}");
    }
}
