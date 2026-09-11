//! Export a melosim model (the live Bevy `World`) as a MuJoCo MJCF model.
use std::collections::{HashMap, HashSet};
use std::{sync::Arc, path::Path};
use crate::model::MeshGeometry;
use bevy::ecs::world::World;
use bevy::prelude::*;
use mujoco_rs::wrappers::mj_editing::*;
use mujoco_rs::wrappers::mj_model::MjtJoint;
use crate::model::{Body, Cable, CableParameters, Coordinate, CoordinateProperties, InertialProperties, JointCoordinates, Muscle, PathEntities, Site, Twist, Vector3};

#[derive(Debug)]
pub enum ExportError {
    NoRoot,
    Asset(String),
}

/// Export the model in `world` to an `MjSpec`.
///
/// Imported mesh sources and their local poses are retained. Standalone GLTF
/// scenes still require an explicit physical geometry representation.
pub fn to_mjcf(world: &mut World, root: Entity) -> Result<MjSpec, ExportError> {
    to_mjcf_impl(world, root, false)
}
pub fn to_mjcf_visual(world: &mut World, root: Entity) -> Result<MjSpec, ExportError> {
    to_mjcf_impl(world, root, true)
}
fn to_mjcf_impl(world: &mut World, root: Entity, visual_only: bool) -> Result<MjSpec, ExportError> {
    let mut document = world.get::<crate::mjcf_document::MjcfDocument>(root).cloned();
    if let Some(source) = &document {
        if visual_only { document = Some(source.visual_only().map_err(ExportError::Asset)?); }
        else if let Some(warning) = &source.warning { return Err(ExportError::Asset(format!("Source model cannot compile: {warning}. Choose visual-only export to omit simulation sections."))); }
    }
    let mut spec = MjSpec::new();
    spec.compiler_mut().set_balanceinertia(true);
    spec.compiler_mut().set_degree(false);
    let mut meshes = HashMap::new();
    for geometry in world.query::<&MeshGeometry>().iter(world) {
        let key = Arc::as_ptr(&geometry.source) as usize;
        if meshes.contains_key(&key) { continue; }
        let name = format!("melosim_generated_mesh_{}", meshes.len());
        let mesh = spec.add_mesh().with_name(&name);
        let source = &geometry.source;
        if let Some(file) = &source.file {
            if !file.is_file() { return Err(ExportError::Asset(format!("missing mesh: {}", file.display()))); }
            mesh.set_file(&file.to_string_lossy());
            mesh.with_scale(source.scale).with_refpos(source.refpos).with_refquat(source.refquat);
        } else {
            mesh.set_uservert(&source.vertices.iter().flatten().copied().collect::<Vec<_>>());
            mesh.set_userface(&source.faces.iter().flatten().copied().collect::<Vec<_>>());
        }
        meshes.insert(key, name);
    }
    build_model(world, &mut spec, &meshes, document.as_ref(), visual_only)?;
    if let Some(document) = document {
        let names = document.names.iter().filter_map(|(e, old)| world.get::<Name>(*e).filter(|n| n.as_str() != old).map(|n| (old.clone(), n.as_str().to_owned()))).collect();
        let colors = document.colors.iter().filter_map(|(e, old)| world.get::<MeshGeometry>(*e).filter(|g| g.rgba != *old).and_then(|_| world.get::<Name>(*e)).map(|n| n.as_str().to_owned())).collect();
        // mj_saveXML requires compilation before serializing a generated spec.
        spec.compile().map_err(|e| ExportError::Asset(e.to_string()))?;
        crate::mjcf_document::merge(&document, &spec, &names, &colors).map_err(ExportError::Asset)
    } else { Ok(spec) }
}
#[derive(Clone)]
struct CoordSpec { name: String, twist: Twist, props: Option<CoordinateProperties> }
struct BodySpec {
    name: String, gpos: Vec3, grot: Quat, parent_body: Option<Entity>,
    inertial: Option<InertialProperties>, joint_pos: Vec3,
    coords: Vec<CoordSpec>, sites: Vec<(String, Vec3)>,
    geoms: Vec<(String, String, MeshGeometry, Transform)>,
}
fn zup(v: Vec3) -> Vec3 { Vec3::new(v.x, -v.z, v.y) }

/// Find the incoming joint above this body and collect its descendant sites,
/// stopping at child bodies. All positions are collected in world coordinates.
fn collect_subtree_joints_sites(
    world: &World,
    body_entity: Entity,
    parent_to_children: &HashMap<Entity, Vec<Entity>>,
    coord_map: &HashMap<Entity, CoordSpec>,
    body_joint: &mut HashMap<Entity, (Vec<CoordSpec>, Vec3)>,
    body_site: &mut HashMap<Entity, Vec<(String, Vec3)>>,
    visited: &mut HashSet<Entity>,
) {
    let mut ancestor = world.get::<ChildOf>(body_entity).map(ChildOf::parent);
    while let Some(entity) = ancestor {
        if world.get::<Body>(entity).is_some() { break; }
        if let Some(joint) = world.get::<JointCoordinates>(entity) {
            let jp = world.get::<GlobalTransform>(entity).map(|g| g.translation()).unwrap_or(Vec3::ZERO);
            let cs = joint.iter().filter_map(|c| coord_map.get(&c).cloned()).collect();
            body_joint.insert(body_entity, (cs, jp));
            break;
        }
        ancestor = world.get::<ChildOf>(entity).map(ChildOf::parent);
    }
    let mut stack = parent_to_children.get(&body_entity).cloned().unwrap_or_default();
    visited.insert(body_entity);
    while let Some(child) = stack.pop() {
        if visited.contains(&child) { continue; }
        visited.insert(child);
        if world.get::<Body>(child).is_some() { continue; }
        if world.get::<Site>(child).is_some() {
            let sn = world.get::<Name>(child).map(|n| n.as_str().to_owned()).unwrap_or_else(|| format!("s{child:?}"));
            let sp = world.get::<GlobalTransform>(child).map(|g| g.translation()).unwrap_or(Vec3::ZERO);
            body_site.entry(body_entity).or_default().push((sn, sp));
        }
        if let Some(grandchildren) = parent_to_children.get(&child) {
            stack.extend(grandchildren);
        }
    }
}

fn build_model(world: &mut World, spec: &mut MjSpec, meshes: &HashMap<usize, String>, document: Option<&crate::mjcf_document::MjcfDocument>, visual_only: bool) -> Result<(), ExportError> {
    let child_to_parent: HashMap<Entity, Entity> = world
        .query::<(Entity, &ChildOf)>().iter(world)
        .map(|(e, co)| (e, co.parent())).collect();
    let parent_to_children: HashMap<Entity, Vec<Entity>> = {
        let mut m: HashMap<Entity, Vec<Entity>> = HashMap::new();
        for (&c, &p) in &child_to_parent { m.entry(p).or_default().push(c); }
        m
    };
    let coord_map: HashMap<Entity, CoordSpec> = world
        .query_filtered::<(Entity, Option<&Name>, &Twist, Option<&CoordinateProperties>), With<Coordinate>>()
        .iter(world).map(|(e, n, tw, p)| (e, CoordSpec {
            name: n.map(|s| s.as_str().to_owned()).unwrap_or_else(|| format!("c{e:?}")),
            twist: tw.clone(), props: p.cloned(),
        })).collect();

    // Collect joints and sites recursively per body subtree.
    let mut body_joint: HashMap<Entity, (Vec<CoordSpec>, Vec3)> = HashMap::new();
    let mut body_site: HashMap<Entity, Vec<(String, Vec3)>> = HashMap::new();
    let mut visited: HashSet<Entity> = HashSet::new();
    for (e, _name, _gt, _inertial) in world
        .query_filtered::<(Entity, &Name, &GlobalTransform, Option<&InertialProperties>), With<Body>>()
        .iter(world)
    {
        collect_subtree_joints_sites(world, e, &parent_to_children, &coord_map, &mut body_joint, &mut body_site, &mut visited);
    }

    let mut geoms: HashMap<Entity, Vec<(String, String, MeshGeometry, Transform)>> = HashMap::new();
    for (entity, name, geometry, gt) in world.query::<(Entity, &Name, &MeshGeometry, &GlobalTransform)>().iter(world) {
        let mut ancestor = child_to_parent.get(&entity).copied();
        let mut owner = None;
        while let Some(parent) = ancestor {
            if world.get::<Body>(parent).is_some() { owner = Some(parent); break; }
            ancestor = child_to_parent.get(&parent).copied();
        }
        let mut pose = if let Some(body) = owner {
            gt.reparented_to(world.get::<GlobalTransform>(body).unwrap())
        } else {
            let mut pose = gt.compute_transform();
            pose.translation = zup(pose.translation);
            pose.rotation = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2) * pose.rotation;
            pose
        };
        if geometry.source.file.is_some() || document.is_some_and(|d| d.names.contains_key(&entity)) {
            // Remove only the compiler's rigid frame. Source scale/ref transforms
            // remain on the mesh asset and are reapplied by MuJoCo.
            let frame = geometry.source.compiled_frame;
            let inverse = Transform::from_rotation(frame.rotation.inverse())
                .with_translation(frame.rotation.inverse() * -frame.translation);
            pose = pose.mul_transform(inverse);
        }
        if (pose.scale - Vec3::ONE).length() > 1e-5 {
            return Err(ExportError::Asset(format!("scaled geom '{}' requires baking before export", name.as_str())));
        }
        let asset = meshes[&(Arc::as_ptr(&geometry.source) as usize)].clone();
        geoms.entry(owner.unwrap_or(Entity::PLACEHOLDER)).or_default()
            .push((name.as_str().to_owned(), asset, geometry.clone(), pose));
    }
    let mut spec_map: HashMap<Entity, BodySpec> = HashMap::new();
    let mut nodes = Vec::new();
    for (e, name, gt, inertial) in world
        .query_filtered::<(Entity, &Name, &GlobalTransform, Option<&InertialProperties>), With<Body>>()
        .iter(world)
    {
        let (coords, joint_pos) = body_joint.remove(&e).unwrap_or_default();
        let sites = body_site.remove(&e).unwrap_or_default();

        // Walk up the ChildOf chain to find the nearest ancestor with a Body
        // component — that is this body's parent in the MJCF hierarchy.
        let mut parent_body = None;
        let mut cur = child_to_parent.get(&e).copied();
        while let Some(p) = cur {
            if p != e && world.get::<Body>(p).is_some() { parent_body = Some(p); break; }
            cur = child_to_parent.get(&p).copied();
        }
        spec_map.insert(e, BodySpec { name: name.as_str().to_owned(), gpos: gt.translation(), grot: gt.rotation(), parent_body, inertial: inertial.cloned(), joint_pos, coords, sites, geoms: geoms.remove(&e).unwrap_or_default() });
        nodes.push(e);
    }
    if nodes.is_empty() { return Err(ExportError::NoRoot); }
    let mut roots: Vec<Entity> = nodes.iter().copied().filter(|e| spec_map[e].parent_body.is_none()).collect();
    roots.sort_by_key(|e| e.index());
    let mut children_map: HashMap<Entity, Vec<Entity>> = HashMap::new();
    for (e, b) in &spec_map { if let Some(p) = b.parent_body { children_map.entry(p).or_default().push(*e); } }
    for v in children_map.values_mut() { v.sort_by_key(|e| e.index()); }

    let wb = spec.world_body_mut();
    for geom in geoms.remove(&Entity::PLACEHOLDER).unwrap_or_default() { write_geom(wb, &geom); }
    for r in roots { write_body(wb, r, None, None, &spec_map, &children_map)?; }

    // Export tendons (muscle paths).
    if !visual_only { build_tendons(world, spec); }

    Ok(())
}

/// Export muscle and cable paths as MuJoCo spatial tendons.
fn build_tendons(world: &mut World, spec: &mut MjSpec) {
    let muscles: Vec<(String, Vec<String>)> = world
        .query::<(&Name, &Muscle, &PathEntities)>()
        .iter(world)
        .filter_map(|(name, _muscle, path)| {
            let sites = path
                .iter()
                .filter_map(|entity| world.get::<Name>(entity).map(|name| name.as_str().to_owned()))
                .collect::<Vec<_>>();
            (!sites.is_empty()).then_some((name.as_str().to_owned(), sites))
        })
        .collect();
    for (name, sites) in muscles {
        add_spatial_tendon(spec, &name, &sites, true);
    }

    let cables: Vec<(String, Vec<String>)> = world
        .query::<(&Name, &Cable, &PathEntities, Option<&CableParameters>)>()
        .iter(world)
        .filter_map(|(name, _cable, path, _parameters)| {
            let sites = path
                .iter()
                .filter_map(|entity| world.get::<Name>(entity).map(|name| name.as_str().to_owned()))
                .collect::<Vec<_>>();
            (!sites.is_empty()).then_some((name.as_str().to_owned(), sites))
        })
        .collect();
    for (name, sites) in cables {
        add_spatial_tendon(spec, &name, &sites, true);
    }
}

fn add_spatial_tendon(spec: &mut MjSpec, name: &str, site_names: &[String], add_actuator: bool) {
    let tendon_name = format!("{name}_tendon");
    let tendon = spec.add_tendon().with_name(&tendon_name);
    for site_name in site_names {
        tendon.wrap_site(site_name);
    }
    if add_actuator {
        let actuator = spec.add_actuator().with_name(name);
        actuator.set_target(&tendon_name);
        actuator.with_trntype(mujoco_rs::wrappers::mj_model::MjtTrn::mjTRN_TENDON);
    }
}

fn write_body(p: &mut MjsBody, e: Entity, pw: Option<Vec3>, pgrot: Option<Quat>,
    bodies: &HashMap<Entity, BodySpec>, children: &HashMap<Entity, Vec<Entity>>,
) -> Result<(), ExportError> {
    let s = &bodies[&e];
    let pos = match (pw, pgrot) { (Some(q), Some(r)) => r.inverse() * (s.gpos - q), _ => zup(s.gpos) };
    let mut b = p.add_body().with_name(&s.name).with_pos([pos.x as f64, pos.y as f64, pos.z as f64]);
    // Body rotation: compute relative rotation from parent. The relative
    // rotation between parent and child GlobalTransforms already accounts
    // for the anchor's Z-up→Y-up rotation (it cancels out), so the result
    // IS the MJCF Z-up rotation (just needs wxyz format conversion).
    {
        let mj_rot = pgrot.map(|pg| pg.inverse() * s.grot)
            .unwrap_or_else(|| Quat::from_rotation_x(std::f32::consts::FRAC_PI_2) * s.grot);
        b = b.with_quat([mj_rot.w as f64, mj_rot.x as f64, mj_rot.y as f64, mj_rot.z as f64]);
    }
    if let Some(i) = &s.inertial {
        let c = i.mass_center;
        // Normalize NaN/zero inertia: fullinertia may have NaN when the MJCF
        // used diaginertia (parsed from included files via MjSpec).
        let fi = i.inertia.0.map(|x: f64| if x.is_finite() { x } else { 0.0 });
        let mass = if i.mass.is_finite() && i.mass > 0.0 { i.mass } else { 0.0 };
        let ipos = [if c.x.is_finite() { c.x } else { 0.0 }, if c.y.is_finite() { c.y } else { 0.0 }, if c.z.is_finite() { c.z } else { 0.0 }];
        if mass > 0.0 {
            b.with_mass(mass).with_fullinertia(fi).with_ipos(ipos).with_explicitinertial(true);
        }
    }
    for geom in &s.geoms { write_geom(b, geom); }

    for (sn, sp) in &s.sites {
        let d = s.grot.inverse() * (*sp - s.gpos);
        b.add_site().with_name(sn).with_pos([d.x as f64, d.y as f64, d.z as f64]);
    }
    if !s.coords.is_empty() {
        let jp = s.grot.inverse() * (s.joint_pos - s.gpos);
        for c in &s.coords { add_joint(b, c, [jp.x as f64, jp.y as f64, jp.z as f64]); }
    }
    let g = s.gpos;
    let gr = s.grot;
    for k in children.get(&e).cloned().unwrap_or_default() { write_body(b, k, Some(g), Some(gr), bodies, children)?; }
    Ok(())
}

fn write_geom(body: &mut MjsBody, geom: &(String, String, MeshGeometry, Transform)) {
    let (name, mesh, properties, pose) = geom;
    let q = pose.rotation;
    body.add_geom().with_name(name)
        .with_type(mujoco_rs::wrappers::mj_model::MjtGeom::mjGEOM_MESH)
        .with_meshname(mesh)
        .with_pos(pose.translation.to_array().map(f64::from))
        .with_quat([q.w as f64, q.x as f64, q.y as f64, q.z as f64])
        .with_rgba(properties.rgba).with_contype(properties.contype)
        .with_conaffinity(properties.conaffinity).with_condim(properties.condim)
        .with_friction(properties.friction).with_margin(properties.margin)
        .with_gap(properties.gap).with_group(properties.group);
}

/// Write a portable MJCF plus a fresh sibling asset directory. Original source
/// files are copied byte-for-byte; shared references are packaged once.
pub fn save_mjcf(world: &mut World, root: Entity, path: &Path) -> Result<(), ExportError> {
    save_mjcf_impl(world, root, path, false)
}
pub fn save_mjcf_visual(world: &mut World, root: Entity, path: &Path) -> Result<(), ExportError> {
    save_mjcf_impl(world, root, path, true)
}
fn save_mjcf_impl(world: &mut World, root: Entity, path: &Path, visual_only: bool) -> Result<(), ExportError> {
    let fail = |error: String| ExportError::Asset(error);
    let mut spec = to_mjcf_impl(world, root, visual_only)?;
    spec.compile().map_err(|e| fail(e.to_string()))?;
    let parent = path.parent().filter(|p| !p.as_os_str().is_empty()).unwrap_or(Path::new("."));
    let directory = std::env::current_dir().map_err(|e| fail(e.to_string()))?;
    let document = crate::mjcf_document::MjcfDocument::capture(&spec, &directory, HashMap::new()).map_err(fail)?;
    document.package(parent, path.file_name().and_then(|s| s.to_str()).ok_or_else(|| fail("Invalid output filename".into()))?).map_err(fail)
}

fn add_joint(b: &mut MjsBody, c: &CoordSpec, pos: [f64; 3]) {
    let ang = c.twist.angular.norm(); let lin = c.twist.linear.norm();
    let hinge = ang > lin && ang > 1e-6;
    let kind = if hinge { MjtJoint::mjJNT_HINGE } else { MjtJoint::mjJNT_SLIDE };
    let v = if hinge { c.twist.angular } else { c.twist.linear };
    let v = if v.norm() > 1e-9 { v.normalize() } else { Vector3::new(1.0, 0.0, 0.0) };
    let axis = [v.x, v.y, v.z];
    let j = b.add_joint().with_name(&c.name).with_type(kind).with_pos(pos).with_axis(axis);
    if let Some(p) = &c.props {
        if p.clamped { j.with_range([p.range.0, p.range.1]).with_limited(MjtLimited::mjLIMITED_TRUE); }
        if p.damping > 0.0 { j.with_damping([p.damping, 0.0, 0.0]); }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Coordinate, CoordinateProperties, Inertia, InertialProperties, Joint, JointCoordinates, Twist};
    #[test]
    fn exports_body_tree_with_joint() {
        let mut world = World::new();
        let inertial = || InertialProperties::new(1.0, Vector3::zeros(), Inertia::diag(0.01, 0.01, 0.01));
        let anchor = world.spawn((Name::new("anchor"), Body, inertial(), Transform::IDENTITY, GlobalTransform::IDENTITY)).id();
        let coord = world.spawn((Name::new("knee_flex"), Coordinate,
            CoordinateProperties { range: (-1.5,1.5), clamped: true, locked: false, stiffness: 0.0, damping: 1.0 },
            Twist::rotation(Vector3::new(0.0, 0.0, 1.0)),
        )).id();
        let joint = world.spawn((Name::new("knee"), Joint, JointCoordinates::new(vec![coord]), Transform::IDENTITY)).id();
        world.entity_mut(joint).insert(ChildOf(anchor));
        let seg1 = world.spawn((Name::new("seg1"), Body, inertial(), Transform::from_xyz(0.0,1.0,0.0), GlobalTransform::from(Transform::from_xyz(0.0,1.0,0.0)))).id();
        world.entity_mut(seg1).insert(ChildOf(joint));
        let mut spec = to_mjcf(&mut world, anchor).unwrap();
        let compiled = spec.compile().expect("valid body/joint hierarchy must compile");
        assert_eq!(compiled.njnt(), 1);
        assert_eq!(compiled.id_to_name(mujoco_rs::wrappers::mj_model::MjtObj::mjOBJ_BODY,
            compiled.jnt_bodyid()[0] as usize), Some("seg1"));
        let xml = spec.save_xml_string(1 << 16).unwrap_or_default();
        eprintln!("xml:\n{}", xml);
        // MuJoCo may collapse trivial bodies; verify the export API works end-to-end.
        assert!(!xml.is_empty(), "export produced empty XML");
    }
}
