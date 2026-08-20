//! MuJoCo MJCF → melosim model (built anew for the *current* model).
//!
//! `mujoco-rs`'s `MjSpec` parses the XML (includes, defaults, classes); we walk
//! the spec and spawn current-model entities so an imported model visualizes in
//! the editor: `Body` (+ `InertialProperties`), `Site`, `Joint` →
//! `Coordinate` (+ `Twist`, `CoordinateProperties`, `CoordinateState`), and
//! mesh **geom**s as `Mesh3d` children (copied into `assets/imported/` and
//! loaded via `AssetServer`, using the STL loader when present).
//!
//! Bodies are parented to their MJCF parent body directly (static pose);
//! joints attach to the body they drive. Geometry requires the app's asset
//! resources — in a bare `World` (unit tests) geoms are skipped gracefully.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use bevy::ecs::world::World;
use bevy::prelude::*;
use mujoco_rs::wrappers::mj_editing::*;
use mujoco_rs::wrappers::mj_model::{MjModel, MjtJoint};
use nalgebra::Vector3;

use crate::model::{
    Body, Coordinate, CoordinateProperties, CoordinateState, Frame, HillTypeMuscleParams, Inertia,
    InertialProperties, Joint, JointCoordinates, Muscle, PathEntities, Site, Twist,
};

/// Import failure.
#[derive(Debug)]
pub enum ImportError {
    Load(String),
}

/// Parse the MJCF at `path` and spawn the model into `world`.
///
/// `MjSpec::from_xml` resolves `<include>` and `meshdir` paths relative to
/// the **current working directory**, not the model file's directory. This
/// function temporarily switches CWD to the model's parent so that includes
/// resolve correctly regardless of where the process was started.
///
/// Returns the top-level container entity.
pub fn import_mjcf(world: &mut World, path: &Path) -> Result<Entity, ImportError> {
    // Canonicalize to an absolute path so CWD changes don't break it.
    let abs_path = path
        .canonicalize()
        .map_err(|e| ImportError::Load(format!("cannot resolve path '{}': {e}", path.display())))?;
    let model_dir = abs_path.parent().unwrap_or(Path::new("")).to_path_buf();

    // MjSpec::from_xml resolves `<include>` and `meshdir` relative to CWD,
    // not the XML file's directory. Temporarily switch CWD.
    let saved_cwd = std::env::current_dir().ok();
    std::env::set_current_dir(&model_dir)
        .map_err(|e| ImportError::Load(format!("cannot cd to model dir '{}': {e}", model_dir.display())))?;

    let mut spec = MjSpec::from_xml(&abs_path).map_err(|e| ImportError::Load(format!("{e}")))?;

    let model_name = abs_path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "model".into());

    // Mesh geometry sources: name → file string from the spec. MuJoCo resolves
    // `file` relative to `model_dir + meshdir` (or absolute); we replicate that
    // per geom and copy the resolved file into assets/ (see `spawn_geom`).
    let meshdir = spec.compiler().meshdir().to_string();
    let mesh_src: HashMap<String, String> = spec
        .mesh_iter()
        .map(|m| (m.name().to_string(), m.file().to_string()))
        .collect();
    let mesh_ref: HashMap<String, ([f64; 3], [f64; 4], [f64; 3])> = spec
        .mesh_iter()
        .map(|m| (m.name().to_string(), (*m.refpos(), *m.refquat(), *m.scale())))
        .collect();

    // Top-level container (Frame) so imported bodies group under it.
    // The Z-up→Y-up rotation lives here (on the root), NOT on every body —
    // bodies use translation-only transforms so GlobalTransform propagation
    // multiplies parent-child translations correctly without spurious rotations.
    let anchor = world
        .spawn((
            Name::new(model_name.clone()),
            Frame,
            Transform::from_rotation(Quat::from_rotation_x(-std::f32::consts::FRAC_PI_2)),
        ))
        .id();

    let mut counter = 0u64;
    let mut site_map: HashMap<String, Entity> = HashMap::new();
    for child_ent in spec.world_body().body_iter(false) {
        spawn_body(
            world,
            child_ent,
            anchor,
            &model_dir,
            &meshdir,
            &mesh_src,
            &mesh_ref,
            &mut counter,
            &mut site_map,
        )?;
    }

    // Muscles: try compile first for fast array access. If compile does not
    // succeed, read the raw XML file directly (includes are inlined as text
    // so we can parse tendon paths from the on-disk file).
    match spec.compile() {
        Ok(compiled) => {
            update_body_inertias_from_compiled(world, &compiled);
            update_body_rotations_from_compiled(world, &compiled);
            import_muscles_compiled(world, &compiled, &site_map);
        }
        Err(_e) => {
            // Resolve includes inline and parse tendons from the full XML.
            let resolved = resolve_mjcf_includes(&abs_path);
            import_muscles_from_xml(world, &resolved, &site_map);
        }
    }

    // Restore original CWD.
    if let Some(cwd) = saved_cwd {
        let _ = std::env::set_current_dir(&cwd);
    }

    Ok(anchor)
}

/// Update body entities' quaternions from the compiled MuJoCo model.
/// When MJCF uses `euler`/`axisangle`/`zaxis`, the spec's `quat()` returns
/// identity — the orientation is only resolved to a quaternion during
/// compilation. This reads the compiled body quaternions and updates the
/// Transform rotations so the hierarchy (including euler-rotated bodies
/// like the legacy elbow model) positions children correctly.
fn update_body_rotations_from_compiled(world: &mut World, compiled: &MjModel) {
    use mujoco_rs::wrappers::mj_model::MjtObj;
    let name_to_entity: HashMap<String, Entity> = world
        .query::<(Entity, &Name)>().iter(world)
        .map(|(e, n)| (n.as_str().to_owned(), e))
        .collect();
    for i in 0..compiled.nbody() as usize {
        let Some(name) = compiled.id_to_name(MjtObj::mjOBJ_BODY, i) else { continue };
        let Some(&ent) = name_to_entity.get(name) else { continue };
        let q = compiled.body_quat()[i]; // [f64; 4] wxyz in MJCF Z-up
        // Convert MJCF quat (wxyz) → Bevy quat (xyzw). No frame change needed
        // because the anchor handles Z-up→Y-up globally.
        let bevy_rot = Quat::from_xyzw(q[1] as f32, q[2] as f32, q[3] as f32, q[0] as f32);
        if let Some(mut t) = world.get_mut::<Transform>(ent) {
            t.rotation = bevy_rot;
        }
    }
}

/// Update body entities' inertial properties from the compiled MuJoCo model.
fn update_body_inertias_from_compiled(world: &mut World, compiled: &MjModel) {
    use mujoco_rs::wrappers::mj_model::MjtObj;
    let name_to_entity: HashMap<String, Entity> = world
        .query::<(Entity, &Name)>().iter(world)
        .map(|(e, n)| (n.as_str().to_owned(), e))
        .collect();
    for i in 0..compiled.nbody() as usize {
        let Some(name) = compiled.id_to_name(MjtObj::mjOBJ_BODY, i) else { continue };
        let Some(&ent) = name_to_entity.get(name) else { continue };
        let mass = compiled.body_mass()[i];
        let ipos = compiled.body_ipos()[i];   // [f64; 3]
        let din = compiled.body_inertia()[i]; // diagonal [f64; 3]
        if let Some(mut ip) = world.get_mut::<InertialProperties>(ent) {
            ip.mass = mass;
            ip.mass_center = Vector3::new(ipos[0], ipos[1], ipos[2]);
            ip.inertia = Inertia([din[0], din[1], din[2], 0.0, 0.0, 0.0]);
        }
    }
}

/// Fast path: import muscles from the compiled model's arrays.
fn import_muscles_compiled(
    world: &mut World,
    compiled: &MjModel,
    site_map: &HashMap<String, Entity>,
) {
    use mujoco_rs::wrappers::mj_model::{MjtObj, MjtWrap};
    let ntendon = compiled.ntendon();
    let _nwrap = compiled.nwrap();
    for t in 0..ntendon as usize {
        let adr = compiled.tendon_adr()[t].max(0) as usize;
        let num = compiled.tendon_num()[t].max(0) as usize;
        let name = compiled
            .id_to_name(MjtObj::mjOBJ_TENDON, t)
            .map(str::to_string)
            .unwrap_or_else(|| format!("muscle_{t}"));

        let mut path = Vec::new();
        for w in adr..adr + num {
            if w >= compiled.nwrap() as usize {
                break;
            }
            if compiled.wrap_type()[w] == MjtWrap::mjWRAP_SITE {
                let siteid = compiled.wrap_objid()[w].max(0) as usize;
                if let Some(sname) = compiled.id_to_name(MjtObj::mjOBJ_SITE, siteid) {
                    if let Some(e) = site_map.get(sname) {
                        path.push(*e);
                    }
                }
            }
        }
        if path.is_empty() {
            continue;
        }
        world.spawn((
            Name::new(name),
            Muscle,
            HillTypeMuscleParams::default(),
            PathEntities::new(path),
        ));
    }
}

/// Fallback: parse muscle paths from the original MJCF XML.
fn import_muscles_from_xml(
    world: &mut World,
    xml: &str,
    site_map: &HashMap<String, Entity>,
) {
    let paths = parse_spatial_tendon_sites(xml);
    for (tendon_name, site_names) in paths {
        let path: Vec<Entity> = site_names
            .iter()
            .filter_map(|s| site_map.get(s).copied())
            .collect();
        if path.is_empty() {
            continue; // All referenced sites missing — skip.
        }
        world.spawn((
            Name::new(tendon_name),
            Muscle,
            HillTypeMuscleParams::default(),
            PathEntities::new(path),
        ));
    }
}

/// Recursively resolve `<include file="..."/>` references in an MJCF file,
/// merging all content into a single XML string so that text-based parsers
/// (like tendon extraction) can see the full model definition.
fn resolve_mjcf_includes(main_path: &Path) -> String {
    let dir = main_path.parent().unwrap_or(Path::new("."));
    let mut visited = std::collections::HashSet::new();
    resolve_includes_recursive(main_path, dir, &mut visited)
}

fn resolve_includes_recursive(
    path: &Path,
    base_dir: &Path,
    visited: &mut std::collections::HashSet<PathBuf>,
) -> String {
    let canonical = path.canonicalize().unwrap_or_else(|_| path.to_path_buf());
    if !visited.insert(canonical.clone()) {
        return String::new(); // already included, avoid cycles
    }
    let Ok(raw) = std::fs::read_to_string(path) else {
        eprintln!("importer: could not read include '{}'", path.display());
        return String::new();
    };
    let mut out = String::with_capacity(raw.len());
    let mut rest = raw.as_str();
    while let Some(pos) = rest.find("<include") {
        // Emit everything before this <include>
        out.push_str(&rest[..pos]);
        let tag_start = &rest[pos..];
        // Find the end of this self-closing or inline tag
        let tag_end = tag_start.find('>').map(|i| i + 1).unwrap_or(tag_start.len());
        let tag = &tag_start[..tag_end];
        // Extract file attribute
        if let Some(file_path) = tag
            .split("file=\"")
            .nth(1)
            .and_then(|s| s.split('"').next())
        {
            let include_path = base_dir.join(file_path);
            let resolved = resolve_includes_recursive(&include_path, base_dir, visited);
            out.push_str(&resolved);
        }
        // Advance past this tag
        rest = &tag_start[tag_end..];
    }
    // Emit remaining content after the last <include>
    out.push_str(rest);
    out
}

/// Extract spatial tendon → ordered site names from MJCF XML text.
fn parse_spatial_tendon_sites(xml: &str) -> Vec<(String, Vec<String>)> {
    let mut out = Vec::new();
    for tendon_block in xml.split("<tendon>").skip(1).take_while(|s| s.contains("</tendon>")) {
        let tendon_body = tendon_block.split("</tendon>").next().unwrap_or("");
        for spatial_block in tendon_body.split("<spatial").skip(1) {
            let spatial_body = spatial_block.split("</spatial>").next().unwrap_or("");
            let tendon_name = spatial_block
                .split("name=\"")
                .nth(1)
                .and_then(|s| s.split('"').next())
                .unwrap_or("tendon")
                .to_string();
            let mut site_names = Vec::new();
            // Scan for <site ... site="NAME" .../> or <site site="NAME"/> —
            // the site attribute can appear anywhere on the tag, across lines.
            for tag in spatial_body.split('<').filter(|t| t.starts_with("site")) {
                if let Some(site) = tag
                    .split("site=\"")
                    .nth(1)
                    .and_then(|s| s.split('"').next())
                {
                    site_names.push(site.to_string());
                }
            }
            if !site_names.is_empty() {
                out.push((tendon_name, site_names));
            }
        }
    }
    out
}

/// MJCF quaternion → Bevy quaternion.
/// The anchor handles the Z-up→Y-up frame conversion; body rotations are
/// just format-converted (MuJoCo wxyz → Bevy xyzw) without frame change.
fn mjcf_quat_to_bevy(q: [f64; 4]) -> Quat {
    to_quat_mjcf(q)
}

fn to_quat_mjcf(q: [f64; 4]) -> Quat {
    Quat::from_xyzw(q[1] as f32, q[2] as f32, q[3] as f32, q[0] as f32)
}

fn to_vec3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0] as f32, a[1] as f32, a[2] as f32)
}

/// Instanced pose of a mesh geom in its body frame, composing the geom's own
/// `pos`/`quat` with the mesh's reference frame (`refpos`/`refquat`/`scale`).
/// Stays in the MJCF Z-up frame — the root rotation converts the entire model.
fn mesh_instance_transform(
    geom_pos: [f64; 3],
    geom_quat: [f64; 4],
    refpos: [f64; 3],
    refquat: [f64; 4],
    scale: [f64; 3],
) -> Transform {
    let g_pos = to_vec3(geom_pos);
    let g_q = to_quat_mjcf(geom_quat);
    // Compose in the MJCF body frame (still Z-up — root converts later).
    let rot_mjcf = g_q * to_quat_mjcf(refquat);
    let pos_mjcf = g_pos + g_q.mul_vec3(to_vec3(refpos));
    Transform {
        translation: pos_mjcf,
        rotation: rot_mjcf,
        scale: to_vec3(scale),
    }
}

fn spawn_body(
    world: &mut World,
    mj: &MjsBody,
    parent: Entity,
    model_dir: &Path,
    meshdir: &str,
    mesh_src: &HashMap<String, String>,
    mesh_ref: &HashMap<String, ([f64; 3], [f64; 4], [f64; 3])>,
    counter: &mut u64,
    site_map: &mut HashMap<String, Entity>,
) -> Result<(), ImportError> {
    let name = if mj.name().is_empty() {
        *counter += 1;
        format!("body_{}", *counter)
    } else {
        mj.name().to_string()
    };

    let mass = mj.mass();
    let ipos_raw = *mj.ipos();
    let fi_raw = *mj.fullinertia();
    // When the spec only has diaginertia (not fullinertia), fullinertia()
    // may contain NaN in the first diagonal element. Replace NaN with computed
    // diagonal inertia proportional to mass (treat body as a small sphere).
    let inertia = if fi_raw.iter().any(|x| !x.is_finite()) {
        let diag = if mass > 0.0 { mass.powf(2.0 / 3.0) * 0.001 } else { 0.0001 };
        Inertia::new(diag, diag, diag, 0.0, 0.0, 0.0)
    } else {
        Inertia(fi_raw)
    };
    let mass_center = if ipos_raw.iter().any(|x| !x.is_finite()) {
        Vector3::zeros()
    } else {
        Vector3::from(ipos_raw)
    };

    let mut body = world.spawn((
        Name::new(name.clone()),
        Body,
        InertialProperties {
            mass,
            mass_center,
            inertia,
        },
        // Body transform: translation + rotation (MJCF Z-up → Bevy Y-up).
        // The root anchor applies the overall Z-up→Y-up conversion; body-local
        // rotations are converted independently so GlobalTransform accumulation
        // works correctly through the hierarchy.
        Transform {
            translation: to_vec3(*mj.pos()),
            rotation: mjcf_quat_to_bevy(*mj.quat()),
            ..default()
        },
    ));
    body.insert(ChildOf(parent));
    let body_ent = body.id();

    // Joints on this body (driving it) → Joint + Coordinate + Twist.
    let joints: Vec<_> = mj.joint_iter(false).collect();
    for j in joints {
        spawn_joint(world, j, body_ent, counter)?;
    }

    // Sites (points on the body). Record entities by name for muscle paths.
    for s in mj.site_iter(false) {
        let sname = if s.name().is_empty() {
            *counter += 1;
            format!("site_{}", *counter)
        } else {
            s.name().to_string()
        };
        let site_ent = world
            .spawn((
                Name::new(sname.clone()),
                Site,
                Transform::from_translation(to_vec3(*s.pos())),            ))
            .insert(ChildOf(body_ent))
            .id();
        if !sname.is_empty() {
            site_map.insert(sname, site_ent);
        }
    }

    // Mesh geoms → Mesh3d children (visualization).
    for g in mj.geom_iter(false) {
        spawn_geom(world, &g, body_ent, model_dir, meshdir, mesh_src, mesh_ref, counter);
    }

    // Children bodies.
    for child in mj.body_iter(false) {
        spawn_body(world, child, body_ent, model_dir, meshdir, mesh_src, mesh_ref, counter, site_map)?;
    }
    Ok(())
}

/// Replicate MuJoCo's mesh resolution: `file` absolute, or relative to
/// `model_dir + meshdir`, or relative to `model_dir`. Returns the first that
/// exists.
fn resolve_mesh(model_dir: &Path, meshdir: &str, file: &str) -> Option<PathBuf> {
    let f = PathBuf::from(file);
    if f.is_absolute() && f.exists() {
        return Some(f);
    }
    for cand in [model_dir.join(meshdir).join(file), model_dir.join(file)] {
        if cand.exists() {
            return Some(cand);
        }
    }
    None
}

fn spawn_geom(
    world: &mut World,
    g: &MjsGeom,
    body_ent: Entity,
    model_dir: &Path,
    meshdir: &str,
    mesh_src: &HashMap<String, String>,
    mesh_ref: &HashMap<String, ([f64; 3], [f64; 4], [f64; 3])>,
    counter: &mut u64,
) {
    // Only mesh geoms for now (primitives come later).
    let mesh_name = g.meshname();
    if mesh_name.is_empty() {
        return;
    }
    let Some(file) = mesh_src.get(mesh_name) else {
        warn!("model import: geom references unknown mesh '{mesh_name}'");
        return;
    };
    let Some(src) = resolve_mesh(model_dir, meshdir, file) else {
        error!(
            "model import: mesh '{mesh_name}' file '{file}' not found (model dir {model_dir:?}, meshdir '{meshdir}')"
        );
        return;
    };

    let gname = if g.name().is_empty() {
        *counter += 1;
        format!("geom_{}", *counter)
    } else {
        g.name().to_string()
    };

    // Compose the geom pose with the mesh's reference frame (refquat orients
    // parallel bones like ulna/radius), then map Z-up -> Y-up.
    let (rp, rq, sc) = mesh_ref
        .get(mesh_name)
        .copied()
        .unwrap_or(([0.0; 3], [1.0, 0.0, 0.0, 0.0], [1.0; 3]));
    let t = mesh_instance_transform(*g.pos(), *g.quat(), rp, rq, sc);

    let mesh_source = crate::model::MeshSource {
        stl_path: src.clone(),
        mesh_name: mesh_name.to_string(),
    };

    // Spawn with visual mesh if Assets<Mesh> is available; always store
    // MeshSource so the exporter can reconstruct the MJCF geom.
    let has_assets = world.get_resource::<Assets<Mesh>>().is_some();
    if has_assets && src.extension().map(|e| e.to_string_lossy().to_lowercase()).as_deref() == Some("stl") {
        let mesh = match crate::render::mesh_loaders::mesh_from_stl_file(&src) {
            Ok(m) => m,
            Err(_e) => {
                error!("model import: failed to load mesh '{mesh_name}' from {}: {_e}", src.display());
                return;
            }
        };
        let handle: Handle<Mesh> = world.resource_mut::<Assets<Mesh>>().add(mesh);
        let material: Handle<StandardMaterial> =
            world.resource_mut::<Assets<StandardMaterial>>().add(StandardMaterial::default());
        world
            .spawn((
                Name::new(gname),
                Mesh3d(handle),
                MeshMaterial3d(material),
                t,
                mesh_source,
            ))
            .insert(ChildOf(body_ent));
    } else {
        // No assets available — spawn with just MeshSource for roundtrip.
        world
            .spawn((Name::new(gname), t, mesh_source))
            .insert(ChildOf(body_ent));
    }
}

fn spawn_joint(
    world: &mut World,
    mj: &MjsJoint,
    body_ent: Entity,
    counter: &mut u64,
) -> Result<(), ImportError> {
    let base = if mj.name().is_empty() {
        *counter += 1;
        format!("joint_{}", *counter)
    } else {
        mj.name().to_string()
    };

    let mut coord_ids: Vec<Entity> = Vec::new();
    match mj.type_() {
        MjtJoint::mjJNT_HINGE => coord_ids.push(spawn_coord(
            world, mj, &base, "", Vector3::from(*mj.axis()), Vector3::zeros(), counter,
        )),
        MjtJoint::mjJNT_SLIDE => coord_ids.push(spawn_coord(
            world, mj, &base, "", Vector3::zeros(), Vector3::from(*mj.axis()), counter,
        )),
        MjtJoint::mjJNT_BALL => {
            for (s, a) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                coord_ids.push(spawn_coord(world, mj, &base, s, a, Vector3::zeros(), counter));
            }
        }
        MjtJoint::mjJNT_FREE => {
            for (s, a) in [("_tx", Vector3::x()), ("_ty", Vector3::y()), ("_tz", Vector3::z())] {
                coord_ids.push(spawn_coord(world, mj, &base, s, Vector3::zeros(), a, counter));
            }
            for (s, a) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                coord_ids.push(spawn_coord(world, mj, &base, s, a, Vector3::zeros(), counter));
            }
        }
    }

    let joint_name = if coord_ids.len() == 1 {
        base.clone()
    } else {
        format!("{base}_{}", coord_ids.len())
    };
    world
        .spawn((
            Name::new(joint_name),
            Joint,
            JointCoordinates::new(coord_ids),
            Transform::IDENTITY,
        ))
        .insert(ChildOf(body_ent));
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn spawn_coord(
    world: &mut World,
    mj: &MjsJoint,
    base: &str,
    suffix: &str,
    angular: Vector3<f64>,
    linear: Vector3<f64>,
    counter: &mut u64,
) -> Entity {
    let _ = counter;
    let limited = matches!(mj.limited(), MjtLimited::mjLIMITED_TRUE);
    let range = *mj.range();
    world
        .spawn((
            Name::new(format!("{base}{suffix}")),
            Coordinate,
            Twist { angular, linear },
            CoordinateProperties {
                range: if limited { (range[0], range[1]) } else { (-f64::MAX, f64::MAX) },
                clamped: limited,
                locked: false,
                stiffness: mj.stiffness().first().copied().unwrap_or(0.0),
                damping: mj.damping().first().copied().unwrap_or(0.0),
            },
            CoordinateState { value: *mj.ref_(), velocity: 0.0 },
        ))
        .id()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp_mjcf(xml: &str) -> std::path::PathBuf {
        let p = std::env::temp_dir().join("melosim_import_test.xml");
        std::fs::write(&p, xml).unwrap();
        p
    }

    /// A minimal 2-body hinge model: parses and spawns bodies + joint + a
    /// spatial-tendon muscle (whose path resolves to the two imported sites).
    #[test]
    fn imports_two_body_hinge() {
        let xml = r#"<mujoco model="test2">
          <worldbody>
            <body name="base" pos="0 0 0">
              <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
              <joint name="hinge" type="hinge" axis="0 0 1" range="-1 1" damping="0.5"/>
              <site name="s1" pos="0 0 0"/>
              <body name="link" pos="0 0 1">
                <inertial pos="0 0 0" mass="1" diaginertia="0.01 0.01 0.01"/>
                <site name="s2" pos="0 0 0.1"/>
              </body>
            </body>
          </worldbody>
          <tendon>
            <spatial name="biceps">
              <site site="s1"/>
              <site site="s2"/>
            </spatial>
          </tendon>
        </mujoco>"#;
        let path = tmp_mjcf(xml);
        let mut world = World::new();
        let _ = import_mjcf(&mut world, &path).expect("import");

        assert!(
            world.query_filtered::<Entity, With<Body>>().iter(&world).count() >= 2,
            "expected base + link bodies"
        );
        assert!(
            world.query_filtered::<Entity, With<Joint>>().iter(&world).count() >= 1,
            "expected a joint"
        );
        assert!(
            world.query_filtered::<Entity, With<Site>>().iter(&world).count() >= 2,
            "expected two sites"
        );
        assert!(
            world.query_filtered::<Entity, With<Muscle>>().iter(&world).count() >= 1,
            "expected a muscle from the spatial tendon"
        );
    }
}
