//! MuJoCo MJCF → melosim model (built anew for the *current* model).
//!
//! `mujoco-rs`'s `MjSpec` parses the XML (includes, defaults, classes); we walk
//! the spec and spawn current-model entities so an imported model visualizes in
//! the editor: `Body` (+ `InertialProperties`), `Site`, `Joint` →
//! `Coordinate` (+ `Twist`, `CoordinateProperties`, `CoordinateState`), and
//! GLTF mesh **geom**s as `WorldAssetRoot` children loaded through Bevy's
//! built-in `AssetServer`. Other mesh formats are reported and skipped; mesh
//! conversion belongs outside the ECS importer.
//!
//! Bodies are parented to their MJCF parent body directly (static pose);
//! joints attach to the body they drive. Geometry requires the app's asset
//! resources — in a bare `World` (unit tests) geoms are skipped gracefully.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use bevy::ecs::world::World;
use bevy::prelude::*;
use bevy::world_serialization::{WorldAsset, WorldAssetRoot};
use mujoco_rs::wrappers::mj_editing::*;
use mujoco_rs::wrappers::mj_model::{MjModel, MjtJoint};
use nalgebra::Vector3;

use super::ImportError;

use crate::model::{
    CoordinateProperties, Frame, HillTypeMuscleParams, Inertia, InitialConditions,
    validate_kinematic_hierarchy, CoordinateSpec, InertialProperties, ModelBuilder, Muscle,
    PathEntities, Twist,
};

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
    {
        let mut builder = ModelBuilder::new(world);
        for child_ent in spec.world_body().body_iter(false) {
            spawn_body(
                &mut builder,
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

    for issue in validate_kinematic_hierarchy(world) {
        warn!("model import: invalid kinematic hierarchy: {issue}");
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
    builder: &mut ModelBuilder,
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

    // A MuJoCo body's pose is fixed at zero joint motion. When the body has
    // joints, represent that fixed placement with a Frame, then put the
    // coordinate-driven Joint below it and the body below the joint. This
    // keeps sync_kinematics from overwriting the fixed body/joint placement.
    let body_pose = Transform {
        translation: to_vec3(*mj.pos()),
        rotation: mjcf_quat_to_bevy(*mj.quat()),
        ..default()
    };
    let joints: Vec<_> = mj.joint_iter(false).collect();
    let body_ent = if joints.is_empty() {
        builder.body(
            name.clone(),
            InertialProperties {
                mass,
                mass_center,
                inertia,
            },
            body_pose,
            Some(parent),
        )
    } else {
        // All MuJoCo joint elements on one body describe one body connection.
        // They become ordered coordinates on one melosim Joint entity.
        let anchor = Vector3::from(*joints[0].pos());
        if joints.iter().skip(1).any(|joint| {
            (Vector3::from(*joint.pos()) - anchor).norm() > 1e-9
        }) {
            warn!(
                "model import: body '{name}' has joint elements with different anchors; using the first anchor for the combined Joint"
            );
        }
        let anchor_bevy = to_vec3([anchor.x, anchor.y, anchor.z]);
        let body_ent = builder.body(
            name.clone(),
            InertialProperties {
                mass,
                mass_center,
                inertia,
            },
            Transform::IDENTITY,
            None,
        );
        let frame_transform = Transform {
            translation: body_pose.translation + body_pose.rotation * anchor_bevy,
            rotation: body_pose.rotation,
            ..default()
        };
        builder
            .joint_connection(
                parent,
                format!("{name}_joint_frame"),
                frame_transform,
                joint_name(&joints, &name, counter),
                body_ent,
                Transform::from_translation(-anchor_bevy),
                joint_coordinate_specs(&joints, counter),
            )
            .map_err(|e| ImportError::Load(format!("invalid joint hierarchy for '{name}': {e}")))?;
        body_ent
    };

    // Sites (points on the body). Record entities by name for muscle paths.
    for s in mj.site_iter(false) {
        let sname = if s.name().is_empty() {
            *counter += 1;
            format!("site_{}", *counter)
        } else {
            s.name().to_string()
        };
        let site_ent = builder.site(
            sname.clone(),
            Transform::from_translation(to_vec3(*s.pos())),
            body_ent,
        );
        if !sname.is_empty() {
            site_map.insert(sname, site_ent);
        }
    }

    // Mesh geoms → Mesh3d children (visualization).
    for g in mj.geom_iter(false) {
        spawn_geom(builder.world_mut(), &g, body_ent, model_dir, meshdir, mesh_src, mesh_ref, counter);
    }

    // Children bodies.
    for child in mj.body_iter(false) {
        spawn_body(builder, child, body_ent, model_dir, meshdir, mesh_src, mesh_ref, counter, site_map)?;
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

    let extension = src
        .extension()
        .and_then(|e| e.to_str())
        .map(|e| e.to_ascii_lowercase());
    if !matches!(extension.as_deref(), Some("glb") | Some("gltf")) {
        warn!(
            "model import: skipping non-GLTF mesh '{}' (convert it to GLTF before import)",
            src.display()
        );
        return;
    }

    let Some(asset_server) = world.get_resource::<AssetServer>().cloned() else {
        warn!("model import: AssetServer unavailable; skipping GLTF geom '{gname}'");
        return;
    };

    let Some(file_name) = src.file_name().and_then(|f| f.to_str()) else {
        warn!("model import: GLTF geom '{mesh_name}' has no usable file name");
        return;
    };
    let safe_name = file_name.replace(' ', "_");
    let asset_dir = Path::new("assets").join("imported");
    if let Err(e) = std::fs::create_dir_all(&asset_dir) {
        warn!("model import: cannot create {}: {e}", asset_dir.display());
        return;
    }
    let destination = asset_dir.join(&safe_name);
    if let Err(e) = std::fs::copy(&src, &destination) {
        warn!(
            "model import: cannot copy GLTF {} → {}: {e}",
            src.display(),
            destination.display()
        );
        return;
    }

    let asset_path = format!("imported/{safe_name}");
    let scene: Handle<WorldAsset> = asset_server.load(format!("{asset_path}#Scene0"));
    world
        .spawn((Name::new(gname), WorldAssetRoot(scene), t))
        .insert(ChildOf(body_ent));
}

fn joint_name(joints: &[&MjsJoint], body_name: &str, counter: &mut u64) -> String {
    if joints.len() == 1 {
        if joints[0].name().is_empty() {
            *counter += 1;
            format!("joint_{}", *counter)
        } else {
            joints[0].name().to_string()
        }
    } else {
        format!("{body_name}_joint")
    }
}

fn joint_coordinate_specs(joints: &[&MjsJoint], _counter: &mut u64) -> Vec<CoordinateSpec> {
    let first_base = joints
        .first()
        .filter(|joint| !joint.name().is_empty())
        .map(|joint| joint.name().to_string())
        .unwrap_or_else(|| "joint".to_string());
    let mut specs = Vec::new();

    for (index, mj) in joints.iter().copied().enumerate() {
        let base = if joints.len() == 1 {
            first_base.clone()
        } else if mj.name().is_empty() {
            format!("{first_base}_{index}")
        } else {
            mj.name().to_string()
        };
        append_joint_coordinate_specs(mj, &base, &mut specs);
    }
    specs
}

fn append_joint_coordinate_specs(mj: &MjsJoint, base: &str, specs: &mut Vec<CoordinateSpec>) {
    let limited = matches!(mj.limited(), MjtLimited::mjLIMITED_TRUE);
    let range = *mj.range();
    let properties = || CoordinateProperties {
        range: if limited { (range[0], range[1]) } else { (-f64::MAX, f64::MAX) },
        clamped: limited,
        locked: false,
        stiffness: mj.stiffness().first().copied().unwrap_or(0.0),
        damping: mj.damping().first().copied().unwrap_or(0.0),
    };
    let spec = |suffix: &str, twist: Twist| CoordinateSpec {
        name: format!("{base}{suffix}"),
        properties: properties(),
        initial: InitialConditions {
            value: *mj.ref_(),
            velocity: 0.0,
        },
        twist: Some(twist),
        driven_by: None,
    };

    match mj.type_() {
        MjtJoint::mjJNT_HINGE => specs.push(spec(
            "",
            Twist::rotation(Vector3::from(*mj.axis())),
        )),
        MjtJoint::mjJNT_SLIDE => specs.push(spec(
            "",
            Twist::translation(Vector3::from(*mj.axis())),
        )),
        MjtJoint::mjJNT_BALL => {
            for (suffix, axis) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                specs.push(spec(suffix, Twist::rotation(axis)));
            }
        }
        MjtJoint::mjJNT_FREE => {
            for (suffix, axis) in [("_tx", Vector3::x()), ("_ty", Vector3::y()), ("_tz", Vector3::z())] {
                specs.push(spec(suffix, Twist::translation(axis)));
            }
            for (suffix, axis) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                specs.push(spec(suffix, Twist::rotation(axis)));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Body, Frame, Joint, Site};

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

        // The imported joint must be in the kinematic transform chain:
        // fixed joint frame → joint → body. The old importer placed the joint
        // beside the body, so articulation could not reach it.
        let base = world
            .query_filtered::<(Entity, &Name), With<Body>>()
            .iter(&world)
            .find(|(_, name)| name.as_str() == "base")
            .map(|(entity, _)| entity)
            .expect("base body");
        let joint = world
            .get::<ChildOf>(base)
            .expect("jointed body should have a joint parent")
            .parent();
        assert!(world.get::<Joint>(joint).is_some());
        let joint_frame = world
            .get::<ChildOf>(joint)
            .expect("joint should have a fixed frame parent")
            .parent();
        assert!(world.get::<Frame>(joint_frame).is_some());

        let link = world
            .query_filtered::<(Entity, &Name), With<Body>>()
            .iter(&world)
            .find(|(_, name)| name.as_str() == "link")
            .map(|(entity, _)| entity)
            .expect("link body");
        assert_eq!(world.get::<ChildOf>(link).unwrap().parent(), base);

        let issues = validate_kinematic_hierarchy(&mut world);
        assert!(issues.is_empty(), "invalid imported hierarchy: {issues:?}");
    }

}
