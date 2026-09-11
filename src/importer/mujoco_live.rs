//! MuJoCo MJCF import with native compiled mesh rendering and source provenance.
//! Mesh geometry is retained even in headless worlds; render assets are optional.

use std::collections::HashMap;
use std::path::{Path, PathBuf};

use bevy::ecs::world::World;
use bevy::prelude::*;

use std::sync::Arc;
use crate::model::{MeshGeometry, MeshSource};
use mujoco_rs::wrappers::mj_editing::*;
use mujoco_rs::wrappers::mj_model::{MjModel, MjtJoint};
use nalgebra::Vector3;

use super::ImportError;

use crate::model::{
    Body, CoordinateProperties, Frame, HillTypeMuscleParams, Inertia, InitialConditions,
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
    static IMPORT_DIRECTORY: std::sync::Mutex<()> = std::sync::Mutex::new(());
    let _directory_lock = IMPORT_DIRECTORY.lock().map_err(|_| ImportError::Load("An earlier import panicked".into()))?;
    // Canonicalize to an absolute path so CWD changes don't break it.
    let abs_path = path
        .canonicalize()
        .map_err(|e| ImportError::Load(format!("cannot resolve path '{}': {e}", path.display())))?;
    let model_dir = abs_path.parent().unwrap_or(Path::new("")).to_path_buf();

    // MjSpec::from_xml resolves `<include>` and `meshdir` relative to CWD,
    // not the XML file's directory. Temporarily switch CWD.
    let saved_cwd = std::env::current_dir().ok();
    let _cwd_guard = RestoreCwd(saved_cwd.clone());
    std::env::set_current_dir(&model_dir)
        .map_err(|e| ImportError::Load(format!("cannot cd to model dir '{}': {e}", model_dir.display())))?;

    let mut spec = MjSpec::from_xml(&abs_path).map_err(|e| ImportError::Load(format!("{e}")))?;

    let model_name = abs_path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "model".into());

    // Retain the authored asset directory for original-file provenance.
    let meshdir = spec.compiler().meshdir().to_string();
    // Give unnamed bodies stable identities before compilation.
    let used: std::collections::HashSet<String> = spec.body_iter().map(|b| b.name().to_owned()).collect();
    for (i, body) in spec.body_iter_mut().enumerate() {
        if body.name().is_empty() {
            let mut name = format!("melosim_body_{i}");
            while used.contains(&name) { name.push('_'); }
            body.set_name(&name).map_err(|e| ImportError::Load(e.to_string()))?;
        }
    }
    let used: std::collections::HashSet<String> = spec.geom_iter().map(|g| g.name().to_owned()).collect();
    for (i, geom) in spec.geom_iter_mut().enumerate() {
        if geom.name().is_empty() {
            let mut name = format!("melosim_geom_{i}");
            while used.contains(&name) { name.push('_'); }
            geom.set_name(&name).map_err(|e| ImportError::Load(e.to_string()))?;
        }
    }
    // Convert the world basis once; all body and geom poses remain local.
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
                &mut counter,
                &mut site_map,
            )?;
        }
    }

    // Compile the complete model when possible. Some real-world MyoSim files
    // contain tendon references that MjSpec rejects even though the body/mesh
    // portion is valid. In that case, compile a geometry-only copy so MuJoCo's
    // mesh canonicalization is still available, and use the text fallback for
    // tendon paths.
    match spec.compile() {
        Ok(compiled) => {
            update_body_inertias_from_compiled(world, &compiled);
            update_body_rotations_from_compiled(world, &compiled);
            import_compiled_meshes(world, &compiled, &spec, &model_dir, &meshdir, anchor);
            import_muscles_compiled(world, &compiled, &site_map);
            let names = world.query::<(Entity, &Name)>().iter(world).map(|(e,n)| (e,n.as_str().to_owned())).collect();
            let mut document = crate::mjcf_document::MjcfDocument::capture(&spec, &model_dir, names)
                .map_err(ImportError::Load)?;
            document.colors = world.query::<(Entity, &MeshGeometry)>().iter(world).map(|(e,g)|(e,g.rgba)).collect();
            world.entity_mut(anchor).insert(document);
        }
        Err(error) => {
            warn!("model import: full MuJoCo compile failed; using geometry-only compile: {error}");
            let mut document = crate::mjcf_document::MjcfDocument::capture_uncompiled(&abs_path, HashMap::new(), error.to_string()).map_err(ImportError::Load)?;
            let geometry = document.visual_only().and_then(|d| MjSpec::from_xml_string(&d.xml).map_err(|e| e.to_string())?.compile().map_err(|e| e.to_string()));
            match geometry {
                Ok(compiled) => {
                    update_body_rotations_from_compiled(world, &compiled);
                    import_compiled_meshes(world, &compiled, &spec, &model_dir, &meshdir, anchor);
                }
                Err(error) => {
                    warn!("geometry-only MuJoCo compile failed: {error}");
                }
            }
            import_muscles_from_xml(world, &document.xml, &site_map);
            document.names = world.query::<(Entity, &Name)>().iter(world).map(|(e,n)| (e,n.as_str().to_owned())).collect();
            document.colors = world.query::<(Entity, &MeshGeometry)>().iter(world).map(|(e,g)|(e,g.rgba)).collect();
            world.entity_mut(anchor).insert(document);
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

/// Update fixed body/joint-frame rotations from the compiled MuJoCo model.
///
/// Jointed bodies in the live hierarchy are represented as:
/// `body_parent → <body>_joint_frame → Joint → Body`. The compiled body
/// quaternion therefore belongs on the fixed joint frame, not on the child
/// body, whose local transform already contains the inverse joint anchor.
fn update_body_rotations_from_compiled(world: &mut World, compiled: &MjModel) {
    use mujoco_rs::wrappers::mj_model::MjtObj;
    let bodies: HashMap<String, Entity> = world
        .query_filtered::<(Entity, &Name), With<Body>>()
        .iter(world)
        .map(|(entity, name)| (name.as_str().to_owned(), entity))
        .collect();
    let joint_frames: HashMap<String, Entity> = world
        .query_filtered::<(Entity, &Name), With<Frame>>()
        .iter(world)
        .filter_map(|(entity, name)| {
            name.as_str()
                .strip_suffix("_joint_frame")
                .map(|body_name| (body_name.to_owned(), entity))
        })
        .collect();

    for body_id in 0..compiled.nbody() as usize {
        let Some(name) = compiled.id_to_name(MjtObj::mjOBJ_BODY, body_id) else { continue };
        let q = compiled.body_quat()[body_id]; // [w, x, y, z] in MJCF
        let bevy_rot = Quat::from_xyzw(q[1] as f32, q[2] as f32, q[3] as f32, q[0] as f32);
        let target = joint_frames.get(name).copied().or_else(|| bodies.get(name).copied());
        if let Some(target) = target {
            if let Some(mut transform) = world.get_mut::<Transform>(target) {
                transform.rotation = bevy_rot;
            }
        }
    }
}

/// Restore process CWD on error as well as success.
struct RestoreCwd(Option<PathBuf>);
impl Drop for RestoreCwd {
    fn drop(&mut self) { if let Some(path) = &self.0 { let _ = std::env::set_current_dir(path); } }
}

fn compiled_mesh_transform(pos: [f64; 3], quat: [f64; 4]) -> Transform {
    Transform::from_translation(to_vec3(pos)).with_rotation(to_quat_mjcf(quat))
}

/// Render compiled vertices with compiled geom poses. Source assets never pass
/// through a second converter or a guessed axis correction.
fn import_compiled_meshes(
    world: &mut World, model: &MjModel, spec: &MjSpec,
    model_dir: &Path, meshdir: &str, root: Entity,
) {
    use mujoco_rs::wrappers::mj_model::{MjtObj, MjtGeom};
    let mut bodies = HashMap::new();
    let mut stack = vec![root];
    while let Some(entity) = stack.pop() {
        if world.get::<Body>(entity).is_some() {
            if let Some(name) = world.get::<Name>(entity) { bodies.insert(name.as_str().to_owned(), entity); }
        }
        if let Some(children) = world.get::<Children>(entity) { stack.extend(children.iter()); }
    }
    let mut sources = HashMap::new();

    for id in 0..model.nmesh() as usize {
        let name = model.id_to_name(MjtObj::mjOBJ_MESH, id).unwrap_or("");
        let authored = spec.mesh_iter().find(|mesh| mesh.name() == name);
        let file = authored.and_then(|mesh| {
            if mesh.file().is_empty() { None } else { resolve_mesh(model_dir, meshdir, mesh.file()) }
        });
        let va = model.mesh_vertadr()[id] as usize;
        let fa = model.mesh_faceadr()[id] as usize;
        let vertices = model.mesh_vert()[va..va + model.mesh_vertnum()[id] as usize].to_vec();
        let faces = model.mesh_face()[fa..fa + model.mesh_facenum()[id] as usize].to_vec();
        let source = Arc::new(MeshSource {
            file,
            scale: authored.map(|m| *m.scale()).unwrap_or([1.0; 3]),
            refpos: authored.map(|m| *m.refpos()).unwrap_or([0.0; 3]),
            refquat: authored.map(|m| *m.refquat()).unwrap_or([1.0, 0.0, 0.0, 0.0]),
            vertices, faces,
            compiled_frame: compiled_mesh_transform(model.mesh_pos()[id], model.mesh_quat()[id]),
        });

        sources.insert(id, source);
    }
    for id in 0..model.ngeom() as usize {
        if model.geom_type()[id] != MjtGeom::mjGEOM_MESH { continue; }
        let body_id = model.geom_bodyid()[id] as usize;
        let parent = if body_id == 0 { root } else {
            let Some(name) = model.id_to_name(MjtObj::mjOBJ_BODY, body_id) else { continue };
            let Some(&entity) = bodies.get(name) else { continue };
            entity
        };
        let mesh_id = model.geom_dataid()[id] as usize;
        let mut rgba = model.geom_rgba()[id];
        let material_id = model.geom_matid()[id];
        if material_id >= 0 && rgba == [0.5, 0.5, 0.5, 1.0] {
            rgba = model.mat_rgba()[material_id as usize];
        }
        let name = model.id_to_name(MjtObj::mjOBJ_GEOM, id)
            .map(str::to_owned).unwrap_or_else(|| format!("melosim_geom_{id}"));
        let geometry = MeshGeometry {
            source: sources[&mesh_id].clone(), rgba,
            contype: model.geom_contype()[id], conaffinity: model.geom_conaffinity()[id],
            condim: model.geom_condim()[id], friction: model.geom_friction()[id],
            margin: model.geom_margin()[id], gap: model.geom_gap()[id], group: model.geom_group()[id],
        };
        world.spawn((Name::new(name), geometry, ChildOf(parent),
            compiled_mesh_transform(model.geom_pos()[id], model.geom_quat()[id])));

    }
}

fn update_body_inertias_from_compiled(world: &mut World, compiled: &MjModel) {
    use mujoco_rs::wrappers::mj_model::MjtObj;
    let name_to_entity: HashMap<String, Entity> = world
        .query_filtered::<(Entity, &Name), With<Body>>().iter(world)
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
            let q = compiled.body_iquat()[i];
            let rotation = nalgebra::UnitQuaternion::new_normalize(
                nalgebra::Quaternion::new(q[0], q[1], q[2], q[3]),
            ).to_rotation_matrix();
            let tensor = rotation.matrix() * nalgebra::Matrix3::from_diagonal(&Vector3::from(din))
                * rotation.matrix().transpose();
            ip.inertia = Inertia::new(tensor[(0, 0)], tensor[(1, 1)], tensor[(2, 2)],
                tensor[(0, 1)], tensor[(0, 2)], tensor[(1, 2)]);
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

fn spawn_body(
    builder: &mut ModelBuilder,
    mj: &MjsBody,
    parent: Entity,
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

    // Children bodies.
    for child in mj.body_iter(false) {
        spawn_body(builder, child, body_ent, counter, site_map)?;
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
    let range = *mj.range();
    let limited = matches!(mj.limited(), MjtLimited::mjLIMITED_TRUE)
        || (matches!(mj.limited(), MjtLimited::mjLIMITED_AUTO) && range[0] < range[1]);
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
