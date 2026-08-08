// ── MuJoCo MJCF Importer ──────────────────────────────

use std::collections::HashMap;

use crate::components::*;
use crate::math::{Quaternion, Transform, Vec3};
use crate::world::World;
use crate::world::WorldExt;
use bevy_ecs::prelude::Entity;

use mujoco_rs::wrappers::mj_model::*;
use mujoco_rs::mujoco_c::*;

/// Import a MuJoCo MJCF file into a melosim World.
pub fn import_mjcf(path: &str) -> Result<(World, HashMap<i32, Entity>), String> {
    let model = MjModel::from_xml(path)
        .map_err(|e| format!("Failed to load MJCF: {}", e))?;

    let mut world = World::new();

    // ── Ground body (entity 0) ──
    let ground = world.spawn_entity();
    world.attach(ground, InertialProperties {
        mass: 0.0, com: [0.0; 3], inertia: [0.0; 6],
    });
    let model_name = model.id_to_name(MjtObj::mjOBJ_BODY, 0)
        .unwrap_or("worldbody").to_string();
    world.attach(ground, Name { value: model_name });

    let mut body_map: HashMap<i32, Entity> = HashMap::new();
    body_map.insert(0, ground);

    let nbody = model.nbody() as usize;
    let njnt = model.njnt() as usize;
    let ngeom = model.ngeom() as usize;
    let nsite = model.nsite() as usize;
    let nu = model.nu() as usize;
    let ntendon = model.ntendon() as usize;

    // ── Import bodies (skip body 0 = worldbody) ──
    for i in 1..nbody {
        let entity = world.spawn_entity();
        let mass = model.body_mass()[i];
        let ipos = model.body_ipos()[i];
        let inertia_diag = model.body_inertia()[i];
        world.attach(entity, InertialProperties {
            mass, com: ipos,
            inertia: [inertia_diag[0], inertia_diag[1], inertia_diag[2], 0.0, 0.0, 0.0],
        });
        let name = model.id_to_name(MjtObj::mjOBJ_BODY, i)
            .unwrap_or("unnamed").to_string();
        world.attach(entity, Name { value: name });
        let parent_id = model.body_parentid()[i];
        let parent = *body_map.get(&parent_id)
            .ok_or_else(|| format!("Body {} has unmapped parent {}", i, parent_id))?;
        let pos = model.body_pos()[i];
        let quat = model.body_quat()[i];
        world.set_parent(entity, parent);
        world.attach(entity, Position::new(pos[0], pos[1], pos[2]));
        world.attach(entity, Rotation { quaternion: Quaternion { w: quat[0], x: quat[1], y: quat[2], z: quat[3] } });
        body_map.insert(i as i32, entity);
    }

    let mut coord_map: HashMap<i32, Entity> = HashMap::new();

    // ── Import joints ──
    for j in 0..njnt {
        let jnt_type = model.jnt_type()[j];
        let body_id = model.jnt_bodyid()[j];
        let body_b = *body_map.get(&body_id)
            .ok_or_else(|| format!("Joint {} attached to unmapped body {}", j, body_id))?;
        let parent_id = model.body_parentid()[body_id as usize];
        let body_a = *body_map.get(&parent_id)
            .ok_or_else(|| format!("Joint {} parent body {} not mapped", j, parent_id))?;
        let jnt_name = model.id_to_name(MjtObj::mjOBJ_JOINT, j)
            .unwrap_or("unnamed_joint").to_string();
        let axis = model.jnt_axis()[j];
        let axis_arr = [axis[0], axis[1], axis[2]];
        let range = model.jnt_range()[j];
        let has_limits = model.jnt_limited()[j];
        let stiffness = model.jnt_stiffness()[j];
        let dof_adr = model.jnt_dofadr()[j];
        let damping = if dof_adr >= 0 { model.dof_damping()[dof_adr as usize] } else { 0.0 };

        let joint_entity = world.spawn_entity();
        world.attach(joint_entity, Name { value: jnt_name });
        world.set_parent(joint_entity, body_a);
        world.set_parent(body_b, joint_entity);

        match jnt_type {
            mjtJoint_::mjJNT_HINGE => {
                let coord_entity = world.spawn_entity();
                let coord_name = model.id_to_name(MjtObj::mjOBJ_JOINT, j)
                    .unwrap_or("unnamed_coord");
                world.set_parent(coord_entity, joint_entity);
                world.attach(coord_entity, Name { value: coord_name.to_string() });
                world.attach(coord_entity, JointCoordinate {
                    range_min: if has_limits { range[0] } else { -1e10 },
                    range_max: if has_limits { range[1] } else { 1e10 },
                    default_value: 0.0, stiffness, damping,
                    clamped: has_limits, locked: false, prescribed_function: None,
                });
                coord_map.insert(j as i32, coord_entity);
                let effect_entity = world.spawn_entity();
                world.set_parent(effect_entity, coord_entity);
                world.attach(effect_entity, CoordinateEffect {
                    component: TransformComponent::RotationAboutAxis(axis_arr),
                    function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
                });
            }
            mjtJoint_::mjJNT_SLIDE => {
                let coord_entity = world.spawn_entity();
                let coord_name = model.id_to_name(MjtObj::mjOBJ_JOINT, j)
                    .unwrap_or("unnamed_coord");
                world.set_parent(coord_entity, joint_entity);
                world.attach(coord_entity, Name { value: coord_name.to_string() });
                world.attach(coord_entity, JointCoordinate {
                    range_min: if has_limits { range[0] } else { -1e10 },
                    range_max: if has_limits { range[1] } else { 1e10 },
                    default_value: 0.0, stiffness, damping,
                    clamped: has_limits, locked: false, prescribed_function: None,
                });
                coord_map.insert(j as i32, coord_entity);
                let effect_entity = world.spawn_entity();
                world.set_parent(effect_entity, coord_entity);
                world.attach(effect_entity, CoordinateEffect {
                    component: TransformComponent::TranslationAlongAxis(axis_arr),
                    function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
                });
            }
            mjtJoint_::mjJNT_BALL => {}
            mjtJoint_::mjJNT_FREE => {}
        }
    }

    // ── Import sites ──
    for s in 0..nsite {
        let body_id = model.site_bodyid()[s];
        let parent = *body_map.get(&body_id)
            .ok_or_else(|| format!("Site {} attached to unmapped body {}", s, body_id))?;
        let pos = model.site_pos()[s];
        let site_entity = world.spawn_entity();
        world.set_parent(site_entity, parent);
        world.attach(site_entity, Position::new(pos[0], pos[1], pos[2]));
        let site_name = model.id_to_name(MjtObj::mjOBJ_SITE, s)
            .unwrap_or("unnamed_site").to_string();
        world.attach(site_entity, Name { value: site_name });
    }

    // ── Import geoms as DisplayGeometry ──
    for g in 0..ngeom {
        let body_id = model.geom_bodyid()[g];
        let body = *body_map.get(&body_id)
            .ok_or_else(|| format!("Geom {} attached to unmapped body {}", g, body_id))?;
        let pos = model.geom_pos()[g];
        let quat = model.geom_quat()[g];
        let rgba = model.geom_rgba()[g];
        let size = model.geom_size()[g];
        let geom_type = model.geom_type()[g];
        let mesh_file = match geom_type {
            mjtGeom_::mjGEOM_MESH => {
                let mesh_id = model.geom_dataid()[g] as usize;
                model.id_to_name(MjtObj::mjOBJ_MESH, mesh_id).map(|s| s.to_string())
            }
            _ => None,
        };

        let mut translation = [pos[0], pos[1], pos[2]];
        let mut rotation = quat;
        let mut scale = [size[0], size[1], size[2]];

        if geom_type == mjtGeom_::mjGEOM_MESH {
            let mid = model.geom_dataid()[g] as usize;
            let mp = model.mesh_pos()[mid];
            let q_pre = qconj(model.mesh_quat()[mid]);
            let t_pre = qrot(q_pre, [-mp[0], -mp[1], -mp[2]]);
            rotation = qmul(quat, q_pre);
            let rt = qrot(quat, t_pre);
            translation = [pos[0] + rt[0], pos[1] + rt[1], pos[2] + rt[2]];
            scale = model.mesh_scale()[mid];
        }

        let geom_entity = world.spawn_entity();
        world.attach(geom_entity, DisplayGeometry {
            body, mesh_file, scale,
            color: [rgba[0] as f64, rgba[1] as f64, rgba[2] as f64],
            opacity: rgba[3] as f64,
            transform: Transform {
                translation: Vec3::new(translation[0], translation[1], translation[2]),
                rotation: Quaternion { w: rotation[0], x: rotation[1], y: rotation[2], z: rotation[3] },
            },
        });
        let geom_name = model.id_to_name(MjtObj::mjOBJ_GEOM, g)
            .unwrap_or("unnamed_geom").to_string();
        world.attach(geom_entity, Name { value: geom_name });
    }

    // ── Tendon name → path data mapping ──
    struct TendonPathData {
        _name: String,
        path_points: Vec<PathPoint>,
    }
    let mut tendon_paths: Vec<TendonPathData> = Vec::new();

    for t in 0..ntendon {
        let tendon_name = model.id_to_name(MjtObj::mjOBJ_TENDON, t)
            .unwrap_or("unnamed_tendon").to_string();
        let wrap_adr = model.tendon_adr()[t] as usize;
        let wrap_num = model.tendon_num()[t] as usize;
        let wrap_types = model.wrap_type();
        let wrap_objids = model.wrap_objid();

        let mut path_points = Vec::new();
        for w in wrap_adr..(wrap_adr + wrap_num) {
            match wrap_types[w] {
                mjtWrap_::mjWRAP_SITE => {
                    let site_id = wrap_objids[w];
                    let site_pos = model.site_pos()[site_id as usize];
                    let site_body_id = model.site_bodyid()[site_id as usize];
                    let body = *body_map.get(&site_body_id).unwrap_or(&ground);
                    path_points.push(PathPoint::BodyFixed { body, location: site_pos });
                }
                mjtWrap_::mjWRAP_SPHERE | mjtWrap_::mjWRAP_CYLINDER => {
                    let geom_id = wrap_objids[w];
                    let geom_pos = model.geom_pos()[geom_id as usize];
                    let geom_body_id = model.geom_bodyid()[geom_id as usize];
                    let body = *body_map.get(&geom_body_id).unwrap_or(&ground);
                    path_points.push(PathPoint::BodyFixed { body, location: geom_pos });
                }
                _ => {}
            }
        }
        tendon_paths.push(TendonPathData { _name: tendon_name, path_points });
    }

    // ── Import actuators ──
    let actuator_trntype = model.actuator_trntype();
    let actuator_dyntype = model.actuator_dyntype();
    let actuator_gaintype = model.actuator_gaintype();
    let actuator_biastype = model.actuator_biastype();
    let actuator_trnid = model.actuator_trnid();
    let actuator_gainprm = model.actuator_gainprm();
    let actuator_biasprm = model.actuator_biasprm();
    let actuator_dynprm = model.actuator_dynprm();
    let actuator_ctrlrange = model.actuator_ctrlrange();
    let actuator_gear = model.actuator_gear();

    for a in 0..nu {
        let act_name = model.id_to_name(MjtObj::mjOBJ_ACTUATOR, a)
            .unwrap_or("unnamed_actuator").to_string();

        let is_muscle = actuator_dyntype[a] == mjtDyn_::mjDYN_MUSCLE
            || actuator_gaintype[a] == mjtGain_::mjGAIN_MUSCLE
            || actuator_biastype[a] == mjtBias_::mjBIAS_MUSCLE;

        if is_muscle {
            let muscle_entity = world.spawn_entity();
            world.attach(muscle_entity, Muscle);
            world.attach(muscle_entity, Name { value: act_name.clone() });
            let max_force = actuator_gear[a][0];
            let opt_fiber = actuator_biasprm[a][0];
            let tendon_slack = actuator_biasprm[a][1];
            let pennation = actuator_biasprm[a][2];
            let act_time = actuator_dynprm[a][0];
            let deact_time = actuator_dynprm[a][1];
            let min_act = actuator_ctrlrange[a][0];
            world.attach(muscle_entity, Millard2012Params {
                muscle: muscle_entity,
                max_isometric_force: max_force,
                optimal_fiber_length: if opt_fiber > 0.0 { opt_fiber } else { 0.1 },
                tendon_slack_length: if tendon_slack > 0.0 { tendon_slack } else { 0.1 },
                pennation_angle_at_optimal: pennation,
                max_contraction_velocity: 10.0,
                activation_time_constant: if act_time > 0.0 { act_time } else { 0.01 },
                deactivation_time_constant: if deact_time > 0.0 { deact_time } else { 0.04 },
                minimum_activation: if min_act > 0.0 { min_act } else { 0.01 },
                fiber_damping: 0.0,
                ignore_activation_dynamics: false,
                ignore_tendon_compliance: false,
            });
            let tendon_id = actuator_trnid[a][0];
            if tendon_id >= 0 && (tendon_id as usize) < tendon_paths.len() {
                let tp = &tendon_paths[tendon_id as usize];
                world.attach(muscle_entity, MusclePath {
                    muscle: muscle_entity,
                    points: tp.path_points.clone(),
                });
            }
        } else if actuator_trntype[a] == mjtTrn_::mjTRN_JOINT {
            let joint_id = actuator_trnid[a][0];
            if let Some(&coord_entity) = coord_map.get(&joint_id) {
                let act_entity = world.spawn_entity();
                world.attach(act_entity, Name { value: act_name });
                world.attach(act_entity, CoordinateActuator {
                    coordinate: coord_entity,
                    optimal_force: actuator_gear[a][0].abs(),
                    min_control: actuator_ctrlrange[a][0],
                    max_control: actuator_ctrlrange[a][1],
                });
            }
        }
    }

    Ok((world, body_map))
}

// ── Quaternion helpers (MuJoCo order: w, x, y, z) ─────

fn qconj(q: [f64; 4]) -> [f64; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

fn qmul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let (aw, ax, ay, az) = (a[0], a[1], a[2], a[3]);
    let (bw, bx, by, bz) = (b[0], b[1], b[2], b[3]);
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

fn qrot(q: [f64; 4], v: [f64; 3]) -> [f64; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    let c = [y * v[2] - z * v[1], z * v[0] - x * v[2], x * v[1] - y * v[0]];
    let cc = [y * c[2] - z * c[1], z * c[0] - x * c[2], x * c[1] - y * c[0]];
    [
        v[0] + 2.0 * (w * c[0] + cc[0]),
        v[1] + 2.0 * (w * c[1] + cc[1]),
        v[2] + 2.0 * (w * c[2] + cc[2]),
    ]
}
