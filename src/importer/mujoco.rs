// ── MuJoCo MJCF Importer ──────────────────────────────
// Loads MJCF files via mujoco-rs (MuJoCo C parser) and
// populates a melosim World from the resolved model.
//
// MuJoCo resolves all <default> classes, <include> files,
// and computes derived quantities (body frames, joint transforms)
// at parse time. We walk the compiled MjModel arrays directly.
//
// Architecture:
//   MJCF file → MjModel::from_xml() → resolved flat arrays
//   → walk bodies/joints/geoms/sites/actuators/tendons
//   → create melosim ECS entities and components

use std::collections::HashMap;

use crate::components::*;
use crate::id::EntityID;
use crate::math::{Quaternion, Transform, Vec3};
use crate::world::World;

use mujoco_rs::wrappers::mj_model::*;
use mujoco_rs::mujoco_c::*;

/// Import a MuJoCo MJCF file into a melosim World.
///
/// Returns the populated World and a map from MuJoCo body IDs
/// to melosim EntityIDs (useful for downstream reference).
pub fn import_mjcf(path: &str) -> Result<(World, HashMap<i32, EntityID>), String> {
    let model = MjModel::from_xml(path)
        .map_err(|e| format!("Failed to load MJCF: {}", e))?;

    let mut world = World::new();

    // ── Ground body (entity 0) ──
    // MuJoCo's body 0 is the worldbody. We map it to our ground entity.
    let ground = world.spawn();
    world.attach(ground, InertialProperties {
        mass: 0.0,
        com: [0.0; 3],
        inertia: [0.0; 6],
    });
    let model_name = model.id_to_name(MjtObj::mjOBJ_BODY, 0)
        .unwrap_or("worldbody")
        .to_string();
    world.attach(ground, Name { value: model_name });

    // ── Body ID → EntityID mapping ──
    let mut body_map: HashMap<i32, EntityID> = HashMap::new();
    body_map.insert(0, ground);

    let nbody = model.nbody() as usize;
    let njnt = model.njnt() as usize;
    let ngeom = model.ngeom() as usize;
    let nsite = model.nsite() as usize;
    let nu = model.nu() as usize;
    let ntendon = model.ntendon() as usize;

    // ── Import bodies (skip body 0 = worldbody) ──
    for i in 1..nbody {
        let entity = world.spawn();

        // Inertial properties
        let mass = model.body_mass()[i];
        let ipos = model.body_ipos()[i];
        let inertia_diag = model.body_inertia()[i];
        world.attach(entity, InertialProperties {
            mass,
            com: ipos,
            inertia: [inertia_diag[0], inertia_diag[1], inertia_diag[2], 0.0, 0.0, 0.0],
        });

        // Name
        let name = model.id_to_name(MjtObj::mjOBJ_BODY, i)
            .unwrap_or("unnamed")
            .to_string();
        world.attach(entity, Name { value: name });

        // Frame (relative to parent body)
        let parent_id = model.body_parentid()[i];
        let parent = *body_map.get(&parent_id)
            .ok_or_else(|| format!("Body {} has unmapped parent {}", i, parent_id))?;
        let pos = model.body_pos()[i];
        let quat = model.body_quat()[i];
        world.attach(entity, Frame {
            parent,
            transform: Transform {
                translation: Vec3::new(pos[0], pos[1], pos[2]),
                rotation: Quaternion { w: quat[0], x: quat[1], y: quat[2], z: quat[3] },
            },
        });

        body_map.insert(i as i32, entity);
    }

    // ── Coordinate name → EntityID mapping (for actuator references) ──
    let mut coord_map: HashMap<i32, EntityID> = HashMap::new();

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
            .unwrap_or("unnamed_joint")
            .to_string();

        let axis = model.jnt_axis()[j];
        let axis_arr = [axis[0], axis[1], axis[2]];
        let range = model.jnt_range()[j];
        let has_limits = model.jnt_limited()[j];
        let limits = if has_limits {
            Some(JointLimits { lower: range[0], upper: range[1] })
        } else {
            None
        };

        let stiffness = model.jnt_stiffness()[j];

        // Get damping from dof (joint may have 0 or more dofs)
        let dof_adr = model.jnt_dofadr()[j];
        let damping = if dof_adr >= 0 {
            model.dof_damping()[dof_adr as usize]
        } else {
            0.0
        };

        let joint_entity = world.spawn();
        world.attach(joint_entity, Name { value: jnt_name });

        match jnt_type {
            mjtJoint_::mjJNT_HINGE => {
                world.attach(joint_entity, HingeJoint {
                    body_a,
                    body_b,
                    limits,
                    axis: axis_arr,
                });
                // Create a coordinate entity for the hinge DOF
                let coord_entity = world.spawn();
                let coord_name = model.id_to_name(MjtObj::mjOBJ_JOINT, j)
                    .unwrap_or("unnamed_coord");
                world.attach(coord_entity, Name { value: coord_name.to_string() });
                world.attach(coord_entity, JointCoordinate {
                    range_min: if has_limits { range[0] } else { -1e10 },
                    range_max: if has_limits { range[1] } else { 1e10 },
                    default_value: 0.0,
                    stiffness,
                    damping,
                    clamped: has_limits,
                    locked: false,
                    prescribed_function: None,
                });
                coord_map.insert(j as i32, coord_entity);
            }
            mjtJoint_::mjJNT_SLIDE => {
                world.attach(joint_entity, SlideJoint {
                    body_a,
                    body_b,
                    limits,
                    axis: axis_arr,
                });
                let coord_entity = world.spawn();
                let coord_name = model.id_to_name(MjtObj::mjOBJ_JOINT, j)
                    .unwrap_or("unnamed_coord");
                world.attach(coord_entity, Name { value: coord_name.to_string() });
                world.attach(coord_entity, JointCoordinate {
                    range_min: if has_limits { range[0] } else { -1e10 },
                    range_max: if has_limits { range[1] } else { 1e10 },
                    default_value: 0.0,
                    stiffness,
                    damping,
                    clamped: has_limits,
                    locked: false,
                    prescribed_function: None,
                });
                coord_map.insert(j as i32, coord_entity);
            }
            mjtJoint_::mjJNT_BALL => {
                world.attach(joint_entity, BallJoint {
                    body_a,
                    body_b,
                    limits,
                });
            }
            mjtJoint_::mjJNT_FREE => {
                world.attach(joint_entity, FreeJoint {
                    body_a,
                    body_b,
                    limits,
                });
            }
            // All four MuJoCo joint types are handled above.
        }
    }

    // ── Import sites ──
    for s in 0..nsite {
        let body_id = model.site_bodyid()[s];
        let parent = *body_map.get(&body_id)
            .ok_or_else(|| format!("Site {} attached to unmapped body {}", s, body_id))?;

        let pos = model.site_pos()[s];
        let site_entity = world.spawn();
        world.attach(site_entity, Site {
            parent,
            offset: Vec3::new(pos[0], pos[1], pos[2]),
        });

        let site_name = model.id_to_name(MjtObj::mjOBJ_SITE, s)
            .unwrap_or("unnamed_site")
            .to_string();
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

        // Map geom type to mesh file or primitive
        let geom_type = model.geom_type()[g];
        let mesh_file = match geom_type {
            mjtGeom_::mjGEOM_MESH => {
                // Mesh geoms reference mesh data; we store the name as reference
                model.id_to_name(MjtObj::mjOBJ_GEOM, g)
                    .map(|s| s.to_string())
            }
            _ => None, // Primitive geoms don't have mesh files
        };

        let geom_entity = world.spawn();
        world.attach(geom_entity, DisplayGeometry {
            body,
            mesh_file,
            scale: [size[0], size[1], size[2]],
            color: [rgba[0] as f64, rgba[1] as f64, rgba[2] as f64],
            opacity: rgba[3] as f64,
            transform: Transform {
                translation: Vec3::new(pos[0], pos[1], pos[2]),
                rotation: Quaternion { w: quat[0], x: quat[1], y: quat[2], z: quat[3] },
            },
        });

        let geom_name = model.id_to_name(MjtObj::mjOBJ_GEOM, g)
            .unwrap_or("unnamed_geom")
            .to_string();
        world.attach(geom_entity, Name { value: geom_name });
    }

    // ── Tendon name → path data mapping ──
    // In MuJoCo, spatial tendons reference sites and wrapping geoms.
    // We store the wrap path for muscle path construction.
    struct TendonPath {
        _name: String,
        path_points: Vec<PathPoint>,
    }
    let mut tendon_paths: Vec<TendonPath> = Vec::new();

    for t in 0..ntendon {
        let tendon_name = model.id_to_name(MjtObj::mjOBJ_TENDON, t)
            .unwrap_or("unnamed_tendon")
            .to_string();

        let wrap_adr = model.tendon_adr()[t] as usize;
        let wrap_num = model.tendon_num()[t] as usize;
        let wrap_types = model.wrap_type();
        let wrap_objids = model.wrap_objid();

        let mut path_points = Vec::new();
        for w in wrap_adr..(wrap_adr + wrap_num) {
            match wrap_types[w] {
                mjtWrap_::mjWRAP_SITE => {
                    // A site reference — objid is the site ID
                    let site_id = wrap_objids[w];
                    let site_pos = model.site_pos()[site_id as usize];
                    let site_body_id = model.site_bodyid()[site_id as usize];
                    let body = *body_map.get(&site_body_id)
                        .unwrap_or(&ground);
                    path_points.push(PathPoint::BodyFixed {
                        body,
                        location: site_pos,
                    });
                }
                mjtWrap_::mjWRAP_SPHERE => {
                    // A sphere wrapping object — objid is the geom ID
                    let geom_id = wrap_objids[w];
                    let geom_pos = model.geom_pos()[geom_id as usize];
                    let geom_body_id = model.geom_bodyid()[geom_id as usize];
                    let body = *body_map.get(&geom_body_id)
                        .unwrap_or(&ground);
                    // Treat as a path point at the wrap surface center
                    path_points.push(PathPoint::BodyFixed {
                        body,
                        location: geom_pos,
                    });
                }
                mjtWrap_::mjWRAP_CYLINDER => {
                    // A cylinder wrapping object
                    let geom_id = wrap_objids[w];
                    let geom_pos = model.geom_pos()[geom_id as usize];
                    let geom_body_id = model.geom_bodyid()[geom_id as usize];
                    let body = *body_map.get(&geom_body_id)
                        .unwrap_or(&ground);
                    path_points.push(PathPoint::BodyFixed {
                        body,
                        location: geom_pos,
                    });
                }
                _ => {
                    // Other wrap types (pulley, etc.) — skip for now
                }
            }
        }

        tendon_paths.push(TendonPath {
            _name: tendon_name,
            path_points,
        });
    }

    // ── Import actuators (muscles and coordinate actuators) ──
    let actuator_trntype = model.actuator_trntype();
    let actuator_dyntype = model.actuator_dyntype();
    let actuator_gaintype = model.actuator_gaintype();
    let actuator_biastype = model.actuator_biastype();
    let actuator_trnid = model.actuator_trnid();
    let actuator_gainprm = model.actuator_gainprm();
    let actuator_biasprm = model.actuator_biasprm();
    let actuator_dynprm = model.actuator_dynprm();
    let actuator_ctrlrange = model.actuator_ctrlrange();
    let _actuator_forcerange = model.actuator_forcerange();
    let actuator_gear = model.actuator_gear();

    for a in 0..nu {
        let act_name = model.id_to_name(MjtObj::mjOBJ_ACTUATOR, a)
            .unwrap_or("unnamed_actuator")
            .to_string();

        let is_muscle = actuator_dyntype[a] == mjtDyn_::mjDYN_MUSCLE
            || actuator_gaintype[a] == mjtGain_::mjGAIN_MUSCLE
            || actuator_biastype[a] == mjtBias_::mjBIAS_MUSCLE;

        if is_muscle {
            // ── Muscle actuator ──
            let muscle_entity = world.spawn();
            world.attach(muscle_entity, Muscle);
            world.attach(muscle_entity, Name { value: act_name.clone() });

            // Muscle parameters from MuJoCo's compiled model:
            // For <muscle> actuators:
            //   gear[0] = max_isometric_force (from MJCF "force" attribute)
            //   gainprm[0] = gain scaling factor
            //   biasprm[0..2] = muscle curve parameters
            //   dynprm[0] = activation time constant
            //   dynprm[1] = deactivation time constant
            //   ctrlrange[0] = minimum activation (from default class)
            //   lengthrange = [tendon_slack_length, max_muscle_length]
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
                max_contraction_velocity: 10.0, // MuJoCo default
                activation_time_constant: if act_time > 0.0 { act_time } else { 0.01 },
                deactivation_time_constant: if deact_time > 0.0 { deact_time } else { 0.04 },
                minimum_activation: if min_act > 0.0 { min_act } else { 0.01 },
                fiber_damping: 0.0,
                ignore_activation_dynamics: false,
                ignore_tendon_compliance: false,
            });

            // Find the tendon path for this muscle actuator
            // In MuJoCo, muscle actuators reference tendons via trnid[0]
            let tendon_id = actuator_trnid[a][0];
            if tendon_id >= 0 && (tendon_id as usize) < tendon_paths.len() {
                let tp = &tendon_paths[tendon_id as usize];
                world.attach(muscle_entity, MusclePath {
                    muscle: muscle_entity,
                    points: tp.path_points.clone(),
                });
            }
        } else if actuator_trntype[a] == mjtTrn_::mjTRN_JOINT {
            // ── Coordinate actuator (torque motor on a joint) ──
            let joint_id = actuator_trnid[a][0];
            if let Some(&coord_entity) = coord_map.get(&joint_id) {
                let act_entity = world.spawn();
                world.attach(act_entity, Name { value: act_name });
                world.attach(act_entity, CoordinateActuator {
                    coordinate: coord_entity,
                    optimal_force: actuator_gear[a][0].abs(),
                    min_control: actuator_ctrlrange[a][0],
                    max_control: actuator_ctrlrange[a][1],
                });
            }
        }
        // Other transmission types (tendon, site, body) — skip for now
    }

    Ok((world, body_map))
}
