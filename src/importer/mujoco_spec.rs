// ── MuJoCo MJCF Importer (MjSpec-based) ──────────────
//
// Loads MJCF files via MjSpec which preserves the original XML
// structure, attributes, and default classes. Walks the spec tree
// to populate a melosim World, then stores the MjSpec itself as
// a resource for lossless export.
//
// Unlike the MjModel-based importer, this approach:
// - Preserves original muscle force/lengthrange attributes
// - Preserves mesh references and asset paths
// - Preserves default class assignments
// - Preserves compiler options (angle units, etc.)
//
// The MjSpec is stored as a World resource so the exporter can
// modify it in-place and save back to XML with full fidelity.

use std::collections::HashMap;

use crate::components::*;
use crate::id::EntityID;
use crate::math::{Quaternion, Transform, Vec3};
use crate::world::World;

use mujoco_rs::wrappers::mj_editing::*;
use mujoco_rs::wrappers::mj_model::MjtJoint;

/// A stored MjSpec for lossless round-trip export.
/// Wraps the spec so it can be stored as a World resource.
pub struct StoredMjSpec {
    pub spec: MjSpec,
}

/// Import a MuJoCo MJCF file into a melosim World using MjSpec.
///
/// Returns the populated World and a map from MuJoCo body names
/// to melosim EntityIDs. The original MjSpec is stored as a
/// resource in the World for lossless export.
pub fn import_mjcf_spec(path: &str) -> Result<(World, HashMap<String, EntityID>), String> {
    let spec = MjSpec::from_xml(path)
        .map_err(|e| format!("Failed to load MJCF: {}", e))?;

    let mut world = World::new();
    let mut body_map: HashMap<String, EntityID> = HashMap::new();

    // ── Ground body (entity 0) ──
    let ground = world.spawn();
    world.attach(ground, InertialProperties {
        mass: 0.0,
        com: [0.0; 3],
        inertia: [0.0; 6],
    });
    world.attach(ground, Name { value: "world".to_string() });
    body_map.insert("world".to_string(), ground);

    // ── Walk the body tree recursively ──
    let world_body = spec.world_body();
    import_body_recursive(&mut world, &mut body_map, world_body, ground, &spec)?;

    // ── Import actuators ──
    for actuator in spec.actuator_iter() {
        import_actuator(&mut world, &body_map, actuator, &spec)?;
    }

    // Store the MjSpec for lossless export
    world.insert_resource(StoredMjSpec { spec });

    Ok((world, body_map))
}

/// Recursively import a body and its children from MjSpec.
fn import_body_recursive(
    world: &mut World,
    body_map: &mut HashMap<String, EntityID>,
    body: &MjsBody,
    parent_entity: EntityID,
    spec: &MjSpec,
) -> Result<(), String> {
    // Skip the worldbody itself (it's our ground)
    if body.name() == "world" {
        // Just recurse into children
        for child in body.body_iter(false) {
            import_body_recursive(world, body_map, child, parent_entity, spec)?;
        }
        return Ok(());
    }

    let entity = world.spawn();
    let body_name = body.name().to_string();

    // Inertial properties
    let mass = body.mass();
    let com = *body.ipos();
    let fullinertia = *body.fullinertia();
    world.attach(entity, InertialProperties {
        mass,
        com,
        inertia: fullinertia,
    });

    // Name
    world.attach(entity, Name { value: body_name.clone() });

    // Frame (relative to parent)
    let pos = *body.pos();
    let quat = *body.quat();
    world.attach(entity, ChildOf { parent: parent_entity });
    world.attach(entity, Position::new(pos[0], pos[1], pos[2]));
    world.attach(entity, Rotation { quaternion: Quaternion { w: quat[0], x: quat[1], y: quat[2], z: quat[3] } });

    body_map.insert(body_name, entity);

    // ── Import joints on this body ──
    // In MJCF, joints are defined on the child body
    import_body_joints(world, body, entity, parent_entity);

    // ── Import sites on this body ──
    for site in body.site_iter(false) {
        let site_entity = world.spawn();
        let site_name = site.name().to_string();
        let site_pos = *site.pos();
        world.attach(site_entity, ChildOf { parent: entity });
        world.attach(site_entity, Position::new(site_pos[0], site_pos[1], site_pos[2]));
        world.attach(site_entity, Site);
        world.attach(site_entity, Name { value: site_name });
    }

    // ── Import geoms on this body ──
    for geom in body.geom_iter(false) {
        let geom_entity = world.spawn();
        let geom_name = geom.name().to_string();
        let geom_pos = *geom.pos();
        let geom_quat = *geom.quat();
        let geom_size = *geom.size();
        let geom_rgba = *geom.rgba();
        let mesh_file = if geom.meshname().is_empty() {
            None
        } else {
            Some(geom.meshname().to_string())
        };

        world.attach(geom_entity, DisplayGeometry {
            body: entity,
            mesh_file,
            scale: geom_size,
            color: [geom_rgba[0] as f64, geom_rgba[1] as f64, geom_rgba[2] as f64],
            opacity: geom_rgba[3] as f64,
            transform: Transform {
                translation: Vec3::new(geom_pos[0], geom_pos[1], geom_pos[2]),
                rotation: Quaternion { w: geom_quat[0], x: geom_quat[1], y: geom_quat[2], z: geom_quat[3] },
            },
        });
        world.attach(geom_entity, Name { value: geom_name });
    }

    // ── Recurse into children ──
    for child in body.body_iter(false) {
        import_body_recursive(world, body_map, child, entity, spec)?;
    }

    Ok(())
}

/// Import joints from a body's joint list.
fn import_body_joints(
    world: &mut World,
    body: &MjsBody,
    child_entity: EntityID,
    parent_entity: EntityID,
) {
    for joint in body.joint_iter(false) {
        let jnt_name = joint.name().to_string();
        let jnt_type = joint.type_();
        let axis = *joint.axis();
        let range = *joint.range();
        let limited = matches!(joint.limited(), MjtLimited::mjLIMITED_TRUE);
        let limits = if limited {
            Some(JointLimits { lower: range[0], upper: range[1] })
        } else {
            None
        };

        let damping_arr = *joint.damping();
        let stiffness_arr = *joint.stiffness();
        let damping = damping_arr[0];
        let stiffness = stiffness_arr[0];

        let joint_entity = world.spawn();
        world.attach(joint_entity, Name { value: jnt_name.clone() });

        match jnt_type {
            MjtJoint::mjJNT_HINGE => {
                // Create coordinate entity
                let coord_entity = world.spawn();
                world.attach(coord_entity, Name { value: jnt_name });
                world.attach(coord_entity, JointCoordinate {
                    range_min: if limited { range[0] } else { -1e10 },
                    range_max: if limited { range[1] } else { 1e10 },
                    default_value: *joint.ref_(),
                    stiffness,
                    damping,
                    clamped: limited,
                    locked: false,
                    prescribed_function: None,
                });

                // Create CoordinateEffect: rotation about the hinge axis
                let effect_entity = world.spawn();
                world.attach(effect_entity, CoordinateEffect {
                    coordinate: coord_entity,
                    joint: joint_entity,
                    component: TransformComponent::RotationAboutAxis(axis),
                    function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
                });

                // Create SpatialTransform
                let st_entity = world.spawn();
                world.attach(st_entity, SpatialTransform {
                    joint: joint_entity,
                    effects: vec![effect_entity],
                });

                // Create the unified Joint
                world.attach(joint_entity, ParentFrame { frame: parent_entity });
                world.attach(joint_entity, ChildFrame { frame: child_entity });
                world.attach(joint_entity, Joint {
                    limits,
                    coordinates: vec![coord_entity],
                });
            }
            MjtJoint::mjJNT_SLIDE => {
                let coord_entity = world.spawn();
                world.attach(coord_entity, Name { value: jnt_name });
                world.attach(coord_entity, JointCoordinate {
                    range_min: if limited { range[0] } else { -1e10 },
                    range_max: if limited { range[1] } else { 1e10 },
                    default_value: *joint.ref_(),
                    stiffness,
                    damping,
                    clamped: limited,
                    locked: false,
                    prescribed_function: None,
                });

                // Create CoordinateEffect: translation along the slide axis
                let effect_entity = world.spawn();
                world.attach(effect_entity, CoordinateEffect {
                    coordinate: coord_entity,
                    joint: joint_entity,
                    component: TransformComponent::TranslationAlongAxis(axis),
                    function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
                });

                // Create SpatialTransform
                let st_entity = world.spawn();
                world.attach(st_entity, SpatialTransform {
                    joint: joint_entity,
                    effects: vec![effect_entity],
                });

                // Create the unified Joint
                world.attach(joint_entity, ParentFrame { frame: parent_entity });
                world.attach(joint_entity, ChildFrame { frame: child_entity });
                world.attach(joint_entity, Joint {
                    limits,
                    coordinates: vec![coord_entity],
                });
            }
            MjtJoint::mjJNT_BALL => {
                // Create the unified Joint
                world.attach(joint_entity, ParentFrame { frame: parent_entity });
                world.attach(joint_entity, ChildFrame { frame: child_entity });
                world.attach(joint_entity, Joint {
                    limits,
                    coordinates: vec![],
                });
            }
            MjtJoint::mjJNT_FREE => {
                // Create the unified Joint
                world.attach(joint_entity, ParentFrame { frame: parent_entity });
                world.attach(joint_entity, ChildFrame { frame: child_entity });
                world.attach(joint_entity, Joint {
                    limits: None,
                    coordinates: vec![],
                });
            }
        }
    }
}

/// Import an actuator from MjSpec.
fn import_actuator(
    world: &mut World,
    body_map: &HashMap<String, EntityID>,
    actuator: &MjsActuator,
    spec: &MjSpec,
) -> Result<(), String> {
    let act_name = actuator.name().to_string();
    let is_muscle = actuator.dyntype() == mujoco_rs::mujoco_c::mjtDyn_::mjDYN_MUSCLE;

    if is_muscle {
        // ── Muscle actuator ──
        let muscle_entity = world.spawn();
        world.attach(muscle_entity, Muscle);
        world.attach(muscle_entity, Name { value: act_name.clone() });

        // Store muscle params — these are the raw MJCF attributes,
        // not the compiled/lossy versions from MjModel.
        // For now, store default params; the MjSpec preserves the
        // original XML attributes for lossless export.
        world.attach(muscle_entity, Millard2012Params {
            muscle: muscle_entity,
            max_isometric_force: 1000.0,  // placeholder — actual value preserved in MjSpec
            optimal_fiber_length: 0.1,
            tendon_slack_length: 0.1,
            pennation_angle_at_optimal: 0.0,
            max_contraction_velocity: 10.0,
            activation_time_constant: 0.01,
            deactivation_time_constant: 0.04,
            minimum_activation: 0.01,
            fiber_damping: 0.0,
            ignore_activation_dynamics: false,
            ignore_tendon_compliance: false,
        });

        // Build muscle path from the tendon if referenced
        let target = actuator.target();
        if !target.is_empty() {
            // Find the spatial tendon with this name
            for tendon in spec.tendon_iter() {
                if tendon.name() == target {
                    let mut path_points = Vec::new();
                    for i in 0..tendon.wrap_num() {
                        let wrap = tendon.wrap(i);
                        // MjsWrap has wrap_type and obj_name
                        let obj_name = wrap.name();
                        match wrap.type_() {
                            mujoco_rs::mujoco_c::mjtWrap_::mjWRAP_SITE => {
                                // Find the site's body and position
                                if let Some((body_entity, location)) =
                                    find_site_info(spec, body_map, obj_name)
                                {
                                    path_points.push(PathPoint::BodyFixed {
                                        body: body_entity,
                                        location,
                                    });
                                }
                            }
                            _ => {
                                // Wrapping objects — treat as path points on their body
                                if let Some((body_entity, location)) =
                                    find_wrap_geom_info(spec, body_map, obj_name)
                                {
                                    path_points.push(PathPoint::BodyFixed {
                                        body: body_entity,
                                        location,
                                    });
                                }
                            }
                        }
                    }
                    world.attach(muscle_entity, MusclePath {
                        muscle: muscle_entity,
                        points: path_points,
                    });
                    break;
                }
            }
        }
    } else if actuator.trntype() == mujoco_rs::mujoco_c::mjtTrn_::mjTRN_JOINT {
        // ── Coordinate actuator ──
        let target = actuator.target();
        if !target.is_empty() {
            // Find the coordinate entity by joint name (collect to avoid borrow conflict)
            let matched_coord = world.iter::<JointCoordinate>()
                .find(|(coord_key, _)| {
                    world.get::<Name>(*coord_key).map(|n| n.value.as_str()) == Some(target)
                })
                .map(|(coord_key, _)| coord_key);

            if let Some(coord_key) = matched_coord {
                let act_entity = world.spawn();
                world.attach(act_entity, Name { value: act_name });
                world.attach(act_entity, CoordinateActuator {
                    coordinate: coord_key,
                    optimal_force: actuator.gear()[0].abs(),
                    min_control: actuator.ctrlrange()[0],
                    max_control: actuator.ctrlrange()[1],
                });
            }
        }
    }

    Ok(())
}

/// Find a site's body entity and position from MjSpec.
fn find_site_info(
    spec: &MjSpec,
    body_map: &HashMap<String, EntityID>,
    site_name: &str,
) -> Option<(EntityID, [f64; 3])> {
    for body in spec.body_iter() {
        for site in body.body_iter(true).flat_map(|b| {
            let mut sites = Vec::new();
            for s in b.site_iter(false) {
                sites.push((b.name(), s));
            }
            sites
        }) {
            if site.1.name() == site_name {
                let body_entity = *body_map.get(site.0)?;
                let pos = *site.1.pos();
                return Some((body_entity, pos));
            }
        }
    }
    None
}

/// Find a wrap geom's body entity and position from MjSpec.
fn find_wrap_geom_info(
    spec: &MjSpec,
    body_map: &HashMap<String, EntityID>,
    geom_name: &str,
) -> Option<(EntityID, [f64; 3])> {
    for body in spec.body_iter() {
        for geom in body.body_iter(true).flat_map(|b| {
            let mut geoms = Vec::new();
            for g in b.geom_iter(false) {
                geoms.push((b.name(), g));
            }
            geoms
        }) {
            if geom.1.name() == geom_name {
                let body_entity = *body_map.get(geom.0)?;
                let pos = *geom.1.pos();
                return Some((body_entity, pos));
            }
        }
    }
    None
}