// ── MuJoCo MJCF Importer (MjSpec-based) ──────────────

use std::collections::HashMap;

use crate::components::*;
use crate::math::{Quaternion, Transform, Vec3};
use crate::world::World;
use bevy_ecs::prelude::Entity;

use mujoco_rs::wrappers::mj_editing::*;
use mujoco_rs::wrappers::mj_model::MjtJoint;

/// A stored MjSpec for lossless round-trip export.
/// SAFETY: MjSpec contains NonNull which is !Send+!Sync, but melosim
/// is single-threaded. We wrap it in a newtype to satisfy Bevy's Resource trait.
pub struct StoredMjSpec {
    pub spec: MjSpec,
}

unsafe impl Send for StoredMjSpec {}
unsafe impl Sync for StoredMjSpec {}
impl bevy_ecs::system::Resource for StoredMjSpec {}

/// Import a MuJoCo MJCF file into a melosim World using MjSpec.
pub fn import_mjcf_spec(path: &str) -> Result<(World, HashMap<String, Entity>), String> {
    let spec = MjSpec::from_xml(path)
        .map_err(|e| format!("Failed to load MJCF: {}", e))?;

    let mut world = World::new();
    let mut body_map: HashMap<String, Entity> = HashMap::new();

    // ── Ground body (entity 0) ──
    let ground = world.spawn(()).id();
    world.entity_mut(ground).insert(InertialProperties {
        mass: 0.0, com: [0.0; 3], inertia: [0.0; 6],
    });
    world.entity_mut(ground).insert(Name { value: "world".to_string() });
    body_map.insert("world".to_string(), ground);

    let world_body = spec.world_body();
    import_body_recursive(&mut world, &mut body_map, world_body, ground, &spec)?;

    for actuator in spec.actuator_iter() {
        import_actuator(&mut world, &body_map, actuator, &spec)?;
    }

    world.insert_resource(StoredMjSpec { spec });
    Ok((world, body_map))
}

fn import_body_recursive(
    world: &mut World,
    body_map: &mut HashMap<String, Entity>,
    body: &MjsBody,
    parent_entity: Entity,
    spec: &MjSpec,
) -> Result<(), String> {
    if body.name() == "world" {
        for child in body.body_iter(false) {
            import_body_recursive(world, body_map, child, parent_entity, spec)?;
        }
        return Ok(());
    }

    let entity = world.spawn(()).id();
    let body_name = body.name().to_string();
    let mass = body.mass();
    let com = *body.ipos();
    let fullinertia = *body.fullinertia();
    world.entity_mut(entity).insert(InertialProperties {
        mass, com, inertia: fullinertia,
    });
    world.entity_mut(entity).insert(Name { value: body_name.clone() });
    let pos = *body.pos();
    let quat = *body.quat();
    world.entity_mut(entity).insert(ChildOf { parent: parent_entity });
    world.entity_mut(entity).insert(Position::new(pos[0], pos[1], pos[2]));
    world.entity_mut(entity).insert(Rotation { quaternion: Quaternion { w: quat[0], x: quat[1], y: quat[2], z: quat[3] } });
    body_map.insert(body_name, entity);

    import_body_joints(world, body, entity, parent_entity);

    for site in body.site_iter(false) {
        let site_entity = world.spawn(()).id();
        let site_name = site.name().to_string();
        let site_pos = *site.pos();
        world.entity_mut(site_entity).insert(ChildOf { parent: entity });
        world.entity_mut(site_entity).insert(Position::new(site_pos[0], site_pos[1], site_pos[2]));
        world.entity_mut(site_entity).insert(Name { value: site_name });
    }

    for geom in body.geom_iter(false) {
        let geom_entity = world.spawn(()).id();
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
        world.entity_mut(geom_entity).insert(DisplayGeometry {
            body: entity, mesh_file, scale: geom_size,
            color: [geom_rgba[0] as f64, geom_rgba[1] as f64, geom_rgba[2] as f64],
            opacity: geom_rgba[3] as f64,
            transform: Transform {
                translation: Vec3::new(geom_pos[0], geom_pos[1], geom_pos[2]),
                rotation: Quaternion { w: geom_quat[0], x: geom_quat[1], y: geom_quat[2], z: geom_quat[3] },
            },
        });
        world.entity_mut(geom_entity).insert(Name { value: geom_name });
    }

    for child in body.body_iter(false) {
        import_body_recursive(world, body_map, child, entity, spec)?;
    }

    Ok(())
}

fn import_body_joints(
    world: &mut World,
    body: &MjsBody,
    child_entity: Entity,
    parent_entity: Entity,
) {
    for joint in body.joint_iter(false) {
        let jnt_name = joint.name().to_string();
        let jnt_type = joint.type_();
        let axis = *joint.axis();
        let range = *joint.range();
        let limited = matches!(joint.limited(), MjtLimited::mjLIMITED_TRUE);
        let damping_arr = *joint.damping();
        let stiffness_arr = *joint.stiffness();
        let damping = damping_arr[0];
        let stiffness = stiffness_arr[0];

        let joint_entity = world.spawn(()).id();
        world.entity_mut(joint_entity).insert(Name { value: jnt_name.clone() });
        world.entity_mut(joint_entity).insert(ChildOf { parent: parent_entity });
        world.entity_mut(child_entity).insert(ChildOf { parent: joint_entity });

        match jnt_type {
            MjtJoint::mjJNT_HINGE => {
                let coord_entity = world.spawn(()).id();
                world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
                world.entity_mut(coord_entity).insert(Name { value: jnt_name });
                world.entity_mut(coord_entity).insert(JointCoordinate {
                    range_min: if limited { range[0] } else { -1e10 },
                    range_max: if limited { range[1] } else { 1e10 },
                    default_value: *joint.ref_(),
                    stiffness, damping, clamped: limited, locked: false,
                    prescribed_function: None,
                });
                let effect_entity = world.spawn(()).id();
                world.entity_mut(effect_entity).insert(ChildOf { parent: coord_entity });
                world.entity_mut(effect_entity).insert(CoordinateEffect {
                    component: TransformComponent::RotationAboutAxis(axis),
                    function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
                });
            }
            MjtJoint::mjJNT_SLIDE => {
                let coord_entity = world.spawn(()).id();
                world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
                world.entity_mut(coord_entity).insert(Name { value: jnt_name });
                world.entity_mut(coord_entity).insert(JointCoordinate {
                    range_min: if limited { range[0] } else { -1e10 },
                    range_max: if limited { range[1] } else { 1e10 },
                    default_value: *joint.ref_(),
                    stiffness, damping, clamped: limited, locked: false,
                    prescribed_function: None,
                });
                let effect_entity = world.spawn(()).id();
                world.entity_mut(effect_entity).insert(ChildOf { parent: coord_entity });
                world.entity_mut(effect_entity).insert(CoordinateEffect {
                    component: TransformComponent::TranslationAlongAxis(axis),
                    function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
                });
            }
            MjtJoint::mjJNT_BALL => {}
            MjtJoint::mjJNT_FREE => {}
        }
    }
}

fn import_actuator(
    world: &mut World,
    body_map: &HashMap<String, Entity>,
    actuator: &MjsActuator,
    spec: &MjSpec,
) -> Result<(), String> {
    let act_name = actuator.name().to_string();
    let is_muscle = actuator.dyntype() == mujoco_rs::mujoco_c::mjtDyn_::mjDYN_MUSCLE;

    if is_muscle {
        let muscle_entity = world.spawn(()).id();
        world.entity_mut(muscle_entity).insert(Muscle);
        world.entity_mut(muscle_entity).insert(Name { value: act_name.clone() });
        world.entity_mut(muscle_entity).insert(Millard2012Params {
            muscle: muscle_entity,
            max_isometric_force: 1000.0,
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

        let target = actuator.target();
        if !target.is_empty() {
            for tendon in spec.tendon_iter() {
                if tendon.name() == target {
                    let mut path_points = Vec::new();
                    for i in 0..tendon.wrap_num() {
                        let wrap = tendon.wrap(i);
                        let obj_name = wrap.name();
                        match wrap.type_() {
                            mujoco_rs::mujoco_c::mjtWrap_::mjWRAP_SITE => {
                                if let Some((body_entity, location)) =
                                    find_site_info(spec, body_map, obj_name)
                                {
                                    path_points.push(PathPoint::BodyFixed {
                                        body: body_entity, location,
                                    });
                                }
                            }
                            _ => {
                                if let Some((body_entity, location)) =
                                    find_wrap_geom_info(spec, body_map, obj_name)
                                {
                                    path_points.push(PathPoint::BodyFixed {
                                        body: body_entity, location,
                                    });
                                }
                            }
                        }
                    }
                    world.entity_mut(muscle_entity).insert(MusclePath {
                        muscle: muscle_entity, points: path_points,
                    });
                    break;
                }
            }
        }
    } else if actuator.trntype() == mujoco_rs::mujoco_c::mjtTrn_::mjTRN_JOINT {
        let target = actuator.target();
        if !target.is_empty() {
            // Find coordinate by name
            let coord_items: Vec<(Entity, JointCoordinate)> = {
                let mut query = world.query::<(Entity, &JointCoordinate)>();
                query.iter(world).map(|(e, c)| (e, c.clone())).collect()
            };
            let matched_coord = coord_items
                .into_iter()
                .find(|(coord_key, _)| {
                    world.get::<Name>(*coord_key).map(|n| n.value.as_str()) == Some(target)
                })
                .map(|(coord_key, _)| coord_key);

            if let Some(coord_key) = matched_coord {
                let act_entity = world.spawn(()).id();
                world.entity_mut(act_entity).insert(Name { value: act_name });
                world.entity_mut(act_entity).insert(CoordinateActuator {
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

fn find_site_info(
    spec: &MjSpec,
    body_map: &HashMap<String, Entity>,
    site_name: &str,
) -> Option<(Entity, [f64; 3])> {
    for body in spec.body_iter() {
        for site in body.body_iter(true).flat_map(|b| {
            let mut sites = Vec::new();
            for s in b.site_iter(false) { sites.push((b.name(), s)); }
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

fn find_wrap_geom_info(
    spec: &MjSpec,
    body_map: &HashMap<String, Entity>,
    geom_name: &str,
) -> Option<(Entity, [f64; 3])> {
    for body in spec.body_iter() {
        for geom in body.body_iter(true).flat_map(|b| {
            let mut geoms = Vec::new();
            for g in b.geom_iter(false) { geoms.push((b.name(), g)); }
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
