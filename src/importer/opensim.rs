// ── OpenSim Importer ──────────────────────────────────
// Intermediate data types and import functions for OpenSim models.

use serde::Deserialize;
use std::collections::HashMap;

use crate::components::*;
use crate::math::Transform;
use crate::world::World;
use bevy_ecs::prelude::Entity;

// ── Intermediate Data Types ───────────────────────────

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimModelData {
    pub name: String,
    pub bodies: Vec<OpenSimBodyData>,
    pub joints: Vec<OpenSimJointData>,
    pub markers: Vec<OpenSimMarkerData>,
    pub muscles: Vec<OpenSimMuscleData>,
    pub wrap_objects: Vec<OpenSimWrapData>,
    pub display_geometries: Vec<OpenSimDisplayGeometryData>,
    #[serde(default)]
    pub coordinate_actuators: Vec<OpenSimCoordinateActuatorData>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimBodyData {
    pub name: String,
    pub mass: f64,
    pub mass_center: [f64; 3],
    pub inertia: [f64; 6],
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimJointData {
    pub name: String,
    pub joint_type: String,
    pub parent_body: String,
    pub child_body: String,
    pub location_in_parent: [f64; 3],
    pub orientation_in_parent: [f64; 3],
    pub location_in_child: [f64; 3],
    pub orientation_in_child: [f64; 3],
    pub axis: Option<[f64; 3]>,
    pub coordinate: Option<OpenSimCoordinateData>,
    pub coordinates: Option<Vec<OpenSimCoordinateData>>,
    pub spatial_transform: Option<OpenSimSpatialTransformData>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimCoordinateData {
    pub name: String,
    pub range_min: f64,
    pub range_max: f64,
    pub default_value: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub clamped: bool,
    pub locked: bool,
    pub prescribed_function: Option<Vec<f64>>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimSpatialTransformData {
    pub rotation_x: Option<OpenSimEffectData>,
    pub rotation_y: Option<OpenSimEffectData>,
    pub rotation_z: Option<OpenSimEffectData>,
    pub translation_x: Option<OpenSimEffectData>,
    pub translation_y: Option<OpenSimEffectData>,
    pub translation_z: Option<OpenSimEffectData>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimEffectData {
    pub coordinate_name: String,
    pub function_type: String,
    pub coefficients: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimMarkerData {
    pub name: String,
    pub body: String,
    pub location: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimMuscleData {
    pub name: String,
    pub muscle_type: String,
    pub max_isometric_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub pennation_angle_at_optimal: f64,
    pub max_contraction_velocity: f64,
    pub activation_time_constant: f64,
    pub deactivation_time_constant: f64,
    pub minimum_activation: f64,
    pub fiber_damping: f64,
    pub ignore_activation_dynamics: bool,
    pub ignore_tendon_compliance: bool,
    pub path_points: Vec<OpenSimPathPointData>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimPathPointData {
    pub point_type: String,
    pub body: String,
    pub location: [f64; 3],
    pub coordinate: Option<String>,
    pub function: Option<Vec<f64>>,
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimWrapData {
    pub name: String,
    pub body: String,
    pub wrap_type: String,
    pub dimensions: Vec<f64>,
    pub location: [f64; 3],
    pub orientation: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimDisplayGeometryData {
    pub body: String,
    pub mesh_file: Option<String>,
    pub scale: [f64; 3],
    pub color: [f64; 3],
    pub opacity: f64,
    pub location: [f64; 3],
    pub orientation: [f64; 3],
}

#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimCoordinateActuatorData {
    pub name: String,
    pub coordinate: String,
    #[serde(default = "default_optimal_force")]
    pub optimal_force: f64,
    #[serde(default = "default_min_control")]
    pub min_control: f64,
    #[serde(default = "default_max_control")]
    pub max_control: f64,
}

fn default_optimal_force() -> f64 { 1.0 }
fn default_min_control() -> f64 { -1.0 }
fn default_max_control() -> f64 { 1.0 }

// ── Import Functions ──────────────────────────────────

pub fn import_opensim_model(
    world: &mut World,
    data: &OpenSimModelData,
) -> Result<(), Vec<String>> {
    let mut body_map: HashMap<String, Entity> = HashMap::new();
    let mut coord_map: HashMap<String, Entity> = HashMap::new();
    let mut errors = Vec::new();

    // Phase 1: Import all bodies
    for body_data in &data.bodies {
        match import_opensim_body(world, body_data) {
            Ok(key) => { body_map.insert(body_data.name.clone(), key); }
            Err(e) => errors.push(e),
        }
    }

    // Phase 2: Import all joints
    for joint_data in &data.joints {
        let parent = body_map.get(&joint_data.parent_body).copied();
        let child = body_map.get(&joint_data.child_body).copied();
        match (parent, child) {
            (Some(parent_key), Some(child_key)) => {
                if let Err(e) = import_opensim_joint(world, joint_data, parent_key, child_key) {
                    errors.push(e);
                }
            }
            (None, _) => errors.push(format!(
                "Joint '{}': parent body '{}' not found",
                joint_data.name, joint_data.parent_body
            )),
            (_, None) => errors.push(format!(
                "Joint '{}': child body '{}' not found",
                joint_data.name, joint_data.child_body
            )),
        }
    }

    // Build coordinate name → entity map
    let coord_items: Vec<(Entity, JointCoordinate)> = {
        let mut query = world.query::<(Entity, &JointCoordinate)>();
        query.iter(world).map(|(e, c)| (e, c.clone())).collect()
    };
    for (key, _coord) in coord_items {
        if let Some(name) = world.get::<Name>(key) {
            coord_map.insert(name.value.clone(), key);
        }
    }

    // Phase 3: Import markers
    for marker_data in &data.markers {
        if let Some(&body_key) = body_map.get(&marker_data.body) {
            import_opensim_marker(world, marker_data, body_key);
        } else {
            errors.push(format!(
                "Marker '{}': body '{}' not found",
                marker_data.name, marker_data.body
            ));
        }
    }

    // Phase 4: Import muscles
    for muscle_data in &data.muscles {
        match import_opensim_muscle(world, muscle_data, &body_map, &coord_map) {
            Ok(_) => {}
            Err(e) => errors.push(e),
        }
    }

    // Phase 5: Import wrap objects
    for wrap_data in &data.wrap_objects {
        match import_opensim_wrap(world, wrap_data, &body_map) {
            Ok(_) => {}
            Err(e) => errors.push(e),
        }
    }

    // Phase 6: Import display geometries
    for geom_data in &data.display_geometries {
        if let Err(e) = import_opensim_display_geometry(world, geom_data, &body_map) {
            errors.push(e);
        }
    }

    // Phase 7: Import coordinate actuators
    for act_data in &data.coordinate_actuators {
        match import_coordinate_actuator(world, act_data, &coord_map) {
            Ok(_) => {}
            Err(e) => errors.push(e),
        }
    }

    if errors.is_empty() { Ok(()) } else { Err(errors) }
}

pub fn import_opensim_body(
    world: &mut World,
    data: &OpenSimBodyData,
) -> Result<Entity, String> {
    let body_entity = world.spawn(()).id();
    world.entity_mut(body_entity).insert(InertialProperties {
        mass: data.mass,
        com: data.mass_center,
        inertia: data.inertia,
    });
    world.entity_mut(body_entity).insert(Name { value: data.name.clone() });
    Ok(body_entity)
}

pub fn import_opensim_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: Entity,
    child_key: Entity,
) -> Result<Entity, String> {
    match data.joint_type.as_str() {
        "PinJoint" => import_pin_joint(world, data, parent_key, child_key),
        "WeldJoint" => import_weld_joint(world, data, parent_key, child_key),
        "BallJoint" => import_ball_joint(world, data, parent_key, child_key),
        "FreeJoint" => import_free_joint(world, data, parent_key, child_key),
        "UniversalJoint" => import_universal_joint(world, data, parent_key, child_key),
        "CustomJoint" => import_custom_joint(world, data, parent_key, child_key),
        other => Err(format!("Joint '{}': unsupported type '{}'", data.name, other)),
    }
}

pub fn import_opensim_marker(
    world: &mut World,
    data: &OpenSimMarkerData,
    body_key: Entity,
) -> Entity {
    let site_entity = world.spawn(()).id();
    world.entity_mut(site_entity).insert(ChildOf { parent: body_key });
    world.entity_mut(site_entity).insert(Position::new(
        data.location[0], data.location[1], data.location[2],
    ));
    world.entity_mut(site_entity).insert(Name { value: data.name.clone() });
    site_entity
}

pub fn import_coordinate_actuator(
    world: &mut World,
    data: &OpenSimCoordinateActuatorData,
    coord_map: &HashMap<String, Entity>,
) -> Result<Entity, String> {
    let coord_key = coord_map.get(&data.coordinate).copied().ok_or_else(|| {
        format!("CoordinateActuator '{}': coordinate '{}' not found", data.name, data.coordinate)
    })?;
    let entity = world.spawn(()).id();
    world.entity_mut(entity).insert(CoordinateActuator {
        coordinate: coord_key,
        optimal_force: data.optimal_force,
        min_control: data.min_control,
        max_control: data.max_control,
    });
    world.entity_mut(entity).insert(Name { value: data.name.clone() });
    Ok(entity)
}

pub fn import_opensim_muscle(
    world: &mut World,
    data: &OpenSimMuscleData,
    body_map: &HashMap<String, Entity>,
    coord_map: &HashMap<String, Entity>,
) -> Result<Entity, String> {
    let muscle_entity = world.spawn(()).id();
    world.entity_mut(muscle_entity).insert(Muscle);
    world.entity_mut(muscle_entity).insert(Name { value: data.name.clone() });

    let params_entity = world.spawn(()).id();
    world.entity_mut(params_entity).insert(Millard2012Params {
        muscle: muscle_entity,
        max_isometric_force: data.max_isometric_force,
        optimal_fiber_length: data.optimal_fiber_length,
        tendon_slack_length: data.tendon_slack_length,
        pennation_angle_at_optimal: data.pennation_angle_at_optimal,
        max_contraction_velocity: data.max_contraction_velocity,
        activation_time_constant: data.activation_time_constant,
        deactivation_time_constant: data.deactivation_time_constant,
        minimum_activation: data.minimum_activation,
        fiber_damping: data.fiber_damping,
        ignore_activation_dynamics: data.ignore_activation_dynamics,
        ignore_tendon_compliance: data.ignore_tendon_compliance,
    });

    let mut path_points: Vec<PathPoint> = Vec::new();
    for pt in &data.path_points {
        let body_key = body_map.get(&pt.body).ok_or_else(|| {
            format!("Muscle '{}': path point references unknown body '{}'", data.name, pt.body)
        })?;

        let path_point = match pt.point_type.as_str() {
            "BodyFixedPathPoint" => PathPoint::BodyFixed {
                body: *body_key,
                location: pt.location,
            },
            "MovingPathPoint" => {
                let coord_key = pt.coordinate.as_ref().and_then(|name| coord_map.get(name)).copied().ok_or_else(|| {
                    format!("Muscle '{}': MovingPathPoint references unknown coordinate '{:?}'", data.name, pt.coordinate)
                })?;
                let empty_fn: Vec<f64> = Vec::new();
                let fn_coeffs = pt.function.as_ref().unwrap_or(&empty_fn);
                let location_functions = [fn_coeffs.clone(), fn_coeffs.clone(), fn_coeffs.clone()];
                PathPoint::Moving {
                    body: *body_key,
                    coordinate: coord_key,
                    location_functions,
                }
            }
            other => {
                return Err(format!("Muscle '{}': unsupported path point type '{}'", data.name, other));
            }
        };
        path_points.push(path_point);
    }

    let path_entity = world.spawn(()).id();
    world.entity_mut(path_entity).insert(MusclePath {
        muscle: muscle_entity,
        points: path_points,
    });

    Ok(muscle_entity)
}

pub fn import_opensim_wrap(
    world: &mut World,
    data: &OpenSimWrapData,
    body_map: &HashMap<String, Entity>,
) -> Result<Entity, String> {
    let body_key = body_map.get(&data.body).copied().ok_or_else(|| {
        format!("Wrap '{}': references unknown body '{}'", data.name, data.body)
    })?;

    let transform = Transform {
        translation: data.location.into(),
        rotation: euler_to_quaternion(data.orientation),
    };

    let geom_type = match data.wrap_type.as_str() {
        "Sphere" => {
            let radius = data.dimensions.first().copied().unwrap_or(0.0);
            WrapGeomType::Sphere { radius }
        }
        "Cylinder" => {
            let radius = data.dimensions.first().copied().unwrap_or(0.0);
            let length = data.dimensions.get(1).copied().unwrap_or(0.0);
            WrapGeomType::Cylinder { radius, length }
        }
        "Ellipsoid" => {
            let radii = [
                data.dimensions.first().copied().unwrap_or(0.0),
                data.dimensions.get(1).copied().unwrap_or(0.0),
                data.dimensions.get(2).copied().unwrap_or(0.0),
            ];
            WrapGeomType::Ellipsoid { radii }
        }
        other => {
            return Err(format!("Wrap '{}': unsupported type '{}'", data.name, other));
        }
    };

    let entity = world.spawn(()).id();
    world.entity_mut(entity).insert(WrapGeom {
        body: body_key,
        transform,
        geom_type,
    });
    world.entity_mut(entity).insert(Name { value: data.name.clone() });
    Ok(entity)
}

pub fn import_opensim_display_geometry(
    world: &mut World,
    data: &OpenSimDisplayGeometryData,
    body_map: &HashMap<String, Entity>,
) -> Result<(), String> {
    let body_key = body_map.get(&data.body).copied().ok_or_else(|| {
        format!("Display geometry: references unknown body '{}'", data.body)
    })?;

    let transform = Transform {
        translation: data.location.into(),
        rotation: euler_to_quaternion(data.orientation),
    };

    let entity = world.spawn(()).id();
    world.entity_mut(entity).insert(DisplayGeometry {
        body: body_key,
        mesh_file: data.mesh_file.clone(),
        scale: data.scale,
        color: data.color,
        opacity: data.opacity,
        transform,
    });

    Ok(())
}

// ── Helpers ───────────────────────────────────────────

fn euler_to_quaternion(euler: [f64; 3]) -> crate::math::Quaternion {
    let (roll, pitch, yaw) = (euler[0], euler[1], euler[2]);
    let cr = (roll * 0.5).cos();
    let sr = (roll * 0.5).sin();
    let cp = (pitch * 0.5).cos();
    let sp = (pitch * 0.5).sin();
    let cy = (yaw * 0.5).cos();
    let sy = (yaw * 0.5).sin();

    crate::math::Quaternion {
        w: cr * cp * cy + sr * sp * sy,
        x: sr * cp * cy - cr * sp * sy,
        y: cr * sp * cy + sr * cp * sy,
        z: cr * cp * sy - sr * sp * cy,
    }
}

// ── Unified Joint Importers ───────────────────────────

fn import_pin_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: Entity,
    child_key: Entity,
) -> Result<Entity, String> {
    let axis = data.axis.ok_or_else(|| format!("PinJoint '{}' missing axis", data.name))?;
    let joint_entity = world.spawn(()).id();
    world.entity_mut(joint_entity).insert(ChildOf { parent: parent_key });
    world.entity_mut(child_key).insert(ChildOf { parent: joint_entity });

    if let Some(coord) = &data.coordinate {
        let coord_entity = world.spawn(()).id();
        world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
        world.entity_mut(coord_entity).insert(JointCoordinate {
            range_min: coord.range_min,
            range_max: coord.range_max,
            default_value: coord.default_value,
            stiffness: coord.stiffness,
            damping: coord.damping,
            clamped: coord.clamped,
            locked: coord.locked,
            prescribed_function: coord.prescribed_function.as_ref().map(|c| {
                JointFunction::Polynomial { coefficients: c.clone() }
            }),
        });
        world.entity_mut(coord_entity).insert(Name { value: coord.name.clone() });

        let effect_entity = world.spawn(()).id();
        world.entity_mut(effect_entity).insert(ChildOf { parent: coord_entity });
        world.entity_mut(effect_entity).insert(CoordinateEffect {
            component: TransformComponent::RotationAboutAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
    }

    update_child_frame(world, parent_key, child_key, data);
    Ok(joint_entity)
}

fn import_weld_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: Entity,
    child_key: Entity,
) -> Result<Entity, String> {
    let joint_entity = world.spawn(()).id();
    world.entity_mut(joint_entity).insert(ChildOf { parent: parent_key });
    world.entity_mut(child_key).insert(ChildOf { parent: joint_entity });
    update_child_frame(world, parent_key, child_key, data);
    Ok(joint_entity)
}

fn import_ball_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: Entity,
    child_key: Entity,
) -> Result<Entity, String> {
    let joint_entity = world.spawn(()).id();
    world.entity_mut(joint_entity).insert(ChildOf { parent: parent_key });
    world.entity_mut(child_key).insert(ChildOf { parent: joint_entity });

    let mut coord_refs = Vec::new();
    if let Some(coord) = &data.coordinate {
        let coord_entity = world.spawn(()).id();
        world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
        world.entity_mut(coord_entity).insert(JointCoordinate {
            range_min: coord.range_min,
            range_max: coord.range_max,
            default_value: coord.default_value,
            stiffness: coord.stiffness,
            damping: coord.damping,
            clamped: coord.clamped,
            locked: coord.locked,
            prescribed_function: None,
        });
        world.entity_mut(coord_entity).insert(Name { value: coord.name.clone() });
        coord_refs.push(coord_entity);
    }

    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let coord_for_effects = coord_refs.first().copied();
    for axis in &axes {
        if let Some(coord_id) = coord_for_effects {
            let effect_entity = world.spawn(()).id();
            world.entity_mut(effect_entity).insert(ChildOf { parent: coord_id });
            world.entity_mut(effect_entity).insert(CoordinateEffect {
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
        }
    }

    update_child_frame(world, parent_key, child_key, data);
    Ok(joint_entity)
}

fn import_free_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: Entity,
    child_key: Entity,
) -> Result<Entity, String> {
    let joint_entity = world.spawn(()).id();
    world.entity_mut(joint_entity).insert(ChildOf { parent: parent_key });
    world.entity_mut(child_key).insert(ChildOf { parent: joint_entity });

    let rot_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for axis in &rot_axes {
        let coord_entity = world.spawn(()).id();
        world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
        world.entity_mut(coord_entity).insert(JointCoordinate {
            range_min: -1e10, range_max: 1e10, default_value: 0.0,
            stiffness: 0.0, damping: 0.0, clamped: false, locked: false,
            prescribed_function: None,
        });
        let effect_entity = world.spawn(()).id();
        world.entity_mut(effect_entity).insert(ChildOf { parent: coord_entity });
        world.entity_mut(effect_entity).insert(CoordinateEffect {
            component: TransformComponent::RotationAboutAxis(*axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
    }
    let trans_axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    for axis in &trans_axes {
        let coord_entity = world.spawn(()).id();
        world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
        world.entity_mut(coord_entity).insert(JointCoordinate {
            range_min: -1e10, range_max: 1e10, default_value: 0.0,
            stiffness: 0.0, damping: 0.0, clamped: false, locked: false,
            prescribed_function: None,
        });
        let effect_entity = world.spawn(()).id();
        world.entity_mut(effect_entity).insert(ChildOf { parent: coord_entity });
        world.entity_mut(effect_entity).insert(CoordinateEffect {
            component: TransformComponent::TranslationAlongAxis(*axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
    }

    update_child_frame(world, parent_key, child_key, data);
    Ok(joint_entity)
}

fn import_universal_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: Entity,
    child_key: Entity,
) -> Result<Entity, String> {
    let coords = data.coordinates.as_ref()
        .ok_or_else(|| format!("UniversalJoint '{}' missing coordinates", data.name))?;
    if coords.len() < 2 {
        return Err(format!("UniversalJoint '{}' needs 2 coordinates, got {}", data.name, coords.len()));
    }

    let joint_entity = world.spawn(()).id();
    world.entity_mut(joint_entity).insert(ChildOf { parent: parent_key });
    world.entity_mut(child_key).insert(ChildOf { parent: joint_entity });

    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];
    let mut coord_refs = Vec::new();
    for coord in coords {
        let coord_entity = world.spawn(()).id();
        world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
        world.entity_mut(coord_entity).insert(JointCoordinate {
            range_min: coord.range_min, range_max: coord.range_max,
            default_value: coord.default_value, stiffness: coord.stiffness,
            damping: coord.damping, clamped: coord.clamped, locked: coord.locked,
            prescribed_function: None,
        });
        world.entity_mut(coord_entity).insert(Name { value: coord.name.clone() });
        coord_refs.push(coord_entity);
    }

    for (i, coord_id) in coord_refs.iter().enumerate() {
        let axis = axes[i % axes.len()];
        let effect_entity = world.spawn(()).id();
        world.entity_mut(effect_entity).insert(ChildOf { parent: *coord_id });
        world.entity_mut(effect_entity).insert(CoordinateEffect {
            component: TransformComponent::RotationAboutAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
    }

    update_child_frame(world, parent_key, child_key, data);
    Ok(joint_entity)
}

fn import_custom_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: Entity,
    child_key: Entity,
) -> Result<Entity, String> {
    let coords = data.coordinates.as_ref()
        .ok_or_else(|| format!("CustomJoint '{}' missing coordinates", data.name))?;
    let st = data.spatial_transform.as_ref()
        .ok_or_else(|| format!("CustomJoint '{}' missing spatial_transform", data.name))?;

    let joint_entity = world.spawn(()).id();
    world.entity_mut(joint_entity).insert(ChildOf { parent: parent_key });
    world.entity_mut(child_key).insert(ChildOf { parent: joint_entity });

    let mut coord_ids: HashMap<String, Entity> = HashMap::new();
    for coord in coords {
        let coord_entity = world.spawn(()).id();
        world.entity_mut(coord_entity).insert(ChildOf { parent: joint_entity });
        world.entity_mut(coord_entity).insert(JointCoordinate {
            range_min: coord.range_min, range_max: coord.range_max,
            default_value: coord.default_value, stiffness: coord.stiffness,
            damping: coord.damping, clamped: coord.clamped, locked: coord.locked,
            prescribed_function: coord.prescribed_function.as_ref().map(|c| {
                JointFunction::Polynomial { coefficients: c.clone() }
            }),
        });
        world.entity_mut(coord_entity).insert(Name { value: coord.name.clone() });
        coord_ids.insert(coord.name.clone(), coord_entity);
    }

    let effects = [
        ("rotation_x", &st.rotation_x),
        ("rotation_y", &st.rotation_y),
        ("rotation_z", &st.rotation_z),
        ("translation_x", &st.translation_x),
        ("translation_y", &st.translation_y),
        ("translation_z", &st.translation_z),
    ];

    for (slot_name, effect_opt) in &effects {
        if let Some(effect) = effect_opt {
            let coord_id = coord_ids.get(&effect.coordinate_name).ok_or_else(|| {
                format!("CustomJoint '{}': effect '{}' references unknown coordinate '{}'",
                    data.name, slot_name, effect.coordinate_name)
            })?;

            let component = match *slot_name {
                "rotation_x" => TransformComponent::RotationX,
                "rotation_y" => TransformComponent::RotationY,
                "rotation_z" => TransformComponent::RotationZ,
                "translation_x" => TransformComponent::TranslationX,
                "translation_y" => TransformComponent::TranslationY,
                "translation_z" => TransformComponent::TranslationZ,
                _ => unreachable!(),
            };

            let function = match effect.function_type.as_str() {
                "Constant" => JointFunction::Constant(effect.coefficients[0]),
                "Linear" => JointFunction::Linear {
                    slope: effect.coefficients[0],
                    intercept: effect.coefficients.get(1).copied().unwrap_or(0.0),
                },
                "Polynomial" | _ => JointFunction::Polynomial {
                    coefficients: effect.coefficients.clone(),
                },
            };

            let effect_entity = world.spawn(()).id();
            world.entity_mut(effect_entity).insert(ChildOf { parent: *coord_id });
            world.entity_mut(effect_entity).insert(CoordinateEffect { component, function });
        }
    }

    update_child_frame(world, parent_key, child_key, data);
    Ok(joint_entity)
}

// ── Frame helper ──────────────────────────────────────

fn update_child_frame(
    world: &mut World,
    _parent_key: Entity,
    child_key: Entity,
    data: &OpenSimJointData,
) {
    world.entity_mut(child_key).insert(Position::new(
        data.location_in_child[0], data.location_in_child[1], data.location_in_child[2],
    ));
    world.entity_mut(child_key).insert(Rotation {
        quaternion: euler_to_quaternion(data.orientation_in_child),
    });
}

// ── JSON loading helper ───────────────────────────────

pub fn load_opensim_json(path: &str) -> Result<OpenSimModelData, String> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read '{}': {}", path, e))?;
    serde_json::from_str(&content)
        .map_err(|e| format!("Failed to parse '{}': {}", path, e))
}
