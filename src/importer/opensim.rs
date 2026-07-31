// ── OpenSim Importer ──────────────────────────────────
// Intermediate data types and import functions for OpenSim models.
//
// Architecture:
//   Python extraction script (on user's machine with OpenSim installed)
//   → JSON file → Rust importer (via serde_json)
//
// Each OpenSim component type has a corresponding *Data struct that
// carries only the fields needed for import. The import functions
// create melosim ECS entities and components from these structs.
//
// The importer is designed to be built incrementally:
//   1. Start with one joint type (PinJoint) + two bodies
//   2. Verify round-trip (JSON → World → validate)
//   3. Add more component types iteratively

use serde::Deserialize;
use std::collections::HashMap;

use crate::components::*;
use crate::id::EntityID;
use crate::math::Transform;
use crate::world::World;

// ── Intermediate Data Types ───────────────────────────
// These mirror the OpenSim model structure and are deserialized
// from JSON produced by the Python extraction script.

/// Full OpenSim model representation.
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

/// An OpenSim Body's essential properties.
#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimBodyData {
    pub name: String,
    pub mass: f64,
    pub mass_center: [f64; 3],
    /// Inertia tensor: [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
    pub inertia: [f64; 6],
}

/// An OpenSim Joint, with type-specific data in optional fields.
#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimJointData {
    pub name: String,
    pub joint_type: String, // "PinJoint", "CustomJoint", "UniversalJoint", etc.
    pub parent_body: String,
    pub child_body: String,
    /// Translation from parent frame to joint frame.
    pub location_in_parent: [f64; 3],
    /// Euler angles (radians) from parent frame to joint frame.
    pub orientation_in_parent: [f64; 3],
    /// Translation from child frame to joint frame.
    pub location_in_child: [f64; 3],
    /// Euler angles (radians) from child frame to joint frame.
    pub orientation_in_child: [f64; 3],
    // PinJoint / hinge-like joints
    pub axis: Option<[f64; 3]>,
    pub coordinate: Option<OpenSimCoordinateData>,
    // CustomJoint
    pub coordinates: Option<Vec<OpenSimCoordinateData>>,
    pub spatial_transform: Option<OpenSimSpatialTransformData>,
}

/// An OpenSim Coordinate (generalized DOF).
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
    /// Polynomial coefficients for prescribed function (optional).
    pub prescribed_function: Option<Vec<f64>>,
}

/// SpatialTransform for CustomJoint — up to 6 CoordinateEffects.
#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimSpatialTransformData {
    pub rotation_x: Option<OpenSimEffectData>,
    pub rotation_y: Option<OpenSimEffectData>,
    pub rotation_z: Option<OpenSimEffectData>,
    pub translation_x: Option<OpenSimEffectData>,
    pub translation_y: Option<OpenSimEffectData>,
    pub translation_z: Option<OpenSimEffectData>,
}

/// A single CoordinateEffect within a SpatialTransform.
#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimEffectData {
    pub coordinate_name: String,
    pub function_type: String, // "Constant", "Linear", "Polynomial"
    pub coefficients: Vec<f64>,
}

/// An OpenSim Marker (anatomical landmark on a body).
#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimMarkerData {
    pub name: String,
    pub body: String,
    pub location: [f64; 3],
}

/// An OpenSim Muscle's essential properties for import.
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

/// A single point along a muscle path, as represented in import data.
#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimPathPointData {
    pub point_type: String,  // "BodyFixedPathPoint", "MovingPathPoint"
    pub body: String,
    pub location: [f64; 3],
    pub coordinate: Option<String>,
    pub function: Option<Vec<f64>>,
}

/// A wrapping surface imported from OpenSim.
#[derive(Clone, Debug, Deserialize)]
pub struct OpenSimWrapData {
    pub name: String,
    pub body: String,
    pub wrap_type: String, // "Sphere", "Cylinder", "Ellipsoid"
    pub dimensions: Vec<f64>,
    pub location: [f64; 3],
    pub orientation: [f64; 3],
}

/// A display (visual) geometry imported from OpenSim.
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

/// A CoordinateActuator — torque on a single coordinate.
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

/// Import a full OpenSim model from intermediate data into a World.
///
/// Returns Ok(()) on success, or Err with a list of error messages
/// if any component failed to import (missing body references, etc.).
pub fn import_opensim_model(
    world: &mut World,
    data: &OpenSimModelData,
) -> Result<(), Vec<String>> {
    let mut body_map: HashMap<String, EntityID> = HashMap::new();
    let mut coord_map: HashMap<String, EntityID> = HashMap::new();
    let mut errors = Vec::new();

    // Phase 1: Import all bodies, build name → key map
    for body_data in &data.bodies {
        match import_opensim_body(world, body_data) {
            Ok(key) => {
                body_map.insert(body_data.name.clone(), key);
            }
            Err(e) => errors.push(e),
        }
    }

    // Phase 2: Import all joints using resolved body keys
    for joint_data in &data.joints {
        let parent = body_map.get(&joint_data.parent_body).copied();
        let child = body_map.get(&joint_data.child_body).copied();
        match (parent, child) {
            (Some(parent_key), Some(child_key)) => {
                if let Err(e) =
                    import_opensim_joint(world, joint_data, parent_key, child_key)
                {
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

    // Build coordinate name → key map from the world
    for (key, _coord) in world.iter::<JointCoordinate>() {
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

    // Phase 4: Import muscles (using body_map AND coord_map)
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

    if errors.is_empty() {
        Ok(())
    } else {
        Err(errors)
    }
}

/// Import a single OpenSim body: creates InertialProperties + Frame entities.
pub fn import_opensim_body(
    world: &mut World,
    data: &OpenSimBodyData,
) -> Result<EntityID, String> {
    let body_entity = world.spawn();
    world.attach(body_entity, InertialProperties {
        mass: data.mass,
        com: data.mass_center,
        inertia: data.inertia,
    });
    world.attach(body_entity, Name { value: data.name.clone() });
    // Frame is a separate entity referencing the body
    let frame_entity = world.spawn();
    world.attach(frame_entity, Frame {
        parent: body_entity,
        transform: Transform::default(),
    });
    Ok(body_entity)
}

/// Import a single OpenSim joint, dispatching by type.
/// All joint types produce a unified `Joint` component.
pub fn import_opensim_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: EntityID,
    child_key: EntityID,
) -> Result<EntityID, String> {
    match data.joint_type.as_str() {
        "PinJoint" => import_pin_joint(world, data, parent_key, child_key),
        "WeldJoint" => import_weld_joint(world, data, parent_key, child_key),
        "BallJoint" => import_ball_joint(world, data, parent_key, child_key),
        "FreeJoint" => import_free_joint(world, data, parent_key, child_key),
        "UniversalJoint" => import_universal_joint(world, data, parent_key, child_key),
        "CustomJoint" => import_custom_joint(world, data, parent_key, child_key),
        other => Err(format!(
            "Joint '{}': unsupported type '{}'",
            data.name, other
        )),
    }
}

/// Import a single OpenSim marker: creates a Site entity with Name.
pub fn import_opensim_marker(
    world: &mut World,
    data: &OpenSimMarkerData,
    body_key: EntityID,
) -> EntityID {
    let site_entity = world.spawn();
    world.attach(site_entity, Site {
        parent: body_key,
        offset: data.location.into(),
    });
    world.attach(site_entity, Name { value: data.name.clone() });
    site_entity
}

/// Import a single CoordinateActuator.
pub fn import_coordinate_actuator(
    world: &mut World,
    data: &OpenSimCoordinateActuatorData,
    coord_map: &HashMap<String, EntityID>,
) -> Result<EntityID, String> {
    let coord_key = coord_map.get(&data.coordinate).copied().ok_or_else(|| {
        format!(
            "CoordinateActuator '{}': coordinate '{}' not found",
            data.name, data.coordinate
        )
    })?;
    let entity = world.spawn();
    world.attach(entity, CoordinateActuator {
        coordinate: coord_key,
        optimal_force: data.optimal_force,
        min_control: data.min_control,
        max_control: data.max_control,
    });
    world.attach(entity, Name { value: data.name.clone() });
    Ok(entity)
}

/// Import a single OpenSim muscle: creates Muscle + MusclePath + Millard2012Params.
pub fn import_opensim_muscle(
    world: &mut World,
    data: &OpenSimMuscleData,
    body_map: &HashMap<String, EntityID>,
    coord_map: &HashMap<String, EntityID>,
) -> Result<EntityID, String> {
    // Step 1: Create the Muscle entity
    let muscle_entity = world.spawn();
    world.attach(muscle_entity, Muscle);
    world.attach(muscle_entity, Name { value: data.name.clone() });

    // Step 2: Create Millard2012Params entity referencing the muscle
    let params_entity = world.spawn();
    world.attach(params_entity, Millard2012Params {
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

    // Step 3: Build PathPoints
    let mut path_points: Vec<PathPoint> = Vec::new();
    for pt in &data.path_points {
        let body_key = body_map.get(&pt.body).ok_or_else(|| {
            format!(
                "Muscle '{}': path point references unknown body '{}'",
                data.name, pt.body
            )
        })?;

        let path_point = match pt.point_type.as_str() {
            "BodyFixedPathPoint" => PathPoint::BodyFixed {
                body: *body_key,
                location: pt.location,
            },
            "MovingPathPoint" => {
                let coord_key = pt.coordinate.as_ref().and_then(|name| coord_map.get(name)).copied().ok_or_else(|| {
                    format!(
                        "Muscle '{}': MovingPathPoint references unknown coordinate '{:?}'",
                        data.name, pt.coordinate
                    )
                })?;

                let empty_fn: Vec<f64> = Vec::new();
                let fn_coeffs = pt.function.as_ref().unwrap_or(&empty_fn);
                let location_functions = [
                    fn_coeffs.clone(),
                    fn_coeffs.clone(),
                    fn_coeffs.clone(),
                ];

                PathPoint::Moving {
                    body: *body_key,
                    coordinate: coord_key,
                    location_functions,
                }
            }
            other => {
                return Err(format!(
                    "Muscle '{}': unsupported path point type '{}'",
                    data.name, other
                ));
            }
        };
        path_points.push(path_point);
    }

    // Step 4: Create MusclePath entity
    let path_entity = world.spawn();
    world.attach(path_entity, MusclePath {
        muscle: muscle_entity,
        points: path_points,
    });

    Ok(muscle_entity)
}

/// Import a single OpenSim wrap object: creates WrapGeom entity.
pub fn import_opensim_wrap(
    world: &mut World,
    data: &OpenSimWrapData,
    body_map: &HashMap<String, EntityID>,
) -> Result<EntityID, String> {
    let body_key = body_map.get(&data.body).copied().ok_or_else(|| {
        format!(
            "Wrap '{}': references unknown body '{}'",
            data.name, data.body
        )
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
            return Err(format!(
                "Wrap '{}': unsupported type '{}'",
                data.name, other
            ));
        }
    };

    let entity = world.spawn();
    world.attach(entity, WrapGeom {
        body: body_key,
        transform,
        geom_type,
    });
    world.attach(entity, Name { value: data.name.clone() });

    Ok(entity)
}

/// Import a single OpenSim display geometry: creates DisplayGeometry entity.
pub fn import_opensim_display_geometry(
    world: &mut World,
    data: &OpenSimDisplayGeometryData,
    body_map: &HashMap<String, EntityID>,
) -> Result<(), String> {
    let body_key = body_map.get(&data.body).copied().ok_or_else(|| {
        format!(
            "Display geometry: references unknown body '{}'",
            data.body
        )
    })?;

    let transform = Transform {
        translation: data.location.into(),
        rotation: euler_to_quaternion(data.orientation),
    };

    let entity = world.spawn();
    world.attach(entity, DisplayGeometry {
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

/// Convert XYZ Euler angles (in radians) to a Quaternion.
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
// Each creates a Joint component with the appropriate joint_type,
// plus coordinate entities, CoordinateEffects, and SpatialTransform.

fn import_pin_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: EntityID,
    child_key: EntityID,
) -> Result<EntityID, String> {
    let axis = data
        .axis
        .ok_or_else(|| format!("PinJoint '{}' missing axis", data.name))?;

    let joint_entity = world.spawn();

    // Create coordinate if present
    let mut coord_refs = Vec::new();
    if let Some(coord) = &data.coordinate {
        let coord_entity = world.spawn();
        world.attach(coord_entity, JointCoordinate {
            range_min: coord.range_min,
            range_max: coord.range_max,
            default_value: coord.default_value,
            stiffness: coord.stiffness,
            damping: coord.damping,
            clamped: coord.clamped,
            locked: coord.locked,
            prescribed_function: coord.prescribed_function.as_ref().map(|c| {
                JointFunction::Polynomial {
                    coefficients: c.clone(),
                }
            }),
        });
        world.attach(coord_entity, Name { value: coord.name.clone() });
        coord_refs.push(coord_entity);

        // Create CoordinateEffect: rotation about the pin axis
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
    }

    let limits = data.coordinate.as_ref().map(|c| JointLimits {
        lower: c.range_min,
        upper: c.range_max,
    });

    world.attach(joint_entity, Joint {
        body_a: parent_key,
        body_b: child_key,
        limits,
        joint_type: "PinJoint",
        coordinates: coord_refs,
    });

    update_child_frame(world, child_key, data);
    Ok(joint_entity)
}

fn import_weld_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: EntityID,
    child_key: EntityID,
) -> Result<EntityID, String> {
    let joint_entity = world.spawn();
    world.attach(joint_entity, Joint {
        body_a: parent_key,
        body_b: child_key,
        limits: None,
        joint_type: "WeldJoint",
        coordinates: vec![],
    });

    update_child_frame(world, child_key, data);
    Ok(joint_entity)
}

fn import_ball_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: EntityID,
    child_key: EntityID,
) -> Result<EntityID, String> {
    let joint_entity = world.spawn();
    let mut coord_refs = Vec::new();
    let mut effect_refs = Vec::new();

    // Import coordinate if present
    if let Some(coord) = &data.coordinate {
        let coord_entity = world.spawn();
        world.attach(coord_entity, JointCoordinate {
            range_min: coord.range_min,
            range_max: coord.range_max,
            default_value: coord.default_value,
            stiffness: coord.stiffness,
            damping: coord.damping,
            clamped: coord.clamped,
            locked: coord.locked,
            prescribed_function: None,
        });
        world.attach(coord_entity, Name { value: coord.name.clone() });
        coord_refs.push(coord_entity);
    }

    // For ball joints, create 3 rotation effects (even if 1 coord drives all 3)
    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let coord_for_effects = coord_refs.first().copied();
    for axis in &axes {
        if let Some(coord_id) = coord_for_effects {
            let effect_entity = world.spawn();
            world.attach(effect_entity, CoordinateEffect {
                coordinate: coord_id,
                joint: joint_entity,
                component: TransformComponent::RotationAboutAxis(*axis),
                function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
            });
            effect_refs.push(effect_entity);
        }
    }

    if !effect_refs.is_empty() {
        let st_entity = world.spawn();
        world.attach(st_entity, SpatialTransform {
            joint: joint_entity,
            effects: effect_refs,
        });
    }

    let limits = data.coordinate.as_ref().map(|c| JointLimits {
        lower: c.range_min,
        upper: c.range_max,
    });

    world.attach(joint_entity, Joint {
        body_a: parent_key,
        body_b: child_key,
        limits,
        joint_type: "BallJoint",
        coordinates: coord_refs,
    });

    update_child_frame(world, child_key, data);
    Ok(joint_entity)
}

fn import_free_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: EntityID,
    child_key: EntityID,
) -> Result<EntityID, String> {
    let joint_entity = world.spawn();
    world.attach(joint_entity, Joint {
        body_a: parent_key,
        body_b: child_key,
        limits: None,
        joint_type: "FreeJoint",
        coordinates: vec![],
    });

    update_child_frame(world, child_key, data);
    Ok(joint_entity)
}

fn import_universal_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: EntityID,
    child_key: EntityID,
) -> Result<EntityID, String> {
    let coords = data
        .coordinates
        .as_ref()
        .ok_or_else(|| format!("UniversalJoint '{}' missing coordinates", data.name))?;

    if coords.len() < 2 {
        return Err(format!(
            "UniversalJoint '{}' needs 2 coordinates, got {}",
            data.name,
            coords.len()
        ));
    }

    let joint_entity = world.spawn();
    let mut coord_refs = Vec::new();
    let mut effect_refs = Vec::new();

    // Default axes: axis1 around X, axis2 around Y if not specified by effects
    let axes = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]];

    // Import coordinates
    for coord in coords {
        let coord_entity = world.spawn();
        world.attach(coord_entity, JointCoordinate {
            range_min: coord.range_min,
            range_max: coord.range_max,
            default_value: coord.default_value,
            stiffness: coord.stiffness,
            damping: coord.damping,
            clamped: coord.clamped,
            locked: coord.locked,
            prescribed_function: None,
        });
        world.attach(coord_entity, Name { value: coord.name.clone() });
        coord_refs.push(coord_entity);
    }

    // Create RotationAboutAxis effects for each coordinate
    for (i, coord_id) in coord_refs.iter().enumerate() {
        let axis = axes[i % axes.len()];
        let effect_entity = world.spawn();
        world.attach(effect_entity, CoordinateEffect {
            coordinate: *coord_id,
            joint: joint_entity,
            component: TransformComponent::RotationAboutAxis(axis),
            function: JointFunction::Linear { slope: 1.0, intercept: 0.0 },
        });
        effect_refs.push(effect_entity);
    }

    let st_entity = world.spawn();
    world.attach(st_entity, SpatialTransform {
        joint: joint_entity,
        effects: effect_refs,
    });

    world.attach(joint_entity, Joint {
        body_a: parent_key,
        body_b: child_key,
        limits: None,
        joint_type: "UniversalJoint",
        coordinates: coord_refs,
    });

    update_child_frame(world, child_key, data);
    Ok(joint_entity)
}

fn import_custom_joint(
    world: &mut World,
    data: &OpenSimJointData,
    parent_key: EntityID,
    child_key: EntityID,
) -> Result<EntityID, String> {
    let coords = data
        .coordinates
        .as_ref()
        .ok_or_else(|| format!("CustomJoint '{}' missing coordinates", data.name))?;

    let st = data
        .spatial_transform
        .as_ref()
        .ok_or_else(|| format!("CustomJoint '{}' missing spatial_transform", data.name))?;

    // Phase 1: Import all coordinates
    let mut coord_ids: HashMap<String, EntityID> = HashMap::new();
    for coord in coords {
        let coord_entity = world.spawn();
        world.attach(coord_entity, JointCoordinate {
            range_min: coord.range_min,
            range_max: coord.range_max,
            default_value: coord.default_value,
            stiffness: coord.stiffness,
            damping: coord.damping,
            clamped: coord.clamped,
            locked: coord.locked,
            prescribed_function: coord.prescribed_function.as_ref().map(|c| {
                JointFunction::Polynomial {
                    coefficients: c.clone(),
                }
            }),
        });
        world.attach(coord_entity, Name { value: coord.name.clone() });
        coord_ids.insert(coord.name.clone(), coord_entity);
    }

    // Phase 2: Create the joint
    let coord_refs: Vec<EntityID> = coords
        .iter()
        .map(|c| coord_ids[&c.name])
        .collect();

    let joint_entity = world.spawn();
    world.attach(joint_entity, Joint {
        body_a: parent_key,
        body_b: child_key,
        limits: None,
        joint_type: "CustomJoint",
        coordinates: coord_refs,
    });

    // Phase 3: Import CoordinateEffects and SpatialTransform
    let mut effect_ids: Vec<EntityID> = Vec::new();

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
                format!(
                    "CustomJoint '{}': effect '{}' references unknown coordinate '{}'",
                    data.name, slot_name, effect.coordinate_name
                )
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

            let effect_entity = world.spawn();
            world.attach(effect_entity, CoordinateEffect {
                coordinate: *coord_id,
                joint: joint_entity,
                component,
                function,
            });
            effect_ids.push(effect_entity);
        }
    }

    // Phase 4: SpatialTransform groups the effects
    let st_entity = world.spawn();
    world.attach(st_entity, SpatialTransform {
        joint: joint_entity,
        effects: effect_ids,
    });

    update_child_frame(world, child_key, data);
    Ok(joint_entity)
}

// ── Frame helper ──────────────────────────────────────

/// Update a body's Frame transform with the joint's parent-frame offset.
/// In OpenSim, the joint's location_in_parent / orientation_in_parent defines
/// where the child body attaches relative to the parent body's frame.
/// For now we store this in the child's Frame component.
fn update_child_frame(_world: &mut World, _child_key: EntityID, _data: &OpenSimJointData) {
    // TODO: Compose location_in_parent + orientation_in_parent into the
    // child body's Frame transform. This requires computing a Transform from
    // the Euler angles and translation, which needs quaternion math.
    //
    // For now, frames use default transforms. The joint parent/child offsets
    // are captured by the joint's body_a/body_b fields. Adding full frame
    // transform computation is the next step after basic import validation.
}

// ── JSON loading helper ───────────────────────────────

/// Load OpenSim model data from a JSON file.
pub fn load_opensim_json(path: &str) -> Result<OpenSimModelData, String> {
    let content = std::fs::read_to_string(path)
        .map_err(|e| format!("Failed to read '{}': {}", path, e))?;
    serde_json::from_str(&content)
        .map_err(|e| format!("Failed to parse '{}': {}", path, e))
}
