//! OpenSim `.osim` import directly into the canonical Bevy model.
//!
//! This adapter parses with the official OpenSim Python API, converts the
//! format-specific objects into small local parsing records, then uses
//! `ModelBuilder` to create the live ECS hierarchy. It intentionally does not
//! introduce a second persistent model representation.

use std::collections::HashMap;
use std::path::Path;

use bevy::prelude::*;
use nalgebra::{Isometry3, Translation, UnitQuaternion, Vector3};
use pyo3::prelude::*;

use crate::model::{
    CoordinateProperties, CoordinateSpec, CouplingKind, Frame, Function, Inertia,
    InitialConditions, InertialProperties, ModelBuilder, Twist, to_bevy_transform,
};

use super::ImportError;

type Res<T> = Result<T, String>;

struct ParsedModel {
    name: String,
    bodies: Vec<ParsedBody>,
    joints: Vec<ParsedJoint>,
}

struct ParsedBody {
    name: String,
    inertial: InertialProperties,
}

struct ParsedJoint {
    name: String,
    parent_body: String,
    child_body: String,
    parent_offset: Isometry3<f64>,
    child_offset: Isometry3<f64>,
    coordinates: Vec<ParsedCoordinate>,
    axes: Vec<ParsedAxis>,
}

#[derive(Clone)]
struct ParsedCoordinate {
    name: String,
    initial: InitialConditions,
    properties: CoordinateProperties,
}

struct ParsedAxis {
    twist: Twist,
    coordinate: Option<String>,
    function: Option<Function>,
}

/// Parse an OpenSim model and spawn it into the canonical Bevy ECS model.
pub fn import_osim(world: &mut World, path: &Path) -> Result<Entity, ImportError> {
    let parsed = parse_osim(path).map_err(ImportError::Load)?;
    let anchor = world
        .spawn((
            Name::new(parsed.name.clone()),
            Frame,
            Transform::IDENTITY,
        ))
        .id();

    let mut builder = ModelBuilder::new(world);
    let ground = builder.body(
        "ground",
        InertialProperties {
            mass: 0.0,
            mass_center: Vector3::zeros(),
            inertia: Inertia::default(),
        },
        Transform::IDENTITY,
        Some(anchor),
    );

    let mut bodies = HashMap::new();
    bodies.insert("ground".to_string(), ground);
    for body in parsed.bodies {
        let entity = builder.body(
            body.name.clone(),
            body.inertial,
            Transform::IDENTITY,
            None,
        );
        bodies.insert(body.name, entity);
    }

    // Source coordinates are created before joints so cross-joint axis
    // references can resolve regardless of joint order in the OpenSim file.
    let mut coordinates = HashMap::new();
    let mut coordinate_specs = HashMap::new();
    for joint in &parsed.joints {
        for coordinate in &joint.coordinates {
            if coordinates.contains_key(&coordinate.name) {
                continue;
            }
            let entity = builder.coordinate(CoordinateSpec {
                name: coordinate.name.clone(),
                properties: coordinate.properties.clone(),
                initial: coordinate.initial.clone(),
                twist: None,
                driven_by: None,
            });
            coordinates.insert(coordinate.name.clone(), entity);
            coordinate_specs.insert(coordinate.name.clone(), coordinate.clone());
        }
    }

    let mut coordinate_owners: HashMap<Entity, Entity> = HashMap::new();
    for joint in &parsed.joints {
        let Some(&parent) = bodies.get(&joint.parent_body) else {
            return Err(ImportError::Load(format!(
                "joint '{}': unknown parent body '{}'",
                joint.name, joint.parent_body
            )));
        };
        let Some(&child) = bodies.get(&joint.child_body) else {
            return Err(ImportError::Load(format!(
                "joint '{}': unknown child body '{}'",
                joint.name, joint.child_body
            )));
        };

        let connection = builder
            .joint_connection(
                parent,
                format!("{}_frame", joint.name),
                to_bevy_transform(&joint.parent_offset),
                joint.name.clone(),
                child,
                to_bevy_transform(&joint.child_offset),
                std::iter::empty::<CoordinateSpec>(),
            )
            .map_err(|e| ImportError::Load(format!("joint '{}': {e}", joint.name)))?;

        for coordinate in &joint.coordinates {
            let Some(&source) = coordinates.get(&coordinate.name) else {
                return Err(ImportError::Load(format!(
                    "joint '{}': missing coordinate '{}'",
                    joint.name, coordinate.name
                )));
            };
            if !coordinate_owners.contains_key(&source) {
                builder
                    .add_coordinate_to_joint(connection.joint, source)
                    .map_err(|e| ImportError::Load(format!("coordinate '{}': {e}", coordinate.name)))?;
                coordinate_owners.insert(source, connection.joint);
            }
        }

        for (axis_index, axis) in joint.axes.iter().enumerate() {
            let Some(source_name) = axis.coordinate.as_deref() else {
                // Constant transform axes require a fixed frame/pose expression;
                // zero constants were already removed during parsing.
                warn!(
                    "OpenSim joint '{}': constant transform axis {} is not imported yet",
                    joint.name, axis_index
                );
                continue;
            };
            let Some(&source) = coordinates.get(source_name) else {
                return Err(ImportError::Load(format!(
                    "joint '{}': axis references unknown coordinate '{}'",
                    joint.name, source_name
                )));
            };
            let source_data = coordinate_specs.get(source_name).ok_or_else(|| {
                ImportError::Load(format!("missing parsed data for coordinate '{source_name}'"))
            })?;
            let effect_name = format!("{}_axis_{axis_index}", joint.name);
            let effect = builder.coordinate(CoordinateSpec {
                name: effect_name,
                properties: source_data.properties.clone(),
                initial: source_data.initial.clone(),
                twist: Some(axis.twist.clone()),
                driven_by: Some((
                    source,
                    CouplingKind::Equality(axis.function.clone().unwrap_or_else(Function::identity)),
                )),
            });
            builder
                .add_coordinate_to_joint(connection.joint, effect)
                .map_err(|e| ImportError::Load(format!("joint '{}': {e}", joint.name)))?;
        }
    }

    for issue in crate::model::validate_kinematic_hierarchy(builder.world_mut()) {
        warn!("OpenSim import: invalid kinematic hierarchy: {issue}");
    }

    Ok(anchor)
}

fn parse_osim(path: &Path) -> Res<ParsedModel> {
    Python::attach(|py| {
        let osim = py.import("opensim").map_err(|e| {
            format!("import opensim failed (check PYTHONPATH/LD_LIBRARY_PATH): {e}")
        })?;
        let model = osim
            .getattr("Model")
            .and_then(|m| m.call1((path.to_string_lossy().as_ref(),)))
            .map_err(|e| format!("failed to load {}: {e}", path.display()))?;

        let body_set = call0(&model, "getBodySet")?;
        let body_count: usize = call0(&body_set, "getSize")?.extract().map_err(err)?;
        let mut bodies = Vec::with_capacity(body_count);
        for i in 0..body_count {
            bodies.push(extract_body(&body_set, i)?);
        }

        let joint_set = call0(&model, "getJointSet")?;
        let joint_count: usize = call0(&joint_set, "getSize")?.extract().map_err(err)?;
        let mut joints = Vec::with_capacity(joint_count);
        for i in 0..joint_count {
            joints.push(extract_joint(&osim, &joint_set, i)?);
        }

        Ok(ParsedModel {
            name: get_str(&model, "getName")?,
            bodies,
            joints,
        })
    })
}

fn extract_body(body_set: &Bound<'_, PyAny>, index: usize) -> Res<ParsedBody> {
    let body = call1(body_set, "get", index)?;
    let inertia = call0(&body, "getInertia")?;
    let moments = vec3(&call0(&inertia, "getMoments")?)?;
    let products = vec3(&call0(&inertia, "getProducts")?)?;
    Ok(ParsedBody {
        name: get_str(&body, "getName")?,
        inertial: InertialProperties {
            mass: f64_or(&body, "getMass", 0.0),
            mass_center: vec3(&call0(&body, "getMassCenter")?)?,
            inertia: Inertia::new(
                moments[0], moments[1], moments[2], products[0], products[1], products[2],
            ),
        },
    })
}

fn extract_joint(
    osim: &Bound<'_, PyModule>,
    joint_set: &Bound<'_, PyAny>,
    index: usize,
) -> Res<ParsedJoint> {
    let joint = call1(joint_set, "get", index)?;
    let name = get_str(&joint, "getName")?;
    let class = get_str(&joint, "getConcreteClassName")?;
    let (parent_body, parent_offset) = resolve_frame(osim, &call0(&joint, "getParentFrame")?)?;
    let (child_body, socket_offset) = resolve_frame(osim, &call0(&joint, "getChildFrame")?)?;
    let child_offset = socket_offset.inverse();

    let count: usize = call0(&joint, "numCoordinates")?.extract().map_err(err)?;
    let mut coordinates = Vec::with_capacity(count);
    for i in 0..count {
        let coordinate = call1(&joint, "get_coordinates", i)?;
        coordinates.push(ParsedCoordinate {
            name: get_str(&coordinate, "getName")?,
            initial: InitialConditions {
                value: f64_or(&coordinate, "getDefaultValue", 0.0),
                velocity: 0.0,
            },
            properties: CoordinateProperties {
                range: (
                    f64_or(&coordinate, "getRangeMin", -std::f64::consts::PI),
                    f64_or(&coordinate, "getRangeMax", std::f64::consts::PI),
                ),
                clamped: bool_or(&coordinate, "get_clamped", false),
                locked: bool_or(&coordinate, "get_locked", false),
                stiffness: f64_or(&coordinate, "getStiffness", 0.0),
                damping: f64_or(&coordinate, "getDamping", 0.0),
            },
        });
    }

    let axes = if class == "CustomJoint" {
        extract_transform_axes(osim, &joint)?
    } else {
        builtin_axes(&name, &class, &coordinates)
    };

    Ok(ParsedJoint {
        name,
        parent_body,
        child_body,
        parent_offset,
        child_offset,
        coordinates,
        axes,
    })
}

fn resolve_frame(osim: &Bound<'_, PyModule>, frame: &Bound<'_, PyAny>) -> Res<(String, Isometry3<f64>)> {
    let mut current = frame.clone();
    let mut composed = Isometry3::identity();
    loop {
        let Some(offset_frame) = downcast(osim, "PhysicalOffsetFrame", &current)? else {
            return Ok((get_str(&current, "getName")?, composed));
        };
        let translation = vec3(&call0(&offset_frame, "get_translation")?)?;
        let orientation = vec3(&call0(&offset_frame, "get_orientation")?)?;
        composed = Isometry3::from_parts(Translation::from(translation), euler_xyz(orientation)) * composed;
        current = call0(&offset_frame, "getParentFrame")?;
    }
}

fn extract_transform_axes(osim: &Bound<'_, PyModule>, joint: &Bound<'_, PyAny>) -> Res<Vec<ParsedAxis>> {
    let Some(custom_joint) = downcast(osim, "CustomJoint", joint)? else {
        return Ok(vec![]);
    };
    let spatial = call0(&custom_joint, "getSpatialTransform")?;
    let getters = [
        ("get_translation1", false),
        ("get_translation2", false),
        ("get_translation3", false),
        ("get_rotation1", true),
        ("get_rotation2", true),
        ("get_rotation3", true),
    ];
    let mut axes = Vec::new();
    for (getter, rotation) in getters {
        let transform_axis = call0(&spatial, getter)?;
        let axis = vec3(&call0(&transform_axis, "get_axis")?)?;
        if axis.norm() < 1e-12 {
            continue;
        }
        let function = extract_function(osim, &call0(&transform_axis, "get_function")?)?;
        let names = call0(&transform_axis, "getCoordinateNames")?;
        let count: usize = call0(&names, "size")?.extract().map_err(err)?;
        let coordinate = if count == 0 {
            None
        } else {
            Some(call1(&names, "getValue", 0usize)?.extract().map_err(err)?)
        };
        axes.push(ParsedAxis {
            twist: if rotation {
                Twist::rotation(axis)
            } else {
                Twist::translation(axis)
            },
            coordinate,
            function,
        });
    }
    Ok(axes)
}

fn builtin_axes(name: &str, class: &str, coordinates: &[ParsedCoordinate]) -> Vec<ParsedAxis> {
    let axis = |index: usize, angular: Vector3<f64>, linear: Vector3<f64>| ParsedAxis {
        twist: Twist { angular, linear },
        coordinate: Some(coordinates[index].name.clone()),
        function: None,
    };
    let (x, y, z, zero) = (Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros());
    match (class, coordinates.len()) {
        ("WeldJoint", _) => vec![],
        ("PinJoint", 1) => vec![axis(0, z, zero)],
        ("SliderJoint", 1) => vec![axis(0, zero, x)],
        ("UniversalJoint", 2) => vec![axis(0, x, zero), axis(1, y, zero)],
        ("BallJoint", 3) => vec![axis(0, x, zero), axis(1, y, zero), axis(2, z, zero)],
        ("FreeJoint", 6) => vec![
            axis(3, zero, x), axis(4, zero, y), axis(5, zero, z),
            axis(0, x, zero), axis(1, y, zero), axis(2, z, zero),
        ],
        (other, _) => {
            warn!("joint '{name}': unsupported OpenSim type '{other}'");
            vec![]
        }
    }
}

fn extract_function(osim: &Bound<'_, PyModule>, function: &Bound<'_, PyAny>) -> Res<Option<Function>> {
    let class = get_str(function, "getConcreteClassName")?;
    match class.as_str() {
        "NullFunction" => Ok(None),
        "LinearFunction" => {
            let Some(linear) = downcast(osim, "LinearFunction", function)? else { return Ok(None) };
            Ok(Some(Function::Linear {
                slope: f64_or(&linear, "getSlope", 1.0),
                intercept: f64_or(&linear, "getIntercept", 0.0),
            }))
        }
        "Constant" => {
            let Some(constant) = downcast(osim, "Constant", function)? else { return Ok(None) };
            Ok(Some(Function::Constant(f64_or(&constant, "getValue", 0.0))))
        }
        "PolynomialFunction" => {
            let Some(polynomial) = downcast(osim, "PolynomialFunction", function)? else { return Ok(None) };
            Ok(Some(Function::Polynomial(double_vector(&call0(&polynomial, "getCoefficients")?)?)))
        }
        "SimmSpline" => {
            let Some(spline) = downcast(osim, "SimmSpline", function)? else { return Ok(None) };
            let count: usize = call0(&spline, "getNumberOfPoints")?.extract().map_err(err)?;
            let mut x = Vec::with_capacity(count);
            let mut y = Vec::with_capacity(count);
            for index in 0..count {
                x.push(call1(&spline, "getX", index)?.extract::<f64>().map_err(err)?);
                y.push(call1(&spline, "getY", index)?.extract::<f64>().map_err(err)?);
            }
            Ok(Some(Function::CubicSpline { x, y }))
        }
        other => {
            warn!("unsupported OpenSim function '{other}', using identity");
            Ok(None)
        }
    }
}

fn err<E: ToString>(error: E) -> String { error.to_string() }

fn call0<'py>(object: &Bound<'py, PyAny>, method: &str) -> Res<Bound<'py, PyAny>> {
    object.call_method0(method).map_err(|e| format!("{method}: {e}"))
}

fn call1<'py, A: IntoPyObject<'py>>(
    object: &Bound<'py, PyAny>,
    method: &str,
    argument: A,
) -> Res<Bound<'py, PyAny>> {
    object.call_method1(method, (argument,)).map_err(|e| format!("{method}: {e}"))
}

fn get_str(object: &Bound<'_, PyAny>, method: &str) -> Res<String> {
    call0(object, method)?.extract().map_err(err)
}

fn f64_or(object: &Bound<'_, PyAny>, method: &str, default: f64) -> f64 {
    object.call_method0(method).and_then(|value| value.extract::<f64>()).unwrap_or(default)
}

fn bool_or(object: &Bound<'_, PyAny>, method: &str, default: bool) -> bool {
    object.call_method0(method).and_then(|value| value.extract::<bool>()).unwrap_or(default)
}

fn vec3(value: &Bound<'_, PyAny>) -> Res<Vector3<f64>> {
    let mut result = Vector3::zeros();
    for index in 0..3 {
        result[index] = value.get_item(index).and_then(|item| item.extract::<f64>()).map_err(err)?;
    }
    Ok(result)
}

fn double_vector(value: &Bound<'_, PyAny>) -> Res<Vec<f64>> {
    let count: usize = call0(value, "__len__")?.extract().map_err(err)?;
    (0..count)
        .map(|index| call1(value, "__getitem__", index)?.extract::<f64>().map_err(err))
        .collect()
}

fn downcast<'py>(
    osim: &Bound<'py, PyModule>,
    class: &str,
    object: &Bound<'py, PyAny>,
) -> Res<Option<Bound<'py, PyAny>>> {
    let cls = osim.getattr(class).map_err(err)?;
    let result = cls.call_method1("safeDownCast", (object,)).map_err(err)?;
    Ok((!result.is_none()).then_some(result))
}

fn euler_xyz(value: Vector3<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), value.x)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), value.y)
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), value.z)
}
