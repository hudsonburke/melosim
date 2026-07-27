use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;

use melosim::components::*;
use melosim::id::EntityKey;
use melosim::importer::opensim::{
    import_opensim_body, import_opensim_joint, import_opensim_marker, OpenSimBodyData,
    OpenSimCoordinateData, OpenSimEffectData, OpenSimJointData, OpenSimMarkerData, OpenSimSpatialTransformData,
};
use melosim::world::World;

/// Extract an OpenSim Body from a Python PyObject and populate the World.
fn extract_body(
    world: &mut World,
    body: &Bound<'_, PyAny>,
) -> PyResult<(String, EntityKey)> {
    let name = body.call_method0("getName")?.extract::<String>()?;
    let mass = body.call_method0("getMass")?.extract::<f64>()?;

    let mass_center = body.call_method0("getMassCenter")?;
    let mc_x = mass_center.get_item(0)?.extract::<f64>()?;
    let mc_y = mass_center.get_item(1)?.extract::<f64>()?;
    let mc_z = mass_center.get_item(2)?.extract::<f64>()?;

    let inertia = body.call_method0("getInertia")?;
    let inertia_vec = [
        inertia.get_item(0)?.extract::<f64>()?,
        inertia.get_item(1)?.extract::<f64>()?,
        inertia.get_item(2)?.extract::<f64>()?,
        inertia.get_item(3)?.extract::<f64>()?,
        inertia.get_item(4)?.extract::<f64>()?,
        inertia.get_item(5)?.extract::<f64>()?,
    ];

    let data = OpenSimBodyData {
        name: name.clone(),
        mass,
        mass_center: [mc_x, mc_y, mc_z],
        inertia: inertia_vec,
    };

    let key = import_opensim_body(world, &data)
        .map_err(|e| PyRuntimeError::new_err(e))?;

    Ok((name, key))
}

/// Extract a coordinate from an OpenSim Coordinate.
fn extract_coordinate(coord: &Bound<'_, PyAny>) -> PyResult<OpenSimCoordinateData> {
    let name = coord.call_method0("getName")?.extract::<String>()?;
    let range_min = coord.call_method0("getRangeMin")?.extract::<f64>()?;
    let range_max = coord.call_method0("getRangeMax")?.extract::<f64>()?;
    let default_value = coord.call_method0("getDefaultValue")?.extract::<f64>()?;
    let stiffness = coord.call_method0("getStiffness")?.extract::<f64>()?;
    let damping = coord.call_method0("getDamping")?.extract::<f64>()?;
    let clamped = coord.call_method0("getClamped")?.extract::<bool>()?;
    let locked = coord.call_method0("getLocked")?.extract::<bool>()?;

    // Try to get prescribed function coefficients
    let prescribed_function = try_extract_prescribed_function(coord)?;

    Ok(OpenSimCoordinateData {
        name,
        range_min,
        range_max,
        default_value,
        stiffness,
        damping,
        clamped,
        locked,
        prescribed_function,
    })
}

/// Extract prescribed function coefficients from a coordinate (if any).
fn try_extract_prescribed_function(
    coord: &Bound<'_, PyAny>,
) -> PyResult<Option<Vec<f64>>> {
    let result = coord.call_method0("getPrescribedFunction");
    if result.is_err() {
        return Ok(None);
    }
    let pf = result?;
    let class_name = pf.call_method0("getConcreteClassName")?;
    let class_name_str = class_name.extract::<String>()?;
    if class_name_str == "NullFunction" {
        return Ok(None);
    }
    if class_name_str == "PolynomialFunction" {
        let size = pf
            .call_method0("getCoefficientSize")?
            .extract::<usize>()?;
        let mut coeffs = Vec::with_capacity(size);
        for i in 0..size {
            let c = pf.call_method1("getCoefficient", (i,))?.extract::<f64>()?;
            coeffs.push(c);
        }
        return Ok(Some(coeffs));
    }
    Ok(None)
}

/// Extract joint frame transforms from an OpenSim joint.
fn vec3_from_py(obj: &Bound<'_, PyAny>) -> PyResult<[f64; 3]> {
    Ok([
        obj.get_item(0)?.extract::<f64>()?,
        obj.get_item(1)?.extract::<f64>()?,
        obj.get_item(2)?.extract::<f64>()?,
    ])
}

/// Extract a joint from an OpenSim Joint PyObject.
fn extract_joint(
    joint: &Bound<'_, PyAny>,
    _body_map: &std::collections::HashMap<String, EntityKey>,
) -> PyResult<OpenSimJointData> {
    let name = joint.call_method0("getName")?.extract::<String>()?;
    let joint_type = joint
        .call_method0("getConcreteClassName")?
        .extract::<String>()?;

    let parent_frame = joint.call_method0("getParentFrame")?;
    let child_frame = joint.call_method0("getChildFrame")?;

    // The parent/child frame names might differ from body names.
    // In OpenSim, the parent frame is usually named after the parent body.
    let parent_name = parent_frame.call_method0("getName")?.extract::<String>()?;
    let child_name = child_frame.call_method0("getName")?.extract::<String>()?;

    let location_in_parent = vec3_from_py(&joint.call_method0("getLocationInParent")?)?;
    let orientation_in_parent =
        vec3_from_py(&joint.call_method0("getOrientationInParent")?)?;
    let location_in_child = vec3_from_py(&joint.call_method0("getLocationInChild")?)?;
    let orientation_in_child =
        vec3_from_py(&joint.call_method0("getOrientationInChild")?)?;

    let mut data = OpenSimJointData {
        name,
        joint_type,
        parent_body: parent_name,
        child_body: child_name,
        location_in_parent,
        orientation_in_parent,
        location_in_child,
        orientation_in_child,
        axis: None,
        coordinate: None,
        coordinates: None,
        spatial_transform: None,
    };

    // Type-specific extraction
    let coord_set = joint.call_method0("getCoordinateSet")?;
    let num_coords = coord_set.call_method0("getSize")?.extract::<usize>()?;

    match data.joint_type.as_str() {
        "PinJoint" => {
            if num_coords > 0 {
                let coord = coord_set.call_method1("get", (0,))?;
                let axis = coord.call_method0("getAxis")?;
                data.axis = Some(vec3_from_py(&axis)?);
                data.coordinate = Some(extract_coordinate(&coord)?);
            }
        }
        "CustomJoint" => {
            let mut coords = Vec::with_capacity(num_coords);
            for i in 0..num_coords {
                let coord = coord_set.call_method1("get", (i,))?;
                coords.push(extract_coordinate(&coord)?);
            }
            data.coordinates = Some(coords);
            data.spatial_transform = Some(extract_spatial_transform(joint)?);
        }
        "UniversalJoint" => {
            let mut coords = Vec::with_capacity(num_coords);
            for i in 0..num_coords {
                let coord = coord_set.call_method1("get", (i,))?;
                coords.push(extract_coordinate(&coord)?);
            }
            data.coordinates = Some(coords);
        }
        "BallJoint" => {
            if num_coords > 0 {
                let coord = coord_set.call_method1("get", (0,))?;
                data.coordinate = Some(extract_coordinate(&coord)?);
            }
        }
        "FreeJoint" | "WeldJoint" => {
            // No coordinates
        }
        _ => {
            // Unknown type — try to extract coordinates anyway
            if num_coords > 0 {
                let coord = coord_set.call_method1("get", (0,))?;
                data.coordinate = Some(extract_coordinate(&coord)?);
            }
        }
    }

    Ok(data)
}

/// Extract a SpatialTransform from a CustomJoint.
fn extract_spatial_transform(
    joint: &Bound<'_, PyAny>,
) -> PyResult<OpenSimSpatialTransformData> {
    // The OpenSim Python API exposes SpatialTransform.get_rotation_x() etc.
    // via property-like access. The exact method names depend on the bindings.
    // Try property access pattern first, fall back to component index access.
    let empty = OpenSimSpatialTransformData {
        rotation_x: None,
        rotation_y: None,
        rotation_z: None,
        translation_x: None,
        translation_y: None,
        translation_z: None,
    };

    let st = match joint.call_method0("getSpatialTransform") {
        Ok(st) => st,
        Err(_) => return Ok(empty),
    };

    // Try OpenSim 4.x+ property access pattern
    let transform_names = [
        "rotation_x",
        "rotation_y",
        "rotation_z",
        "translation_x",
        "translation_y",
        "translation_z",
    ];

    let mut result = OpenSimSpatialTransformData {
        rotation_x: None,
        rotation_y: None,
        rotation_z: None,
        translation_x: None,
        translation_y: None,
        translation_z: None,
    };

    for (idx, name) in transform_names.iter().enumerate() {
        // Try getPropertyByName first
        let component = match st.call_method1("getPropertyByName", (name,)) {
            Ok(c) => c,
            Err(_) => {
                // Fallback: try index-based access
                if let Ok(c) = st.call_method1("getComponent", (idx,)) {
                    c
                } else {
                    continue;
                }
            }
        };

        if let Ok(effect) = extract_effect(&component) {
            match idx {
                0 => result.rotation_x = effect,
                1 => result.rotation_y = effect,
                2 => result.rotation_z = effect,
                3 => result.translation_x = effect,
                4 => result.translation_y = effect,
                5 => result.translation_z = effect,
                _ => {}
            }
        }
    }

    Ok(result)
}

/// Extract a coordinate effect from a transform component.
fn extract_effect(
    component: &Bound<'_, PyAny>,
) -> PyResult<Option<OpenSimEffectData>> {
    let function = component.call_method0("getFunction")?;
    let func_type = function
        .call_method0("getConcreteClassName")?
        .extract::<String>()?;

    if func_type == "NullFunction" {
        return Ok(None);
    }

    let coordinate = component.call_method0("getCoordinate")?;
    let coord_name = coordinate.call_method0("getName")?.extract::<String>()?;

    let mut coefficients = Vec::new();
    match func_type.as_str() {
        "Constant" | "ConstantFunction" => {
            if let Ok(v) = function.call_method0("getValue") {
                coefficients.push(v.extract::<f64>()?);
            }
        }
        "LinearFunction" => {
            if let Ok(slope) = function.call_method0("getSlope") {
                coefficients.push(slope.extract::<f64>()?);
            }
            if let Ok(intercept) = function.call_method0("getIntercept") {
                coefficients.push(intercept.extract::<f64>()?);
            }
        }
        "PolynomialFunction" | _ => {
            if let Ok(size) = function.call_method0("getCoefficientSize") {
                let n = size.extract::<usize>()?;
                for i in 0..n {
                    if let Ok(c) = function.call_method1("getCoefficient", (i,)) {
                        coefficients.push(c.extract::<f64>()?);
                    }
                }
            }
        }
    }

    Ok(Some(OpenSimEffectData {
        coordinate_name: coord_name,
        function_type: func_type.replace("Function", ""),
        coefficients,
    }))
}

/// Extract a marker from an OpenSim Marker PyObject.
fn extract_marker(marker: &Bound<'_, PyAny>) -> PyResult<OpenSimMarkerData> {
    let name = marker.call_method0("getName")?.extract::<String>()?;
    let body = marker.call_method0("getBodyName")?.extract::<String>()?;
    let location = vec3_from_py(&marker.call_method0("getLocation")?)?;

    Ok(OpenSimMarkerData { name, body, location })
}

/// Import an OpenSim model from a .osim file path.
/// Returns a JSON summary of the imported model (component counts).
#[pyfunction]
fn import_osim(_py: Python<'_>, path: &str) -> PyResult<String> {
    // Load the model
    let opensim = _py
        .import("opensim")
        .map_err(|e| PyRuntimeError::new_err(format!("Failed to import opensim: {}", e)))?;

    let model = opensim
        .call_method1("Model", (path,))
        .map_err(|e| PyRuntimeError::new_err(format!("Failed to load model '{}': {}", path, e)))?;

    model
        .call_method0("initSystem")
        .map_err(|e| PyRuntimeError::new_err(format!("Failed to initSystem: {}", e)))?;

    let mut world = World::new();
    let mut body_map: std::collections::HashMap<String, EntityKey> =
        std::collections::HashMap::new();

    // Extract bodies
    let body_set = model.call_method0("getBodySet")?;
    let num_bodies = body_set.call_method0("getSize")?.extract::<usize>()?;

    for i in 0..num_bodies {
        let body = body_set.call_method1("get", (i,))?;
        let (name, key) = extract_body(&mut world, &body)?;
        body_map.insert(name, key);
    }

    // Add ground if not in the set (OpenSim's BodySet doesn't include ground by default)
    if !body_map.contains_key("ground") {
        let ground_data = OpenSimBodyData {
            name: "ground".to_string(),
            mass: 0.0,
            mass_center: [0.0, 0.0, 0.0],
            inertia: [0.0; 6],
        };
        let ground_key = import_opensim_body(&mut world, &ground_data)
            .map_err(|e| PyRuntimeError::new_err(e))?;
        body_map.insert("ground".to_string(), ground_key);
    }

    // Extract joints
    let joint_set = model.call_method0("getJointSet")?;
    let num_joints = joint_set.call_method0("getSize")?.extract::<usize>()?;

    for i in 0..num_joints {
        let joint = joint_set.call_method1("get", (i,))?;
        let joint_data = extract_joint(&joint, &body_map)?;

        let parent_key = body_map.get(&joint_data.parent_body).copied().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "Joint '{}': parent body '{}' not found",
                joint_data.name, joint_data.parent_body
            ))
        })?;
        let child_key = body_map.get(&joint_data.child_body).copied().ok_or_else(|| {
            PyRuntimeError::new_err(format!(
                "Joint '{}': child body '{}' not found",
                joint_data.name, joint_data.child_body
            ))
        })?;

        import_opensim_joint(&mut world, &joint_data, parent_key, child_key)
            .map_err(|e| PyRuntimeError::new_err(e))?;
    }

    // Extract markers
    let marker_set = model.call_method0("getMarkerSet")?;
    let num_markers = marker_set.call_method0("getSize")?.extract::<usize>()?;

    for i in 0..num_markers {
        let marker = marker_set.call_method1("get", (i,))?;
        let marker_data = extract_marker(&marker)?;
        if let Some(&body_key) = body_map.get(&marker_data.body) {
            import_opensim_marker(&mut world, &marker_data, body_key);
        } else {
            return Err(PyRuntimeError::new_err(format!(
                "Marker '{}': body '{}' not found",
                marker_data.name, marker_data.body
            )));
        }
    }

    // Return a JSON summary instead of serializing the full World
    // (World contains AnyMaps which don't implement Serialize)
    let summary = serde_json::json!({
        "status": "ok",
        "bodies": world.count::<InertialProperties>(),
        "frames": world.count::<Frame>(),
        "hinge_joints": world.count::<HingeJoint>(),
        "slide_joints": world.count::<SlideJoint>(),
        "ball_joints": world.count::<BallJoint>(),
        "free_joints": world.count::<FreeJoint>(),
        "fixed_joints": world.count::<FixedJoint>(),
        "universal_joints": world.count::<UniversalJoint>(),
        "custom_joints": world.count::<CustomJoint>(),
        "coordinates": world.count::<JointCoordinate>(),
        "coordinate_effects": world.count::<CoordinateEffect>(),
        "spatial_transforms": world.count::<SpatialTransform>(),
        "sites": world.count::<Site>(),
        "landmarks": world.count::<Landmark>(),
        "muscle_params": world.count::<HillTypeMuscleParams>(),
    });

    Ok(summary.to_string())
}

/// Validate the World and return a JSON result.
/// Takes the same JSON summary format as import_osim returns.
/// For now this is a placeholder — real validation happens on the Rust side.
#[pyfunction]
fn validate_world(_py: Python<'_>, _path: &str) -> PyResult<String> {
    Ok(serde_json::json!({"status": "Placeholder — validation via Rust API" }).to_string())
}

/// Python module definition.
#[pymodule]
fn melosim_py(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_function(wrap_pyfunction!(import_osim, m)?)?;
    m.add_function(wrap_pyfunction!(validate_world, m)?)?;
    Ok(())
}
