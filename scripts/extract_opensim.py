#!/usr/bin/env python3
"""
Extract OpenSim model data to JSON for melosim Rust importer.

Usage:
    python scripts/extract_opensim.py Rajagopal2015.osim [output.json]
    qemu-x86_64 python3 scripts/extract_opensim.py model.osim model.json   (on aarch64)

Produces a JSON file matching the OpenSimModelData struct in
melosim::importer::opensim. The Rust importer reads this JSON
and populates an ECS World.

Architecture:
    OpenSim host ──JSON──→ Rust (any platform)
"""

import json
import sys
from pathlib import Path


def vec3(v):
    """Convert OpenSim Vec3 to Python list using subscript access."""
    return [v[i] for i in range(3)]


def get_frame_transform(frame):
    """Get translation and orientation from a PhysicalOffsetFrame using transform."""
    try:
        t = frame.findTransformInBaseFrame()
        trans = [t.p()[i] for i in range(3)]
        rot = [t.R().convertRotationToBodyFixedXYZ()[i] for i in range(3)]
        return trans, rot
    except Exception:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]


def get_f64(obj, method, default=0.0):
    try:
        return getattr(obj, method)()
    except Exception:
        return default


def get_bool(obj, method, default=False):
    try:
        return bool(getattr(obj, method)())
    except Exception:
        return default


def extract_body(body):
    mc = body.getMassCenter()
    inertia = body.getInertia()
    try:
        moments = [inertia.getMoments()[i] for i in range(3)]
        products = [inertia.getProducts()[i] for i in range(3)]
    except Exception:
        try:
            moments = [mc[i] for i in range(3)]
            products = [0.0, 0.0, 0.0]
        except Exception:
            moments, products = [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    return {
        "name": body.getName(),
        "mass": get_f64(body, "getMass"),
        "mass_center": [mc[i] for i in range(3)],
        "inertia": moments + products,
    }


def extract_coordinate(coord):
    prescribed = None
    try:
        pf = coord.getPrescribedFunction()
        if pf and pf.getConcreteClassName() != "NullFunction":
            func_type = pf.getConcreteClassName()
            coeffs = []
            if func_type == "PolynomialFunction":
                for i in range(pf.getCoefficientSize()):
                    coeffs.append(pf.getCoefficient(i))
            prescribed = {"function_type": func_type, "coefficients": coeffs}
    except Exception:
        pass

    return {
        "name": coord.getName(),
        "range_min": get_f64(coord, "getRangeMin", -1.0),
        "range_max": get_f64(coord, "getRangeMax", 1.0),
        "default_value": get_f64(coord, "getDefaultValue"),
        "stiffness": 0.0,
        "damping": 0.0,
        "clamped": get_bool(coord, "get_clamped"),
        "locked": get_bool(coord, "get_locked"),
        "prescribed_function": prescribed,
    }


def extract_effect(effect):
    try:
        function = effect.getFunction()
        func_type = function.getConcreteClassName()
    except Exception:
        return None
    coeffs = []
    try:
        if func_type == "PolynomialFunction":
            for i in range(function.getCoefficientSize()):
                coeffs.append(function.getCoefficient(i))
        elif func_type == "LinearFunction":
            coeffs = [function.getSlope(), function.getIntercept()]
        elif func_type == "Constant":
            coeffs = [function.getValue()]
        elif func_type == "NullFunction":
            return None
    except Exception:
        pass

    try:
        coord_name = effect.getCoordinate().getName()
    except Exception:
        coord_name = "unknown"

    return {
        "coordinate_name": coord_name,
        "function_type": func_type.replace("Function", ""),
        "coefficients": coeffs,
    }


def extract_spatial_transform(joint):
    """Extract SpatialTransform from a CustomJoint.
    Available via property system in Python bindings.
    """
    if joint.getConcreteClassName() != "CustomJoint":
        return None
    names = ["rotation_x", "rotation_y", "rotation_z",
             "translation_x", "translation_y", "translation_z"]
    result = {n: None for n in names}
    try:
        st_prop = joint.getPropertyByName("SpatialTransform")
        st_obj = st_prop.getValueAsObject(0)
        # Try to get components from the SpatialTransform object
        for idx, name in enumerate(names):
            try:
                comp = st_obj.getComponent(idx)
                effect = extract_effect(comp)
                if effect:
                    result[name] = effect
            except Exception:
                pass
    except Exception:
        pass
    return result


def extract_joint(joint):
    joint_type = joint.getConcreteClassName()
    try:
        pf = joint.getParentFrame()
        cf = joint.getChildFrame()
        # PhysicalOffsetFrame names differ from body names — findBaseFrame() gives the actual body
        parent_name = pf.findBaseFrame().getName()
        child_name = cf.findBaseFrame().getName()
        loc_in_parent, ori_in_parent = get_frame_transform(pf)
        loc_in_child, ori_in_child = get_frame_transform(cf)
    except Exception:
        try:
            parent_name = pf.getName()
            child_name = cf.getName()
        except Exception:
            parent_name = "ground"
            child_name = "ground"

    data = {
        "name": joint.getName(),
        "joint_type": joint_type,
        "parent_body": parent_name,
        "child_body": child_name,
        "location_in_parent": loc_in_parent,
        "orientation_in_parent": ori_in_parent,
        "location_in_child": loc_in_child,
        "orientation_in_child": ori_in_child,
        "axis": None,
        "coordinate": None,
        "coordinates": None,
        "spatial_transform": None,
    }

    try:
        if joint_type == "PinJoint":
            coord = joint.getCoordinate()
            data["axis"] = [coord.getAxis()[i] for i in range(3)]
            data["coordinate"] = extract_coordinate(coord)
        elif joint_type == "CustomJoint":
            data["coordinates"] = [
                extract_coordinate(joint.get_coordinates(i))
                for i in range(joint.numCoordinates())
            ]
            data["spatial_transform"] = extract_spatial_transform(joint)
        elif joint_type == "UniversalJoint":
            data["coordinates"] = [
                extract_coordinate(joint.get_coordinates(i))
                for i in range(joint.numCoordinates())
            ]
        elif joint_type == "BallJoint":
            coord = joint.getCoordinate()
            data["coordinate"] = extract_coordinate(coord)
        elif joint_type in ("FreeJoint", "WeldJoint"):
            pass
        else:
            if joint.numCoordinates() > 0:
                coord = joint.getCoordinate()
                data["coordinate"] = extract_coordinate(coord)
    except Exception:
        pass

    return data


def extract_marker(marker):
    try:
        loc = [marker.getLocation()[i] for i in range(3)]
    except Exception:
        loc = [0.0, 0.0, 0.0]
    return {
        "name": marker.getName(),
        "body": marker.getBodyName(),
        "location": loc,
    }


def extract_muscle_path_points(force):
    points = []
    try:
        path = force.getGeometryPath()
        pset = path.getPathPointSet()
        for j in range(pset.getSize()):
            pp = pset.get(j)
            pp_type = pp.getConcreteClassName()
            body = pp.getBody().getName()
            loc = vec3(pp.getLocation())
            coordinate = None
            function = None
            if pp_type == "MovingPathPoint":
                try:
                    coordinate = pp.getCoordinate().getName()
                    func = pp.getFunction()
                    func_type = func.getConcreteClassName()
                    if "Polynomial" in func_type:
                        coeffs = [func.getCoefficient(k) for k in range(func.getCoefficientSize())]
                        function = coeffs
                except Exception:
                    pass
            points.append({
                "point_type": pp_type,
                "body": body,
                "location": loc,
                "coordinate": coordinate,
                "function": function,
            })
    except Exception:
        pass
    return points


def extract_muscle(force):
    path_points = extract_muscle_path_points(force)
    return {
        "name": force.getName(),
        "muscle_type": force.getConcreteClassName(),
        "max_isometric_force": get_f64(force, "getMaxIsometricForce"),
        "optimal_fiber_length": get_f64(force, "getOptimalFiberLength"),
        "tendon_slack_length": get_f64(force, "getTendonSlackLength"),
        "pennation_angle_at_optimal": get_f64(force, "getPennationAngleAtOptimalFiberLength"),
        "max_contraction_velocity": get_f64(force, "getMaxContractionVelocity", 10.0),
        "activation_time_constant": get_f64(force, "getActivationTimeConstant", 0.01),
        "deactivation_time_constant": get_f64(force, "getDeactivationTimeConstant", 0.04),
        "minimum_activation": get_f64(force, "getMinimumActivation", 0.01),
        "fiber_damping": get_f64(force, "getFiberDamping", 0.1),
        "ignore_activation_dynamics": get_bool(force, "getIgnoreActivationDynamics"),
        "ignore_tendon_compliance": get_bool(force, "getIgnoreTendonCompliance"),
        "path_points": path_points,
    }


def extract_wrap(wrap):
    name = wrap.getName()
    wrap_type = wrap.getConcreteClassName()
    try:
        body = wrap.getFrame().getName()
    except Exception:
        body = "ground"
    try:
        location = [wrap.getLocation()[i] for i in range(3)]
        orientation = [wrap.getOrientation()[i] for i in range(3)]
    except Exception:
        location = [0.0, 0.0, 0.0]
        orientation = [0.0, 0.0, 0.0]

    if "Sphere" in wrap_type:
        dimensions = [get_f64(wrap, "getRadius")]
    elif "Cylinder" in wrap_type:
        dimensions = [get_f64(wrap, "getRadius"), get_f64(wrap, "getLength")]
    elif "Ellipsoid" in wrap_type:
        try:
            dims = wrap.getDimensions()
            dimensions = [dims[i] for i in range(3)]
        except Exception:
            dimensions = [0.0, 0.0, 0.0]
    else:
        dimensions = []

    return {
        "name": name,
        "body": body,
        "wrap_type": wrap_type,
        "dimensions": dimensions,
        "location": location,
        "orientation": orientation,
    }


def extract_display_geometry(body):
    geoms = []
    try:
        for i in range(body.getPropertyByName("display_geometry").size()):
            dg_obj = body.getPropertyByName("display_geometry").getValueAsObject(i)
            geoms.append({
                "body_name": body.getName(),
                "mesh_file": dg_obj.getPropertyByName("display_geometry_file").getValueAsObject(0).getName(),
                "scale_factors": [1.0, 1.0, 1.0],
                "color": [0.8, 0.8, 0.8],
                "opacity": 1.0,
                "transform": None,
            })
    except Exception:
        pass
    return geoms


def extract_model(osim_path):
    import opensim as osim

    print(f"Loading {osim_path}...")
    model = osim.Model(osim_path)
    model.initSystem()

    data = {
        "name": model.getName(),
        "bodies": [],
        "joints": [],
        "markers": [],
        "muscles": [],
        "wrap_objects": [],
        "display_geometries": [],
    }

    # Bodies
    body_set = model.getBodySet()
    for i in range(body_set.getSize()):
        data["bodies"].append(extract_body(body_set.get(i)))

    has_ground = any(b["name"] == "ground" for b in data["bodies"])
    if not has_ground:
        data["bodies"].append({
            "name": "ground", "mass": 0.0,
            "mass_center": [0.0, 0.0, 0.0], "inertia": [0.0] * 6,
        })

    # Joints
    joint_set = model.getJointSet()
    for i in range(joint_set.getSize()):
        data["joints"].append(extract_joint(joint_set.get(i)))

    # Markers
    try:
        marker_set = model.getMarkerSet()
        for i in range(marker_set.getSize()):
            data["markers"].append(extract_marker(marker_set.get(i)))
    except Exception:
        pass

    # Muscles (from ForceSet)
    try:
        force_set = model.getForceSet()
        for i in range(force_set.getSize()):
            force = force_set.get(i)
            class_name = force.getConcreteClassName()
            if "Muscle" in class_name:
                data["muscles"].append(extract_muscle(force))
    except Exception:
        pass

    # Wrap objects
    try:
        wrap_set = model.getWrapObjectSet()
        for i in range(wrap_set.getSize()):
            data["wrap_objects"].append(extract_wrap(wrap_set.get(i)))
    except Exception:
        pass

    # Display geometry
    for i in range(body_set.getSize()):
        data["display_geometries"].extend(extract_display_geometry(body_set.get(i)))

    return data


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    osim_path = sys.argv[1]
    json_path = sys.argv[2] if len(sys.argv) > 2 else Path(osim_path).with_suffix(".json")
    data = extract_model(osim_path)

    print(f"  Bodies:       {len(data['bodies'])}")
    print(f"  Joints:       {len(data['joints'])}")
    print(f"  Markers:      {len(data['markers'])}")
    print(f"  Muscles:      {len(data['muscles'])}")
    print(f"  Wrap objects: {len(data['wrap_objects'])}")
    print(f"  Display geom: {len(data['display_geometries'])}")

    with open(json_path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"Wrote {json_path}")


if __name__ == "__main__":
    main()
