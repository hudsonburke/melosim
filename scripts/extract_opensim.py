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
    return [v.get(i) for i in range(3)]


def extract_body(body):
    mc = body.getMassCenter()
    inertia = body.getInertia()
    return {
        "name": body.getName(),
        "mass": body.getMass(),
        "mass_center": vec3(mc),
        "inertia": [inertia.get(i) for i in range(6)],
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
        "range_min": coord.getRangeMin(),
        "range_max": coord.getRangeMax(),
        "default_value": coord.getDefaultValue(),
        "stiffness": coord.getStiffness(),
        "damping": coord.getDamping(),
        "clamped": coord.getClamped(),
        "locked": coord.getLocked(),
        "prescribed_function": prescribed,
    }


def extract_effect(effect):
    function = effect.getFunction()
    func_type = function.getConcreteClassName()
    coeffs = []
    if func_type == "PolynomialFunction":
        for i in range(function.getCoefficientSize()):
            coeffs.append(function.getCoefficient(i))
    elif func_type == "LinearFunction":
        coeffs = [function.getSlope(), function.getIntercept()]
    elif func_type == "Constant":
        coeffs = [function.getValue()]
    elif func_type == "NullFunction":
        return None

    return {
        "coordinate_name": effect.getCoordinate().getName(),
        "function_type": func_type.replace("Function", ""),
        "coefficients": coeffs,
    }


def extract_spatial_transform(joint):
    if joint.getConcreteClassName() != "CustomJoint":
        return None
    st = joint.getSpatialTransform()
    names = ["rotation_x", "rotation_y", "rotation_z",
             "translation_x", "translation_y", "translation_z"]
    result = {n: None for n in names}
    for idx, name in enumerate(names):
        try:
            prop = st.getPropertyByName(name)
            effect = extract_effect(prop)
            if effect:
                result[name] = effect
        except Exception:
            try:
                comp = st.getComponent(idx)
                effect = extract_effect(comp)
                if effect:
                    result[name] = effect
            except Exception:
                pass
    return result


def extract_joint(joint):
    joint_type = joint.getConcreteClassName()
    data = {
        "name": joint.getName(),
        "joint_type": joint_type,
        "parent_body": joint.getParentFrame().getName(),
        "child_body": joint.getChildFrame().getName(),
        "location_in_parent": list(joint.getLocationInParent()),
        "orientation_in_parent": list(joint.getOrientationInParent()),
        "location_in_child": list(joint.getLocationInChild()),
        "orientation_in_child": list(joint.getOrientationInChild()),
        "axis": None,
        "coordinate": None,
        "coordinates": None,
        "spatial_transform": None,
    }

    if joint_type == "PinJoint":
        coord = joint.getCoordinate()
        data["axis"] = list(coord.getAxis())
        data["coordinate"] = extract_coordinate(coord)
    elif joint_type == "CustomJoint":
        coords = joint.getCoordinateSet()
        data["coordinates"] = [extract_coordinate(coords.get(i)) for i in range(coords.getSize())]
        data["spatial_transform"] = extract_spatial_transform(joint)
    elif joint_type == "UniversalJoint":
        coords = joint.getCoordinateSet()
        data["coordinates"] = [extract_coordinate(coords.get(i)) for i in range(coords.getSize())]
    elif joint_type == "BallJoint":
        coord = joint.getCoordinate()
        data["coordinate"] = extract_coordinate(coord)
    elif joint_type in ("FreeJoint", "WeldJoint"):
        pass
    else:
        try:
            cs = joint.getCoordinateSet()
            if cs.getSize() > 0:
                data["coordinate"] = extract_coordinate(cs.get(0))
        except Exception:
            pass

    return data


def extract_marker(marker):
    return {
        "name": marker.getName(),
        "body": marker.getBodyName(),
        "location": list(marker.getLocation()),
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


def extract_muscle(force, body_names, coord_names):
    name = force.getName()
    class_name = force.getConcreteClassName()

    def get_f64(method, default=0.0):
        try:
            return getattr(force, method)()
        except Exception:
            return default

    def get_bool(method, default=False):
        try:
            return getattr(force, method)()
        except Exception:
            return default

    # Geometry path points
    path_points = extract_muscle_path_points(force)

    return {
        "name": name,
        "muscle_type": class_name,
        "max_isometric_force": get_f64("getMaxIsometricForce"),
        "optimal_fiber_length": get_f64("getOptimalFiberLength"),
        "tendon_slack_length": get_f64("getTendonSlackLength"),
        "pennation_angle_at_optimal": get_f64("getPennationAngleAtOptimalFiberLength"),
        "max_contraction_velocity": get_f64("getMaxContractionVelocity", 10.0),
        "activation_time_constant": get_f64("getActivationTimeConstant", 0.01),
        "deactivation_time_constant": get_f64("getDeactivationTimeConstant", 0.04),
        "minimum_activation": get_f64("getMinimumActivation", 0.01),
        "fiber_damping": get_f64("getFiberDamping", 0.1),
        "ignore_activation_dynamics": get_bool("getIgnoreActivationDynamics"),
        "ignore_tendon_compliance": get_bool("getIgnoreTendonCompliance"),
        "path_points": path_points,
    }


def extract_wrap(wrap):
    name = wrap.getName()
    wrap_type = wrap.getConcreteClassName()
    body = wrap.getFrame().getName()
    location = list(wrap.getLocation())
    orientation = list(wrap.getOrientation())

    if "Sphere" in wrap_type:
        dimensions = [wrap.getRadius()]
    elif "Cylinder" in wrap_type:
        dimensions = [wrap.getRadius(), wrap.getLength()]
    elif "Ellipsoid" in wrap_type:
        dims = wrap.getDimensions()
        dimensions = [dims.get(i) for i in range(3)]
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
            dg = body.getPropertyByName("display_geometry").getValue(i)
            geoms.append({
                "body_name": body.getName(),
                "mesh_file": dg.getPropertyByName("display_geometry_file").getValueString(0),
                "scale_factors": list(dg.getPropertyByName("scale_factors").getValue(0)),
                "color": list(dg.getPropertyByName("color").getValue(0)),
                "opacity": dg.getPropertyByName("opacity").getValue(0),
                "transform": list(dg.getPropertyByName("transform").getValue(0)) if dg.getPropertyByName("transform").size() > 0 else None,
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

    body_names = {b["name"] for b in data["bodies"]}

    # Joints
    joint_set = model.getJointSet()
    for i in range(joint_set.getSize()):
        data["joints"].append(extract_joint(joint_set.get(i)))

    # Coordinates name map (for muscle path points)
    coord_names = set()
    for j in data["joints"]:
        if j["coordinate"]:
            coord_names.add(j["coordinate"]["name"])
        if j["coordinates"]:
            for c in j["coordinates"]:
                coord_names.add(c["name"])

    # Markers
    marker_set = model.getMarkerSet()
    for i in range(marker_set.getSize()):
        data["markers"].append(extract_marker(marker_set.get(i)))

    # Muscles (from ForceSet)
    force_set = model.getForceSet()
    for i in range(force_set.getSize()):
        force = force_set.get(i)
        class_name = force.getConcreteClassName()
        if "Muscle" in class_name:
            data["muscles"].append(extract_muscle(force, body_names, coord_names))

    # Wrap objects
    try:
        wrap_set = model.getWrapObjectSet()
        for i in range(wrap_set.getSize()):
            data["wrap_objects"].append(extract_wrap(wrap_set.get(i)))
    except Exception:
        pass

    # Display geometry (attached to bodies)
    for i in range(body_set.getSize()):
        body = body_set.get(i)
        data["display_geometries"].extend(extract_display_geometry(body))

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
