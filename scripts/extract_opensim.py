#!/usr/bin/env python3
"""
Extract OpenSim model data to JSON for melosim Rust importer.

Usage:
    python extract_opensim.py Rajagopal2015.osim output.json

Requires:
    opensim (pip install opensim)  — only on your local machine

Produces a JSON file matching the OpenSimModelData struct in
melosim::importer::opensim. The Rust importer reads this JSON
and populates an ECS World.

Architecture:
    Your machine (OpenSim) ──JSON──→ Any machine (Rust importer)
"""

import json
import sys
from pathlib import Path


def extract_body(body):
    """Extract body data from an OpenSim Body object."""
    mass_center = body.getMassCenter()
    inertia = body.getInertia()
    return {
        "name": body.getName(),
        "mass": body.getMass(),
        "mass_center": [mass_center.get(i) for i in range(3)],
        "inertia": [
            inertia.get(0), inertia.get(1), inertia.get(2),  # Ixx, Iyy, Izz
            inertia.get(3), inertia.get(4), inertia.get(5),  # Ixy, Ixz, Iyz
        ],
    }


def extract_coordinate(coord):
    """Extract a coordinate from an OpenSim Coordinate object."""
    # Check for prescribed function
    prescribed = None
    try:
        pf = coord.getPrescribedFunction()
        if pf and pf.getConcreteClassName() != "NullFunction":
            func_type = pf.getConcreteClassName()
            coeffs = []
            if func_type == "PolynomialFunction":
                for i in range(pf.getCoefficientSize()):
                    coeffs.append(pf.getCoefficient(i))
            prescribed = {
                "function_type": func_type,
                "coefficients": coeffs,
            }
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
    """Extract a CoordinateEffect from a SpatialTransform component."""
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
        return None  # No effect defined for this component

    return {
        "coordinate_name": effect.getCoordinate().getName(),
        "function_type": func_type.replace("Function", ""),
        "coefficients": coeffs,
    }


def extract_spatial_transform(joint):
    """Extract the SpatialTransform from a CustomJoint."""
    if joint.getConcreteClassName() != "CustomJoint":
        return None

    st = joint.getSpatialTransform()
    transform = {}
    # The 6 transform components
    for component_name in [
        "rotation_x", "rotation_y", "rotation_z",
        "translation_x", "translation_y", "translation_z",
    ]:
        # In OpenSim Python API, transform components are accessed
        # by index (0-5) rather than name directly
        transform[component_name] = None

    # TODO: OpenSim Python API needs proper iteration
    # For now, this is a placeholder — the actual iteration
    # depends on how PyO3 exposes SpatialTransform components.
    # In practice you'd do:
    # for i in range(6):
    #     component = st.getComponent(i)
    #     effect = extract_effect(component)
    #     if effect:
    #         transform[component_name_map[i]] = effect
    return transform


def extract_joint(joint):
    """Extract joint data from an OpenSim Joint object."""
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
        data["coordinates"] = [
            extract_coordinate(coords.get(i))
            for i in range(coords.getSize())
        ]
        data["spatial_transform"] = extract_spatial_transform(joint)

    elif joint_type == "UniversalJoint":
        coords = joint.getCoordinateSet()
        data["coordinates"] = [
            extract_coordinate(coords.get(i))
            for i in range(coords.getSize())
        ]

    elif joint_type == "BallJoint":
        coord = joint.getCoordinate()
        data["coordinate"] = extract_coordinate(coord)

    elif joint_type in ("FreeJoint", "WeldJoint"):
        pass  # No coordinates to extract

    else:
        # For unknown types, try to get coordinates if they exist
        try:
            if joint.getCoordinateSet().getSize() > 0:
                coord = joint.getCoordinate(0)
                data["coordinate"] = extract_coordinate(coord)
        except Exception:
            pass

    return data


def extract_marker(marker):
    """Extract marker data from an OpenSim Marker object."""
    location = marker.getLocation()
    return {
        "name": marker.getName(),
        "body": marker.getBodyName(),
        "location": [location.get(i) for i in range(3)],
    }


def extract_model(osim_path):
    """Load an OpenSim model and extract its data structure."""
    import opensim as osim

    model = osim.Model(osim_path)
    model.initSystem()

    data = {
        "name": model.getName(),
        "bodies": [],
        "joints": [],
        "markers": [],
    }

    # Extract bodies
    body_set = model.getBodySet()
    for i in range(body_set.getSize()):
        body = body_set.get(i)
        data["bodies"].append(extract_body(body))

    # Ground is implicit — add it if not already in the set
    # (OpenSim's BodySet doesn't include ground by default)
    has_ground = any(b["name"] == "ground" for b in data["bodies"])
    if not has_ground:
        data["bodies"].append({
            "name": "ground",
            "mass": 0.0,
            "mass_center": [0.0, 0.0, 0.0],
            "inertia": [0.0] * 6,
        })

    # Extract joints
    joint_set = model.getJointSet()
    for i in range(joint_set.getSize()):
        joint = joint_set.get(i)
        data["joints"].append(extract_joint(joint))

    # Extract markers
    marker_set = model.getMarkerSet()
    for i in range(marker_set.getSize()):
        marker = marker_set.get(i)
        data["markers"].append(extract_marker(marker))

    return data


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)

    osim_path = sys.argv[1]
    json_path = sys.argv[2] if len(sys.argv) > 2 else "model.json"

    print(f"Loading model from {osim_path}...")
    data = extract_model(osim_path)

    print(f"  Bodies: {len(data['bodies'])}")
    print(f"  Joints: {len(data['joints'])}")
    print(f"  Markers: {len(data['markers'])}")
    print(f"Writing to {json_path}...")

    with open(json_path, "w") as f:
        json.dump(data, f, indent=2)

    print("Done!")


if __name__ == "__main__":
    main()
