// ── MuJoCo MJCF XML Exporter ──────────────────────────
//
// Walks the melosim World and produces a valid MJCF XML file.
// Works alongside the MJCF importer to enable round-trip testing:
//   import (.mjcf) → World → export (.mjcf)
//
// MJCF structure:
//   <mujoco>
//     <worldbody>
//       <body name="...">         ← InertialProperties + Name
//         <inertial .../>
//         <joint name="..." />    ← HingeJoint / SlideJoint / etc.
//         <geom name="..." />     ← DisplayGeometry
//         <site name="..." />     ← Site
//         <body name="...">       ← recursive children
//         </body>
//       </body>
//     </worldbody>
//     <tendon>
//       <spatial name="...">      ← from MusclePath
//         <site site="..." />
//       </spatial>
//     </tendon>
//     <actuator>
//       <muscle name="..." />     ← Muscle + Millard2012Params
//     </actuator>
//   </mujoco>

use std::collections::HashMap;

use crate::components::*;
use crate::id::EntityID;
use crate::world::World;

/// Export the World to an MJCF XML string.
pub fn world_to_mjcf(world: &World, model_name: &str) -> String {
    let mut xml = String::new();


    xml.push_str("<mujoco model=\"");
    xml.push_str(&escape_attr(model_name));
    xml.push_str("\">\n");
    // MuJoCo defaults to degrees; our internal representation is radians.
    xml.push_str("  <compiler angle=\"radian\"/>\n");

    // ── Build body hierarchy ──
    let children_map = build_children_map(world);
    let body_names = build_body_name_map(world);

    // ── Find root bodies (no Frame parent, or parent is ground entity 0) ──
    let roots = find_root_bodies(world);

    // ── worldbody ──
    // In MJCF, the worldbody is implicitly the ground/world body.
    // We don't wrap ground in a <body> — root children go directly inside worldbody.
    xml.push_str("  <worldbody>\n");

    for &root_id in &roots {
        emit_body_recursive(world, &mut xml, root_id, &children_map, &body_names, 2);
    }

    xml.push_str("  </worldbody>\n");

    // ── tendon section ──
    let has_tendons = world.iter::<MusclePath>().next().is_some();
    if has_tendons {
        xml.push_str("\n  <tendon>\n");
        for (muscle_key, path) in world.iter::<MusclePath>() {
            let tendon_name = world.get::<Name>(muscle_key)
                .map(|n| format!("{}_tendon", n.value))
                .unwrap_or_else(|| format!("tendon_{}", muscle_key.0));
            xml.push_str(&format!("    <spatial name=\"{}\">\n", escape_attr(&tendon_name)));
            for point in &path.points {
                match point {
                    PathPoint::BodyFixed { body, location } => {
                        // Find the site name for this path point
                        // Look for a Site entity on this body at this location
                        if let Some(site_name) = find_site_name(world, *body, location) {
                            xml.push_str(&format!("      <site site=\"{}\"/>\n", escape_attr(&site_name)));
                        }
                    }
                    PathPoint::Moving { .. } => {
                        // Moving path points aren't directly representable in MJCF spatial tendons
                        // Skip for now — they need a different export strategy
                    }
                }
            }
            xml.push_str("    </spatial>\n");
        }
        xml.push_str("  </tendon>\n");
    }

    // ── actuator section ──
    let has_muscles = world.iter::<Muscle>().next().is_some();
    let has_coord_actuators = world.iter::<CoordinateActuator>().next().is_some();
    if has_muscles || has_coord_actuators {
        xml.push_str("\n  <actuator>\n");

        // Muscles
        for (muscle_key, _muscle) in world.iter::<Muscle>() {
            let name = world.get::<Name>(muscle_key)
                .map(|n| n.value.as_str())
                .unwrap_or("unnamed_muscle");

            let params = world.get::<Millard2012Params>(muscle_key);
            let path = world.get::<MusclePath>(muscle_key);

            let tendon_name = path.and_then(|_p| {
                // Find the tendon name from the muscle key
                Some(format!("{}_tendon", name))
            });

            let force = params.map(|p| p.max_isometric_force).unwrap_or(1000.0);
            let range_min = params.map(|p| p.minimum_activation).unwrap_or(0.01);
            let range_max = 1.0;
            let lengthrange = params.map(|p| {
                format!("{} {}", p.tendon_slack_length, p.tendon_slack_length + p.optimal_fiber_length)
            });

            xml.push_str(&format!("    <muscle name=\"{}\"", escape_attr(name)));
            xml.push_str(&format!(" force=\"{}\"", force));
            xml.push_str(&format!(" range=\"{} {}\"", range_min, range_max));
            if let Some(ref lr) = lengthrange {
                xml.push_str(&format!(" lengthrange=\"{}\"", lr));
            }
            if let Some(ref tn) = tendon_name {
                xml.push_str(&format!(" tendon=\"{}\"", escape_attr(tn)));
            }
            xml.push_str("/>\n");
        }

        // Coordinate actuators
        for (act_key, act) in world.iter::<CoordinateActuator>() {
            let name = world.get::<Name>(act_key)
                .map(|n| n.value.as_str())
                .unwrap_or("unnamed_actuator");

            // Find the joint name for the coordinate
            let coord_name = world.get::<Name>(act.coordinate)
                .map(|n| n.value.as_str())
                .unwrap_or("unnamed_coord");

            xml.push_str(&format!("    <general name=\"{}\"", escape_attr(name)));
            xml.push_str(&format!(" joint=\"{}\"", escape_attr(coord_name)));
            xml.push_str(&format!(" gear=\"{}\"", act.optimal_force));
            xml.push_str(&format!(" ctrlrange=\"{} {}\"", act.min_control, act.max_control));
            xml.push_str("/>\n");
        }

        xml.push_str("  </actuator>\n");
    }

    xml.push_str("</mujoco>\n");
    xml
}

/// Write MJCF to a file.
pub fn write_mjcf(world: &World, path: &str, model_name: &str) -> Result<(), String> {
    let xml = world_to_mjcf(world, model_name);
    std::fs::write(path, &xml).map_err(|e| format!("Failed to write {}: {}", path, e))
}

// ── Helper functions ──

/// Build parent → children map from ChildOf components.
fn build_children_map(world: &World) -> HashMap<EntityID, Vec<EntityID>> {
    let mut children: HashMap<EntityID, Vec<EntityID>> = HashMap::new();
    for (entity, child_of) in world.iter::<ChildOf>() {
        children.entry(child_of.parent).or_default().push(entity);
    }
    children
}

/// Build entity → name map for bodies (entities with InertialProperties).
fn build_body_name_map(world: &World) -> HashMap<EntityID, String> {
    let mut names = HashMap::new();
    for (entity, _) in world.iter::<InertialProperties>() {
        if let Some(name) = world.get::<Name>(entity) {
            names.insert(entity, name.value.clone());
        }
    }
    names
}

/// Find root bodies: entities with InertialProperties whose ChildOf
/// parent is entity 0 (ground) or that have no ChildOf at all.
fn find_root_bodies(world: &World) -> Vec<EntityID> {
    let mut roots = Vec::new();
    for (entity, child_of) in world.iter::<ChildOf>() {
        if child_of.parent == EntityID(0) {
            // This body's parent is ground — it's a root
            if world.get::<InertialProperties>(entity).is_some() {
                roots.push(entity);
            }
        }
    }
    // Also include bodies with no ChildOf (top-level bodies)
    for (entity, _) in world.iter::<InertialProperties>() {
        if entity == EntityID(0) {
            continue; // Skip ground itself
        }
        if world.get::<ChildOf>(entity).is_none() {
            roots.push(entity);
        }
    }
    roots
}

/// Recursively emit a body and its children.
fn emit_body_recursive(
    world: &World,
    xml: &mut String,
    entity: EntityID,
    children_map: &HashMap<EntityID, Vec<EntityID>>,
    body_names: &HashMap<EntityID, String>,
    depth: usize,
) {
    let indent = "  ".repeat(depth);
    let name = body_names.get(&entity).map(|s| s.as_str()).unwrap_or("unnamed");

    xml.push_str(&format!("{}<body name=\"{}\"", indent, escape_attr(name)));

    // Position and orientation from Position/Rotation components
    if let Some(pos) = world.get::<Position>(entity) {
        if pos.x != 0.0 || pos.y != 0.0 || pos.z != 0.0 {
            xml.push_str(&format!(" pos=\"{} {} {}\"", pos.x, pos.y, pos.z));
        }
    }
    if let Some(rot) = world.get::<Rotation>(entity) {
        let r = &rot.quaternion;
        if r.w != 1.0 || r.x != 0.0 || r.y != 0.0 || r.z != 0.0 {
            xml.push_str(&format!(" quat=\"{} {} {} {}\"", r.w, r.x, r.y, r.z));
        }
    }
    xml.push_str(">\n");

    // Inertial properties
    if let Some(inertial) = world.get::<InertialProperties>(entity) {
        xml.push_str(&format!("{}  <inertial pos=\"{} {} {}\" mass=\"{}\"",
            indent, inertial.com[0], inertial.com[1], inertial.com[2], inertial.mass));
        // fullinertia: Ixx Iyy Izz Ixy Ixz Iyz
        let i = &inertial.inertia;
        xml.push_str(&format!(" fullinertia=\"{} {} {} {} {} {}\"",
            i[0], i[1], i[2], i[3], i[4], i[5]));
        xml.push_str("/>\n");
    }

    // Joints attached to this body
    emit_body_joints(world, xml, entity, indent.len());

    // Display geometries
    for (_geom_key, geom) in world.iter::<DisplayGeometry>() {
        if geom.body == entity {
            xml.push_str(&format!("{}  <geom name=\"{}\"",
                indent, escape_attr(
                    world.get::<Name>(_geom_key).map(|n| n.value.as_str()).unwrap_or("geom")
                )));
            // Use sphere type for all display geoms.
            // Mesh references require mesh definitions in <asset> section
            // which we don't export yet. Use the first scale value as radius.
            xml.push_str(" type=\"sphere\"");
            xml.push_str(&format!(" size=\"{} {} {}\"", geom.scale[0], geom.scale[1], geom.scale[2]));
            xml.push_str(&format!(" pos=\"{} {} {}\"",
                geom.transform.translation.x, geom.transform.translation.y, geom.transform.translation.z));
            xml.push_str(&format!(" rgba=\"{} {} {} {}\"",
                geom.color[0], geom.color[1], geom.color[2], geom.opacity));
            xml.push_str("/>\n");
        }
    }

    // Sites on this body
    for (site_key, _site) in world.iter::<Site>() {
        if world.get::<ChildOf>(site_key).map_or(false, |co| co.parent == entity) {
            let site_name = world.get::<Name>(site_key)
                .map(|n| n.value.as_str())
                .unwrap_or("unnamed_site");
            let pos = world.get::<Position>(site_key);
            let (px, py, pz) = pos.map(|p| (p.x, p.y, p.z)).unwrap_or((0.0, 0.0, 0.0));
            xml.push_str(&format!("{}  <site name=\"{}\" pos=\"{} {} {}\"/>\n",
                indent, escape_attr(site_name), px, py, pz));
        }
    }

    // Wrap geometries (as reference geoms for muscle wrapping)
    for (_wrap_key, wrap) in world.iter::<WrapGeom>() {
        if wrap.body == entity {
            let wrap_name = world.get::<Name>(_wrap_key)
                .map(|n| n.value.as_str())
                .unwrap_or("wrap");
            match &wrap.geom_type {
                WrapGeomType::Sphere { radius } => {
                    xml.push_str(&format!("{}  <geom name=\"{}\" type=\"sphere\" size=\"{}\" pos=\"{} {} {}\" rgba=\"0.5 0.5 0.9 0.4\" group=\"2\"/>\n",
                        indent, escape_attr(wrap_name), radius,
                        wrap.transform.translation.x, wrap.transform.translation.y, wrap.transform.translation.z));
                }
                WrapGeomType::Cylinder { radius, length } => {
                    xml.push_str(&format!("{}  <geom name=\"{}\" type=\"cylinder\" size=\"{} {}\" pos=\"{} {} {}\" rgba=\"0.5 0.5 0.9 0.4\" group=\"2\"/>\n",
                        indent, escape_attr(wrap_name), radius, length,
                        wrap.transform.translation.x, wrap.transform.translation.y, wrap.transform.translation.z));
                }
                WrapGeomType::Ellipsoid { radii } => {
                    xml.push_str(&format!("{}  <geom name=\"{}\" type=\"ellipsoid\" size=\"{} {} {}\" pos=\"{} {} {}\" rgba=\"0.5 0.5 0.9 0.4\" group=\"2\"/>\n",
                        indent, escape_attr(wrap_name), radii[0], radii[1], radii[2],
                        wrap.transform.translation.x, wrap.transform.translation.y, wrap.transform.translation.z));
                }
            }
        }
    }

    // Recurse into children
    if let Some(children) = children_map.get(&entity) {
        for &child in children {
            emit_body_recursive(world, xml, child, children_map, body_names, depth + 1);
        }
    }

    xml.push_str(&format!("{}</body>\n", indent));
}

/// Emit joints attached to a body.
fn emit_body_joints(world: &World, xml: &mut String, body: EntityID, indent: usize) {
    let ind = "  ".repeat(indent + 1);

    for (key, joint) in world.iter::<Joint>() {
        if !world.get::<ChildFrame>(key).map_or(false, |cf| cf.frame == body) {
            continue;
        }
        let name = world.get::<Name>(key).map(|n| n.value.as_str()).unwrap_or("joint");

        // Extract axis from the first RotationAboutAxis or TranslationAlongAxis effect
        let axis = extract_joint_axis(world, key);

        let kind = infer_joint_kind(world, joint);
        match kind {
            "PinJoint" => {
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"hinge\" axis=\"{} {} {}\"",
                    ind, escape_attr(name), axis[0], axis[1], axis[2]));
                if let Some(ref lim) = joint.limits {
                    xml.push_str(&format!(" limited=\"true\" range=\"{} {}\"", lim.lower, lim.upper));
                }
                append_joint_dynamics(world, xml, key);
                xml.push_str("/>\n");
            }
            "SlideJoint" => {
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"slide\" axis=\"{} {} {}\"",
                    ind, escape_attr(name), axis[0], axis[1], axis[2]));
                if let Some(ref lim) = joint.limits {
                    xml.push_str(&format!(" limited=\"true\" range=\"{} {}\"", lim.lower, lim.upper));
                }
                append_joint_dynamics(world, xml, key);
                xml.push_str("/>\n");
            }
            "BallJoint" => {
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"ball\"", ind, escape_attr(name)));
                if let Some(ref lim) = joint.limits {
                    xml.push_str(&format!(" limited=\"true\" range=\"{} {}\"", lim.lower, lim.upper));
                }
                xml.push_str("/>\n");
            }
            "FreeJoint" => {
                xml.push_str(&format!("{}<freejoint name=\"{}\"/>\n", ind, escape_attr(name)));
            }
            "WeldJoint" => {
                // MuJoCo has no explicit fixed joint — bodies without joints are fixed
            }
            "UniversalJoint" => {
                // Emit two hinge joints for the two axes
                let axis2 = extract_joint_axis2(world, key);
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"hinge\" axis=\"{} {} {}\"",
                    ind, escape_attr(name), axis[0], axis[1], axis[2]));
                if let Some(ref lim) = joint.limits {
                    xml.push_str(&format!(" limited=\"true\" range=\"{} {}\"", lim.lower, lim.upper));
                }
                xml.push_str("/>\n");
                xml.push_str(&format!("{}<joint name=\"{}_2\" type=\"hinge\" axis=\"{} {} {}\"/>\n",
                    ind, escape_attr(name), axis2[0], axis2[1], axis2[2]));
            }
            "CustomJoint" => {
                // One hinge per coordinate
                for (i, coord_key) in joint.coordinates.iter().enumerate() {
                    let coord_name = world.get::<Name>(*coord_key).map(|n| n.value.as_str()).unwrap_or("coord");
                    if i == 0 {
                        xml.push_str(&format!("{}<joint name=\"{}\" type=\"hinge\" axis=\"0 0 1\"",
                            ind, escape_attr(coord_name)));
                    } else {
                        xml.push_str(&format!("{}<joint name=\"{}\" type=\"hinge\" axis=\"0 0 1\"",
                            ind, escape_attr(coord_name)));
                    }
                    xml.push_str("/>\n");
                }
            }
            _ => {}
        }
    }
}

/// Extract the axis from a joint's first RotationAboutAxis or TranslationAlongAxis effect.
fn extract_joint_axis(world: &World, joint_key: EntityID) -> [f64; 3] {
    for (_st_key, st) in world.iter::<SpatialTransform>() {
        if st.joint == joint_key {
            for effect_key in &st.effects {
                if let Some(effect) = world.get::<CoordinateEffect>(*effect_key) {
                    match &effect.component {
                        TransformComponent::RotationAboutAxis(axis)
                        | TransformComponent::TranslationAlongAxis(axis) => {
                            return *axis;
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    [0.0, 0.0, 1.0] // default
}

/// Extract the second axis from a joint's effects (for UniversalJoint).
fn extract_joint_axis2(world: &World, joint_key: EntityID) -> [f64; 3] {
    let mut found_first = false;
    for (_st_key, st) in world.iter::<SpatialTransform>() {
        if st.joint == joint_key {
            for effect_key in &st.effects {
                if let Some(effect) = world.get::<CoordinateEffect>(*effect_key) {
                    match &effect.component {
                        TransformComponent::RotationAboutAxis(axis) => {
                            if found_first {
                                return *axis;
                            }
                            found_first = true;
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    [0.0, 1.0, 0.0] // default
}

/// Append stiffness/damping from JointCoordinate if available.
fn append_joint_dynamics(world: &World, xml: &mut String, joint_key: EntityID) {
    // Look for a JointCoordinate whose Name matches the joint's Name
    let joint_name = world.get::<Name>(joint_key).map(|n| n.value.as_str());
    if let Some(jname) = joint_name {
        for (_coord_key, coord) in world.iter::<JointCoordinate>() {
            let coord_name = world.get::<Name>(_coord_key).map(|n| n.value.as_str());
            if coord_name == Some(jname) {
                if coord.stiffness != 0.0 {
                    xml.push_str(&format!(" stiffness=\"{}\"", coord.stiffness));
                }
                if coord.damping != 0.0 {
                    xml.push_str(&format!(" damping=\"{}\"", coord.damping));
                }
                return;
            }
        }
    }
}

/// Find a site name matching a body and location (for tendon path references).
fn find_site_name(world: &World, body: EntityID, location: &[f64; 3]) -> Option<String> {
    for (site_key, _site) in world.iter::<Site>() {
        if world.get::<ChildOf>(site_key).map_or(false, |co| co.parent == body) {
            if let Some(pos) = world.get::<Position>(site_key) {
                let dx = pos.x - location[0];
                let dy = pos.y - location[1];
                let dz = pos.z - location[2];
                if (dx * dx + dy * dy + dz * dz) < 1e-12 {
                    return world.get::<Name>(site_key).map(|n| n.value.clone());
                }
            }
        }
    }
    None
}

/// Escape special characters for XML attribute values.
fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}