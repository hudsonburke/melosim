// ── OpenSim .osim XML Exporter ────────────────────────
//
// Walks the melosim World and produces a valid .osim XML file.
// Works alongside the PyO3 importer to enable round-trip testing:
//   import (.osim) → World → export (.osim)
//
// The exporter handles the inverse of the importer: it looks up which
// joint connects each body to its parent, and nests the joint inside
// the child body's XML element, matching OpenSim's serialization format.
//
// Supports export of: bodies, joints, markers, muscles (Millard2012),
// wrap geometry, and display geometry.

use std::collections::HashMap;

use crate::components::*;
use crate::id::EntityID;
use crate::world::World;

use crate::components::Name;

/// Export the World to an .osim XML string.
pub fn world_to_osim(world: &World, model_name: &str) -> String {
    let mut xml = String::new();

    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str("<OpenSimDocument Version=\"30000\">\n");
    xml.push_str(&format!("<Model name=\"{}\">\n", escape_attr(model_name)));

    // ── Build inverse lookup: for each entity, find its joint (where body_b == entity) ──
    let child_joint = build_child_joint_map(world);
    // Build set of bodies that are parents (body_a) of at least one joint ──
    let parent_set = build_parent_set(world);
    // ── Build name → key lookup for body names ──
    let body_names = build_body_name_map(world);
    // ── Build name → key lookup for coordinate names ──
    let coord_names = build_coordinate_name_map(world);

    // ── BodySet ──
    xml.push_str("  <BodySet>\n");
    xml.push_str("    <objects>\n");

    for (body_key, _) in world.iter::<InertialProperties>() {
        let name = body_names
            .get(&body_key)
            .map(|s| s.as_str())
            .unwrap_or("unknown");

        // In OpenSim, the root body (ground) is not emitted in BodySet
        // unless it has a joint. Detect ground as: no joint where this
        // is body_b (no parent joint) AND is a parent of at least one joint.
        let has_parent_joint = child_joint.contains_key(&body_key);
        let is_parent = parent_set.contains(&body_key);

        if !has_parent_joint && is_parent {
            continue; // Skip root body — it's implicit ground in OpenSim
        }

        xml.push_str(&format!("      <Body name=\"{}\">\n", escape_attr(name)));
        xml.push_str(&emit_body_properties(world, body_key));

        // ── Emit joint if this body is the child of one ──
        if let Some(joint) = find_parent_joint(world, body_key) {
            xml.push_str(&joint);
        } else {
            // FreeJoint to ground for root bodies without explicit joint
            xml.push_str("        <Joint>\n");
            xml.push_str("          <FreeJoint name=\"ground_to_body\">\n");
            xml.push_str("            <parent_body>ground</parent_body>\n");
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </FreeJoint>\n");
            xml.push_str("        </Joint>\n");
        }

        // ── Display geometry for this body ──
        xml.push_str(&emit_body_display_geometry(world, body_key));

        xml.push_str("      </Body>\n");
    }

    xml.push_str("    </objects>\n");
    xml.push_str("  </BodySet>\n");

    // ── ForceSet (muscles + actuators) ──
    let muscle_count = world.count::<Muscle>();
    if muscle_count > 0 {
        xml.push_str("  <ForceSet>\n");
        xml.push_str("    <objects>\n");
        xml.push_str(&emit_muscles(world, &body_names, &coord_names));
        xml.push_str("    </objects>\n");
        xml.push_str("  </ForceSet>\n");
    }

    // ── MarkerSet ──
    let marker_count = world.count::<Landmark>();
    if marker_count > 0 {
        xml.push_str("  <MarkerSet>\n");
        xml.push_str("    <objects>\n");
        for (landmark_key, landmark) in world.iter::<Landmark>() {
            if let Some(site) = world.get::<Site>(landmark.site) {
                let landmark_name = world.get::<Name>(landmark_key).map(|n| n.value.as_str()).unwrap_or("marker");
                xml.push_str(&format!(
                    "      <Marker name=\"{}\">\n",
                    escape_attr(landmark_name)
                ));
                // Get parent body name for marker
                let parent_name = body_names
                    .get(&site.parent)
                    .map(|s| s.as_str())
                    .unwrap_or("ground");
                xml.push_str(&format!(
                    "        <body>{}</body>\n",
                    escape_attr(parent_name)
                ));
                xml.push_str(&format!(
                    "        <location>{} {} {}</location>\n",
                    site.offset.x, site.offset.y, site.offset.z
                ));
                xml.push_str("        <fixed>true</fixed>\n");
                xml.push_str("      </Marker>\n");
            }
        }
        xml.push_str("    </objects>\n");
        xml.push_str("  </MarkerSet>\n");
    }

    // ── WrapObjectSet (at Model level for simplicity) ──
    let wrap_count = world.count::<WrapGeom>();
    if wrap_count > 0 {
        xml.push_str("  <WrapObjectSet>\n");
        xml.push_str("    <objects>\n");
        xml.push_str(&emit_wrap_objects(world, &body_names));
        xml.push_str("    </objects>\n");
        xml.push_str("  </WrapObjectSet>\n");
    }

    xml.push_str("</Model>\n");
    xml.push_str("</OpenSimDocument>\n");

    xml
}

/// Write the World to an .osim file.
pub fn write_osim(world: &World, path: &str, model_name: &str) -> Result<(), String> {
    let xml = world_to_osim(world, model_name);
    std::fs::write(path, &xml).map_err(|e| format!("Failed to write '{}': {}", path, e))
}

// ── Helpers ───────────────────────────────────────────

/// Build a map from child body key → joint key for all joint types.
fn build_child_joint_map(world: &World) -> HashMap<EntityID, EntityID> {
    let mut map = HashMap::new();

    for (key, joint) in world.iter::<HingeJoint>() {
        map.insert(joint.body_b, key);
    }
    for (key, joint) in world.iter::<SlideJoint>() {
        map.insert(joint.body_b, key);
    }
    for (key, joint) in world.iter::<BallJoint>() {
        map.insert(joint.body_b, key);
    }
    for (key, joint) in world.iter::<FreeJoint>() {
        map.insert(joint.body_b, key);
    }
    for (key, joint) in world.iter::<FixedJoint>() {
        map.insert(joint.body_b, key);
    }
    for (key, joint) in world.iter::<UniversalJoint>() {
        map.insert(joint.body_b, key);
    }
    for (key, joint) in world.iter::<CustomJoint>() {
        map.insert(joint.body_b, key);
    }

    map
}

/// Build a set of body EntityIDs that appear as body_a (parent) in any joint.
fn build_parent_set(world: &World) -> std::collections::HashSet<EntityID> {
    let mut set = std::collections::HashSet::new();

    for (_, joint) in world.iter::<HingeJoint>() { set.insert(joint.body_a); }
    for (_, joint) in world.iter::<SlideJoint>() { set.insert(joint.body_a); }
    for (_, joint) in world.iter::<BallJoint>() { set.insert(joint.body_a); }
    for (_, joint) in world.iter::<FreeJoint>() { set.insert(joint.body_a); }
    for (_, joint) in world.iter::<FixedJoint>() { set.insert(joint.body_a); }
    for (_, joint) in world.iter::<UniversalJoint>() { set.insert(joint.body_a); }
    for (_, joint) in world.iter::<CustomJoint>() { set.insert(joint.body_a); }

    set
}

/// Build a map from entity key → body name.
fn build_body_name_map(world: &World) -> HashMap<EntityID, String> {
    let mut map = HashMap::new();

    for (id, name) in world.iter::<Name>() {
        if world.get::<InertialProperties>(id).is_some() {
            map.insert(id, name.value.clone());
        }
    }

    map
}

/// Build a map from coordinate entity key → coordinate name.
fn build_coordinate_name_map(world: &World) -> HashMap<EntityID, String> {
    let mut map = HashMap::new();
    for (id, name) in world.iter::<Name>() {
        if world.get::<JointCoordinate>(id).is_some() {
            map.insert(id, name.value.clone());
        }
    }
    map
}

/// Emit body properties XML (mass, mass_center, inertia).
fn emit_body_properties(world: &World, body_key: EntityID) -> String {
    let mut xml = String::new();

    if let Some(inertial) = world.get::<InertialProperties>(body_key) {
        xml.push_str(&format!(
            "        <mass>{}</mass>\n",
            inertial.mass
        ));
        xml.push_str(&format!(
            "        <mass_center>{} {} {}</mass_center>\n",
            inertial.com[0], inertial.com[1], inertial.com[2]
        ));
        xml.push_str(&format!(
            "        <inertia>{} {} {} {} {} {}</inertia>\n",
            inertial.inertia[0],
            inertial.inertia[1],
            inertial.inertia[2],
            inertial.inertia[3],
            inertial.inertia[4],
            inertial.inertia[5],
        ));
    }

    xml
}

/// Emit display geometry for a body (VisibleObject / GeometrySet).
fn emit_body_display_geometry(world: &World, body_key: EntityID) -> String {
    let mut xml = String::new();

    // Collect all DisplayGeometry attached to this body
    let mut has_geom = false;
    let mut geom_xml = String::new();

    for (_key, dg) in world.iter::<DisplayGeometry>() {
        if dg.body == body_key {
            has_geom = true;
            geom_xml.push_str("              <DisplayGeometry>\n");
            if let Some(ref mesh_file) = dg.mesh_file {
                geom_xml.push_str(&format!(
                    "                <mesh_file>{}</mesh_file>\n",
                    escape_attr(mesh_file)
                ));
            }
            geom_xml.push_str(&format!(
                "                <scale>{} {} {}</scale>\n",
                dg.scale[0], dg.scale[1], dg.scale[2]
            ));
            geom_xml.push_str(&format!(
                "                <color>{} {} {}</color>\n",
                dg.color[0], dg.color[1], dg.color[2]
            ));
            geom_xml.push_str(&format!(
                "                <opacity>{}</opacity>\n",
                dg.opacity
            ));
            geom_xml.push_str("                <transform>\n");
            geom_xml.push_str(&format!(
                "                  <translation>{} {} {}</translation>\n",
                dg.transform.translation.x,
                dg.transform.translation.y,
                dg.transform.translation.z,
            ));
            geom_xml.push_str("                  <rotation>0 0 0 1</rotation>\n");
            geom_xml.push_str("                </transform>\n");
            geom_xml.push_str("              </DisplayGeometry>\n");
        }
    }

    if has_geom {
        xml.push_str("        <VisibleObject>\n");
        xml.push_str("          <GeometrySet>\n");
        xml.push_str("            <objects>\n");
        xml.push_str(&geom_xml);
        xml.push_str("            </objects>\n");
        xml.push_str("          </GeometrySet>\n");
        xml.push_str("          <scale_factors>1 1 1</scale_factors>\n");
        xml.push_str("        </VisibleObject>\n");
    }

    xml
}

/// Emit muscles (Millard2012EquilibriumMuscle) as part of a ForceSet.
fn emit_muscles(
    world: &World,
    body_names: &HashMap<EntityID, String>,
    coord_names: &HashMap<EntityID, String>,
) -> String {
    let mut xml = String::new();

    for (muscle_key, _muscle) in world.iter::<Muscle>() {
        // Find the MusclePath for this muscle
        let path = world
            .iter::<MusclePath>()
            .find(|(_, p)| p.muscle == muscle_key)
            .map(|(_, p)| p);

        // Find the Millard2012Params for this muscle
        let params = world
            .iter::<Millard2012Params>()
            .find(|(_, p)| p.muscle == muscle_key)
            .map(|(_, p)| p);

        let muscle_name = world.get::<Name>(muscle_key).map(|n| n.value.as_str()).unwrap_or("muscle");
        xml.push_str(&format!(
            "        <Millard2012EquilibriumMuscle name=\"{}\">\n",
            escape_attr(muscle_name)
        ));

        // Millard2012 param fields
        if let Some(p) = params {
            xml.push_str(&format!(
                "          <max_isometric_force>{}</max_isometric_force>\n",
                p.max_isometric_force
            ));
            xml.push_str(&format!(
                "          <optimal_fiber_length>{}</optimal_fiber_length>\n",
                p.optimal_fiber_length
            ));
            xml.push_str(&format!(
                "          <tendon_slack_length>{}</tendon_slack_length>\n",
                p.tendon_slack_length
            ));
            xml.push_str(&format!(
                "          <pennation_angle_at_optimal>{}</pennation_angle_at_optimal>\n",
                p.pennation_angle_at_optimal
            ));
            xml.push_str(&format!(
                "          <max_contraction_velocity>{}</max_contraction_velocity>\n",
                p.max_contraction_velocity
            ));
            xml.push_str(&format!(
                "          <activation_time_constant>{}</activation_time_constant>\n",
                p.activation_time_constant
            ));
            xml.push_str(&format!(
                "          <deactivation_time_constant>{}</deactivation_time_constant>\n",
                p.deactivation_time_constant
            ));
            xml.push_str(&format!(
                "          <minimum_activation>{}</minimum_activation>\n",
                p.minimum_activation
            ));
            xml.push_str(&format!(
                "          <fiber_damping>{}</fiber_damping>\n",
                p.fiber_damping
            ));
            xml.push_str(&format!(
                "          <ignore_activation_dynamics>{}</ignore_activation_dynamics>\n",
                p.ignore_activation_dynamics
            ));
            xml.push_str(&format!(
                "          <ignore_tendon_compliance>{}</ignore_tendon_compliance>\n",
                p.ignore_tendon_compliance
            ));
        }

        // GeometryPath with PathPointSet
        xml.push_str("          <GeometryPath>\n");
        xml.push_str("            <PathPointSet>\n");
        xml.push_str("              <objects>\n");

        if let Some(p) = path {
            for (i, pt) in p.points.iter().enumerate() {
                let body_name = match pt {
                    PathPoint::BodyFixed { body, .. } => body_names
                        .get(body)
                        .map(|s| s.as_str())
                        .unwrap_or("ground"),
                    PathPoint::Moving { body, .. } => body_names
                        .get(body)
                        .map(|s| s.as_str())
                        .unwrap_or("ground"),
                };

                xml.push_str(&format!(
                    "                <PathPoint name=\"pp{}\">\n",
                    i + 1
                ));
                xml.push_str(&format!(
                    "                  <body>{}</body>\n",
                    escape_attr(body_name)
                ));

                match pt {
                    PathPoint::BodyFixed { location, .. } => {
                        xml.push_str(&format!(
                            "                  <location>{} {} {}</location>\n",
                            location[0], location[1], location[2]
                        ));
                    }
                    PathPoint::Moving {
                        coordinate, location_functions, ..
                    } => {
                        let coord_name = coord_names
                            .get(coordinate)
                            .map(|s| s.as_str())
                            .unwrap_or("unknown_coord");
                        xml.push_str(&format!(
                            "                  <coordinate>{}</coordinate>\n",
                            escape_attr(coord_name)
                        ));
                        let loc_at_zero: [f64; 3] =
                            std::array::from_fn(|axis| {
                                location_functions[axis]
                                    .last()
                                    .copied()
                                    .unwrap_or(0.0)
                            });
                        xml.push_str(&format!(
                            "                  <location>{} {} {}</location>\n",
                            loc_at_zero[0], loc_at_zero[1], loc_at_zero[2]
                        ));
                    }
                }

                xml.push_str("                </PathPoint>\n");
            }
        }

        xml.push_str("              </objects>\n");
        xml.push_str("            </PathPointSet>\n");
        xml.push_str("          </GeometryPath>\n");

        xml.push_str(&format!(
            "        </Millard2012EquilibriumMuscle>\n"
        ));
    }

    xml
}

/// Emit wrap geometry objects (WrapObjectSet).
fn emit_wrap_objects(
    world: &World,
    body_names: &HashMap<EntityID, String>,
) -> String {
    let mut xml = String::new();

    for (wg_key, wg) in world.iter::<WrapGeom>() {
        let body_name = body_names
            .get(&wg.body)
            .map(|s| s.as_str())
            .unwrap_or("ground");

        let elem_name = match wg.geom_type {
            WrapGeomType::Sphere { .. } => "WrapSphere",
            WrapGeomType::Cylinder { .. } => "WrapCylinder",
            WrapGeomType::Ellipsoid { .. } => "WrapEllipsoid",
        };

        let wrap_name = world.get::<Name>(wg_key).map(|n| n.value.as_str()).unwrap_or("wrap");
        xml.push_str(&format!(
            "          <{} name=\"{}\">\n",
            elem_name,
            escape_attr(wrap_name)
        ));
        xml.push_str(&format!(
            "            <frame>{}</frame>\n",
            escape_attr(body_name)
        ));
        xml.push_str(&format!(
            "            <xyz_body>{}</xyz_body>\n",
            escape_attr(body_name)
        ));
        xml.push_str(&format!(
            "            <translation>{} {} {}</translation>\n",
            wg.transform.translation.x,
            wg.transform.translation.y,
            wg.transform.translation.z,
        ));

        match wg.geom_type {
            WrapGeomType::Sphere { radius } => {
                xml.push_str(&format!("            <radius>{}</radius>\n", radius));
            }
            WrapGeomType::Cylinder { radius, length } => {
                xml.push_str(&format!("            <radius>{}</radius>\n", radius));
                xml.push_str(&format!("            <length>{}</length>\n", length));
            }
            WrapGeomType::Ellipsoid { radii } => {
                xml.push_str(&format!(
                    "            <dimensions>{} {} {}</dimensions>\n",
                    radii[0], radii[1], radii[2]
                ));
            }
        }

        xml.push_str(&format!("          </{}>\n", elem_name));
    }

    xml
}

/// Find the parent joint (where body_b == child_key) and return its XML.
fn find_parent_joint(world: &World, child_key: EntityID) -> Option<String> {
    let body_names = build_body_name_map(world);

    // Check each joint type in order. We compare body_b directly.
    for (_, joint) in world.iter::<HingeJoint>() {
        if joint.body_b == child_key {
            let parent_name = body_names.get(&joint.body_a).map(|s| s.as_str()).unwrap_or("ground");
            let mut xml = format!("        <Joint>\n          <PinJoint name=\"hinge_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("            <CoordinateSet>\n");
            xml.push_str("              <Coordinate name=\"hinge_coord\">\n");
            xml.push_str(&format!("                <axis>{} {} {}</axis>\n", joint.axis[0], joint.axis[1], joint.axis[2]));
            if let Some(limits) = &joint.limits {
                xml.push_str(&format!("                <range_min>{}</range_min>\n", limits.lower));
                xml.push_str(&format!("                <range_max>{}</range_max>\n", limits.upper));
            }
            xml.push_str("              </Coordinate>\n");
            xml.push_str("            </CoordinateSet>\n");
            xml.push_str("            <reverse>false</reverse>\n");
            xml.push_str("          </PinJoint>\n        </Joint>\n");
            return Some(xml);
        }
    }
    for (_, joint) in world.iter::<FreeJoint>() {
        if joint.body_b == child_key {
            let parent_name = body_names.get(&joint.body_a).map(|s| s.as_str()).unwrap_or("ground");
            let mut xml = format!("        <Joint>\n          <FreeJoint name=\"free_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </FreeJoint>\n        </Joint>\n");
            return Some(xml);
        }
    }
    for (_, joint) in world.iter::<FixedJoint>() {
        if joint.body_b == child_key {
            let parent_name = body_names.get(&joint.body_a).map(|s| s.as_str()).unwrap_or("ground");
            let mut xml = format!("        <Joint>\n          <WeldJoint name=\"weld_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </WeldJoint>\n        </Joint>\n");
            return Some(xml);
        }
    }
    for (_, joint) in world.iter::<BallJoint>() {
        if joint.body_b == child_key {
            let parent_name = body_names.get(&joint.body_a).map(|s| s.as_str()).unwrap_or("ground");
            let mut xml = format!("        <Joint>\n          <BallJoint name=\"ball_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </BallJoint>\n        </Joint>\n");
            return Some(xml);
        }
    }
    for (_, joint) in world.iter::<UniversalJoint>() {
        if joint.body_b == child_key {
            let parent_name = body_names.get(&joint.body_a).map(|s| s.as_str()).unwrap_or("ground");
            let mut xml = format!("        <Joint>\n          <UniversalJoint name=\"universal_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </UniversalJoint>\n        </Joint>\n");
            return Some(xml);
        }
    }
    for (joint_key, joint) in world.iter::<CustomJoint>() {
        if joint.body_b == child_key {
            let parent_name = body_names.get(&joint.body_a).map(|s| s.as_str()).unwrap_or("ground");
            let mut xml = format!("        <Joint>\n          <CustomJoint name=\"custom_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");

            // Emit coordinates
            for coord_key in &joint.coordinates {
                if let Some(coord) = world.get::<JointCoordinate>(*coord_key) {
                    let coord_name = world.get::<Name>(*coord_key).map(|n| n.value.as_str()).unwrap_or("coord");
                    xml.push_str("            <CoordinateSet>\n");
                    xml.push_str(&format!(
                        "              <Coordinate name=\"{}\">\n",
                        escape_attr(coord_name)
                    ));
                    if coord.clamped {
                        xml.push_str(&format!("                <range_min>{}</range_min>\n", coord.range_min));
                        xml.push_str(&format!("                <range_max>{}</range_max>\n", coord.range_max));
                    }
                    xml.push_str(&format!("                <clamped>{}</clamped>\n", coord.clamped));
                    xml.push_str(&format!("                <locked>{}</locked>\n", coord.locked));
                    if let Some(ref pf) = coord.prescribed_function {
                        emit_joint_function(&mut xml, "prescribed_function", pf);
                    }
                    xml.push_str("              </Coordinate>\n");
                    xml.push_str("            </CoordinateSet>\n");
                }
            }

            // Emit SpatialTransform
            emit_spatial_transform(world, joint_key, &mut xml);

            xml.push_str("          </CustomJoint>\n        </Joint>\n");
            return Some(xml);
        }
    }

    None
}

/// Emit a joint XML element. (Currently unused — keep for future refactoring.)
#[allow(dead_code)]
fn emit_joint(
    world: &World,
    joint_key: EntityID,
    body_names: &HashMap<EntityID, String>,
) -> String {
    let mut xml = String::new();

    // Determine joint type and emit the appropriate element
    if let Some(joint) = world.get::<HingeJoint>(joint_key) {
        xml.push_str("        <Joint>\n");
        let parent_name = body_names
            .get(&joint.body_a)
            .map(|s| s.as_str())
            .unwrap_or("ground");
        xml.push_str(&format!("          <PinJoint name=\"hinge_joint\">\n"));
        xml.push_str(&format!(
            "            <parent_body>{}</parent_body>\n",
            escape_attr(parent_name)
        ));
        xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
        xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
        xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
        xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
        // ... (rest of joint XML emission)
        xml.push_str("        </Joint>\n");
    }

    xml
}

/// Emit a JointFunction XML element.
fn emit_joint_function(xml: &mut String, tag: &str, function: &JointFunction) {
    match function {
        JointFunction::Constant(c) => {
            xml.push_str(&format!("            <{}>\n", tag));
            xml.push_str("              <Constant>\n");
            xml.push_str(&format!("                <value>{}</value>\n", c));
            xml.push_str("              </Constant>\n");
            xml.push_str(&format!("            </{}>\n", tag));
        }
        JointFunction::Linear { slope, intercept } => {
            xml.push_str(&format!("            <{}>\n", tag));
            xml.push_str("              <LinearFunction>\n");
            xml.push_str(&format!("                <slope>{}</slope>\n", slope));
            xml.push_str(&format!("                <intercept>{}</intercept>\n", intercept));
            xml.push_str("              </LinearFunction>\n");
            xml.push_str(&format!("            </{}>\n", tag));
        }
        JointFunction::Polynomial { coefficients } => {
            xml.push_str(&format!("            <{}>\n", tag));
            xml.push_str("              <PolynomialFunction>\n");
            xml.push_str(&format!(
                "                <coefficients>{}</coefficients>\n",
                coefficients
                    .iter()
                    .map(|c| c.to_string())
                    .collect::<Vec<_>>()
                    .join(" ")
            ));
            xml.push_str("              </PolynomialFunction>\n");
            xml.push_str(&format!("            </{}>\n", tag));
        }
    }
}

/// Emit SpatialTransform XML for a CustomJoint.
fn emit_spatial_transform(world: &World, joint_key: EntityID, xml: &mut String) {
    // Find the SpatialTransform for this joint
    for (_st_key, st) in world.iter::<SpatialTransform>() {
        if st.joint == joint_key {
            xml.push_str("            <SpatialTransform>\n");

            let mut effects_by_slot: std::collections::HashMap<String, &CoordinateEffect> =
                std::collections::HashMap::new();

            for effect_key in &st.effects {
                if let Some(effect) = world.get::<CoordinateEffect>(*effect_key) {
                    let slot = match effect.component {
                        TransformComponent::RotationX => "rotation_x",
                        TransformComponent::RotationY => "rotation_y",
                        TransformComponent::RotationZ => "rotation_z",
                        TransformComponent::TranslationX => "translation_x",
                        TransformComponent::TranslationY => "translation_y",
                        TransformComponent::TranslationZ => "translation_z",
                    };
                    effects_by_slot.insert(slot.to_string(), effect);
                }
            }

            for slot_name in [
                "rotation_x",
                "rotation_y",
                "rotation_z",
                "translation_x",
                "translation_y",
                "translation_z",
            ] {
                if let Some(effect) = effects_by_slot.get(slot_name) {
                    xml.push_str(&format!("              <{}>\n", slot_name));
                    xml.push_str("                <CoordinateEffect>\n");
                    if let Some(_coord) = world.get::<JointCoordinate>(effect.coordinate) {
                        let coord_name = world.get::<Name>(effect.coordinate).map(|n| n.value.as_str()).unwrap_or("coord");
                        xml.push_str(&format!(
                            "                  <coordinate>{}</coordinate>\n",
                            escape_attr(coord_name)
                        ));
                    }
                    emit_joint_function(xml, "function", &effect.function);
                    xml.push_str("                </CoordinateEffect>\n");
                    xml.push_str(&format!("              </{}>\n", slot_name));
                } else {
                    xml.push_str(&format!("              <{}>\n", slot_name));
                    xml.push_str("                <CoordinateEffect>\n");
                    xml.push_str("                  <coordinate></coordinate>\n");
                    xml.push_str("                  <NullFunction/>\n");
                    xml.push_str("                </CoordinateEffect>\n");
                    xml.push_str(&format!("              </{}>\n", slot_name));
                }
            }

            xml.push_str("            </SpatialTransform>\n");
        }
    }
}

/// Escape special characters in XML attribute/text content.
fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
