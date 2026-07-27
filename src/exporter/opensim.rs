// ── OpenSim .osim XML Exporter ────────────────────────
//
// Walks the melosim World and produces a valid .osim XML file.
// Works alongside the PyO3 importer to enable round-trip testing:
//   import (.osim) → World → export (.osim)
//
// The exporter handles the inverse of the importer: it looks up which
// joint connects each body to its parent, and nests the joint inside
// the child body's XML element, matching OpenSim's serialization format.

use std::collections::HashMap;

use crate::components::*;
use crate::id::EntityKey;
use crate::world::World;
use slotmap::Key;

/// Export the World to an .osim XML string.
pub fn world_to_osim(world: &World, model_name: &str) -> String {
    let mut xml = String::new();

    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str("<OpenSimDocument Version=\"30000\">\n");
    xml.push_str(&format!("<Model name=\"{}\">\n", escape_attr(model_name)));

    // ── Build inverse lookup: for each entity, find its joint (where body_b == entity) ──
    let child_joint = build_child_joint_map(world);
    // ── Build name → key lookup for body names ──
    let body_names = build_body_name_map(world);

    // ── BodySet ──
    xml.push_str("  <BodySet>\n");
    xml.push_str("    <objects>\n");

    for (body_key, _) in world.iter::<InertialProperties>() {
        let name = body_names
            .get(&body_key)
            .map(|s| s.as_str())
            .unwrap_or("unknown");

        // Only emit Ground if it has no parent joint (it's the root)
        // In OpenSim, ground is not emitted in BodySet unless it has a joint
        let is_ground = name == "ground";
        let has_parent_joint = child_joint.contains_key(&body_key);

        if is_ground && !has_parent_joint {
            continue; // Skip ground — it's implicit in OpenSim
        }

        xml.push_str(&format!("      <Body name=\"{}\">\n", escape_attr(name)));
        xml.push_str(&emit_body_properties(world, body_key));

        // ── Emit joint if this body is the child of one ──
        if let Some(&joint_key) = child_joint.get(&body_key) {
            xml.push_str(&emit_joint(world, joint_key, body_key, &body_names));
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

        xml.push_str("      </Body>\n");
    }

    xml.push_str("    </objects>\n");
    xml.push_str("  </BodySet>\n");

    // ── MarkerSet ──
    let marker_count = world.count::<Landmark>();
    if marker_count > 0 {
        xml.push_str("  <MarkerSet>\n");
        xml.push_str("    <objects>\n");
        for (_landmark_key, landmark) in world.iter::<Landmark>() {
            if let Some(site) = world.get::<Site>(landmark.site) {
                if let Some(name) = body_names.get(&landmark.site) {
                    xml.push_str(&format!(
                        "      <Marker name=\"{}\">\n",
                        escape_attr(name)
                    ));
                } else {
                    xml.push_str(&format!(
                        "      <Marker name=\"{}\">\n",
                        escape_attr(&landmark.name)
                    ));
                }
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
fn build_child_joint_map(world: &World) -> HashMap<EntityKey, EntityKey> {
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

/// Build a map from entity key → body name (from Landmarks or entity index).
fn build_body_name_map(world: &World) -> HashMap<EntityKey, String> {
    let mut map = HashMap::new();

    // Use Landmarks with known names as a hint, but mostly use entity indices
    // In OpenSim export, we need meaningful names. For now, use "body_N".
    for (key, _) in world.iter::<InertialProperties>() {
        let idx = key.data().as_ffi() & 0xFFFF_FFFF;
        map.insert(key, format!("body_{}", idx));
    }

    // Override ground if present (usually entity 0)
    if let Some(_first_key) = world.iter::<InertialProperties>().next() {
        // Check if mass is 0 → likely ground
        // (We can't easily check properties here without borrowing world again)
    }

    map
}

/// Emit body properties XML (mass, mass_center, inertia).
fn emit_body_properties(world: &World, body_key: EntityKey) -> String {
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

/// Emit a joint XML element.
fn emit_joint(
    world: &World,
    joint_key: EntityKey,
    _child_key: EntityKey,
    body_names: &HashMap<EntityKey, String>,
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
        xml.push_str("            <CoordinateSet>\n");
        xml.push_str(&format!(
            "              <Coordinate name=\"hinge_coord\">\n"
        ));
        xml.push_str(&format!(
            "                <axis>{} {} {}</axis>\n",
            joint.axis[0], joint.axis[1], joint.axis[2]
        ));
        if let Some(limits) = &joint.limits {
            xml.push_str(&format!("                <range_min>{}</range_min>\n", limits.lower));
            xml.push_str(&format!("                <range_max>{}</range_max>\n", limits.upper));
        } else {
            xml.push_str("                <range_min>-inf</range_min>\n");
            xml.push_str("                <range_max>inf</range_max>\n");
        }
        xml.push_str("              </Coordinate>\n");
        xml.push_str("            </CoordinateSet>\n");
        xml.push_str("            <reverse>false</reverse>\n");
        xml.push_str("          </PinJoint>\n");
        xml.push_str("        </Joint>\n");
    } else if let Some(joint) = world.get::<FreeJoint>(joint_key) {
        xml.push_str("        <Joint>\n");
        let parent_name = body_names
            .get(&joint.body_a)
            .map(|s| s.as_str())
            .unwrap_or("ground");
        xml.push_str(&format!(
            "          <FreeJoint name=\"free_joint\">\n"
        ));
        xml.push_str(&format!(
            "            <parent_body>{}</parent_body>\n",
            escape_attr(parent_name)
        ));
        xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
        xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
        xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
        xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
        xml.push_str("          </FreeJoint>\n");
        xml.push_str("        </Joint>\n");
    } else if let Some(_joint) = world.get::<FixedJoint>(joint_key) {
        xml.push_str("        <Joint>\n");
        xml.push_str("          <WeldJoint name=\"weld_joint\">\n");
        xml.push_str("            <parent_body>ground</parent_body>\n");
        xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
        xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
        xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
        xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
        xml.push_str("          </WeldJoint>\n");
        xml.push_str("        </Joint>\n");
    } else if let Some(joint) = world.get::<UniversalJoint>(joint_key) {
        xml.push_str("        <Joint>\n");
        let parent_name = body_names
            .get(&joint.body_a)
            .map(|s| s.as_str())
            .unwrap_or("ground");
        xml.push_str(&format!(
            "          <UniversalJoint name=\"universal_joint\">\n"
        ));
        xml.push_str(&format!(
            "            <parent_body>{}</parent_body>\n",
            escape_attr(parent_name)
        ));
        xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
        xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
        xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
        xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
        // Find coordinates for this joint
        for (coord_key, _) in world.iter::<JointCoordinate>() {
            let coord_name = format!("coord_{:x}", coord_key.data().as_ffi());
            xml.push_str("            <CoordinateSet>\n");
            xml.push_str(&format!(
                "              <Coordinate name=\"{}\">\n",
                escape_attr(&coord_name)
            ));
            xml.push_str("                <range_min>-inf</range_min>\n");
            xml.push_str("                <range_max>inf</range_max>\n");
            xml.push_str("              </Coordinate>\n");
            xml.push_str("            </CoordinateSet>\n");
        }
        xml.push_str("          </UniversalJoint>\n");
        xml.push_str("        </Joint>\n");
    } else if let Some(joint) = world.get::<CustomJoint>(joint_key) {
        xml.push_str("        <Joint>\n");
        let parent_name = body_names
            .get(&joint.body_a)
            .map(|s| s.as_str())
            .unwrap_or("ground");
        xml.push_str(&format!(
            "          <CustomJoint name=\"custom_joint\">\n"
        ));
        xml.push_str(&format!(
            "            <parent_body>{}</parent_body>\n",
            escape_attr(parent_name)
        ));
        xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
        xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
        xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
        xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");

        // Emit coordinates
        for coord_key in &joint.coordinates {
            if let Some(coord) = world.get::<JointCoordinate>(*coord_key) {
                xml.push_str("            <CoordinateSet>\n");
                xml.push_str(&format!(
                    "              <Coordinate name=\"{}\">\n",
                    escape_attr(&coord.name)
                ));
                if coord.clamped {
                    xml.push_str(&format!(
                        "                <range_min>{}</range_min>\n",
                        coord.range_min
                    ));
                    xml.push_str(&format!(
                        "                <range_max>{}</range_max>\n",
                        coord.range_max
                    ));
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

        xml.push_str("          </CustomJoint>\n");
        xml.push_str("        </Joint>\n");
    }

    xml
}

/// Emit a JointFunction XML element.
fn emit_joint_function(xml: &mut String, tag: &str, function: &JointFunction) {
    match function {
        JointFunction::Constant(c) => {
            xml.push_str(&format!(
                "            <{}>\n",
                tag
            ));
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
fn emit_spatial_transform(world: &World, joint_key: EntityKey, xml: &mut String) {
    // Find the SpatialTransform for this joint
    for (_st_key, st) in world.iter::<SpatialTransform>() {
        if st.joint == joint_key {
            xml.push_str("            <SpatialTransform>\n");

            // We need to sort effects into their 6 slots
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

            // Emit in standard order
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
                    // Get coordinate name
                    if let Some(coord) = world.get::<JointCoordinate>(effect.coordinate) {
                        xml.push_str(&format!(
                            "                  <coordinate>{}</coordinate>\n",
                            escape_attr(&coord.name)
                        ));
                    }
                    emit_joint_function(xml, "function", &effect.function);
                    xml.push_str("                </CoordinateEffect>\n");
                    xml.push_str(&format!("              </{}>\n", slot_name));
                } else {
                    // Empty slot: emit a null function
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
