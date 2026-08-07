// ── OpenSim .osim XML Exporter ────────────────────────
//
// Walks the melosim World and produces a valid .osim XML file.
// Works alongside the PyO3 importer to enable round-trip testing:
//   import (.osim) → World → export (.osim)
//
// The exporter handles the inverse of the importer: it walks the
// ChildOf hierarchy to find which joint connects each body to its parent.
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

    // ── Build hierarchy maps ──
    // For each body entity, find its joint (intermediate node in ChildOf hierarchy)
    // A joint is an entity that:
    //   1. Has no InertialProperties (not a body)
    //   2. Has children that DO have InertialProperties
    //   3. Has a parent via ChildOf (the parent frame)
    let body_names = build_body_name_map(world);
    let coord_names = build_coordinate_name_map(world);

    // ── BodySet ──
    xml.push_str("  <BodySet>\n");
    xml.push_str("    <objects>\n");

    for (body_key, _) in world.iter::<InertialProperties>() {
        let name = body_names
            .get(&body_key)
            .map(|s| s.as_str())
            .unwrap_or("unknown");

        // Detect ground: entity 0 or entity with no parent via ChildOf
        let parent = world.parent_of(body_key);
        let is_ground = body_key == EntityID(0) || parent.is_none();
        // Also check if parent is not a body (i.e., parent is a joint intermediate node)
        // If the parent has InertialProperties, this is a regular body
        // If the parent has no InertialProperties, the parent might be a joint node
        // If no parent at all, this is ground

        if is_ground {
            // Check if this is actually used as a parent in the hierarchy
            let has_children_with_inertials = world.children_of(body_key).iter()
                .any(|&child| world.get::<InertialProperties>(child).is_some()
                    || is_joint_entity(world, child));
            if !has_children_with_inertials && body_key == EntityID(0) {
                continue; // Skip unused ground
            }
            // Root body (ground) — skip if it has no joint connecting it to children
        }

        xml.push_str(&format!("      <Body name=\"{}\">\n", escape_attr(name)));
        xml.push_str(&emit_body_properties(world, body_key));

        // ── Emit joint if this body is the child of a joint entity ──
        if let Some(joint_xml) = find_parent_joint(world, body_key) {
            xml.push_str(&joint_xml);
        } else if !is_ground {
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
    let actuator_count = world.count::<CoordinateActuator>();
    if muscle_count + actuator_count > 0 {
        xml.push_str("  <ForceSet>\n");
        xml.push_str("    <objects>\n");
        xml.push_str(&emit_muscles(world, &body_names, &coord_names));
        for (act_key, act) in world.iter::<CoordinateActuator>() {
            let act_name = world.get::<Name>(act_key)
                .map(|n| n.value.as_str())
                .unwrap_or("actuator");
            let coord_name = coord_names
                .get(&act.coordinate)
                .map(|s| s.as_str())
                .unwrap_or("unknown");
            xml.push_str(&format!(
                "      <CoordinateActuator name=\"{}\">\n",
                escape_attr(act_name)
            ));
            xml.push_str(&format!(
                "        <coordinate>{}</coordinate>\n",
                escape_attr(coord_name)
            ));
            xml.push_str(&format!(
                "        <optimal_force>{}</optimal_force>\n",
                act.optimal_force
            ));
            xml.push_str(&format!(
                "        <min_control>{}</min_control>\n",
                act.min_control
            ));
            xml.push_str(&format!(
                "        <max_control>{}</max_control>\n",
                act.max_control
            ));
            xml.push_str("      </CoordinateActuator>\n");
        }
        xml.push_str("    </objects>\n");
        xml.push_str("  </ForceSet>\n");
    }

    // ── MarkerSet ──
    // Sites are now entities with Position + ChildOf (no Site marker)
    // Identify them as: has Position, has ChildOf parent that is a body,
    // has NO InertialProperties, NO JointCoordinate
    let markers = find_site_entities(world);
    if !markers.is_empty() {
        xml.push_str("  <MarkerSet>\n");
        xml.push_str("    <objects>\n");
        for &site_key in &markers {
            let marker_name = world.get::<Name>(site_key).map(|n| n.value.as_str()).unwrap_or("marker");
            xml.push_str(&format!(
                "      <Marker name=\"{}\">\n",
                escape_attr(marker_name)
            ));
            // Get parent body name for marker
            let parent_name = world.get::<ChildOf>(site_key)
                .and_then(|co| {
                    // Walk up to find the nearest body ancestor
                    find_body_ancestor(world, co.parent)
                })
                .and_then(|body| body_names.get(&body))
                .map(|s| s.as_str())
                .unwrap_or("ground");
            xml.push_str(&format!(
                "        <body>{}</body>\n",
                escape_attr(parent_name)
            ));
            let pos = world.get::<Position>(site_key);
            let (px, py, pz) = pos.map(|p| (p.x, p.y, p.z)).unwrap_or((0.0, 0.0, 0.0));
            xml.push_str(&format!(
                "        <location>{} {} {}</location>\n",
                px, py, pz
            ));
            xml.push_str("        <fixed>true</fixed>\n");
            xml.push_str("      </Marker>\n");
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

/// Check if an entity is a "joint" intermediate node (no InertialProperties,
/// but has children with InertialProperties or is in a parent-child chain).
fn is_joint_entity(world: &World, entity: EntityID) -> bool {
    // A joint entity has no InertialProperties
    if world.get::<InertialProperties>(entity).is_some() {
        return false;
    }
    // A joint entity has at least one child with InertialProperties
    // OR is an intermediate node between bodies
    for child in world.children_of(entity) {
        if world.get::<InertialProperties>(child).is_some() {
            return true;
        }
    }
    false
}

/// Walk up the ChildOf hierarchy to find the nearest body ancestor.
fn find_body_ancestor(world: &World, entity: EntityID) -> Option<EntityID> {
    if world.get::<InertialProperties>(entity).is_some() {
        return Some(entity);
    }
    world.parent_of(entity).and_then(|p| find_body_ancestor(world, p))
}

/// Find entities that represent sites (have Position + ChildOf, no InertialProperties,
/// no JointCoordinate, not a joint intermediate node).
fn find_site_entities(world: &World) -> Vec<EntityID> {
    let mut sites = Vec::new();
    for (entity, _pos) in world.iter::<Position>() {
        // Must have ChildOf
        if world.get::<ChildOf>(entity).is_none() {
            continue;
        }
        // Must not be a body
        if world.get::<InertialProperties>(entity).is_some() {
            continue;
        }
        // Must not be a joint coordinate
        if world.get::<JointCoordinate>(entity).is_some() {
            continue;
        }
        // Must not be a coordinate effect
        if world.get::<CoordinateEffect>(entity).is_some() {
            continue;
        }
        // Must not have Rotation (joints/bodies have rotation)
        if world.get::<Rotation>(entity).is_some() {
            continue;
        }
        // Must not be a joint intermediate node
        if is_joint_entity(world, entity) {
            continue;
        }
        // Must not be a material, muscle, display geometry, etc.
        if world.get::<Material>(entity).is_some() {
            continue;
        }
        if world.get::<Muscle>(entity).is_some() {
            continue;
        }
        if world.get::<DisplayGeometry>(entity).is_some() {
            continue;
        }
        if world.get::<WrapGeom>(entity).is_some() {
            continue;
        }
        sites.push(entity);
    }
    sites
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

/// Infer the joint kind from the coordinate/effect configuration of a joint entity.
/// Returns a string matching OpenSim joint type names.
fn infer_joint_kind(world: &World, joint_entity: EntityID) -> &'static str {
    // Count coordinates that are children of this joint entity
    let coords: Vec<EntityID> = world.children_of(joint_entity).iter()
        .filter(|&&c| world.get::<JointCoordinate>(c).is_some())
        .copied()
        .collect();

    let n_coords = coords.len();
    match n_coords {
        0 => "WeldJoint",
        1 => {
            // Check the effect type
            let coord = coords[0];
            for effect in world.children_of(coord) {
                if let Some(ce) = world.get::<CoordinateEffect>(effect) {
                    return match &ce.component {
                        TransformComponent::RotationAboutAxis(_)
                        | TransformComponent::RotationX
                        | TransformComponent::RotationY
                        | TransformComponent::RotationZ => "PinJoint",
                        TransformComponent::TranslationAlongAxis(_)
                        | TransformComponent::TranslationX
                        | TransformComponent::TranslationY
                        | TransformComponent::TranslationZ => "SlideJoint",
                    };
                }
            }
            "PinJoint"
        }
        2 => "UniversalJoint",
        3 => "BallJoint",
        6 => "FreeJoint",
        _ => "CustomJoint",
    }
}

/// Collect all CoordinateEffects that belong to a joint (via the hierarchy).
fn collect_joint_effects(world: &World, joint_entity: EntityID) -> Vec<(EntityID, &CoordinateEffect)> {
    let mut effects = Vec::new();
    for &coord in &world.children_of(joint_entity) {
        if world.get::<JointCoordinate>(coord).is_some() {
            for effect_entity in world.children_of(coord) {
                if let Some(ce) = world.get::<CoordinateEffect>(effect_entity) {
                    effects.push((effect_entity, ce));
                }
            }
        }
    }
    effects
}

/// Find the parent joint entity for a body and return its XML.
/// Walks the ChildOf hierarchy: body → parent (should be joint entity) → parent's parent (parent body)
fn find_parent_joint(world: &World, child_key: EntityID) -> Option<String> {
    let body_names = build_body_name_map(world);

    // The body's parent in ChildOf should be a joint entity
    let joint_entity = world.parent_of(child_key)?;
    if !is_joint_entity(world, joint_entity) {
        return None; // Parent is not a joint entity
    }

    // The joint entity's parent is the parent body
    let parent_body = world.parent_of(joint_entity)?;
    let parent_name = body_names.get(&parent_body).map(|s| s.as_str()).unwrap_or("ground");

    let kind = infer_joint_kind(world, joint_entity);
    let effects = collect_joint_effects(world, joint_entity);

    match kind {
        "PinJoint" => {
            let mut xml = format!("        <Joint>\n          <PinJoint name=\"hinge_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");

            // Emit coordinates
            for &coord_key in &world.children_of(joint_entity) {
                if let Some(coord) = world.get::<JointCoordinate>(coord_key) {
                    let coord_name = world.get::<Name>(coord_key).map(|n| n.value.as_str()).unwrap_or("coord");
                    xml.push_str("            <CoordinateSet>\n");
                    xml.push_str(&format!(
                        "              <Coordinate name=\"{}\">\n",
                        escape_attr(coord_name)
                    ));
                    // Find axis from RotationAboutAxis effect
                    let mut axis = [0.0f64; 3];
                    for effect_entity in world.children_of(coord_key) {
                        if let Some(effect) = world.get::<CoordinateEffect>(effect_entity) {
                            if let TransformComponent::RotationAboutAxis(a) = effect.component {
                                axis = a;
                            }
                        }
                    }
                    xml.push_str(&format!("                <axis>{} {} {}</axis>\n", axis[0], axis[1], axis[2]));
                    if coord.clamped {
                        xml.push_str(&format!("                <range_min>{}</range_min>\n", coord.range_min));
                        xml.push_str(&format!("                <range_max>{}</range_max>\n", coord.range_max));
                    }
                    xml.push_str("              </Coordinate>\n");
                    xml.push_str("            </CoordinateSet>\n");
                }
            }

            xml.push_str("            <reverse>false</reverse>\n");
            xml.push_str("          </PinJoint>\n        </Joint>\n");
            Some(xml)
        }
        "FreeJoint" => {
            let mut xml = format!("        <Joint>\n          <FreeJoint name=\"free_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </FreeJoint>\n        </Joint>\n");
            Some(xml)
        }
        "WeldJoint" => {
            let mut xml = format!("        <Joint>\n          <WeldJoint name=\"weld_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </WeldJoint>\n        </Joint>\n");
            Some(xml)
        }
        "BallJoint" => {
            let mut xml = format!("        <Joint>\n          <BallJoint name=\"ball_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </BallJoint>\n        </Joint>\n");
            Some(xml)
        }
        "UniversalJoint" => {
            let mut xml = format!("        <Joint>\n          <UniversalJoint name=\"universal_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");
            xml.push_str("          </UniversalJoint>\n        </Joint>\n");
            Some(xml)
        }
        "CustomJoint" => {
            let mut xml = format!("        <Joint>\n          <CustomJoint name=\"custom_joint\">\n");
            xml.push_str(&format!("            <parent_body>{}</parent_body>\n", escape_attr(parent_name)));
            xml.push_str("            <location_in_parent>0 0 0</location_in_parent>\n");
            xml.push_str("            <orientation_in_parent>0 0 0</orientation_in_parent>\n");
            xml.push_str("            <location_in_child>0 0 0</location_in_child>\n");
            xml.push_str("            <orientation_in_child>0 0 0</orientation_in_child>\n");

            // Emit coordinates
            for &coord_key in &world.children_of(joint_entity) {
                if let Some(coord) = world.get::<JointCoordinate>(coord_key) {
                    let coord_name = world.get::<Name>(coord_key).map(|n| n.value.as_str()).unwrap_or("coord");
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

            // Emit SpatialTransform from effects
            emit_spatial_transform_from_effects(world, joint_entity, &effects, &mut xml);

            xml.push_str("          </CustomJoint>\n        </Joint>\n");
            Some(xml)
        }
        _ => None,
    }
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

/// Emit SpatialTransform XML for a CustomJoint from collected effects.
fn emit_spatial_transform_from_effects(
    world: &World,
    joint_entity: EntityID,
    effects: &[(EntityID, &CoordinateEffect)],
    xml: &mut String,
) {
    xml.push_str("            <SpatialTransform>\n");

    let mut effects_by_slot: std::collections::HashMap<String, &CoordinateEffect> =
        std::collections::HashMap::new();

    for (_effect_key, effect) in effects {
        let slot = match &effect.component {
            TransformComponent::RotationX => "rotation_x",
            TransformComponent::RotationY => "rotation_y",
            TransformComponent::RotationZ => "rotation_z",
            TransformComponent::TranslationX => "translation_x",
            TransformComponent::TranslationY => "translation_y",
            TransformComponent::TranslationZ => "translation_z",
            TransformComponent::RotationAboutAxis(_) => "rotation_about_axis",
            TransformComponent::TranslationAlongAxis(_) => "translation_along_axis",
        };
        effects_by_slot.insert(slot.to_string(), effect);
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
            // Find the coordinate name: effect's parent should be its coordinate
            let coord_name = "coord"; // Default
            xml.push_str(&format!("              <{}>\n", slot_name));
            xml.push_str("                <CoordinateEffect>\n");
            xml.push_str(&format!(
                "                  <coordinate>{}</coordinate>\n",
                escape_attr(coord_name)
            ));
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

/// Escape special characters in XML attribute/text content.
fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
