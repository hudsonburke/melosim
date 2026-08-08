// ── OpenSim .osim XML Exporter ────────────────────────

use std::collections::HashMap;

use crate::components::*;
use crate::world::World;
use bevy_ecs::prelude::Entity;

pub fn world_to_osim(world: &mut World, model_name: &str) -> String {
    let mut xml = String::new();

    xml.push_str("<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    xml.push_str("<OpenSimDocument Version=\"30000\">\n");
    xml.push_str(&format!("<Model name=\"{}\">\n", escape_attr(model_name)));

    let body_names = build_body_name_map(world);
    let coord_names = build_coordinate_name_map(world);

    // ── BodySet ──
    xml.push_str("  <BodySet>\n");
    xml.push_str("    <objects>\n");

    let body_entities: Vec<Entity> = {
        let mut query = world.query::<Entity>();
        query.iter(world).collect()
    };

    for body_key in body_entities {
        if world.get::<InertialProperties>(body_key).is_none() { continue; }
        let name = body_names.get(&body_key).map(|s| s.as_str()).unwrap_or("unknown");
        let parent = world.get::<ChildOf>(body_key).map(|co| co.parent);
        let is_ground = parent.is_none();

        if is_ground {
            let child_entities = children_of(world, body_key);
            let has_children_with_inertials = child_entities.iter()
                .any(|&child| world.get::<InertialProperties>(child).is_some()
                    || is_joint_entity(world, child));
            if !has_children_with_inertials {
                continue;
            }
        }

        xml.push_str(&format!("      <Body name=\"{}\">\n", escape_attr(name)));
        xml.push_str(&emit_body_properties(world, body_key));

        if let Some(joint_xml) = find_parent_joint(world, body_key) {
            xml.push_str(&joint_xml);
        } else if !is_ground {
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

        xml.push_str(&emit_body_display_geometry(world, body_key));
        xml.push_str("      </Body>\n");
    }

    xml.push_str("    </objects>\n");
    xml.push_str("  </BodySet>\n");

    // ── ForceSet ──
    let muscle_count = world.query::<&Muscle>().iter(&world).count();
    let actuator_count = world.query::<&CoordinateActuator>().iter(&world).count();
    if muscle_count + actuator_count > 0 {
        xml.push_str("  <ForceSet>\n");
        xml.push_str("    <objects>\n");
        xml.push_str(&emit_muscles(world, &body_names, &coord_names));

        let actuator_entities: Vec<(Entity, CoordinateActuator)> = {
            let mut query = world.query::<(Entity, &CoordinateActuator)>();
            query.iter(world).map(|(e, a)| (e, a.clone())).collect()
        };
        for (act_key, act) in actuator_entities {
            let act_name = world.get::<Name>(act_key).map(|n| n.value.as_str()).unwrap_or("actuator");
            let coord_name = coord_names.get(&act.coordinate).map(|s| s.as_str()).unwrap_or("unknown");
            xml.push_str(&format!("      <CoordinateActuator name=\"{}\">\n", escape_attr(act_name)));
            xml.push_str(&format!("        <coordinate>{}</coordinate>\n", escape_attr(coord_name)));
            xml.push_str(&format!("        <optimal_force>{}</optimal_force>\n", act.optimal_force));
            xml.push_str(&format!("        <min_control>{}</min_control>\n", act.min_control));
            xml.push_str(&format!("        <max_control>{}</max_control>\n", act.max_control));
            xml.push_str("      </CoordinateActuator>\n");
        }
        xml.push_str("    </objects>\n");
        xml.push_str("  </ForceSet>\n");
    }

    // ── MarkerSet ──
    let markers = find_site_entities(world);
    if !markers.is_empty() {
        xml.push_str("  <MarkerSet>\n");
        xml.push_str("    <objects>\n");
        for &site_key in &markers {
            let marker_name = world.get::<Name>(site_key).map(|n| n.value.as_str()).unwrap_or("marker");
            xml.push_str(&format!("      <Marker name=\"{}\">\n", escape_attr(marker_name)));
            let parent = world.get::<ChildOf>(site_key).map(|co| co.parent);
            let parent_name = parent
                .and_then(|p| find_body_ancestor(world, p))
                .and_then(|body| body_names.get(&body))
                .map(|s| s.as_str())
                .unwrap_or("ground");
            xml.push_str(&format!("        <body>{}</body>\n", escape_attr(parent_name)));
            let pos = world.get::<Position>(site_key);
            let (px, py, pz) = pos.map(|p| (p.x, p.y, p.z)).unwrap_or((0.0, 0.0, 0.0));
            xml.push_str(&format!("        <location>{} {} {}</location>\n", px, py, pz));
            xml.push_str("        <fixed>true</fixed>\n");
            xml.push_str("      </Marker>\n");
        }
        xml.push_str("    </objects>\n");
        xml.push_str("  </MarkerSet>\n");
    }

    // ── WrapObjectSet ──
    let wrap_count = world.query::<&WrapGeom>().iter(&world).count();
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

pub fn write_osim(world: &mut World, path: &str, model_name: &str) -> Result<(), String> {
    let xml = world_to_osim(world, model_name);
    std::fs::write(path, &xml).map_err(|e| format!("Failed to write '{}': {}", path, e))
}

// ── Helpers ───────────────────────────────────────────

fn children_of(world: &mut World, entity: Entity) -> Vec<Entity> {
    let mut query = world.query::<(Entity, &ChildOf)>();
    query.iter(world)
        .filter(|(_, co)| co.parent == entity)
        .map(|(e, _)| e)
        .collect()
}

fn is_joint_entity(world: &mut World, entity: Entity) -> bool {
    if world.get::<InertialProperties>(entity).is_some() {
        return false;
    }
    let child_entities = children_of(world, entity);
    for child in child_entities {
        if world.get::<InertialProperties>(child).is_some() {
            return true;
        }
    }
    false
}

fn find_body_ancestor(world: &mut World, entity: Entity) -> Option<Entity> {
    if world.get::<InertialProperties>(entity).is_some() {
        return Some(entity);
    }
    let parent = world.get::<ChildOf>(entity).map(|co| co.parent);
    parent.and_then(|p| find_body_ancestor(world, p))
}

fn find_site_entities(world: &mut World) -> Vec<Entity> {
    let mut sites = Vec::new();
    let entities: Vec<(Entity, Position)> = {
        let mut query = world.query::<(Entity, &Position)>();
        query.iter(world).map(|(e, p)| (e, p.clone())).collect()
    };
    for (entity, _pos) in entities {
        if world.get::<ChildOf>(entity).is_none() { continue; }
        if world.get::<InertialProperties>(entity).is_some() { continue; }
        if world.get::<JointCoordinate>(entity).is_some() { continue; }
        if world.get::<CoordinateEffect>(entity).is_some() { continue; }
        if world.get::<Rotation>(entity).is_some() { continue; }
        if is_joint_entity(world, entity) { continue; }
        if world.get::<Material>(entity).is_some() { continue; }
        if world.get::<Muscle>(entity).is_some() { continue; }
        if world.get::<DisplayGeometry>(entity).is_some() { continue; }
        if world.get::<WrapGeom>(entity).is_some() { continue; }
        sites.push(entity);
    }
    sites
}

fn build_body_name_map(world: &mut World) -> HashMap<Entity, String> {
    let mut map = HashMap::new();
    let name_entities: Vec<(Entity, Name)> = {
        let mut query = world.query::<(Entity, &Name)>();
        query.iter(world).map(|(e, n)| (e, n.clone())).collect()
    };
    for (id, name) in name_entities {
        if world.get::<InertialProperties>(id).is_some() {
            map.insert(id, name.value.clone());
        }
    }
    map
}

fn build_coordinate_name_map(world: &mut World) -> HashMap<Entity, String> {
    let mut map = HashMap::new();
    let name_entities: Vec<(Entity, Name)> = {
        let mut query = world.query::<(Entity, &Name)>();
        query.iter(world).map(|(e, n)| (e, n.clone())).collect()
    };
    for (id, name) in name_entities {
        if world.get::<JointCoordinate>(id).is_some() {
            map.insert(id, name.value.clone());
        }
    }
    map
}

fn emit_body_properties(world: &mut World, body_key: Entity) -> String {
    let mut xml = String::new();
    if let Some(inertial) = world.get::<InertialProperties>(body_key) {
        xml.push_str(&format!("        <mass>{}</mass>\n", inertial.mass));
        xml.push_str(&format!("        <mass_center>{} {} {}</mass_center>\n",
            inertial.com[0], inertial.com[1], inertial.com[2]));
        xml.push_str(&format!("        <inertia>{} {} {} {} {} {}</inertia>\n",
            inertial.inertia[0], inertial.inertia[1], inertial.inertia[2],
            inertial.inertia[3], inertial.inertia[4], inertial.inertia[5]));
    }
    xml
}

fn emit_body_display_geometry(world: &mut World, body_key: Entity) -> String {
    let mut xml = String::new();
    let mut has_geom = false;
    let mut geom_xml = String::new();

    let dg_entities: Vec<(Entity, DisplayGeometry)> = {
        let mut query = world.query::<(Entity, &DisplayGeometry)>();
        query.iter(world).map(|(e, g)| (e, g.clone())).collect()
    };

    for (_key, dg) in dg_entities {
        if dg.body == body_key {
            has_geom = true;
            geom_xml.push_str("              <DisplayGeometry>\n");
            if let Some(ref mesh_file) = dg.mesh_file {
                geom_xml.push_str(&format!("                <mesh_file>{}</mesh_file>\n", escape_attr(mesh_file)));
            }
            geom_xml.push_str(&format!("                <scale>{} {} {}</scale>\n", dg.scale[0], dg.scale[1], dg.scale[2]));
            geom_xml.push_str(&format!("                <color>{} {} {}</color>\n", dg.color[0], dg.color[1], dg.color[2]));
            geom_xml.push_str(&format!("                <opacity>{}</opacity>\n", dg.opacity));
            geom_xml.push_str("                <transform>\n");
            geom_xml.push_str(&format!("                  <translation>{} {} {}</translation>\n",
                dg.transform.translation.x, dg.transform.translation.y, dg.transform.translation.z));
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

fn emit_muscles(
    world: &mut World,
    body_names: &HashMap<Entity, String>,
    coord_names: &HashMap<Entity, String>,
) -> String {
    let mut xml = String::new();

    let muscle_entities: Vec<(Entity, Muscle)> = {
        let mut query = world.query::<(Entity, &Muscle)>();
        query.iter(world).map(|(e, m)| (e, m.clone())).collect()
    };
    let all_params: Vec<Millard2012Params> = {
        let mut query = world.query::<&Millard2012Params>();
        query.iter(world).map(|p| p.clone()).collect()
    };
    let all_paths: Vec<MusclePath> = {
        let mut query = world.query::<&MusclePath>();
        query.iter(world).map(|p| p.clone()).collect()
    };

    for (muscle_key, _muscle) in muscle_entities {
        let path = all_paths.iter().find(|p| p.muscle == muscle_key).cloned();
        let params = all_params.iter().find(|p| p.muscle == muscle_key).cloned();

        let muscle_name = world.get::<Name>(muscle_key).map(|n| n.value.as_str()).unwrap_or("muscle");
        xml.push_str(&format!("        <Millard2012EquilibriumMuscle name=\"{}\">\n", escape_attr(muscle_name)));

        if let Some(ref p) = params {
            xml.push_str(&format!("          <max_isometric_force>{}</max_isometric_force>\n", p.max_isometric_force));
            xml.push_str(&format!("          <optimal_fiber_length>{}</optimal_fiber_length>\n", p.optimal_fiber_length));
            xml.push_str(&format!("          <tendon_slack_length>{}</tendon_slack_length>\n", p.tendon_slack_length));
            xml.push_str(&format!("          <pennation_angle_at_optimal>{}</pennation_angle_at_optimal>\n", p.pennation_angle_at_optimal));
            xml.push_str(&format!("          <max_contraction_velocity>{}</max_contraction_velocity>\n", p.max_contraction_velocity));
            xml.push_str(&format!("          <activation_time_constant>{}</activation_time_constant>\n", p.activation_time_constant));
            xml.push_str(&format!("          <deactivation_time_constant>{}</deactivation_time_constant>\n", p.deactivation_time_constant));
            xml.push_str(&format!("          <minimum_activation>{}</minimum_activation>\n", p.minimum_activation));
            xml.push_str(&format!("          <fiber_damping>{}</fiber_damping>\n", p.fiber_damping));
            xml.push_str(&format!("          <ignore_activation_dynamics>{}</ignore_activation_dynamics>\n", p.ignore_activation_dynamics));
            xml.push_str(&format!("          <ignore_tendon_compliance>{}</ignore_tendon_compliance>\n", p.ignore_tendon_compliance));
        }

        xml.push_str("          <GeometryPath>\n");
        xml.push_str("            <PathPointSet>\n");
        xml.push_str("              <objects>\n");

        if let Some(ref p) = path {
            for (i, pt) in p.points.iter().enumerate() {
                let body_name = match pt {
                    PathPoint::BodyFixed { body, .. } | PathPoint::Moving { body, .. } => {
                        body_names.get(body).map(|s| s.as_str()).unwrap_or("ground")
                    }
                };
                xml.push_str(&format!("                <PathPoint name=\"pp{}\">\n", i + 1));
                xml.push_str(&format!("                  <body>{}</body>\n", escape_attr(body_name)));
                match pt {
                    PathPoint::BodyFixed { location, .. } => {
                        xml.push_str(&format!("                  <location>{} {} {}</location>\n",
                            location[0], location[1], location[2]));
                    }
                    PathPoint::Moving { coordinate, location_functions, .. } => {
                        let coord_name = coord_names.get(coordinate).map(|s| s.as_str()).unwrap_or("unknown_coord");
                        xml.push_str(&format!("                  <coordinate>{}</coordinate>\n", escape_attr(coord_name)));
                        let loc_at_zero: [f64; 3] = std::array::from_fn(|axis| {
                            location_functions[axis].last().copied().unwrap_or(0.0)
                        });
                        xml.push_str(&format!("                  <location>{} {} {}</location>\n",
                            loc_at_zero[0], loc_at_zero[1], loc_at_zero[2]));
                    }
                }
                xml.push_str("                </PathPoint>\n");
            }
        }

        xml.push_str("              </objects>\n");
        xml.push_str("            </PathPointSet>\n");
        xml.push_str("          </GeometryPath>\n");
        xml.push_str("        </Millard2012EquilibriumMuscle>\n");
    }
    xml
}

fn emit_wrap_objects(
    world: &mut World,
    body_names: &HashMap<Entity, String>,
) -> String {
    let mut xml = String::new();

    let wg_entities: Vec<(Entity, WrapGeom)> = {
        let mut query = world.query::<(Entity, &WrapGeom)>();
        query.iter(world).map(|(e, w)| (e, w.clone())).collect()
    };

    for (wg_key, wg) in wg_entities {
        let body_name = body_names.get(&wg.body).map(|s| s.as_str()).unwrap_or("ground");
        let elem_name = match wg.geom_type {
            WrapGeomType::Sphere { .. } => "WrapSphere",
            WrapGeomType::Cylinder { .. } => "WrapCylinder",
            WrapGeomType::Ellipsoid { .. } => "WrapEllipsoid",
        };
        let wrap_name = world.get::<Name>(wg_key).map(|n| n.value.as_str()).unwrap_or("wrap");
        xml.push_str(&format!("          <{} name=\"{}\">\n", elem_name, escape_attr(wrap_name)));
        xml.push_str(&format!("            <frame>{}</frame>\n", escape_attr(body_name)));
        xml.push_str(&format!("            <xyz_body>{}</xyz_body>\n", escape_attr(body_name)));
        xml.push_str(&format!("            <translation>{} {} {}</translation>\n",
            wg.transform.translation.x, wg.transform.translation.y, wg.transform.translation.z));

        match wg.geom_type {
            WrapGeomType::Sphere { radius } => {
                xml.push_str(&format!("            <radius>{}</radius>\n", radius));
            }
            WrapGeomType::Cylinder { radius, length } => {
                xml.push_str(&format!("            <radius>{}</radius>\n", radius));
                xml.push_str(&format!("            <length>{}</length>\n", length));
            }
            WrapGeomType::Ellipsoid { radii } => {
                xml.push_str(&format!("            <dimensions>{} {} {}</dimensions>\n",
                    radii[0], radii[1], radii[2]));
            }
        }

        xml.push_str(&format!("          </{}>\n", elem_name));
    }
    xml
}

fn infer_joint_kind(world: &mut World, joint_entity: Entity) -> &'static str {
    let child_entities = children_of(world, joint_entity);
    let coords: Vec<Entity> = child_entities.iter()
        .filter(|&&c| world.get::<JointCoordinate>(c).is_some())
        .copied()
        .collect();
    let n_coords = coords.len();
    match n_coords {
        0 => "WeldJoint",
        1 => {
            let coord = coords[0];
            let effect_entities = children_of(world, coord);
            for effect_entity in effect_entities {
                if let Some(ce) = world.get::<CoordinateEffect>(effect_entity) {
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

fn collect_joint_effects(world: &mut World, joint_entity: Entity) -> Vec<(Entity, CoordinateEffect)> {
    let mut effects = Vec::new();
    let coord_entities = children_of(world, joint_entity);
    for coord in coord_entities {
        if world.get::<JointCoordinate>(coord).is_some() {
            let effect_entities = children_of(world, coord);
            for effect_entity in effect_entities {
                if let Some(ce) = world.get::<CoordinateEffect>(effect_entity) {
                    effects.push((effect_entity, ce.clone()));
                }
            }
        }
    }
    effects
}

fn find_parent_joint(world: &mut World, child_key: Entity) -> Option<String> {
    let body_names = build_body_name_map(world);
    let joint_entity = world.get::<ChildOf>(child_key).map(|co| co.parent)?;
    if !is_joint_entity(world, joint_entity) { return None; }
    let parent_body = world.get::<ChildOf>(joint_entity).map(|co| co.parent)?;
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

            // Collect coord data to avoid borrow conflicts
            let joint_children = children_of(world, joint_entity);
            let coord_entities: Vec<Entity> = joint_children.iter()
                .filter(|&&c| world.get::<JointCoordinate>(c).is_some())
                .copied()
                .collect();

            let coord_infos: Vec<(String, bool, f64, f64, Vec<Entity>)> = coord_entities.iter()
                .map(|&c| {
                    let name = world.get::<Name>(c).map(|n| n.value.clone()).unwrap_or_else(|| "coord".to_string());
                    let coord = world.get::<JointCoordinate>(c);
                    let (clamped, mn, mx) = coord.map(|c| (c.clamped, c.range_min, c.range_max)).unwrap_or((false, 0.0, 0.0));
                    let effects = children_of(world, c);
                    (name, clamped, mn, mx, effects)
                })
                .collect();

            for (coord_name, clamped, range_min, range_max, effect_keys) in coord_infos {
                xml.push_str("            <CoordinateSet>\n");
                xml.push_str(&format!("              <Coordinate name=\"{}\">\n", escape_attr(&coord_name)));
                let mut axis = [0.0f64; 3];
                for effect_entity in effect_keys {
                    if let Some(effect) = world.get::<CoordinateEffect>(effect_entity) {
                        if let TransformComponent::RotationAboutAxis(a) = effect.component {
                            axis = a;
                        }
                    }
                }
                xml.push_str(&format!("                <axis>{} {} {}</axis>\n", axis[0], axis[1], axis[2]));
                if clamped {
                    xml.push_str(&format!("                <range_min>{}</range_min>\n", range_min));
                    xml.push_str(&format!("                <range_max>{}</range_max>\n", range_max));
                }
                xml.push_str("              </Coordinate>\n");
                xml.push_str("            </CoordinateSet>\n");
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

            let joint_children = children_of(world, joint_entity);
            for coord_key in joint_children {
                if let Some(coord) = world.get::<JointCoordinate>(coord_key) {
                    let coord_name = world.get::<Name>(coord_key).map(|n| n.value.as_str()).unwrap_or("coord");
                    xml.push_str("            <CoordinateSet>\n");
                    xml.push_str(&format!("              <Coordinate name=\"{}\">\n", escape_attr(coord_name)));
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
            emit_spatial_transform_from_effects(&mut xml, &effects);
            xml.push_str("          </CustomJoint>\n        </Joint>\n");
            Some(xml)
        }
        _ => None,
    }
}

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
            xml.push_str(&format!("                <coefficients>{}</coefficients>\n",
                coefficients.iter().map(|c| c.to_string()).collect::<Vec<_>>().join(" ")));
            xml.push_str("              </PolynomialFunction>\n");
            xml.push_str(&format!("            </{}>\n", tag));
        }
    }
}

fn emit_spatial_transform_from_effects(
    xml: &mut String,
    effects: &[(Entity, CoordinateEffect)],
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

    for slot_name in ["rotation_x", "rotation_y", "rotation_z", "translation_x", "translation_y", "translation_z"] {
        if let Some(effect) = effects_by_slot.get(slot_name) {
            let coord_name = "coord";
            xml.push_str(&format!("              <{}>\n", slot_name));
            xml.push_str("                <CoordinateEffect>\n");
            xml.push_str(&format!("                  <coordinate>{}</coordinate>\n", escape_attr(coord_name)));
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

fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
