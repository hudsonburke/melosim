// ── MuJoCo MJCF XML Exporter ──────────────────────────

use std::collections::HashMap;

use crate::components::*;
use crate::world::World;
use bevy_ecs::prelude::Entity;

pub fn world_to_mjcf(world: &mut World, model_name: &str) -> String {
    let mut xml = String::new();
    xml.push_str("<mujoco model=\"");
    xml.push_str(&escape_attr(model_name));
    xml.push_str("\">\n");
    xml.push_str("  <compiler angle=\"radian\"/>\n");

    let children_map = build_children_map(world);
    let body_names = build_body_name_map(world);
    let roots = find_root_bodies(world);

    xml.push_str("  <worldbody>\n");
    for &root_id in &roots {
        emit_body_recursive(world, &mut xml, root_id, &children_map, &body_names, 2);
    }
    xml.push_str("  </worldbody>\n");

    // ── tendon section ──
    let has_tendons = world.iter::<MusclePath>().into_iter().next().is_some();
    if has_tendons {
        xml.push_str("\n  <tendon>\n");
        for (muscle_key, path) in world.iter::<MusclePath>() {
            let tendon_name = world.get::<Name>(muscle_key)
                .map(|n| format!("{}_tendon", n.value))
                .unwrap_or_else(|| format!("tendon_{}", muscle_key.index()));
            xml.push_str(&format!("    <spatial name=\"{}\">\n", escape_attr(&tendon_name)));
            for point in &path.points {
                if let PathPoint::BodyFixed { body, location } = point {
                    if let Some(site_name) = find_site_name(world, *body, location) {
                        xml.push_str(&format!("      <site site=\"{}\"/>\n", escape_attr(&site_name)));
                    }
                }
            }
            xml.push_str("    </spatial>\n");
        }
        xml.push_str("  </tendon>\n");
    }

    // ── actuator section ──
    let has_muscles = world.iter::<Muscle>().into_iter().next().is_some();
    let has_coord_actuators = world.iter::<CoordinateActuator>().into_iter().next().is_some();
    if has_muscles || has_coord_actuators {
        xml.push_str("\n  <actuator>\n");

        for (muscle_key, _muscle) in world.iter::<Muscle>() {
            let name = world.get::<Name>(muscle_key).map(|n| n.value.as_str()).unwrap_or("unnamed_muscle");
            let params = world.get::<Millard2012Params>(muscle_key);
            let path = world.get::<MusclePath>(muscle_key);
            let tendon_name = path.map(|_| format!("{}_tendon", name));
            let force = params.map(|p| p.max_isometric_force).unwrap_or(1000.0);
            let range_min = params.map(|p| p.minimum_activation).unwrap_or(0.01);
            let lengthrange = params.map(|p| {
                format!("{} {}", p.tendon_slack_length, p.tendon_slack_length + p.optimal_fiber_length)
            });

            xml.push_str(&format!("    <muscle name=\"{}\"", escape_attr(name)));
            xml.push_str(&format!(" force=\"{}\"", force));
            xml.push_str(&format!(" range=\"{} 1.0\"", range_min));
            if let Some(ref lr) = lengthrange {
                xml.push_str(&format!(" lengthrange=\"{}\"", lr));
            }
            if let Some(ref tn) = tendon_name {
                xml.push_str(&format!(" tendon=\"{}\"", escape_attr(tn)));
            }
            xml.push_str("/>\n");
        }

        for (act_key, act) in world.iter::<CoordinateActuator>() {
            let name = world.get::<Name>(act_key).map(|n| n.value.as_str()).unwrap_or("unnamed_actuator");
            let coord_name = world.get::<Name>(act.coordinate).map(|n| n.value.as_str()).unwrap_or("unnamed_coord");
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

pub fn write_mjcf(world: &mut World, path: &str, model_name: &str) -> Result<(), String> {
    let xml = world_to_mjcf(world, model_name);
    std::fs::write(path, &xml).map_err(|e| format!("Failed to write {}: {}", path, e))
}

// ── Helper functions ──

fn build_children_map(world: &mut World) -> HashMap<Entity, Vec<Entity>> {
    let mut children: HashMap<Entity, Vec<Entity>> = HashMap::new();
    for (entity, child_of) in world.iter::<ChildOf>() {
        children.entry(child_of.parent).or_default().push(entity);
    }
    children
}

fn build_body_name_map(world: &mut World) -> HashMap<Entity, String> {
    let mut names = HashMap::new();
    for (entity, _) in world.iter::<InertialProperties>() {
        if let Some(name) = world.get::<Name>(entity) {
            names.insert(entity, name.value.clone());
        }
    }
    names
}

fn find_root_bodies(world: &mut World) -> Vec<Entity> {
    let mut roots = Vec::new();
    for (entity, child_of) in world.iter::<ChildOf>() {
        // Root bodies: entities whose parent has no InertialProperties
        // (i.e., parent is a joint or ground)
        if world.get::<InertialProperties>(entity).is_some() {
            if !world.get::<InertialProperties>(child_of.parent).is_some() {
                roots.push(entity);
            }
        }
    }
    // Also include bodies with no ChildOf (top-level bodies)
    for (entity, _) in world.iter::<InertialProperties>() {
        if world.get::<ChildOf>(entity).is_none() {
            roots.push(entity);
        }
    }
    roots
}

fn is_joint_entity(world: &mut World, entity: Entity) -> bool {
    if world.get::<InertialProperties>(entity).is_some() {
        return false;
    }
    for child in world.children_of(entity) {
        if world.get::<InertialProperties>(child).is_some() {
            return true;
        }
    }
    false
}

fn emit_body_recursive(
    world: &mut World,
    xml: &mut String,
    entity: Entity,
    children_map: &HashMap<Entity, Vec<Entity>>,
    body_names: &HashMap<Entity, String>,
    depth: usize,
) {
    let indent = "  ".repeat(depth);
    let name = body_names.get(&entity).map(|s| s.as_str()).unwrap_or("unnamed");
    xml.push_str(&format!("{}<body name=\"{}\"", indent, escape_attr(name)));

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

    if let Some(inertial) = world.get::<InertialProperties>(entity) {
        xml.push_str(&format!("{}  <inertial pos=\"{} {} {}\" mass=\"{}\"",
            indent, inertial.com[0], inertial.com[1], inertial.com[2], inertial.mass));
        let i = &inertial.inertia;
        xml.push_str(&format!(" fullinertia=\"{} {} {} {} {} {}\"",
            i[0], i[1], i[2], i[3], i[4], i[5]));
        xml.push_str("/>\n");
    }

    emit_body_joints(world, xml, entity, depth);

    for (_geom_key, geom) in world.iter::<DisplayGeometry>() {
        if geom.body == entity {
            xml.push_str(&format!("{}  <geom name=\"{}\"",
                indent, escape_attr(
                    world.get::<Name>(_geom_key).map(|n| n.value.as_str()).unwrap_or("geom")
                )));
            xml.push_str(" type=\"sphere\"");
            xml.push_str(&format!(" size=\"{} {} {}\"", geom.scale[0], geom.scale[1], geom.scale[2]));
            xml.push_str(&format!(" pos=\"{} {} {}\"",
                geom.transform.translation.x, geom.transform.translation.y, geom.transform.translation.z));
            xml.push_str(&format!(" rgba=\"{} {} {} {}\"",
                geom.color[0], geom.color[1], geom.color[2], geom.opacity));
            xml.push_str("/>\n");
        }
    }

    for (site_key, _site_pos) in world.iter::<Position>() {
        if world.get::<ChildOf>(site_key).map_or(false, |co| co.parent == entity) {
            if world.get::<InertialProperties>(site_key).is_some() { continue; }
            if world.get::<JointCoordinate>(site_key).is_some() { continue; }
            if world.get::<Rotation>(site_key).is_some() { continue; }
            let site_name = world.get::<Name>(site_key).map(|n| n.value.as_str()).unwrap_or("unnamed_site");
            let pos = world.get::<Position>(site_key);
            let (px, py, pz) = pos.map(|p| (p.x, p.y, p.z)).unwrap_or((0.0, 0.0, 0.0));
            xml.push_str(&format!("{}  <site name=\"{}\" pos=\"{} {} {}\"/>\n",
                indent, escape_attr(site_name), px, py, pz));
        }
    }

    for (_wrap_key, wrap) in world.iter::<WrapGeom>() {
        if wrap.body == entity {
            let wrap_name = world.get::<Name>(_wrap_key).map(|n| n.value.as_str()).unwrap_or("wrap");
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

    if let Some(children) = children_map.get(&entity) {
        for &child in children {
            if world.get::<InertialProperties>(child).is_some() {
                emit_body_recursive(world, xml, child, children_map, body_names, depth + 1);
            }
        }
    }

    xml.push_str(&format!("{}</body>\n", indent));
}

fn emit_body_joints(world: &mut World, xml: &mut String, body: Entity, depth: usize) {
    let ind = "  ".repeat(depth + 1);

    for &child in &world.children_of(body) {
        if !is_joint_entity(world, child) { continue; }
        let name = world.get::<Name>(child).map(|n| n.value.as_str()).unwrap_or("joint");

        let coords: Vec<Entity> = world.children_of(child).iter()
            .filter(|&&c| world.get::<JointCoordinate>(c).is_some())
            .copied()
            .collect();
        let n_coords = coords.len();

        let kind = match n_coords {
            0 => "WeldJoint",
            1 => {
                let mut found_kind = "PinJoint";
                for effect_entity in world.children_of(coords[0]) {
                    if let Some(effect) = world.get::<CoordinateEffect>(effect_entity) {
                        match &effect.component {
                            TransformComponent::TranslationAlongAxis(_)
                            | TransformComponent::TranslationX
                            | TransformComponent::TranslationY
                            | TransformComponent::TranslationZ => { found_kind = "SlideJoint"; }
                            _ => {}
                        }
                    }
                }
                found_kind
            }
            2 => "UniversalJoint",
            3 => "BallJoint",
            6 => "FreeJoint",
            _ => "CustomJoint",
        };

        match kind {
            "PinJoint" => {
                let axis = extract_joint_axis(world, child);
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"hinge\" axis=\"{} {} {}\"",
                    ind, escape_attr(name), axis[0], axis[1], axis[2]));
                if let Some(&coord_key) = coords.first() {
                    if let Some(coord) = world.get::<JointCoordinate>(coord_key) {
                        if coord.clamped {
                            xml.push_str(&format!(" limited=\"true\" range=\"{} {}\"", coord.range_min, coord.range_max));
                        }
                    }
                }
                append_joint_dynamics(world, xml, child);
                xml.push_str("/>\n");
            }
            "SlideJoint" => {
                let axis = extract_joint_axis(world, child);
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"slide\" axis=\"{} {} {}\"",
                    ind, escape_attr(name), axis[0], axis[1], axis[2]));
                if let Some(&coord_key) = coords.first() {
                    if let Some(coord) = world.get::<JointCoordinate>(coord_key) {
                        if coord.clamped {
                            xml.push_str(&format!(" limited=\"true\" range=\"{} {}\"", coord.range_min, coord.range_max));
                        }
                    }
                }
                append_joint_dynamics(world, xml, child);
                xml.push_str("/>\n");
            }
            "BallJoint" => {
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"ball\"", ind, escape_attr(name)));
                xml.push_str("/>\n");
            }
            "FreeJoint" => {
                xml.push_str(&format!("{}<freejoint name=\"{}\"/>\n", ind, escape_attr(name)));
            }
            "WeldJoint" => {}
            "UniversalJoint" => {
                let axes = extract_joint_axes(world, child, 2);
                xml.push_str(&format!("{}<joint name=\"{}\" type=\"hinge\" axis=\"{} {} {}\"",
                    ind, escape_attr(name), axes[0][0], axes[0][1], axes[0][2]));
                xml.push_str("/>\n");
                xml.push_str(&format!("{}<joint name=\"{}_2\" type=\"hinge\" axis=\"{} {} {}\"/>",
                    ind, escape_attr(name), axes[1][0], axes[1][1], axes[1][2]));
                xml.push_str("\n");
            }
            "CustomJoint" => {
                for coord_key in &coords {
                    let coord_name = world.get::<Name>(*coord_key).map(|n| n.value.as_str()).unwrap_or("coord");
                    xml.push_str(&format!("{}<joint name=\"{}\" type=\"hinge\" axis=\"0 0 1\"",
                        ind, escape_attr(coord_name)));
                    xml.push_str("/>\n");
                }
            }
            _ => {}
        }
    }
}

fn extract_joint_axis(world: &mut World, joint_entity: Entity) -> [f64; 3] {
    for &coord in &world.children_of(joint_entity) {
        if world.get::<JointCoordinate>(coord).is_some() {
            for effect_entity in world.children_of(coord) {
                if let Some(effect) = world.get::<CoordinateEffect>(effect_entity) {
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
    [0.0, 0.0, 1.0]
}

fn extract_joint_axes(world: &mut World, joint_entity: Entity, n: usize) -> Vec<[f64; 3]> {
    let mut axes = Vec::new();
    for &coord in &world.children_of(joint_entity) {
        if world.get::<JointCoordinate>(coord).is_some() {
            for effect_entity in world.children_of(coord) {
                if let Some(effect) = world.get::<CoordinateEffect>(effect_entity) {
                    match &effect.component {
                        TransformComponent::RotationAboutAxis(axis)
                        | TransformComponent::TranslationAlongAxis(axis) => {
                            axes.push(*axis);
                            if axes.len() >= n { return axes; }
                        }
                        _ => {}
                    }
                }
            }
        }
    }
    while axes.len() < n { axes.push([0.0, 0.0, 1.0]); }
    axes
}

fn append_joint_dynamics(world: &mut World, xml: &mut String, joint_entity: Entity) {
    for &coord_key in &world.children_of(joint_entity) {
        if let Some(coord) = world.get::<JointCoordinate>(coord_key) {
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

fn find_site_name(world: &mut World, body: Entity, location: &[f64; 3]) -> Option<String> {
    for (site_key, _pos) in world.iter::<Position>() {
        if !world.get::<ChildOf>(site_key).map_or(false, |co| co.parent == body) { continue; }
        if world.get::<InertialProperties>(site_key).is_some() { continue; }
        if world.get::<Rotation>(site_key).is_some() { continue; }
        if world.get::<JointCoordinate>(site_key).is_some() { continue; }
        if let Some(pos) = world.get::<Position>(site_key) {
            let dx = pos.x - location[0];
            let dy = pos.y - location[1];
            let dz = pos.z - location[2];
            if (dx * dx + dy * dy + dz * dz) < 1e-12 {
                return world.get::<Name>(site_key).map(|n| n.value.clone());
            }
        }
    }
    None
}

fn escape_attr(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
