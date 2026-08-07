// ── MuJoCo MJCF XML Exporter (trait-based) ───────────
//
// Component rendering delegated to ExportAs<Mjcf> impls
// in mjcf_components.rs. This file handles only format structure:
// body hierarchy, section organization, and cross-cutting concerns.

use std::collections::HashMap;

use super::trait_export::{escape_attr, ExportAs, ExportCtx};
use crate::components::*;
use crate::id::EntityID;
use crate::world::World;

/// Export the World to an MJCF XML string.
pub fn world_to_mjcf(world: &World, model_name: &str) -> String {
    let ctx = ExportCtx::new(world);
    let mut xml = String::new();

    xml.push_str("<mujoco model=\"");
    xml.push_str(&escape_attr(model_name));
    xml.push_str("\">\n");
    xml.push_str("  <compiler angle=\"radian\"/>\n");

    let children_map = build_children_map(world);
    let roots = find_root_bodies(world);

    // ── worldbody ──
    xml.push_str("  <worldbody>\n");
    for &root_id in &roots {
        emit_body_recursive(world, &ctx, &mut xml, root_id, &children_map, 2);
    }
    xml.push_str("  </worldbody>\n");

    // ── tendon section ──
    let has_tendons = world.iter::<MusclePath>().next().is_some();
    if has_tendons {
        xml.push_str("\n  <tendon>\n");
        for (muscle_key, path) in world.iter::<MusclePath>() {
            let tendon_name = ctx
                .name(muscle_key)
                .map(|n| format!("{}_tendon", n))
                .unwrap_or_else(|| format!("tendon_{}", muscle_key.0));
            xml.push_str(&format!("    <spatial name=\"{}\">\n", escape_attr(&tendon_name)));
            for point in &path.points {
                match point {
                    PathPoint::BodyFixed { body, location } => {
                        if let Some(site_name) = find_site_name(world, *body, location) {
                            xml.push_str(&format!(
                                "      <site site=\"{}\"/>\n",
                                escape_attr(&site_name)
                            ));
                        }
                    }
                    PathPoint::Moving { .. } => {}
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
        for (muscle_key, muscle) in world.iter::<Muscle>() {
            if let Some(element) = muscle.export_as(muscle_key, &ctx) {
                xml.push_str("    ");
                xml.push_str(&element);
                xml.push('\n');
            }
        }
        for (act_key, act) in world.iter::<CoordinateActuator>() {
            if let Some(element) = act.export_as(act_key, &ctx) {
                xml.push_str("    ");
                xml.push_str(&element);
                xml.push('\n');
            }
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

// ── Body hierarchy ────────────────────────────────────

fn emit_body_recursive(
    world: &World,
    ctx: &ExportCtx,
    xml: &mut String,
    entity: EntityID,
    children_map: &HashMap<EntityID, Vec<EntityID>>,
    depth: usize,
) {
    let indent = "  ".repeat(depth);
    let name = ctx.name_or_unnamed(entity);

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

    // Inertial properties
    if let Some(inertial) = world.get::<InertialProperties>(entity) {
        if let Some(element) = inertial.export_as(entity, ctx) {
            xml.push_str(&format!("{}  {}\n", indent, element));
        }
    }

    // Joints (child entities that are joint intermediate nodes)
    emit_body_joints(world, ctx, xml, entity, &indent);

    // Display geometries
    for (geom_key, geom) in world.iter::<DisplayGeometry>() {
        if geom.body == entity {
            if let Some(element) = geom.export_as(geom_key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }

    // Sites (entities with Position + ChildOf, no InertialProperties, no Rotation, no JointCoordinate)
    for (site_key, _site_pos) in world.iter::<Position>() {
        if world.get::<ChildOf>(site_key).map_or(false, |co| co.parent == entity) {
            if world.get::<InertialProperties>(site_key).is_some() { continue; }
            if world.get::<Rotation>(site_key).is_some() { continue; }
            if world.get::<JointCoordinate>(site_key).is_some() { continue; }
            let site_name = ctx.name_or_unnamed(site_key);
            let pos = world.get::<Position>(site_key);
            let (x, y, z) = pos.map(|p| (p.x, p.y, p.z)).unwrap_or((0.0, 0.0, 0.0));
            xml.push_str(&format!("{}  <site name=\"{}\" pos=\"{} {} {}\"/>\n",
                indent, escape_attr(site_name), x, y, z));
        }
    }

    // Wrap geometries
    for (wrap_key, wrap) in world.iter::<WrapGeom>() {
        if wrap.body == entity {
            if let Some(element) = wrap.export_as(wrap_key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }

    // Recurse into children that are bodies
    if let Some(children) = children_map.get(&entity) {
        for &child in children {
            if world.get::<InertialProperties>(child).is_some() {
                emit_body_recursive(world, ctx, xml, child, children_map, depth + 1);
            }
        }
    }

    xml.push_str(&format!("{}</body>\n", indent));
}

/// Check if an entity is a joint intermediate node.
fn is_joint_entity(world: &World, entity: EntityID) -> bool {
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

fn emit_body_joints(
    world: &World,
    ctx: &ExportCtx,
    xml: &mut String,
    body: EntityID,
    indent: &str,
) {
    // Find children of this body that are joint entities
    let joints: Vec<(EntityID, String)> = world.children_of(body).iter()
        .filter(|&&k| is_joint_entity(world, k))
        .filter_map(|&k| {
            // Export the joint based on its coordinate count
            let coords: Vec<EntityID> = world.children_of(k).iter()
                .filter(|&&c| world.get::<JointCoordinate>(c).is_some())
                .copied()
                .collect();
            let name = ctx.name_or_unnamed(k);
            let n_coords = coords.len();
            let kind = match n_coords {
                0 => "WeldJoint",
                1 => {
                    let mut found = "PinJoint";
                    for effect_entity in world.children_of(coords[0]) {
                        if let Some(effect) = world.get::<CoordinateEffect>(effect_entity) {
                            match &effect.component {
                                TransformComponent::TranslationAlongAxis(_)
                                | TransformComponent::TranslationX
                                | TransformComponent::TranslationY
                                | TransformComponent::TranslationZ => { found = "SlideJoint"; }
                                _ => {}
                            }
                        }
                    }
                    found
                }
                2 => "UniversalJoint",
                3 => "BallJoint",
                6 => "FreeJoint",
                _ => "CustomJoint",
            };
            // Generate simple joint XML
            match kind {
                "WeldJoint" => None,
                "FreeJoint" => Some(format!(r#"<freejoint name="{}"/>"#, escape_attr(name))),
                _ => {
                    let axis = [0.0, 0.0, 1.0]; // Default
                    let jtype = match kind {
                        "PinJoint" => "hinge",
                        "SlideJoint" => "slide",
                        "BallJoint" => "ball",
                        _ => "hinge",
                    };
                    Some(format!(r#"<joint name="{}" type="{}" axis="{} {} {}"/>"#,
                        escape_attr(name), jtype, axis[0], axis[1], axis[2]))
                }
            }
            .map(|s| (k, s))
        })
        .collect();

    for (_, element) in joints {
        xml.push_str(&format!("{}  {}\n", indent, element));
    }
}

fn build_children_map(world: &World) -> HashMap<EntityID, Vec<EntityID>> {
    let mut children: HashMap<EntityID, Vec<EntityID>> = HashMap::new();
    for (entity, child_of) in world.iter::<ChildOf>() {
        children.entry(child_of.parent).or_default().push(entity);
    }
    children
}

fn find_root_bodies(world: &World) -> Vec<EntityID> {
    let mut roots = Vec::new();
    for (entity, child_of) in world.iter::<ChildOf>() {
        if child_of.parent == EntityID(0) {
            if world.get::<InertialProperties>(entity).is_some() {
                roots.push(entity);
            }
        }
    }
    for (entity, _) in world.iter::<InertialProperties>() {
        if entity == EntityID(0) {
            continue;
        }
        if world.get::<ChildOf>(entity).is_none() {
            roots.push(entity);
        }
    }
    roots
}

fn find_site_name(world: &World, body: EntityID, location: &[f64; 3]) -> Option<String> {
    for (site_key, _pos) in world.iter::<Position>() {
        if !world.get::<ChildOf>(site_key).map_or(false, |co| co.parent == body) {
            continue;
        }
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
