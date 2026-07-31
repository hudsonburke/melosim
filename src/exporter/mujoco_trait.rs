// ── MuJoCo MJCF XML Exporter (trait-based) ───────────
//
// Component rendering delegated to ExportAs<Mjcf> impls
// in mjcf_components.rs. This file handles only format structure:
// body hierarchy, section organization, and cross-cutting concerns.

use std::collections::HashMap;

use super::trait_export::{escape_attr, ExportAs, ExportCtx, Mjcf};
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

    if let Some(frame) = world.get::<Frame>(entity) {
        let t = &frame.transform.translation;
        if t.x != 0.0 || t.y != 0.0 || t.z != 0.0 {
            xml.push_str(&format!(" pos=\"{} {} {}\"", t.x, t.y, t.z));
        }
        let r = &frame.transform.rotation;
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

    // Joints
    emit_body_joints(world, ctx, xml, entity, &indent);

    // Display geometries
    for (geom_key, geom) in world.iter::<DisplayGeometry>() {
        if geom.body == entity {
            if let Some(element) = geom.export_as(geom_key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }

    // Sites
    for (site_key, site) in world.iter::<Site>() {
        if site.parent == entity {
            if let Some(element) = site.export_as(site_key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
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

    // Recurse into children
    if let Some(children) = children_map.get(&entity) {
        for &child in children {
            emit_body_recursive(world, ctx, xml, child, children_map, depth + 1);
        }
    }

    xml.push_str(&format!("{}</body>\n", indent));
}

fn emit_body_joints(
    world: &World,
    ctx: &ExportCtx,
    xml: &mut String,
    body: EntityID,
    indent: &str,
) {
    let joints: Vec<(EntityID, String)> = world.iter::<Joint>()
        .filter(|(_, j)| j.body_b == body)
        .filter_map(|(k, j)| j.export_as(k, ctx).map(|s| (k, s)))
        .collect();

    for (_, element) in joints {
        xml.push_str(&format!("{}  {}\n", indent, element));
    }
}

fn build_children_map(world: &World) -> HashMap<EntityID, Vec<EntityID>> {
    let mut children: HashMap<EntityID, Vec<EntityID>> = HashMap::new();
    for (entity, frame) in world.iter::<Frame>() {
        children.entry(frame.parent).or_default().push(entity);
    }
    children
}

fn find_root_bodies(world: &World) -> Vec<EntityID> {
    let mut roots = Vec::new();
    for (entity, frame) in world.iter::<Frame>() {
        if frame.parent == EntityID(0) {
            if world.get::<InertialProperties>(entity).is_some() {
                roots.push(entity);
            }
        }
    }
    for (entity, _) in world.iter::<InertialProperties>() {
        if entity == EntityID(0) {
            continue;
        }
        if world.get::<Frame>(entity).is_none() {
            roots.push(entity);
        }
    }
    roots
}

fn find_site_name(world: &World, body: EntityID, location: &[f64; 3]) -> Option<String> {
    for (site_key, site) in world.iter::<Site>() {
        if site.parent == body {
            let dx = site.offset.x - location[0];
            let dy = site.offset.y - location[1];
            let dz = site.offset.z - location[2];
            if (dx * dx + dy * dy + dz * dz) < 1e-12 {
                return world.get::<Name>(site_key).map(|n| n.value.clone());
            }
        }
    }
    None
}
