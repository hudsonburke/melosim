// ── MuJoCo MJCF XML Exporter (trait-based) ───────────
//
// Walks the melosim World and produces a valid MJCF XML file.
//
// Component rendering is delegated to `ExportAs<Mjcf>` impls
// in mjcf_components.rs. This file handles only format structure:
// body hierarchy, section organization, and cross-cutting concerns.
//
// To add a new component type:
//   1. Implement `ExportAs<Mjcf>` on the component (mjcf_components.rs)
//   2. Add iteration logic here where the component belongs in MJCF
//
// To add a new format:
//   1. Define a marker type (trait_export.rs already has OsIm)
//   2. Implement `ExportAs<OsIm>` on each component (osim_components.rs)
//   3. Write a coordinator function (world_to_osim) using those impls

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

    // ── Build body hierarchy ──
    let children_map = build_children_map(world);

    // ── Find root bodies ──
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
            xml.push_str(&format!(
                "    <spatial name=\"{}\">\n",
                escape_attr(&tendon_name)
            ));
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
                    PathPoint::Moving { .. } => {
                        // Moving path points need a different export strategy
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

        // Muscles — delegate to ExportAs<Mjcf> for Muscle
        for (muscle_key, muscle) in world.iter::<Muscle>() {
            if let Some(element) = muscle.export_as(muscle_key, &ctx) {
                xml.push_str("    ");
                xml.push_str(&element);
                xml.push('\n');
            }
        }

        // Coordinate actuators — delegate to ExportAs<Mjcf> for CoordinateActuator
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

// ── Body hierarchy (format-specific structure) ────────

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

    // Position/orientation from Frame
    if let Some(frame) = world.get::<Frame>(entity) {
        let t = &frame.transform;
        if t.translation.x != 0.0 || t.translation.y != 0.0 || t.translation.z != 0.0 {
            xml.push_str(&format!(
                " pos=\"{} {} {}\"",
                t.translation.x, t.translation.y, t.translation.z
            ));
        }
        let r = &frame.transform.rotation;
        if r.w != 1.0 || r.x != 0.0 || r.y != 0.0 || r.z != 0.0 {
            xml.push_str(&format!(" quat=\"{} {} {} {}\"", r.w, r.x, r.y, r.z));
        }
    }
    xml.push_str(">\n");

    // Inertial properties — delegate to trait
    if let Some(element) = export_component_as::<InertialProperties, Mjcf>(world, entity, ctx) {
        xml.push_str(&format!("{}  {}\n", indent, element));
    }

    // Joints — delegate to trait (dispatches on concrete joint type)
    emit_body_joints(world, ctx, xml, entity, &indent);

    // Display geometries on this body
    for (geom_key, geom) in world.iter::<DisplayGeometry>() {
        if geom.body == entity {
            if let Some(element) = geom.export_as(geom_key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }

    // Sites on this body
    for (site_key, site) in world.iter::<Site>() {
        if site.parent == entity {
            if let Some(element) = site.export_as(site_key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }

    // Wrap geometries on this body
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

/// Emit joints attached to a body. Tries each joint type via the trait.
fn emit_body_joints(
    world: &World,
    ctx: &ExportCtx,
    xml: &mut String,
    body: EntityID,
    indent: &str,
) {
    // Each joint type — the trait dispatches on concrete type.
    // Order matters for round-trip fidelity (matches import order).
    for (key, joint) in world.iter::<HingeJoint>() {
        if joint.body_b == body {
            if let Some(element) = joint.export_as(key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }
    for (key, joint) in world.iter::<SlideJoint>() {
        if joint.body_b == body {
            if let Some(element) = joint.export_as(key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }
    for (key, joint) in world.iter::<BallJoint>() {
        if joint.body_b == body {
            if let Some(element) = joint.export_as(key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }
    for (key, joint) in world.iter::<FreeJoint>() {
        if joint.body_b == body {
            if let Some(element) = joint.export_as(key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }
    for (key, joint) in world.iter::<UniversalJoint>() {
        if joint.body_b == body {
            if let Some(element) = joint.export_as(key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }
    for (key, joint) in world.iter::<CustomJoint>() {
        if joint.body_b == body {
            if let Some(element) = joint.export_as(key, ctx) {
                xml.push_str(&format!("{}  {}\n", indent, element));
            }
        }
    }
    // FixedJoint → no-op (MuJoCo has no explicit fixed joint)
}

/// Try to export a component of type C from an entity using format F.
fn export_component_as<C, F>(world: &World, entity: EntityID, ctx: &ExportCtx) -> Option<String>
where
    C: ExportAs<F, Output = String>,
{
    world.get::<C>(entity).and_then(|c| c.export_as(entity, ctx))
}

// ── Hierarchy helpers (same as before) ────────────────

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
