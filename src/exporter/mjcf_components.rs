// ── MJCF component exporters ─────────────────────────
//
// Each component type implements ExportAs<Mjcf> to define how it
// renders into MJCF XML. The exporter (mod.rs) calls these via
// the trait, keeping component logic decoupled from format structure.

use super::trait_export::{escape_attr, ExportAs, ExportCtx, Mjcf};
use crate::components::*;
use crate::id::EntityID;

// ── Joints ────────────────────────────────────────────

impl ExportAs<Mjcf> for HingeJoint {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let mut xml = format!(
            r#"<joint name="{}" type="hinge" axis="{} {} {}""#,
            name, self.axis[0], self.axis[1], self.axis[2]
        );
        if let Some(ref lim) = self.limits {
            xml.push_str(&format!(r#" limited="true" range="{} {}""#, lim.lower, lim.upper));
        }
        append_dynamics(ctx, entity, &mut xml);
        xml.push_str("/>");
        Some(xml)
    }
}

impl ExportAs<Mjcf> for SlideJoint {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let mut xml = format!(
            r#"<joint name="{}" type="slide" axis="{} {} {}""#,
            name, self.axis[0], self.axis[1], self.axis[2]
        );
        if let Some(ref lim) = self.limits {
            xml.push_str(&format!(r#" limited="true" range="{} {}""#, lim.lower, lim.upper));
        }
        append_dynamics(ctx, entity, &mut xml);
        xml.push_str("/>");
        Some(xml)
    }
}

impl ExportAs<Mjcf> for BallJoint {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let mut xml = format!(r#"<joint name="{}" type="ball""#, name);
        if let Some(ref lim) = self.limits {
            xml.push_str(&format!(r#" limited="true" range="{} {}""#, lim.lower, lim.upper));
        }
        xml.push_str("/>");
        Some(xml)
    }
}

impl ExportAs<Mjcf> for FreeJoint {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        Some(format!(r#"<freejoint name="{}"/>"#, name))
    }
}

impl ExportAs<Mjcf> for FixedJoint {
    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        None // MuJoCo has no explicit fixed joint
    }
}

impl ExportAs<Mjcf> for UniversalJoint {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        Some(format!(
            r#"<joint name="{}" type="hinge" axis="{} {} {}"/><joint name="{}_2" type="hinge" axis="{} {} {}"/>"#,
            name, self.axis1[0], self.axis1[1], self.axis1[2],
            name, self.axis2[0], self.axis2[1], self.axis2[2]
        ))
    }
}

impl ExportAs<Mjcf> for CustomJoint {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        if self.coordinates.is_empty() {
            return None;
        }
        if self.coordinates.len() == 1 {
            return Some(format!(r#"<joint name="{}" type="hinge" axis="0 0 1"/>"#, name));
        }
        let mut xml = String::new();
        for i in 0..self.coordinates.len() {
            xml.push_str(&format!(
                r#"<joint name="{}_{}" type="hinge" axis="0 0 1"/>"#, name, i
            ));
        }
        Some(xml)
    }
}

// ── Muscles ───────────────────────────────────────────

impl ExportAs<Mjcf> for Muscle {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let params = ctx.world.get::<Millard2012Params>(entity);
        let path = ctx.world.get::<MusclePath>(entity);

        let tendon_name = path.map(|_| format!("{}_tendon", name));
        let force = params.map(|p| p.max_isometric_force).unwrap_or(1000.0);
        let range_min = params.map(|p| p.minimum_activation).unwrap_or(0.01);
        let lengthrange = params.map(|p| {
            format!("{} {}", p.tendon_slack_length, p.tendon_slack_length + p.optimal_fiber_length)
        });

        let mut xml = format!(r#"<muscle name="{}" force="{}""#, escape_attr(name), force);
        xml.push_str(&format!(r#" range="{} 1.0""#, range_min));
        if let Some(ref lr) = lengthrange {
            xml.push_str(&format!(r#" lengthrange="{}""#, lr));
        }
        if let Some(ref tn) = tendon_name {
            xml.push_str(&format!(r#" tendon="{}""#, escape_attr(tn)));
        }
        xml.push_str("/>");
        Some(xml)
    }
}

// ── Actuators ─────────────────────────────────────────

impl ExportAs<Mjcf> for CoordinateActuator {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let coord_name = ctx.name_or_unnamed(self.coordinate);
        Some(format!(
            r#"<general name="{}" joint="{}" gear="{}" ctrlrange="{} {}"/>"#,
            escape_attr(name), escape_attr(coord_name),
            self.optimal_force, self.min_control, self.max_control
        ))
    }
}

// ── Geometry ──────────────────────────────────────────

impl ExportAs<Mjcf> for DisplayGeometry {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        Some(format!(
            r#"<geom name="{}" type="sphere" size="{} {} {}" pos="{} {} {}" rgba="{} {} {} {}"/>"#,
            escape_attr(name),
            self.scale[0], self.scale[1], self.scale[2],
            self.transform.translation.x, self.transform.translation.y, self.transform.translation.z,
            self.color[0], self.color[1], self.color[2], self.opacity
        ))
    }
}

impl ExportAs<Mjcf> for WrapGeom {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let pos = format!("{} {} {}",
            self.transform.translation.x, self.transform.translation.y, self.transform.translation.z);
        match &self.geom_type {
            WrapGeomType::Sphere { radius } => Some(format!(
                r#"<geom name="{}" type="sphere" size="{}" pos="{}" rgba="0.5 0.5 0.9 0.4" group="2"/>"#,
                escape_attr(name), radius, pos
            )),
            WrapGeomType::Cylinder { radius, length } => Some(format!(
                r#"<geom name="{}" type="cylinder" size="{} {}" pos="{}" rgba="0.5 0.5 0.9 0.4" group="2"/>"#,
                escape_attr(name), radius, length, pos
            )),
            WrapGeomType::Ellipsoid { radii } => Some(format!(
                r#"<geom name="{}" type="ellipsoid" size="{} {} {}" pos="{}" rgba="0.5 0.5 0.9 0.4" group="2"/>"#,
                escape_attr(name), radii[0], radii[1], radii[2], pos
            )),
        }
    }
}

impl ExportAs<Mjcf> for Site {
    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        Some(format!(
            r#"<site name="{}" pos="{} {} {}"/>"#,
            escape_attr(name), self.offset.x, self.offset.y, self.offset.z
        ))
    }
}

impl ExportAs<Mjcf> for InertialProperties {
    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        let i = &self.inertia;
        Some(format!(
            r#"<inertial pos="{} {} {}" mass="{}" fullinertia="{} {} {} {} {} {}"/>"#,
            self.com[0], self.com[1], self.com[2], self.mass,
            i[0], i[1], i[2], i[3], i[4], i[5]
        ))
    }
}

// ── Components with no standalone MJCF representation ──

impl ExportAs<Mjcf> for Millard2012Params {
    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        None // Consumed by Muscle's export_as
    }
}

impl ExportAs<Mjcf> for MusclePath {
    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        None // Emitted as <tendon><spatial> in the coordinator
    }
}

impl ExportAs<Mjcf> for Frame {
    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        None // Encoded as pos/quat on the <body> element
    }
}

impl ExportAs<Mjcf> for JointCoordinate {
    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        None // Appended to joint element by append_dynamics()
    }
}

impl ExportAs<Mjcf> for Name {
    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        None // Consumed by ctx.name()
    }
}

// ── Helper: append coordinate dynamics to a joint element ──

fn append_dynamics(ctx: &ExportCtx, joint_entity: EntityID, xml: &mut String) {
    if let Some(jname) = ctx.name(joint_entity) {
        for (coord_key, coord) in ctx.world.iter::<JointCoordinate>() {
            if ctx.name(coord_key) == Some(jname) {
                if coord.stiffness != 0.0 {
                    xml.push_str(&format!(r#" stiffness="{}""#, coord.stiffness));
                }
                if coord.damping != 0.0 {
                    xml.push_str(&format!(r#" damping="{}""#, coord.damping));
                }
                return;
            }
        }
    }
}
