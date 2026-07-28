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
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let mut xml = format!(
            r#"<joint name="{}" type="hinge" axis="{} {} {}""#,
            name, self.axis[0], self.axis[1], self.axis[2]
        );
        if let Some(ref lim) = self.limits {
            xml.push_str(&format!(r#" limited="true" range="{} {}""#, lim.lower, lim.upper));
        }
        // Append dynamics from JointCoordinate if available
        append_dynamics(ctx, entity, &mut xml);
        xml.push_str("/>");
        Some(xml)
    }
}

impl ExportAs<Mjcf> for SlideJoint {
    type Output = String;

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
    type Output = String;

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
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        Some(format!(r#"<freejoint name="{}"/>"#, name))
    }
}

impl ExportAs<Mjcf> for FixedJoint {
    type Output = String;

    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        // MuJoCo has no explicit fixed joint — bodies without joints are fixed.
        None
    }
}

impl ExportAs<Mjcf> for UniversalJoint {
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        // UniversalJoint (2-DOF) → two hinge joints on same body
        let name = ctx.name_or_unnamed(entity);
        let xml = format!(
            r#"<joint name="{}" type="hinge" axis="{} {} {}"/><joint name="{}_2" type="hinge" axis="{} {} {}"/>"#,
            name, self.axis1[0], self.axis1[1], self.axis1[2],
            name, self.axis2[0], self.axis2[1], self.axis2[2]
        );
        Some(xml)
    }
}

impl ExportAs<Mjcf> for CustomJoint {
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        // CustomJoint with SpatialTransform → approximate as hinge(s)
        // This is lossy — coupled DOFs can't be fully represented.
        let name = ctx.name_or_unnamed(entity);
        if self.coordinates.is_empty() {
            return None;
        }
        // For single-DOF CustomJoints, emit as hinge
        if self.coordinates.len() == 1 {
            let xml = format!(
                r#"<joint name="{}" type="hinge" axis="0 0 1"/>"#,
                name
            );
            return Some(xml);
        }
        // Multi-DOF: emit multiple hinges (lossy)
        let mut xml = String::new();
        for (i, _coord) in self.coordinates.iter().enumerate() {
            xml.push_str(&format!(
                r#"<joint name="{}_{}" type="hinge" axis="0 0 1"/>"#,
                name, i
            ));
        }
        Some(xml)
    }
}

// ── Muscles ───────────────────────────────────────────

impl ExportAs<Mjcf> for Muscle {
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);

        // Gather params and path from the same entity
        let params = ctx.world.get::<Millard2012Params>(entity);
        let path = ctx.world.get::<MusclePath>(entity);

        let tendon_name = path.map(|_| format!("{}_tendon", name));

        let force = params.map(|p| p.max_isometric_force).unwrap_or(1000.0);
        let range_min = params.map(|p| p.minimum_activation).unwrap_or(0.01);
        let range_max = 1.0;
        let lengthrange = params.map(|p| {
            format!("{} {}", p.tendon_slack_length, p.tendon_slack_length + p.optimal_fiber_length)
        });

        let mut xml = format!(r#"<muscle name="{}" force="{}""#, escape_attr(name), force);
        xml.push_str(&format!(r#" range="{} {}""#, range_min, range_max));
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
    type Output = String;

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
    type Output = String;

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
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        match &self.geom_type {
            WrapGeomType::Sphere { radius } => Some(format!(
                r#"<geom name="{}" type="sphere" size="{}" pos="{} {} {}" rgba="0.5 0.5 0.9 0.4" group="2"/>"#,
                escape_attr(name), radius,
                self.transform.translation.x, self.transform.translation.y, self.transform.translation.z
            )),
            WrapGeomType::Cylinder { radius, length } => Some(format!(
                r#"<geom name="{}" type="cylinder" size="{} {}" pos="{} {} {}" rgba="0.5 0.5 0.9 0.4" group="2"/>"#,
                escape_attr(name), radius, length,
                self.transform.translation.x, self.transform.translation.y, self.transform.translation.z
            )),
            WrapGeomType::Ellipsoid { radii } => Some(format!(
                r#"<geom name="{}" type="ellipsoid" size="{} {} {}" pos="{} {} {}" rgba="0.5 0.5 0.9 0.4" group="2"/>"#,
                escape_attr(name), radii[0], radii[1], radii[2],
                self.transform.translation.x, self.transform.translation.y, self.transform.translation.z
            )),
        }
    }
}

impl ExportAs<Mjcf> for Site {
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        Some(format!(
            r#"<site name="{}" pos="{} {} {}"/>"#,
            escape_attr(name), self.offset.x, self.offset.y, self.offset.z
        ))
    }
}

impl ExportAs<Mjcf> for InertialProperties {
    type Output = String;

    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        let i = &self.inertia;
        Some(format!(
            r#"<inertial pos="{} {} {}" mass="{}" fullinertia="{} {} {} {} {} {}"/>"#,
            self.com[0], self.com[1], self.com[2], self.mass,
            i[0], i[1], i[2], i[3], i[4], i[5]
        ))
    }
}

// ── Components with no MJCF representation ────────────

impl ExportAs<Mjcf> for Millard2012Params {
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        // Millard params are consumed by Muscle's export_as,
        // not emitted as a standalone element.
        // But you could override this to produce a more detailed
        // MJCF representation (e.g., explicit force-length curves).
        None
    }
}

impl ExportAs<Mjcf> for MusclePath {
    type Output = String;

    fn export_as(&self, entity: EntityID, ctx: &ExportCtx) -> Option<String> {
        // MusclePath is emitted as <tendon><spatial> in the exporter's
        // tendon section, not as a standalone element here.
        None
    }
}

impl ExportAs<Mjcf> for Frame {
    type Output = String;

    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        // Frame transform is encoded as pos/quat on the <body> element,
        // not as a standalone element.
        None
    }
}

impl ExportAs<Mjcf> for JointCoordinate {
    type Output = String;

    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        // Coordinate dynamics are appended to the joint element
        // by the joint's export_as via append_dynamics().
        None
    }
}

impl ExportAs<Mjcf> for Name {
    type Output = String;

    fn export_as(&self, _entity: EntityID, _ctx: &ExportCtx) -> Option<String> {
        // Name is consumed by other components via ctx.name(),
        // not emitted as a standalone element.
        None
    }
}

// ── Helper: append coordinate dynamics to a joint element ──

fn append_dynamics(ctx: &ExportCtx, joint_entity: EntityID, xml: &mut String) {
    let joint_name = ctx.name(joint_entity);
    if let Some(jname) = joint_name {
        for (coord_key, coord) in ctx.world.iter::<JointCoordinate>() {
            let coord_name = ctx.name(coord_key);
            if coord_name == Some(jname) {
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
