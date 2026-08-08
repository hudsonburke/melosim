// ── MJCF component exporters ─────────────────────────

use super::trait_export::{escape_attr, ExportAs, ExportCtx, Mjcf};
use crate::components::*;
use bevy_ecs::prelude::Entity;

impl ExportAs<Mjcf> for Muscle {
    fn export_as(&self, entity: Entity, ctx: &ExportCtx) -> Option<String> {
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

impl ExportAs<Mjcf> for CoordinateActuator {
    fn export_as(&self, entity: Entity, ctx: &ExportCtx) -> Option<String> {
        let name = ctx.name_or_unnamed(entity);
        let coord_name = ctx.name_or_unnamed(self.coordinate);
        Some(format!(
            r#"<general name="{}" joint="{}" gear="{}" ctrlrange="{} {}"/>"#,
            escape_attr(name), escape_attr(coord_name),
            self.optimal_force, self.min_control, self.max_control
        ))
    }
}

impl ExportAs<Mjcf> for DisplayGeometry {
    fn export_as(&self, entity: Entity, ctx: &ExportCtx) -> Option<String> {
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
    fn export_as(&self, entity: Entity, ctx: &ExportCtx) -> Option<String> {
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

impl ExportAs<Mjcf> for InertialProperties {
    fn export_as(&self, _entity: Entity, _ctx: &ExportCtx) -> Option<String> {
        let i = &self.inertia;
        Some(format!(
            r#"<inertial pos="{} {} {}" mass="{}" fullinertia="{} {} {} {} {} {}"/>"#,
            self.com[0], self.com[1], self.com[2], self.mass,
            i[0], i[1], i[2], i[3], i[4], i[5]
        ))
    }
}

impl ExportAs<Mjcf> for Millard2012Params {
    fn export_as(&self, _entity: Entity, _ctx: &ExportCtx) -> Option<String> { None }
}

impl ExportAs<Mjcf> for MusclePath {
    fn export_as(&self, _entity: Entity, _ctx: &ExportCtx) -> Option<String> { None }
}

impl ExportAs<Mjcf> for JointCoordinate {
    fn export_as(&self, _entity: Entity, _ctx: &ExportCtx) -> Option<String> { None }
}

impl ExportAs<Mjcf> for Name {
    fn export_as(&self, _entity: Entity, _ctx: &ExportCtx) -> Option<String> { None }
}
