//! MuJoCo MJCF → melosim model (built anew for the *current* model).
//!
//! `mujoco-rs`'s `MjSpec` parses the XML (includes, defaults, classes); we walk
//! the spec and spawn current-model entities so an imported model visualizes in
//! the editor: `Body` (+ `InertialProperties`), `Site`, `Joint` →
//! `Coordinate` (+ `Twist`, `CoordinateProperties`, `CoordinateState`).
//!
//! Bodies are parented to their MJCF parent body directly (static pose for
//! visualization); joints attach to the body they drive. See
//! `docs/plans/2026-08-20-mujoco-exporter.md` for the round-trip plan.

use std::path::Path;

use bevy::ecs::world::World;
use bevy::prelude::*;
use mujoco_rs::wrappers::mj_editing::*;
use mujoco_rs::wrappers::mj_model::MjtJoint;
use nalgebra::Vector3;

use crate::model::{
    Body, Coordinate, CoordinateProperties, CoordinateState, Frame, Inertia, InertialProperties,
    Joint, JointCoordinates, Site, Twist,
};

/// Import failure.
#[derive(Debug)]
pub enum ImportError {
    Load(String),
}

/// Parse the MJCF at `path` and spawn the model into `world`.
/// Returns the top-level container entity.
pub fn import_mjcf(world: &mut World, path: &Path) -> Result<Entity, ImportError> {
    let spec = MjSpec::from_xml(path).map_err(|e| ImportError::Load(format!("{e}")))?;
    let model_name = path
        .file_stem()
        .map(|s| s.to_string_lossy().into_owned())
        .unwrap_or_else(|| "model".into());

    // Top-level container (Frame) so imported bodies group under it.
    let anchor = world.spawn((Name::new(model_name), Frame)).id();

    let world_body = spec.world_body();
    let mut counter = 0u64;
    for child_ent in world_body.body_iter(false) {
        spawn_body(world, child_ent, anchor, &mut counter)?;
    }
    Ok(anchor)
}

/// MJCF (Z-up) → melosim (Y-up): invert the exporter's `zup`.
fn zup_to_yup(v: Vec3) -> Vec3 {
    Vec3::new(v.x, v.z, -v.y)
}

fn zup_quat_to_yup(q: [f64; 4]) -> Quat {
    // MuJoCo quat is (w,x,y,z); Bevy is xyzw.
    let mj = Quat::from_xyzw(q[1] as f32, q[2] as f32, q[3] as f32, q[0] as f32);
    // Rotation mapping Z-up→Y-up: +90° about X.
    let r = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
    r * mj
}

fn to_vec3(a: [f64; 3]) -> Vec3 {
    Vec3::new(a[0] as f32, a[1] as f32, a[2] as f32)
}

fn spawn_body(
    world: &mut World,
    mj: &MjsBody,
    parent: Entity,
    counter: &mut u64,
) -> Result<(), ImportError> {
    let name = if mj.name().is_empty() {
        *counter += 1;
        format!("body_{}", *counter)
    } else {
        mj.name().to_string()
    };

    let mut body = world.spawn((
        Name::new(name.clone()),
        Body,
        InertialProperties {
            mass: mj.mass(),
            mass_center: Vector3::from(*mj.ipos()),
            inertia: Inertia(*mj.fullinertia()),
        },
        Transform::from_translation(zup_to_yup(to_vec3(*mj.pos())))
            .with_rotation(zup_quat_to_yup(*mj.quat())),
    ));
    if parent != Entity::PLACEHOLDER {
        body.insert(ChildOf(parent));
    }
    let body_ent = body.id();

    // Joints on this body (driving it) → Joint + Coordinate + Twist.
    let joints: Vec<_> = mj.joint_iter(false).collect();
    for j in joints {
        spawn_joint(world, j, body_ent, counter)?;
    }

    // Sites (points on the body).
    for s in mj.site_iter(false) {
        let sname = if s.name().is_empty() {
            *counter += 1;
            format!("site_{}", *counter)
        } else {
            s.name().to_string()
        };
        world
            .spawn((
                Name::new(sname),
                Site,
                Transform::from_translation(zup_to_yup(to_vec3(*s.pos()))),
            ))
            .insert(ChildOf(body_ent));
    }

    // Children bodies.
    for child in mj.body_iter(false) {
        spawn_body(world, child, body_ent, counter)?;
    }
    Ok(())
}

fn spawn_joint(
    world: &mut World,
    mj: &MjsJoint,
    body_ent: Entity,
    counter: &mut u64,
) -> Result<(), ImportError> {
    let base = if mj.name().is_empty() {
        *counter += 1;
        format!("joint_{}", *counter)
    } else {
        mj.name().to_string()
    };

    let mut coord_ids: Vec<Entity> = Vec::new();
    match mj.type_() {
        MjtJoint::mjJNT_HINGE => coord_ids.push(spawn_coord(
            world, mj, &base, "", Vector3::from(*mj.axis()), Vector3::zeros(), counter,
        )),
        MjtJoint::mjJNT_SLIDE => coord_ids.push(spawn_coord(
            world, mj, &base, "", Vector3::zeros(), Vector3::from(*mj.axis()), counter,
        )),
        MjtJoint::mjJNT_BALL => {
            for (s, a) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                coord_ids.push(spawn_coord(world, mj, &base, s, a, Vector3::zeros(), counter));
            }
        }
        MjtJoint::mjJNT_FREE => {
            for (s, a) in [("_tx", Vector3::x()), ("_ty", Vector3::y()), ("_tz", Vector3::z())] {
                coord_ids.push(spawn_coord(world, mj, &base, s, Vector3::zeros(), a, counter));
            }
            for (s, a) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                coord_ids.push(spawn_coord(world, mj, &base, s, a, Vector3::zeros(), counter));
            }
        }
    }

    let joint_name = if coord_ids.len() == 1 {
        base.clone()
    } else {
        format!("{base}_{}", coord_ids.len())
    };
    world
        .spawn((
            Name::new(joint_name),
            Joint,
            JointCoordinates::new(coord_ids),
            Transform::IDENTITY,
        ))
        .insert(ChildOf(body_ent));
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn spawn_coord(
    world: &mut World,
    mj: &MjsJoint,
    base: &str,
    suffix: &str,
    angular: Vector3<f64>,
    linear: Vector3<f64>,
    counter: &mut u64,
) -> Entity {
    let _ = counter;
    let limited = matches!(mj.limited(), MjtLimited::mjLIMITED_TRUE);
    let range = *mj.range();
    world
        .spawn((
            Name::new(format!("{base}{suffix}")),
            Coordinate,
            Twist { angular, linear },
            CoordinateProperties {
                range: if limited { (range[0], range[1]) } else { (-f64::MAX, f64::MAX) },
                clamped: limited,
                locked: false,
                stiffness: mj.stiffness().first().copied().unwrap_or(0.0),
                damping: mj.damping().first().copied().unwrap_or(0.0),
            },
            CoordinateState { value: *mj.ref_(), velocity: 0.0 },
        ))
        .id()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp_mjcf(xml: &str) -> std::path::PathBuf {
        let p = std::env::temp_dir().join("melosim_import_test.xml");
        std::fs::write(&p, xml).unwrap();
        p
    }

    /// A minimal 2-body hinge model: parses and spawns bodies + joint.
    #[test]
    fn imports_two_body_hinge() {
        let xml = r#"<mujoco model="test2">
          <worldbody>
            <body name="base" pos="0 0 0">
              <joint name="hinge" type="hinge" axis="0 0 1" range="-1 1" damping="0.5"/>
              <body name="link" pos="0 0 1">
                <site name="tip" pos="0 0 0.1"/>
              </body>
            </body>
          </worldbody>
        </mujoco>"#;
        let path = tmp_mjcf(xml);
        let mut world = World::new();
        let anchor = import_mjcf(&mut world, &path).expect("import");
        let _ = anchor;

        assert!(
            world.query_filtered::<Entity, With<Body>>().iter(&world).count() >= 2,
            "expected base + link bodies"
        );
        assert!(
            world.query_filtered::<Entity, With<Joint>>().iter(&world).count() >= 1,
            "expected a joint"
        );
        assert!(
            world.query_filtered::<Entity, With<Site>>().iter(&world).count() >= 1,
            "expected a site"
        );
    }
}
