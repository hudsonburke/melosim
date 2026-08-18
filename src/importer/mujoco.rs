//! MuJoCo MJCF import via `mujoco-rs`'s `MjSpec`: MuJoCo itself parses the
//! XML (includes, compiler defaults, classes) and we walk the spec.
//!
//! Mapping notes:
//! - A MuJoCo body becomes a [`BodyData`]; its joint element(s) become ONE
//!   [`JointData`] whose axes compose — exactly how MuJoCo composes multiple
//!   joints within a body.
//! - The joint anchor (`pos`) folds into the parent/child offsets:
//!   `parent_offset = body_placement ∘ anchor`, `child_offset = -anchor`.
//! - MuJoCo ball/free joints use quaternion qpos; here they become 3
//!   exponential-map rotation coordinates. Structure is preserved; qpos
//!   interop with a running MuJoCo instance is NOT (yet).
//! - Sites, actuators, tendons, equality constraints: not imported (yet).

use std::collections::HashMap;
use std::path::Path;

use bevy::log::warn;
use bevy::asset::io::Reader;
use bevy::asset::{AssetLoader, LoadContext};
use bevy::reflect::TypePath;
use mujoco_rs::wrappers::mj_editing::*; // MjSpec, MjsBody, MjsJoint, MjtLimited, SpecItem (name())
use mujoco_rs::wrappers::mj_model::{MjtJoint, MjtObj};
use nalgebra::{Isometry3, Quaternion, Translation, UnitQuaternion, Vector3};

use super::{AxisData, BodyData, CoordinateData, GeometryData, JointData, ModelData};
use crate::model::{Inertia, InertialProperties, Twist};

/// Parse the MJCF file at `path` into the format-neutral IR.
pub fn extract_mjcf(path: &Path) -> Result<ModelData, String> {
    let mut spec =
        MjSpec::from_xml(path).map_err(|e| format!("failed to load MJCF: {e}"))?;

    let mut model = ModelData {
        name: path
            .file_stem()
            .map(|s| s.to_string_lossy().into_owned())
            .unwrap_or_else(|| "model".into()),
        // MuJoCo resolves mesh files as <model dir>/<compiler meshdir>/<file>.
        mesh_dir: path
            .parent()
            .unwrap_or(Path::new(""))
            .join(spec.compiler().meshdir()),
        bodies: vec![BodyData {
            name: "ground".into(),
            inertial: InertialProperties {
                mass: 0.0,
                mass_center: Vector3::zeros(),
                inertia: Inertia::default(),
            },
            geometry: vec![],
        }],
        joints: vec![],
    };

    let meshes: HashMap<String, String> = spec
        .mesh_iter()
        .map(|m| (m.name().to_string(), m.file().to_string()))
        .collect();

    for body in spec.world_body().body_iter(false) {
        walk_body(body, "ground", &meshes, &mut model)?;
    }

    // Compile the spec to resolve geom orientations. Mesh geoms may
    // specify orientation via `euler`/`axisangle` (resolved into the mesh's
    // reference frame, not the geom's quat), and the mesh itself may have a
    // reference frame offset baked in. The compiled model has the final
    // resolved values.
    match spec.compile() {
        Ok(compiled) => {
            for body in &mut model.bodies {
                for geom in &mut body.geometry {
                    if let Some(gid) = compiled.name_to_id(MjtObj::mjOBJ_GEOM, &geom.name) {
                        let gq = compiled.geom_quat()[gid];
                        let gp = compiled.geom_pos()[gid];
                        let mut offset = iso_from_pos_quat(gp, gq);

                        // Compose the mesh's reference frame (refpos/refquat
                        // baked into the mesh data during compilation).
                        let mesh_id = compiled.geom_dataid()[gid];
                        if mesh_id >= 0 {
                            let mq = compiled.mesh_quat()[mesh_id as usize];
                            let mp = compiled.mesh_pos()[mesh_id as usize];
                            offset *= iso_from_pos_quat(mp, mq);
                        }

                        geom.offset = offset;
                    }
                }
            }
        }
        Err(e) => {
            warn!("MjSpec compile failed (geom orientations may be wrong): {e}");
        }
    }
    Ok(model)
}

fn walk_body(
    body: &MjsBody,
    parent_name: &str,
    meshes: &HashMap<String, String>,
    model: &mut ModelData,
) -> Result<(), String> {
    let name = body.name().to_string();
    let placement = iso_from_pos_quat(*body.pos(), *body.quat());

    let geometry = body
        .geom_iter(false)
        .filter(|g| !g.meshname().is_empty())
        .filter_map(|g| {
            meshes.get(g.meshname()).map(|file| GeometryData {
                name: g.name().to_string(),
                mesh: file.clone(),
                offset: iso_from_pos_quat(*g.pos(), *g.quat()),
            })
        })
        .collect();

    model.bodies.push(BodyData {
        name: name.clone(),
        inertial: InertialProperties {
            mass: body.mass(),
            mass_center: Vector3::from(*body.ipos()),
            inertia: Inertia(*body.fullinertia()),
        },
        geometry,
    });

    // All joint elements of this body compose into one JointData.
    let joints: Vec<_> = body.joint_iter(false).collect();
    let anchor = joints.first().map(|j| Vector3::from(*j.pos())).unwrap_or_default();

    let mut coordinates = Vec::new();
    let mut axes = Vec::new();
    for &j in &joints {
        push_joint_axes(j, &name, &mut coordinates, &mut axes);
    }

    model.joints.push(JointData {
        name: joints
            .first()
            .filter(|j| !j.name().is_empty())
            .map(|j| j.name().to_string())
            .unwrap_or_else(|| format!("{name}_joint")),
        parent_body: parent_name.to_string(),
        child_body: name.clone(),
        parent_offset: placement * Translation::from(anchor),
        child_offset: Isometry3::from(Translation::from(-anchor)),
        coordinates,
        axes,
    });

    for child in body.body_iter(false) {
        walk_body(child, &name, meshes, model)?;
    }
    Ok(())
}

fn push_joint_axes(
    joint: &MjsJoint,
    body_name: &str,
    coordinates: &mut Vec<CoordinateData>,
    axes: &mut Vec<AxisData>,
) {
    let base = if joint.name().is_empty() {
        format!("{body_name}_{:?}", joint.type_())
    } else {
        joint.name().to_string()
    };
    let limited = matches!(joint.limited(), MjtLimited::mjLIMITED_TRUE);
    let range = *joint.range();
    let mut coord = |suffix: &str, default_value: f64| {
        let c = CoordinateData {
            name: format!("{base}{suffix}"),
            default_value,
            range: if limited { (range[0], range[1]) } else { (-f64::MAX, f64::MAX) },
            clamped: limited,
            locked: false,
            stiffness: joint.stiffness().first().copied().unwrap_or(0.0),
            damping: joint.damping().first().copied().unwrap_or(0.0),
        };
        coordinates.push(c);
        format!("{base}{suffix}")
    };
    let mut axis = |coordinate: String, angular: Vector3<f64>, linear: Vector3<f64>| {
        axes.push(AxisData {
            twist: Twist { angular, linear },
            coordinate: Some(coordinate),
            function: None,
        });
    };

    match joint.type_() {
        MjtJoint::mjJNT_HINGE => {
            let name = coord("", *joint.ref_());
            axis(name, Vector3::from(*joint.axis()), Vector3::zeros());
        }
        MjtJoint::mjJNT_SLIDE => {
            let name = coord("", *joint.ref_());
            axis(name, Vector3::zeros(), Vector3::from(*joint.axis()));
        }
        MjtJoint::mjJNT_BALL => {
            for (suffix, dir) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                let name = coord(suffix, 0.0);
                axis(name, dir, Vector3::zeros());
            }
        }
        MjtJoint::mjJNT_FREE => {
            for (suffix, dir) in [("_tx", Vector3::x()), ("_ty", Vector3::y()), ("_tz", Vector3::z())] {
                let name = coord(suffix, 0.0);
                axis(name, Vector3::zeros(), dir);
            }
            for (suffix, dir) in [("_rx", Vector3::x()), ("_ry", Vector3::y()), ("_rz", Vector3::z())] {
                let name = coord(suffix, 0.0);
                axis(name, dir, Vector3::zeros());
            }
        }
    }
}

fn iso_from_pos_quat(pos: [f64; 3], quat: [f64; 4]) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation::from(Vector3::from(pos)),
        // MuJoCo quaternions are (w, x, y, z).
        UnitQuaternion::from_quaternion(Quaternion::new(quat[0], quat[1], quat[2], quat[3])),
    )
}

/// `.xml` → [`ModelData`] asset loader (registered by `ImporterPlugin`).
#[derive(TypePath)]
pub struct MjcfLoader {
    /// Joined with the asset path to get the real filesystem path MjSpec
    /// parses from (it resolves mesh includes itself).
    pub asset_root: std::path::PathBuf,
}

impl AssetLoader for MjcfLoader {
    type Asset = ModelData;
    type Settings = ();
    type Error = std::io::Error;

    async fn load(
        &self,
        _reader: &mut dyn Reader,
        _settings: &Self::Settings,
        load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let real_path = self.asset_root.join(load_context.path().path());
        let mut model = extract_mjcf(&real_path)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        // Mesh paths must be asset-root-relative for later AssetServer loads.
        if let Ok(rel) = model.mesh_dir.strip_prefix(&self.asset_root) {
            model.mesh_dir = rel.to_path_buf();
        }
        Ok(model)
    }

    fn extensions(&self) -> &[&str] {
        &["xml"]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The myo_sim model imports with referential integrity: every joint's
    /// parent/child bodies and every axis's coordinate must exist.
    #[test]
    fn extract_myolegs_structure() {
        let model = extract_mjcf(Path::new("tests/fixtures/myo_sim/osl/myolegs_osl.xml")).unwrap();
        assert!(model.bodies.len() > 3, "expected a real skeleton");

      // Debug: verify euler resolution — print all geoms in compiled model
        {
            let mut spec2 = MjSpec::from_xml(std::path::Path::new(
                "tests/fixtures/myo_sim/osl/myolegs_osl.xml",
            ))
            .unwrap();
            let compiled = spec2.compile().unwrap();
            println!("DEBUG ngeom={}", compiled.ngeom());
            for i in 0..compiled.ngeom() as usize {
                let name = compiled.id_to_name(MjtObj::mjOBJ_GEOM, i as _);
                let q = compiled.geom_quat()[i];
                println!("  geom[{i}] name={name:?} quat=[{:.4} {:.4} {:.4} {:.4}]",
                    q[0], q[1], q[2], q[3]);
            }
            // Also print what our walk found
            for b in &model.bodies {
                for g in &b.geometry {
                    let q = g.offset.rotation;
                    println!("  OURS body={} geom={} quat=[{:.4} {:.4} {:.4} {:.4}]",
                        b.name, g.name, q.w, q.i, q.j, q.k);
                }
            }
        }


        for j in &model.joints {
            assert!(model.bodies.iter().any(|b| b.name == j.parent_body), "parent {}", j.parent_body);
            assert!(model.bodies.iter().any(|b| b.name == j.child_body), "child {}", j.child_body);
            for a in &j.axes {
                let Some(name) = &a.coordinate else { continue };
                assert!(
                    j.coordinates.iter().any(|c| &c.name == name),
                    "axis coordinate {name} of joint {}",
                    j.name,
                );
            }
        }

        // And the whole thing spawns.
        let mut app = bevy::app::App::new();
        app.add_plugins(bevy::app::TaskPoolPlugin::default());
        let root = app.world_mut().spawn_empty().id();
        let spawned = super::super::spawn_model(app.world_mut(), root, &model);
        assert_eq!(spawned.bodies.len(), model.bodies.len());
    }
}

