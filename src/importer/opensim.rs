//! OpenSim `.osim` import via the official Python API (PyO3).
//!
//! OpenSim itself parses the XML — including older document versions, which
//! it upgrades on load — and we walk the live model object. Environment on
//! this machine comes from `.cargo/config.toml` (venv + library paths).
//!
//! Mapping notes:
//! - Socket frames are resolved to `(body, composed offset)` by walking up
//!   the `PhysicalOffsetFrame` chain; those become the joint's parent/child
//!   offsets.
//! - `CustomJoint` spatial transforms become axes; the built-in joint types
//!   (Pin, Slider, Ball, Universal, Free, Weld) map to their fixed axis
//!   layouts. Other types import their coordinates but no axes (warned).
//! - `TransformAxis` entries with no coordinate are constant offsets and are
//!   skipped, matching the old importer's behavior.
//! - Markers, muscles, wrap objects, actuators, contact: not imported (yet).

use std::path::Path;

use bevy::asset::io::Reader;
use bevy::asset::{AssetLoader, LoadContext};
use bevy::log::warn;
use bevy::reflect::TypePath;
use nalgebra::{Isometry3, Translation, UnitQuaternion, Vector3};
use pyo3::prelude::*;

use super::{AxisData, BodyData, CoordinateData, GeometryData, JointData, ModelData};
use crate::model::{Function, Inertia, InertialProperties, Twist};

type Res<T> = Result<T, String>;

/// Parse the `.osim` file at `path` into the format-neutral IR.
pub fn extract_osim(path: &Path) -> Res<ModelData> {
    Python::attach(|py| {
        let osim = py.import("opensim").map_err(|e| {
            format!("import opensim failed (check PYTHONPATH/LD_LIBRARY_PATH): {e}")
        })?;
        let model = osim
            .getattr("Model")
            .and_then(|m| m.call1((path.to_string_lossy().as_ref(),)))
            .map_err(|e| format!("failed to load {}: {e}", path.display()))?;

        let mut data = ModelData {
            name: get_str(&model, "getName")?,
            mesh_dir: path.parent().unwrap_or(Path::new("")).to_path_buf(),
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

        let body_set = call0(&model, "getBodySet")?;
        let n: usize = call0(&body_set, "getSize")?.extract().map_err(err)?;
        for i in 0..n {
            data.bodies
                .push(extract_body(&osim, &call1(&body_set, "get", i)?)?);
        }

        let joint_set = call0(&model, "getJointSet")?;
        let n: usize = call0(&joint_set, "getSize")?.extract().map_err(err)?;
        for i in 0..n {
            data.joints
                .push(extract_joint(&osim, &call1(&joint_set, "get", i)?)?);
        }

        Ok(data)
    })
}

// ── Bodies ────────────────────────────────────────────

fn extract_body(osim: &Bound<'_, PyModule>, body: &Bound<'_, PyAny>) -> Res<BodyData> {
    let inertia = call0(body, "getInertia")?;
    let moments = vec3(&call0(&inertia, "getMoments")?)?;
    let products = vec3(&call0(&inertia, "getProducts")?)?;

    Ok(BodyData {
        name: get_str(body, "getName")?,
        inertial: InertialProperties {
            mass: f64_or(body, "getMass", 0.0),
            mass_center: vec3(&call0(body, "getMassCenter")?)?,
            inertia: Inertia::new(
                moments[0],
                moments[1],
                moments[2],
                products[0],
                products[1],
                products[2],
            ),
        },
        geometry: extract_meshes(osim, body),
    })
}

fn extract_meshes(osim: &Bound<'_, PyModule>, body: &Bound<'_, PyAny>) -> Vec<GeometryData> {
    let mut out = Vec::new();
    // get_attached_geometry(i) takes an index, not a zero-arg collection.
    for i in 0..32 {
        let Ok(geom) = call1(body, "get_attached_geometry", i) else {
            break;
        };
        let Ok(Some(mesh)) = downcast(osim, "Mesh", &geom) else {
            continue;
        };
        let Ok(file) = get_str(&mesh, "get_mesh_file") else {
            continue;
        };
        // OpenSim convention: mesh files are relative to <model_dir>/Geometry/.
        let file = if file.contains('/') || file.contains('\\') {
            file
        } else {
            format!("Geometry/{file}")
        };
        out.push(GeometryData {
            name: file.clone(),
            mesh: file,
            offset: Isometry3::identity(),
        });
    }
    out
}

// ── Joints ────────────────────────────────────────────

fn extract_joint(osim: &Bound<'_, PyModule>, joint: &Bound<'_, PyAny>) -> Res<JointData> {
    let name = get_str(joint, "getName")?;
    let class = get_str(joint, "getConcreteClassName")?;

    // Socket frames → (body name, composed offset from that body).
    let (parent_body, parent_offset) = resolve_frame(osim, &call0(joint, "getParentFrame")?)?;
    let (child_body, socket_offset) = resolve_frame(osim, &call0(joint, "getChildFrame")?)?;
    // resolve_frame gives T(child_body → socket frame); the tree needs the
    // child body's offset from the JOINT frame — the inverse.
    let child_offset = socket_offset.inverse();

    // Coordinates.
    let n: usize = call0(joint, "numCoordinates")?.extract().map_err(err)?;
    let mut coordinates = Vec::with_capacity(n);
    for i in 0..n {
        coordinates.push(extract_coordinate(&call1(joint, "get_coordinates", i)?)?);
    }

    // Motion axes.
    let axes = if class == "CustomJoint" {
        extract_transform_axes(osim, joint)?
    } else {
        builtin_axes(&name, &class, &coordinates)
    };

    Ok(JointData {
        name,
        parent_body,
        child_body,
        parent_offset,
        child_offset,
        coordinates,
        axes,
    })
}

/// Walk a socket frame up its `PhysicalOffsetFrame` chain to the owning
/// body, composing the fixed offsets along the way.
fn resolve_frame(
    osim: &Bound<'_, PyModule>,
    frame: &Bound<'_, PyAny>,
) -> Res<(String, Isometry3<f64>)> {
    let mut current = frame.clone();
    let mut composed = Isometry3::identity();
    loop {
        let Some(offset_frame) = downcast(osim, "PhysicalOffsetFrame", &current)? else {
            // Not an offset frame — it's the body itself.
            return Ok((get_str(&current, "getName")?, composed));
        };
        let t = vec3(&call0(&offset_frame, "get_translation")?)?;
        let r = vec3(&call0(&offset_frame, "get_orientation")?)?;
        composed = Isometry3::from_parts(Translation::from(t), euler_xyz(r)) * composed;
        current = call0(&offset_frame, "getParentFrame")?;
    }
}

fn extract_coordinate(coord: &Bound<'_, PyAny>) -> Res<CoordinateData> {
    Ok(CoordinateData {
        name: get_str(coord, "getName")?,
        default_value: f64_or(coord, "getDefaultValue", 0.0),
        range: (
            f64_or(coord, "getRangeMin", -std::f64::consts::PI),
            f64_or(coord, "getRangeMax", std::f64::consts::PI),
        ),
        clamped: bool_or(coord, "get_clamped", false),
        locked: bool_or(coord, "get_locked", false),
        stiffness: f64_or(coord, "getStiffness", 0.0),
        damping: f64_or(coord, "getDamping", 0.0),
    })
}

/// The six `TransformAxis` slots of a CustomJoint. OpenSim composes the
/// joint transform as rotations first, then translations in the parent
/// frame — T = T₁·T₂·T₃·R₁·R₂·R₃ — so translation axes are emitted FIRST
/// (PoE composes children in insertion order).
fn extract_transform_axes(
    osim: &Bound<'_, PyModule>,
    joint: &Bound<'_, PyAny>,
) -> Res<Vec<AxisData>> {
    let Some(cj) = downcast(osim, "CustomJoint", joint)? else {
        return Ok(vec![]);
    };
    let st = call0(&cj, "getSpatialTransform")?;
    let mut axes = Vec::new();
    let getters = [
        ("get_translation1", false),
        ("get_translation2", false),
        ("get_translation3", false),
        ("get_rotation1", true),
        ("get_rotation2", true),
        ("get_rotation3", true),
    ];
    for (getter, is_rotation) in getters {
        let ta = call0(&st, getter)?;
        let axis = vec3(&call0(&ta, "get_axis")?)?;
        if axis.norm() < 1e-12 {
            continue; // unused slot
        }
        let function = extract_function(osim, &call0(&ta, "get_function")?)?;
        let twist = if is_rotation {
            Twist {
                angular: axis,
                linear: Vector3::zeros(),
            }
        } else {
            Twist {
                angular: Vector3::zeros(),
                linear: axis,
            }
        };
        let names = call0(&ta, "getCoordinateNames")?;
        let count: usize = call0(&names, "size")?.extract().map_err(err)?;
        if count == 0 {
            // No driving coordinate: a fixed offset carried by the function.
            // Zero constants are the identity — skip them entirely.
            match &function {
                None => continue,
                Some(Function::Constant(c)) if c.abs() < 1e-12 => continue,
                _ => {}
            }
            axes.push(AxisData {
                twist,
                coordinate: None,
                function,
            });
            continue;
        }
        let coordinate = call1(&names, "getValue", 0usize)?.extract().map_err(err)?;
        axes.push(AxisData {
            twist,
            coordinate: Some(coordinate),
            function,
        });
    }
    Ok(axes)
}

/// Fixed axis layouts for OpenSim's built-in joint types.
fn builtin_axes(name: &str, class: &str, coordinates: &[CoordinateData]) -> Vec<AxisData> {
    let axis = |i: usize, angular: Vector3<f64>, linear: Vector3<f64>| AxisData {
        twist: Twist { angular, linear },
        coordinate: Some(coordinates[i].name.clone()),
        function: None,
    };
    let (x, y, z, o) = (Vector3::x(), Vector3::y(), Vector3::z(), Vector3::zeros());
    match (class, coordinates.len()) {
        ("WeldJoint", _) => vec![],
        ("PinJoint", 1) => vec![axis(0, z, o)],
        ("SliderJoint", 1) => vec![axis(0, o, x)],
        ("UniversalJoint", 2) => vec![axis(0, x, o), axis(1, y, o)],
        ("BallJoint", 3) => vec![axis(0, x, o), axis(1, y, o), axis(2, z, o)],
        // OpenSim composes rotations first, then translations — so
        // translation axes come first in the PoE chain.
        ("FreeJoint", 6) => vec![
            axis(3, o, x),
            axis(4, o, y),
            axis(5, o, z),
            axis(0, x, o),
            axis(1, y, o),
            axis(2, z, o),
        ],
        (other, _) => {
            warn!("joint '{name}': unsupported type '{other}', coordinates imported but no axes");
            vec![]
        }
    }
}

// ── Functions ─────────────────────────────────────────

fn extract_function(osim: &Bound<'_, PyModule>, f: &Bound<'_, PyAny>) -> Res<Option<Function>> {
    let class = get_str(f, "getConcreteClassName")?;
    match class.as_str() {
        "NullFunction" => Ok(None),
        "LinearFunction" => {
            let Some(lf) = downcast(osim, "LinearFunction", f)? else {
                return Ok(None);
            };
            Ok(Some(Function::Linear {
                slope: f64_or(&lf, "getSlope", 1.0),
                intercept: f64_or(&lf, "getIntercept", 0.0),
            }))
        }
        "Constant" => {
            let Some(c) = downcast(osim, "Constant", f)? else {
                return Ok(None);
            };
            Ok(Some(Function::Constant(f64_or(&c, "getValue", 0.0))))
        }
        "PolynomialFunction" => {
            let Some(p) = downcast(osim, "PolynomialFunction", f)? else {
                return Ok(None);
            };
            let coeffs = double_vector(&call0(&p, "getCoefficients")?)?;
            Ok(Some(Function::Polynomial(coeffs)))
        }
        "SimmSpline" => {
            let Some(s) = downcast(osim, "SimmSpline", f)? else {
                return Ok(None);
            };
            let n: usize = call0(&s, "getNumberOfPoints")?.extract().map_err(err)?;
            let mut x = Vec::with_capacity(n);
            let mut y = Vec::with_capacity(n);
            for i in 0..n {
                x.push(call1(&s, "getX", i)?.extract::<f64>().map_err(err)?);
                y.push(call1(&s, "getY", i)?.extract::<f64>().map_err(err)?);
            }
            Ok(Some(Function::CubicSpline { x, y }))
        }
        other => {
            warn!("unsupported function type '{other}', using identity");
            Ok(None)
        }
    }
}

// ── py helpers ────────────────────────────────────────

fn err<E: ToString>(e: E) -> String {
    e.to_string()
}

fn call0<'py>(obj: &Bound<'py, PyAny>, method: &str) -> Res<Bound<'py, PyAny>> {
    obj.call_method0(method)
        .map_err(|e| format!("{method}: {e}"))
}

fn call1<'py, A: IntoPyObject<'py>>(
    obj: &Bound<'py, PyAny>,
    method: &str,
    arg: A,
) -> Res<Bound<'py, PyAny>> {
    obj.call_method1(method, (arg,))
        .map_err(|e| format!("{method}: {e}"))
}

fn get_str(obj: &Bound<'_, PyAny>, method: &str) -> Res<String> {
    call0(obj, method)?.extract().map_err(err)
}

fn f64_or(obj: &Bound<'_, PyAny>, method: &str, default: f64) -> f64 {
    obj.call_method0(method)
        .and_then(|v| v.extract::<f64>())
        .unwrap_or(default)
}

fn bool_or(obj: &Bound<'_, PyAny>, method: &str, default: bool) -> bool {
    obj.call_method0(method)
        .and_then(|v| v.extract::<bool>())
        .unwrap_or(default)
}

fn vec3(v: &Bound<'_, PyAny>) -> Res<Vector3<f64>> {
    let mut out = Vector3::zeros();
    for i in 0..3 {
        out[i] = v
            .get_item(i)
            .and_then(|x| x.extract::<f64>())
            .map_err(err)?;
    }
    Ok(out)
}

/// A SWIG-wrapped `std::vector<double>` (supports `__len__`/`__getitem__`).
fn double_vector(v: &Bound<'_, PyAny>) -> Res<Vec<f64>> {
    let n: usize = call0(v, "__len__")?.extract().map_err(err)?;
    (0..n)
        .map(|i| call1(v, "__getitem__", i)?.extract::<f64>().map_err(err))
        .collect()
}

/// `osim.<class>.safeDownCast(obj)` — `None` when `obj` isn't a `<class>`.
fn downcast<'py>(
    osim: &Bound<'py, PyModule>,
    class: &str,
    obj: &Bound<'py, PyAny>,
) -> Res<Option<Bound<'py, PyAny>>> {
    let cls = osim.getattr(class).map_err(err)?;
    let result = cls.call_method1("safeDownCast", (obj,)).map_err(err)?;
    Ok((!result.is_none()).then_some(result))
}

/// OpenSim orientations are body-fixed Euler XYZ angles (radians):
/// R = Rx(x) · Ry(y) · Rz(z).
fn euler_xyz(v: Vector3<f64>) -> UnitQuaternion<f64> {
    UnitQuaternion::from_axis_angle(&Vector3::x_axis(), v.x)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), v.y)
        * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), v.z)
}

/// `.osim` → [`ModelData`] asset loader (registered by `ImporterPlugin`).
#[derive(TypePath)]
pub struct OpensimLoader {
    /// Joined with the asset path to get the real filesystem path the
    /// OpenSim API parses from.
    pub asset_root: std::path::PathBuf,
}

impl AssetLoader for OpensimLoader {
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
        let mut model = extract_osim(&real_path)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;
        // Mesh paths must be asset-root-relative for later AssetServer loads.
        if let Ok(rel) = model.mesh_dir.strip_prefix(&self.asset_root) {
            model.mesh_dir = rel.to_path_buf();
        }
        Ok(model)
    }

    fn extensions(&self) -> &[&str] {
        &["osim"]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::prelude::{App, GlobalTransform, IntoScheduleConfigs, PostUpdate, Transform};

    fn check_integrity(model: &ModelData) {
        for j in &model.joints {
            assert!(
                model.bodies.iter().any(|b| b.name == j.parent_body),
                "parent {}",
                j.parent_body
            );
            assert!(
                model.bodies.iter().any(|b| b.name == j.child_body),
                "child {}",
                j.child_body
            );
            for a in &j.axes {
                let Some(name) = &a.coordinate else { continue };
                assert!(
                    model
                        .joints
                        .iter()
                        .flat_map(|j| &j.coordinates)
                        .any(|c| &c.name == name),
                    "axis coordinate {name} of joint {}",
                    j.name,
                );
            }
        }
    }

    #[test]
    fn extract_rat_hindlimb() {
        let model = extract_osim(Path::new(
            "tests/fixtures/rat_hindlimb/rat_hindlimb_bilateral.osim",
        ))
        .unwrap();
        assert_eq!(model.bodies.len(), 10); // 9 + ground
        assert_eq!(model.joints.len(), 9);
        check_integrity(&model);

        let mut app = bevy::app::App::new();
        app.add_plugins(bevy::app::TaskPoolPlugin::default());
        let root = app.world_mut().spawn_empty().id();
        let spawned = super::super::spawn_model(app.world_mut(), root, &model);
        assert_eq!(spawned.bodies.len(), model.bodies.len());
        assert_eq!(spawned.joints.len(), model.joints.len());
    }

    #[test]
    fn extract_rajagopal() {
        let model = extract_osim(Path::new("tests/fixtures/Rajagopal/Rajagopal2015.osim")).unwrap();
        assert_eq!(model.bodies.len(), 23); // 22 + ground
        assert_eq!(model.joints.len(), 22);
        check_integrity(&model);
    }

    /// FK oracle: our world transforms must match OpenSim's own forward
    /// kinematics on the same file, same coordinate values. Poses set one
    /// coordinate at a time (isolating each axis's convention), plus the
    /// all-default pose.
    #[test]
    fn fk_matches_opensim_rat_hindlimb() {
        use crate::model::CoordinateState;

        let path = "tests/fixtures/rat_hindlimb/rat_hindlimb_bilateral.osim";
        const TEST_VALUE: f64 = 0.1;

        // ── Our world ──
        let model = extract_osim(Path::new(path)).unwrap();
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            bevy::transform::TransformPlugin,
        ));
        app.add_systems(
            PostUpdate,
            (
                crate::render::sync::sync_fixed_frames,
                crate::render::sync::sync_kinematics,
            )
                .chain()
                .before(bevy::transform::TransformSystems::Propagate),
        );
        let root = app.world_mut().spawn(Transform::default()).id();
        let spawned = crate::importer::spawn_model(app.world_mut(), root, &model);

        // ── Reference: OpenSim in-process ──
        Python::attach(|py| {
            let osim = py.import("opensim").unwrap();
            let m = osim.getattr("Model").unwrap().call1((path,)).unwrap();
            let body_set = call0(&m, "getBodySet").unwrap();
            let coord_set = call0(&m, "getCoordinateSet").unwrap();
            let n: usize = call0(&coord_set, "getSize").unwrap().extract().unwrap();
            let coords: Vec<_> = (0..n)
                .map(|i| call1(&coord_set, "get", i).unwrap())
                .collect();
            let names: Vec<String> = coords
                .iter()
                .map(|c| get_str(c, "getName").unwrap())
                .collect();
            let defaults: Vec<f64> = coords
                .iter()
                .map(|c| f64_or(c, "getDefaultValue", 0.0))
                .collect();

            // Pose values must stay inside each coordinate's range —
            // OpenSim clamps on setValue, we don't.
            let ranges: Vec<(f64, f64)> = coords
                .iter()
                .map(|c| (f64_or(c, "getRangeMin", 0.0), f64_or(c, "getRangeMax", 0.0)))
                .collect();
            let mut poses: Vec<(String, Vec<f64>)> = vec![("default".into(), defaults.clone())];
            for (i, name) in names.iter().enumerate() {
                let mut v = defaults.clone();
                v[i] = (defaults[i] + TEST_VALUE).clamp(ranges[i].0, ranges[i].1);
                poses.push((name.clone(), v));
            }

            for (pose, values) in poses {
                let state = call0(&m, "initSystem").unwrap();
                for (coord, &v) in coords.iter().zip(&values) {
                    // Some models lock coordinates (rat sacroiliac) — unlock
                    // before posing or OpenSim silently keeps the default.
                    coord.call_method1("setLocked", (&state, false)).unwrap();
                    coord.call_method1("setValue", (&state, v)).unwrap();
                }
                call1(&m, "realizePosition", &state).unwrap();

                for (name, &v) in names.iter().zip(&values) {
                    if let Some(&e) = spawned.coordinates.get(name) {
                        app.world_mut().get_mut::<CoordinateState>(e).unwrap().value = v;
                    }
                }
                app.update();

                for (body_name, &entity) in &spawned.bodies {
                    if body_name == "ground" {
                        continue; // identity on both sides
                    }
                    let ours = *app.world().get::<GlobalTransform>(entity).unwrap();
                    let body = call1(&body_set, "get", body_name.as_str()).unwrap();
                    let t = call1(&body, "getTransformInGround", &state).unwrap();
                    let p = vec3(&call0(&t, "p").unwrap()).unwrap();
                    // Simbody vectors aren't subscriptable from Python;
                    // angle-axis Vec4 (angle, x, y, z) is read via `.get(i)`.
                    let aa = call0(&call0(&t, "R").unwrap(), "convertRotationToAngleAxis").unwrap();
                    let aa: [f64; 4] =
                        core::array::from_fn(|i| call1(&aa, "get", i).unwrap().extract().unwrap());

                    let ot = ours.translation();
                    assert!(
                        (f64::from(ot.x) - p[0]).abs() < 1e-4
                            && (f64::from(ot.y) - p[1]).abs() < 1e-4
                            && (f64::from(ot.z) - p[2]).abs() < 1e-4,
                        "pose '{pose}', body '{body_name}': translation ours={ot:?} opensim={p:?}"
                    );
                    let (half, s) = ((aa[0] / 2.0) as f32, ((aa[0] / 2.0) as f32).sin());
                    let theirs = bevy::math::Quat::from_xyzw(
                        aa[1] as f32 * s,
                        aa[2] as f32 * s,
                        aa[3] as f32 * s,
                        half.cos(),
                    );
                    let angle = ours.rotation().angle_between(theirs);
                    assert!(
                        angle < 1e-3,
                        "pose '{pose}', body '{body_name}': rotation differs by {angle} rad"
                    );
                }
            }
        });
    }
}
