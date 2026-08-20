//! Format-neutral rigid-body model data (the import IR) and the spawner
//! that turns it into entities.

use std::collections::HashMap;
use std::path::PathBuf;

use bevy::prelude::*;
use nalgebra::Isometry3;

use crate::model::{
    Body, Coordinate, CoordinateProperties, CoordinateState, DrivesCoordinate, FixedFrame,
    Function, Geometry, InertialProperties, InitialConditions, Joint, ModelDir, Twist,
};

/// A rigid-body model, produced by the format loaders and consumed by
/// [`spawn_model`]. Format-specific detail (OpenSim sockets, MuJoCo defaults)
/// is resolved by the loader; what remains is bodies, joint offsets,
/// coordinates, and motion axes.
#[derive(Asset, TypePath, Debug)]
pub struct ModelData {
    pub name: String,
    /// Directory prepended to geometry mesh paths at spawn time (the model
    /// file's directory, relative to the asset root).
    pub mesh_dir: PathBuf,
    /// Bodies, including a mass-less ground/world body as the tree root.
    pub bodies: Vec<BodyData>,
    pub joints: Vec<JointData>,
}

#[derive(Debug)]
pub struct BodyData {
    pub name: String,
    pub inertial: InertialProperties,
    pub geometry: Vec<GeometryData>,
}

#[derive(Debug)]
pub struct GeometryData {
    /// The geom's name (for matching against compiled-model data).
    pub name: String,
    /// Mesh path, relative to [`ModelData::mesh_dir`].
    pub mesh: String,
    /// Mesh offset from the body frame.
    pub offset: Isometry3<f64>,
}

#[derive(Debug)]
pub struct JointData {
    pub name: String,
    pub parent_body: String,
    pub child_body: String,
    /// Joint frame offset from the parent body frame.
    pub parent_offset: Isometry3<f64>,
    /// Child body frame offset from the joint frame.
    pub child_offset: Isometry3<f64>,
    pub coordinates: Vec<CoordinateData>,
    /// Motion axes, in composition order (PoE is order-dependent).
    pub axes: Vec<AxisData>,
}

#[derive(Debug)]
pub struct CoordinateData {
    pub name: String,
    pub default_value: f64,
    pub range: (f64, f64),
    pub clamped: bool,
    pub locked: bool,
    pub stiffness: f64,
    pub damping: f64,
}

#[derive(Debug)]
pub struct AxisData {
    pub twist: Twist,
    /// Name of the driving coordinate. Usually one of this joint's
    /// `coordinates`, but OpenSim joints can reference coordinates owned by
    /// other joints (patella driven by the tibiofemoral coordinate).
    /// `None` = no driving coordinate — a fixed offset carried in
    /// `function` (OpenSim TransformAxis with a constant function).
    pub coordinate: Option<String>,
    /// Motion mapping applied to the coordinate value; `None` = identity.
    pub function: Option<Function>,
}

/// Handles to everything [`spawn_model`] created, keyed by model name.
#[derive(Default)]
pub struct SpawnedModel {
    pub bodies: HashMap<String, Entity>,
    pub joints: HashMap<String, Entity>,
    pub coordinates: HashMap<String, Entity>,
}

/// Spawn a [`ModelData`] as a child hierarchy of `parent`.
///
/// Two phases: all bodies first (joints reference them by name), then
/// joints. Axes whose coordinate lives on a later joint (cross-joint
/// coupling) are wired in a deferred pass.
pub fn spawn_model(world: &mut World, parent: Entity, model: &ModelData) -> SpawnedModel {
    let mut spawned = SpawnedModel::default();

    // Stash the asset-root-relative mesh directory on the root so exporters
    // can reconstruct mesh file paths without a separate parameter.
    world
        .entity_mut(parent)
        .insert(ModelDir(model.mesh_dir.clone()));

    // ── Phase 1: bodies (+ geometry) ──
    for b in &model.bodies {
        let body = world
            .spawn((
                Body,
                Name::new(b.name.clone()),
                b.inertial.clone(),
                ChildOf(parent),
            ))
            .id();

        for g in &b.geometry {
            // Only .stl meshes attach (no .vtp loader yet), and only when
            // the app has asset infrastructure — headless spawns (tests,
            // exporters) get plain frame entities.
            let mesh_and_material = if g.mesh.ends_with(".stl") {
                let server = world.get_resource::<AssetServer>().cloned();
                let materials = world.get_resource_mut::<Assets<StandardMaterial>>();
                match (server, materials) {
                    (Some(s), Some(mut m)) => Some((
                        s.load::<Mesh>(model.mesh_dir.join(&g.mesh).to_string_lossy().into_owned()),
                        m.add(StandardMaterial {
                            base_color: Color::srgb(0.82, 0.72, 0.62),
                            ..default()
                        }),
                    )),
                    _ => None,
                }
            } else {
                None
            };
            let mut geom = world.spawn((
                FixedFrame(g.offset),
                Geometry,
                Name::new(g.mesh.clone()),
                ChildOf(body),
            ));
            if let Some((mesh, material)) = mesh_and_material {
                geom.insert((Mesh3d(mesh), MeshMaterial3d(material)));
            }
        }

        spawned.bodies.insert(b.name.clone(), body);
    }

    // ── Phase 2: joints ──
    // (twist, coordinate name, function) for axes whose coordinate entity
    // doesn't exist yet.
    let mut deferred_axes = Vec::new();

    for j in &model.joints {
        let Some(&parent_body) = spawned.bodies.get(&j.parent_body) else {
            warn!(
                "joint '{}': unknown parent body '{}', skipped",
                j.name, j.parent_body
            );
            continue;
        };
        let Some(&child_body) = spawned.bodies.get(&j.child_body) else {
            warn!(
                "joint '{}': unknown child body '{}', skipped",
                j.name, j.child_body
            );
            continue;
        };

        let parent_frame = world
            .spawn((
                FixedFrame(j.parent_offset),
                Name::new(format!("{}_parent", j.name)),
                ChildOf(parent_body),
            ))
            .id();
        let joint = world
            .spawn((Joint, Name::new(j.name.clone()), ChildOf(parent_frame)))
            .id();

        for c in &j.coordinates {
            let coord = world
                .spawn((
                    Coordinate,
                    CoordinateProperties {
                        range: c.range,
                        clamped: c.clamped,
                        locked: c.locked,
                        stiffness: c.stiffness,
                        damping: c.damping,
                    },
                    InitialConditions {
                        value: c.default_value,
                        velocity: 0.0,
                    },
                    CoordinateState {
                        value: c.default_value,
                        velocity: 0.0,
                    },
                    Name::new(c.name.clone()),
                    ChildOf(joint),
                ))
                .id();
            spawned.coordinates.insert(c.name.clone(), coord);
        }

        for a in &j.axes {
            match &a.coordinate {
                None => spawn_axis(world, joint, None, a),
                Some(name) => match spawned.coordinates.get(name) {
                    Some(&coord) => spawn_axis(world, joint, Some(coord), a),
                    None => deferred_axes.push((joint, a)),
                },
            }
        }

        let child_frame = world
            .spawn((
                FixedFrame(j.child_offset),
                Name::new(format!("{}_child", j.name)),
                ChildOf(joint),
            ))
            .id();
        world.entity_mut(child_body).insert(ChildOf(child_frame));

        spawned.joints.insert(j.name.clone(), joint);
    }

    for (joint, a) in deferred_axes {
        let name = a
            .coordinate
            .as_deref()
            .expect("deferred axes always name one");
        if let Some(&coord) = spawned.coordinates.get(name) {
            spawn_axis(world, joint, Some(coord), a);
        } else {
            warn!("axis references unknown coordinate '{name}', skipped");
        }
    }

    spawned
}

fn spawn_axis(world: &mut World, joint: Entity, coord: Option<Entity>, axis: &AxisData) {
    let mut entity = world.spawn((axis.twist.clone(), ChildOf(joint)));
    if let Some(coord) = coord {
        entity.insert(DrivesCoordinate(coord));
    }
    if let Some(f) = &axis.function {
        entity.insert(f.clone());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Inertia;
    use nalgebra::Vector3;

    fn two_body_model() -> ModelData {
        ModelData {
            name: "test".into(),
            mesh_dir: PathBuf::new(),
            bodies: vec![
                BodyData {
                    name: "ground".into(),
                    inertial: InertialProperties {
                        mass: 0.0,
                        mass_center: Vector3::zeros(),
                        inertia: Inertia::default(),
                    },
                    geometry: vec![],
                },
                BodyData {
                    name: "child".into(),
                    inertial: InertialProperties {
                        mass: 1.0,
                        mass_center: Vector3::zeros(),
                        inertia: Inertia::new(1.0, 1.0, 1.0, 0.0, 0.0, 0.0),
                    },
                    geometry: vec![],
                },
            ],
            joints: vec![JointData {
                name: "hinge".into(),
                parent_body: "ground".into(),
                child_body: "child".into(),
                parent_offset: Isometry3::translation(1.0, 0.0, 0.0),
                child_offset: Isometry3::identity(),
                coordinates: vec![CoordinateData {
                    name: "hinge_flex".into(),
                    default_value: 0.25,
                    range: (0.0, 1.5),
                    clamped: true,
                    locked: false,
                    stiffness: 0.0,
                    damping: 0.0,
                }],
                axes: vec![AxisData {
                    twist: Twist {
                        angular: Vector3::z(),
                        linear: Vector3::zeros(),
                    },
                    coordinate: Some("hinge_flex".into()),
                    function: None,
                }],
            }],
        }
    }

    #[test]
    fn spawn_builds_wired_kinematic_chain() {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            bevy::transform::TransformPlugin,
        ));
        app.add_systems(
            PostUpdate,
            crate::model::sync_kinematics
                .before(bevy::transform::TransformSystems::Propagate),
        );

        let root = app.world_mut().spawn(Transform::default()).id();
        let spawned = spawn_model(app.world_mut(), root, &two_body_model());

        // Structure: child body's ancestors are child frame → joint → parent
        // frame → ground.
        let child = spawned.bodies["child"];
        let mut ancestors = app.world_mut().query::<&ChildOf>();
        let frame = ancestors.get(app.world(), child).unwrap().0;
        let joint = ancestors.get(app.world(), frame).unwrap().0;
        assert_eq!(joint, spawned.joints["hinge"]);
        let parent_frame = ancestors.get(app.world(), joint).unwrap().0;
        let ground = ancestors.get(app.world(), parent_frame).unwrap().0;
        assert_eq!(ground, spawned.bodies["ground"]);

        // Coordinate spawned with its default value.
        let coord = spawned.coordinates["hinge_flex"];
        assert_eq!(
            app.world().get::<CoordinateState>(coord).unwrap().value,
            0.25
        );

        // Kinematics: flex the coordinate, the child body's GlobalTransform
        // is parent_offset ∘ rotz(q).
        app.world_mut()
            .get_mut::<CoordinateState>(coord)
            .unwrap()
            .value = 0.5;
        app.update();
        let global = app.world().get::<GlobalTransform>(child).unwrap();
        let t = global.translation();
        assert!((t.x - 1.0).abs() < 1e-6 && t.y.abs() < 1e-6);
        let expected = bevy::math::Quat::from_rotation_z(0.5);
        assert!(global.rotation().angle_between(expected) < 1e-5);
    }
}
