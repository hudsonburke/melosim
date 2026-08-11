use bevy::ecs::template::EntityTemplate;
use bevy::prelude::*;
use nalgebra::{Isometry3, Translation, UnitQuaternion, Vector3};

use super::Function;

// nalgebra type aliases — all f64 for simulation precision
type Vec3 = Vector3<f64>;
type UQuat = UnitQuaternion<f64>;
type Iso3 = Isometry3<f64>;

// ── Core types ────────────────────────────────────────

/// A generalized coordinate definition (persistent model data).
///
/// Defines the properties of a scalar degree of freedom: its range,
/// defaults, stiffness, and damping. Written by the editor, read by
/// the simulation. The current value lives in `CoordinateState`.
#[derive(Component, Clone, Debug)]
pub struct Coordinate {
    pub default_value: f64,
    pub default_speed: f64,
    pub range: (f64, f64),
    pub clamped: bool,
    pub locked: bool,
    pub stiffness: f64,
    pub damping: f64,
}

impl Default for Coordinate {
    fn default() -> Self {
        Self {
            default_value: 0.0,
            default_speed: 0.0,
            range: (-std::f64::consts::PI, std::f64::consts::PI),
            clamped: true,
            locked: false,
            stiffness: 0.0,
            damping: 0.0,
        }
    }
}

/// Runtime state of a generalized coordinate (transient simulation data).
///
/// Written by the simulation integrator, read for display and muscle
/// path computation. Separated from `Coordinate` so the model definition
/// is never accidentally mutated during simulation.
#[derive(Component, Clone, Debug, Default)]
pub struct CoordinateState {
    pub value: f64,
    pub velocity: f64,
}

/// Marker for a joint entity — the connection between two frames.
///
/// Nesting position IS the wiring: the joint is a child of the parent frame
/// (a `Body` or `FixedFrame` entity), and the child frame/body nests inside
/// the joint's `Children`:
///
///   Body (parent)
///     └─ FixedFrame (optional parent-side offset, e.g. distal end)
///          └─ Joint
///               ├─ Coordinate + CoordinateState (entity per coordinate)
///               ├─ axis entities (Twist + DrivesCoordinate ──► sibling
///               │    Coordinate) — one per axis, usually written with the
///               │    `rotation_axis` / `translation_axes` scene helpers
///               └─ FixedFrame (optional child-side offset)
///                    └─ Body (child)
///
/// The joint's own `Transform` carries the product-of-exponentials motion
/// (written each frame by `sync_kinematics`); Bevy's transform propagation
/// composes the rest of the chain.
///
/// `Transform` and `Visibility` are required so propagation reaches the
/// child body through this entity (a child with `GlobalTransform`/
/// `InheritedVisibility` whose parent lacks them triggers warning B0004);
/// the simulation never reads them.
#[derive(Component, Clone, Debug, Default)]
#[require(Transform, Visibility)]
pub struct Joint;

/// A fixed frame in the kinematic chain — an offset point on a body where
/// things attach: joints (proximal/distal ends), muscle path points,
/// markers, exoskeleton parts.
///
/// Attach via `ChildOf` (usually by nesting in `bsn!`); the `Isometry3` is
/// the constant offset from the parent entity's frame, in simulation f64.
/// `sync_fixed_frames` copies it into the entity's `Transform` so Bevy's
/// propagation includes it in the render chain.
///
/// `Default` (identity) is the BSN patch base.
#[derive(Component, Clone, Debug, Default)]
#[require(Transform, Visibility)]
pub struct FixedFrame(pub Iso3);

impl FixedFrame {
    pub fn identity() -> Self {
        Self(Iso3::identity())
    }
}

/// A twist in se(3) — the Lie algebra of SE(3).
///
/// A pure rotation has angular = axis, linear = zero.
/// A pure translation has angular = zero, linear = direction.
/// A screw has both.
///
/// `Default` (zero twist) is the BSN patch base; exp(0·θ) is the identity.
#[derive(Component, Clone, Debug, Default)]
pub struct Twist {
    pub angular: Vec3, // ω — rotation axis (not necessarily unit)
    pub linear: Vec3,  // v — translation direction
}

// ── Relationships ─────────────────────────────────────

/// Axis → Coordinate: "this axis is driven by this coordinate."
///
/// Multiple axes can drive the same coordinate (OpenSim coupled knee).
/// Multiple coordinates can drive one joint (ball, free, custom).
///
/// `FromTemplate` lets `bsn!` resolve `DrivesCoordinate(#coord)` — a named
/// entity reference — into the spawned coordinate's `Entity` at spawn time.
#[derive(Component, Clone, FromTemplate)]
#[relationship(relationship_target = CoordinateAxes)]
pub struct DrivesCoordinate(pub Entity);

/// Back-reference: all axes that drive a given coordinate.
#[derive(Component)]
#[relationship_target(relationship = DrivesCoordinate)]
pub struct CoordinateAxes(Vec<Entity>);

/// Coordinate → Coordinate: "these coordinates are coupled."
///
/// Used for MuJoCo equality constraints, OpenSim CoordinateCoupler,
/// tendon coupling, gear ratios.
#[derive(Component)]
#[relationship(relationship_target = LinkedBy)]
pub struct LinkedTo(pub Entity);

/// Back-reference: all coordinates linked to a given coordinate.
#[derive(Component)]
#[relationship_target(relationship = LinkedTo)]
pub struct LinkedBy(Vec<Entity>);

/// The type of coupling between linked coordinates.
#[derive(Component, Clone, Debug)]
pub struct Coupling {
    pub kind: CouplingKind,
}

#[derive(Clone, Debug)]
pub enum CouplingKind {
    /// q_a = f(q_b) — coordinate coupler with function.
    Equality(Function),
    /// Tendon: path length = f(q_a, q_b, ...).
    Tendon,
    /// Gear: q_a * ratio = q_b.
    Gear { ratio: f64 },
}

// ── Twist → SE(3) via exponential map ─────────────────

impl Twist {
    /// Compute exp(ξ · θ) where ξ is this twist and θ is the scalar factor.
    ///
    /// Returns the rigid body transform in SE(3) (f64).
    pub fn exp(&self, theta: f64) -> Iso3 {
        if theta.abs() < 1e-12 {
            return Iso3::identity();
        }

        let w = self.angular;
        let v = self.linear;
        let w_norm = w.norm();

        if w_norm < 1e-12 {
            // Pure translation
            return Iso3::from_parts(Translation::from(v * theta), UQuat::identity());
        }

        // Rotation via exponential map so(3) → SO(3)
        let angle = w_norm * theta;
        let q = UQuat::from_scaled_axis(w * theta);

        // Translation from the se(3) exponential formula:
        // t = v·θ + (1-cos(‖ωθ‖))/(‖ω‖²)·(ω×v) + (‖ωθ‖-sin(‖ωθ‖))/(‖ω‖³)·(ω×(ω×v))
        let sin_angle = angle.sin();
        let cos_angle = angle.cos();

        let w_cross_v = w.cross(&v);
        let w_cross_w_cross_v = w.cross(&w_cross_v);
        let w2 = w_norm * w_norm;

        let t = v * theta
            + w_cross_v * (1.0 - cos_angle) / w2
            + w_cross_w_cross_v * (angle - sin_angle) / (w2 * w_norm);

        Iso3::from_parts(Translation::from(t), q)
    }
}

// ── Axis scene helpers ────────────────────────────────
//
// The joint zoo, decomposed: a joint is its coordinate entities (declared
// inline, named, referenceable) plus a bundle of axes. These helpers build
// the axes; combine them to get the classic joint types:
//
//   hinge  = 1 coordinate + rotation_axis
//   slider = 1 coordinate + translation_axis
//   ball   = 3 coordinates + rotation_axes
//   free   = 6 coordinates + translation_axes + rotation_axes
//   weld   = no axes at all
//   custom = inline Twist entities (add a Function for coupled motion)
//
// `coord` is an `EntityTemplate`, which is what a `#Name` reference inside
// `bsn!` produces — so call these with the coordinate's name:
// `rotation_axis(#r_elbow_flex, Vec3::z())`.

/// A rotational axis driven by `coord` — a hinge joint's moving part.
pub fn rotation_axis(coord: EntityTemplate, axis: Vec3) -> impl Scene {
    bsn! {
        Twist { angular: {axis} }
        DrivesCoordinate({coord})
    }
}

/// A translational axis driven by `coord` — a slider joint's moving part.
pub fn translation_axis(coord: EntityTemplate, axis: Vec3) -> impl Scene {
    bsn! {
        Twist { linear: {axis} }
        DrivesCoordinate({coord})
    }
}

/// X, Y, Z rotational axes driven by three coordinates — a ball joint.
///
/// Axes compose in X, Y, Z order (product of exponentials is order-dependent).
pub fn rotation_axes(x: EntityTemplate, y: EntityTemplate, z: EntityTemplate) -> impl SceneList {
    bsn_list![
        (Twist { angular: Vec3::x() } DrivesCoordinate({x})),
        (Twist { angular: Vec3::y() } DrivesCoordinate({y})),
        (Twist { angular: Vec3::z() } DrivesCoordinate({z})),
    ]
}

/// X, Y, Z translational axes driven by three coordinates — with
/// `rotation_axes`, a free joint.
pub fn translation_axes(x: EntityTemplate, y: EntityTemplate, z: EntityTemplate) -> impl SceneList {
    bsn_list![
        (Twist { linear: Vec3::x() } DrivesCoordinate({x})),
        (Twist { linear: Vec3::y() } DrivesCoordinate({y})),
        (Twist { linear: Vec3::z() } DrivesCoordinate({z})),
    ]
}

// ── PoE evaluation ────────────────────────────────────

/// Evaluate the product-of-exponentials transform for a joint.
///
/// Walks the joint's axis children in insertion order, composing
/// exp(ξ_i · f_i(q_i)) for each axis. Reads the current coordinate
/// value from `CoordinateState`.
pub fn evaluate_joint(
    children: &Children,
    twists: &Query<(&Twist, &DrivesCoordinate, Option<&Function>)>,
    states: &Query<&CoordinateState>,
) -> Iso3 {
    let mut transform = Iso3::identity();

    for child in children.iter() {
        let Ok((twist, drives, func)) = twists.get(child) else {
            continue;
        };

        let q = states.get(drives.0).map(|s| s.value).unwrap_or(0.0);
        let f = func.map(|f| f.evaluate(q)).unwrap_or(q);

        transform *= twist.exp(f);
    }

    transform
}

/// Compute the Jacobian contribution of one axis for a given coordinate.
///
/// Returns the spatial velocity (angular, linear) contributed by
/// this axis to the coordinate's generalized velocity.
pub fn axis_jacobian(twist: &Twist, func: &Function, q: f64) -> (Vec3, Vec3) {
    let df = func.derivative().evaluate(q);
    (twist.angular * df, twist.linear * df)
}

// ── Rendering boundary: nalgebra → glam ───────────────

/// Convert a simulation isometry to a Bevy Transform (f32).
///
/// Use this when writing simulation state back to rendering components.
pub fn to_bevy_transform(iso: &Iso3) -> Transform {
    let t = iso.translation.vector;
    let r = iso.rotation;
    Transform {
        translation: bevy::math::Vec3::new(t.x as f32, t.y as f32, t.z as f32),
        rotation: bevy::math::Quat::from_xyzw(r.i as f32, r.j as f32, r.k as f32, r.w as f32),
        ..default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::Body;
    use bevy::{app::App, asset::AssetPlugin, scene::ScenePlugin};

    /// A `bsn!` joint must resolve `DrivesCoordinate(#coord)` to the spawned
    /// coordinate entity and maintain the `CoordinateAxes` back-reference.
    #[test]
    fn bsn_axis_coordinate_wiring() {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            AssetPlugin::default(),
            ScenePlugin::default(),
        ));

        app.world_mut()
            .spawn_scene(bsn! {
                Joint
                Children [
                    (
                        #flex
                        Coordinate
                        CoordinateState { value: 0.25 }
                    ),
                    (
                        Twist { angular: Vec3::z() }
                        DrivesCoordinate(#flex)
                    ),
                ]
            })
            .unwrap();

        let world = app.world_mut();
        let mut axes = world.query::<(Entity, &DrivesCoordinate)>();
        let (axis, drives) = axes.single(world).expect("one axis entity");

        // The reference resolved to the coordinate entity.
        let coord = drives.0;
        assert!(world.get::<Coordinate>(coord).is_some());
        assert_eq!(world.get::<CoordinateState>(coord).unwrap().value, 0.25);

        // The relationship target is maintained on the coordinate.
        let axes_of_coord = &world.get::<CoordinateAxes>(coord).unwrap().0;
        assert_eq!(axes_of_coord.as_slice(), &[axis]);

        // Required components fire through the scene spawn path: tree entities
        // carry the propagation set, so children with GlobalTransform /
        // InheritedVisibility never trip warning B0004.
        let mut joints = world.query_filtered::<Entity, With<Joint>>();
        let joint = joints.single(world).expect("one joint entity");
        assert!(world.get::<GlobalTransform>(joint).is_some());
        assert!(world.get::<InheritedVisibility>(joint).is_some());
    }

    /// All roots of one `bsn_list!` share a name scope — including names
    /// nested deep inside another root's `Children` subtree. This is what
    /// lets a flat model file declare muscles/constraints as sibling roots
    /// that reference coordinates buried in the skeleton tree.
    #[test]
    fn bsn_list_shares_scope_across_roots() {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            AssetPlugin::default(),
            ScenePlugin::default(),
        ));

        app.world_mut()
            .spawn_scene_list(bsn_list![
                (
                    Joint
                    Children [
                        (
                            #flex
                            Coordinate
                            CoordinateState { value: 0.25 }
                        ),
                        (
                            Twist { angular: Vec3::z() }
                            DrivesCoordinate(#flex)
                        ),
                    ]
                ),
                (
                    // A second root (e.g. a coupler/constraint entity) referencing a
                    // coordinate nested inside the first root's subtree.
                    Twist { linear: Vec3::x() }
                    DrivesCoordinate(#flex)
                ),
            ])
            .unwrap();

        let world = app.world_mut();
        let mut axes = world.query::<&DrivesCoordinate>();
        let targets: Vec<Entity> = axes.iter(world).map(|d| d.0).collect();
        assert_eq!(targets.len(), 2);
        // Both references resolved to the same coordinate entity.
        assert_eq!(targets[0], targets[1]);
        assert!(world.get::<Coordinate>(targets[0]).is_some());
    }

    /// The full frame → joint → body chain: a parent-side fixed offset
    /// composes with the joint's PoE motion, and Bevy propagation carries
    /// the result to the child body's GlobalTransform.
    #[test]
    fn frame_joint_body_chain() {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            AssetPlugin::default(),
            ScenePlugin::default(),
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

        app.world_mut()
            .spawn_scene(bsn! {
                Body
                Children [
                    (
                        // Parent-side frame: 1 m along X from the body origin.
                        FixedFrame({Iso3::translation(1.0, 0.0, 0.0)})
                        Children [
                            (
                                Joint
                                Children [
                                    (
                                        #flex
                                        Coordinate
                                        CoordinateState { value: 0.5 }
                                    ),
                                    rotation_axis(#flex, Vec3::z()),
                                    (Body),
                                ]
                            )
                        ]
                    )
                ]
            })
            .unwrap();

        app.update();

        let world = app.world_mut();
        let mut bodies = world.query_filtered::<&GlobalTransform, (With<Body>, With<ChildOf>)>();
        let global = bodies.single(world).expect("one child body");
        let t = global.translation();
        assert!((t.x - 1.0).abs() < 1e-6 && t.y.abs() < 1e-6 && t.z.abs() < 1e-6);
        let expected = bevy::math::Quat::from_rotation_z(0.5);
        assert!(global.rotation().angle_between(expected) < 1e-5);
    }

    /// Flat authoring: entities declared as sibling roots in one `bsn_list!`,
    /// parented by name via `ChildOf(#...)`. Proves forward references
    /// resolve and the kinematics pipeline finds the axes regardless of
    /// declaration order — the shape large models are authored in.
    #[test]
    fn flat_bsn_list_chain() {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            AssetPlugin::default(),
            ScenePlugin::default(),
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

        app.world_mut()
            .spawn_scene_list(bsn_list![
                (#root Body),
                (
                    #joint Joint
                    ChildOf(#root)
                    Children [
                        (
                            #flex
                            Coordinate
                            CoordinateState { value: 0.5 }
                        ),
                        rotation_axis(#flex, Vec3::z()),
                    ]
                ),
                (
                    // Forward reference: the joint is declared above this body
                    // uses it, but any order resolves.
                    #child Body
                    ChildOf(#joint)
                ),
            ])
            .unwrap();

        app.update();

        let world = app.world_mut();
        // The child body (the Body with a ChildOf pointing at a Joint).
        let mut bodies = world.query_filtered::<(&GlobalTransform, &ChildOf), With<Body>>();
        let global = bodies
            .iter(world)
            .find_map(|(g, p)| world.get::<Joint>(p.0).map(|_| g))
            .expect("child body under the joint");
        let expected = bevy::math::Quat::from_rotation_z(0.5);
        assert!(global.rotation().angle_between(expected) < 1e-5);
    }
}
