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

/// Marker for a joint entity.
///
/// A joint connects a parent frame to a child frame.
/// Its axes (child entities) define the product-of-exponentials
/// transform from parent to child frame.
///
/// Hierarchy:
///   ParentFrame (entity)
///     └─ Joint (entity, via ChildOf)
///          ├─ Axis (entity, via Children) ── DrivesCoordinate ── Coordinate
///          ├─ Axis (entity, via Children) ── DrivesCoordinate ── Coordinate
///          └─ ChildFrame (entity, via Children)
#[derive(Component, Clone, Debug)]
pub struct Joint;

/// A fixed frame in the kinematic chain.
///
/// Attached to a body or joint via ChildOf. The Isometry3 is the
/// fixed offset from the parent entity's frame.
#[derive(Component, Clone, Debug)]
pub struct FixedFrame(pub Iso3);

/// A twist in se(3) — the Lie algebra of SE(3).
///
/// A pure rotation has angular = axis, linear = zero.
/// A pure translation has angular = zero, linear = direction.
/// A screw has both.
#[derive(Component, Clone, Debug)]
pub struct Twist {
    pub angular: Vec3, // ω — rotation axis (not necessarily unit)
    pub linear: Vec3,  // v — translation direction
}

// ── Relationships ─────────────────────────────────────

/// Axis → Coordinate: "this axis is driven by this coordinate."
///
/// Multiple axes can drive the same coordinate (OpenSim coupled knee).
/// Multiple coordinates can drive one joint (ball, free, custom).
#[derive(Component)]
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

// ── PoE evaluation ────────────────────────────────────

/// Evaluate the product-of-exponentials transform for a joint.
///
/// Walks the joint's axis children in insertion order, composing
/// exp(ξ_i · f_i(q_i)) for each axis. Reads the current coordinate
/// value from `CoordinateState`.
pub fn evaluate_joint(joint: Entity, world: &World) -> Iso3 {
    let Some(children) = world.get::<Children>(joint) else {
        return Iso3::identity();
    };

    let mut transform = Iso3::identity();

    for child in children.iter() {
        let Some(twist) = world.get::<Twist>(child) else {
            continue;
        };

        let Some(drives) = world.get::<DrivesCoordinate>(child) else {
            continue;
        };

        let state = world.get::<CoordinateState>(drives.0);
        let func = world.get::<Function>(child);

        let q = state.map(|s| s.value).unwrap_or(0.0);
        let f = func.map(|f| f.evaluate(q)).unwrap_or(q);

        transform = transform * twist.exp(f);
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

// ── Spawn helpers ─────────────────────────────────────

/// Spawn a hinge joint between two frames.
pub fn spawn_hinge(
    commands: &mut Commands,
    parent_frame: Entity,
    child_frame: Entity,
    axis: [f64; 3],
) -> Entity {
    let axis = Vec3::from(axis);
    let coord = commands
        .spawn((Coordinate::default(), CoordinateState::default()))
        .id();
    let axis_ent = commands
        .spawn((
            Twist {
                angular: axis,
                linear: Vec3::zeros(),
            },
            DrivesCoordinate(coord),
            Function::Polynomial(vec![0.0, 1.0]),
        ))
        .id();

    let joint = commands.spawn((Joint, ChildOf(parent_frame))).id();
    commands.entity(axis_ent).insert(ChildOf(joint));
    commands.entity(child_frame).insert(ChildOf(joint));
    joint
}

/// Spawn a custom joint with arbitrary axes.
///
/// Each entry in `axes` is (twist, default_value, function).
pub fn spawn_custom(
    commands: &mut Commands,
    parent_frame: Entity,
    child_frame: Entity,
    axes: Vec<(Twist, f64, Function)>,
) -> Entity {
    let joint = commands.spawn((Joint, ChildOf(parent_frame))).id();
    for (twist, default_val, func) in axes {
        let coord = commands
            .spawn((
                Coordinate {
                    default_value: default_val,
                    ..default()
                },
                CoordinateState::default(),
            ))
            .id();
        let axis_ent = commands.spawn((twist, DrivesCoordinate(coord), func)).id();
        commands.entity(axis_ent).insert(ChildOf(joint));
    }
    commands.entity(child_frame).insert(ChildOf(joint));
    joint
}

// ── Rendering boundary: nalgebra → glam ───────────────

/// Convert a simulation isometry to a Bevy Transform (f32).
///
/// Use this when writing simulation state back to rendering components.
pub fn to_bevy_transform(iso: &Iso3) -> Transform {
    let t = iso.translation.vector;
    let r = iso.rotation;
    Transform {
        translation: bevy::math::Vec3::new(t.x as f32, t.y as f32, t.z as f32).into(),
        rotation: bevy::math::Quat::from_xyzw(r.i as f32, r.j as f32, r.k as f32, r.w as f32)
            .into(),
        ..default()
    }
}
