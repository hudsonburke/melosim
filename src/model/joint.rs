use bevy::prelude::*;
use nalgebra::{Isometry3, Point3, Quaternion, UnitQuaternion, Vector3};

// nalgebra type aliases — all f64 for simulation precision
type Vec3 = Vector3<f64>;
type Quat = Quaternion<f64>;
type UQuat = UnitQuaternion<f64>;
type Iso3 = Isometry3<f64>;

// ── Core types ────────────────────────────────────────

/// An independent scalar degree of freedom.
///
/// Coordinates are the generalized coordinates of the system.
/// Each is a single float that can be driven by actuators, constrained
/// by coupling, and mapped to rigid body transforms via axes.
#[derive(Component, Clone, Debug)]
pub struct Coordinate {
    pub value: f64,
    pub default_value: f64,
    pub range: (f64, f64),
    pub clamped: bool,
    pub locked: bool,
    pub stiffness: f64,
    pub damping: f64,
}

impl Default for Coordinate {
    fn default() -> Self {
        Self {
            value: 0.0,
            default_value: 0.0,
            range: (-std::f64::consts::PI, std::f64::consts::PI),
            clamped: true,
            locked: false,
            stiffness: 0.0,
            damping: 0.0,
        }
    }
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

/// Polynomial mapping f(q) = a0 + a1*q + a2*q^2 + ...
///
/// Applied to a coordinate value before scaling the twist.
/// Identity (f(q) = q) is [0.0, 1.0].
#[derive(Component, Clone, Debug)]
pub struct Polynomial(pub Vec<f64>);

impl Polynomial {
    pub fn evaluate(&self, q: f64) -> f64 {
        self.0
            .iter()
            .enumerate()
            .fold(0.0, |acc, (i, &c)| acc + c * q.powi(i as i32))
    }

    pub fn derivative(&self) -> Polynomial {
        Polynomial(
            self.0
                .iter()
                .enumerate()
                .skip(1)
                .map(|(i, &c)| c * i as f64)
                .collect(),
        )
    }
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
    /// q_a = f(q_b) — direct equality with optional polynomial.
    Equality(Polynomial),
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
            return Iso3::new(v * theta, UQuat::identity());
        }

        // Rotation via exponential map so(3) → SO(3)
        // exp(ω·θ) = (cos(‖ωθ‖/2), sin(‖ωθ‖/2)·ωθ/‖ωθ‖)
        let w_theta = w * theta;
        let half_norm = w_theta.norm() * 0.5;
        let q = if half_norm < 1e-12 {
            UQuat::identity()
        } else {
            let axis = w_theta / (half_norm * 2.0);
            UQuat::new(Quat::from_parts(half_norm.cos(), axis * half_norm.sin()))
        };

        // Translation from the se(3) exponential formula
        let sin_full = half_norm.sin() * 2.0 * half_norm.cos(); // sin(‖ωθ‖)
        let cos_full = 1.0 - 2.0 * half_norm.sin().powi(2); // cos(‖ωθ‖)

        let w_cross_v = w.cross(&v);
        let w_cross_w_cross_v = w.cross(&w_cross_v);
        let w2 = w_norm * w_norm;

        let t = v * theta
            + w_cross_v * (1.0 - cos_full) / w2
            + w_cross_w_cross_v * (theta * w_norm - sin_full) / (w2 * w_norm);

        Iso3::new(t, q)
    }
}

// ── PoE evaluation ────────────────────────────────────

/// Evaluate the product-of-exponentials transform for a joint.
///
/// Walks the joint's axis children in insertion order, composing
/// exp(ξ_i · f_i(q_i)) for each axis.
pub fn evaluate_joint(joint: Entity, world: &World) -> Iso3 {
    let Some(children) = world.get::<Children>(joint) else {
        return Iso3::identity();
    };

    let mut transform = Iso3::identity();

    for &child in &children.0 {
        let Some(twist) = world.get::<Twist>(child) else {
            continue;
        };

        let Some(drives) = world.get::<DrivesCoordinate>(child) else {
            continue;
        };

        let coord = world.get::<Coordinate>(drives.0);
        let poly = world.get::<Polynomial>(child);

        let q = coord.map(|c| c.value).unwrap_or(0.0);
        let f = poly.map(|p| p.evaluate(q)).unwrap_or(q);

        transform = transform * twist.exp(f);
    }

    transform
}

/// Compute the Jacobian contribution of one axis for a given coordinate.
///
/// Returns the spatial velocity (angular, linear) contributed by
/// this axis to the coordinate's generalized velocity.
pub fn axis_jacobian(twist: &Twist, poly: &Polynomial, q: f64) -> (Vec3, Vec3) {
    let df = poly.derivative().evaluate(q);
    (twist.angular * df, twist.linear * df)
}

// ── BSN scene templates ───────────────────────────────

/// Revolute joint: one coordinate, rotation about an axis.
pub fn hinge(axis: [f64; 3]) -> impl Scene {
    let axis = Vec3::from(axis);
    bsn! {
        Joint
        Children [
            (#q0 Coordinate { ..default() })
            (Twist { angular: axis, linear: Vec3::zeros() }
                DrivesCoordinate(#q0)
                Polynomial(vec![0.0, 1.0]))
        ]
    }
}

/// Prismatic joint: one coordinate, translation along an axis.
pub fn slide(axis: [f64; 3]) -> impl Scene {
    let axis = Vec3::from(axis);
    bsn! {
        Joint
        Children [
            (#q0 Coordinate { ..default() })
            (Twist { angular: Vec3::zeros(), linear: axis }
                DrivesCoordinate(#q0)
                Polynomial(vec![0.0, 1.0]))
        ]
    }
}

/// Ball joint: three coordinates, sequential rotations about X, Y, Z.
pub fn ball() -> impl Scene {
    bsn! {
        Joint
        Children [
            (#qx Coordinate { ..default() })
            (#qy Coordinate { ..default() })
            (#qz Coordinate { ..default() })
            (Twist { angular: Vec3::x(), linear: Vec3::zeros() }
                DrivesCoordinate(#qx)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: Vec3::y(), linear: Vec3::zeros() }
                DrivesCoordinate(#qy)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: Vec3::z(), linear: Vec3::zeros() }
                DrivesCoordinate(#qz)
                Polynomial(vec![0.0, 1.0]))
        ]
    }
}

/// Free joint: six coordinates, three translation + three rotation.
pub fn free() -> impl Scene {
    bsn! {
        Joint
        Children [
            (#tx Coordinate { range: (-1e10, 1e10), ..default() })
            (#ty Coordinate { range: (-1e10, 1e10), ..default() })
            (#tz Coordinate { range: (-1e10, 1e10), ..default() })
            (#rx Coordinate { ..default() })
            (#ry Coordinate { ..default() })
            (#rz Coordinate { ..default() })
            (Twist { angular: Vec3::zeros(), linear: Vec3::x() }
                DrivesCoordinate(#tx)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: Vec3::zeros(), linear: Vec3::y() }
                DrivesCoordinate(#ty)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: Vec3::zeros(), linear: Vec3::z() }
                DrivesCoordinate(#tz)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: Vec3::x(), linear: Vec3::zeros() }
                DrivesCoordinate(#rx)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: Vec3::y(), linear: Vec3::zeros() }
                DrivesCoordinate(#ry)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: Vec3::z(), linear: Vec3::zeros() }
                DrivesCoordinate(#rz)
                Polynomial(vec![0.0, 1.0]))
        ]
    }
}

/// Weld (fixed) joint: no coordinates, no axes.
pub fn weld() -> impl Scene {
    bsn! { Joint }
}

/// Universal joint: two coordinates, rotation about two axes.
pub fn universal(axis_a: [f64; 3], axis_b: [f64; 3]) -> impl Scene {
    let a = Vec3::from(axis_a);
    let b = Vec3::from(axis_b);
    bsn! {
        Joint
        Children [
            (#qa Coordinate { ..default() })
            (#qb Coordinate { ..default() })
            (Twist { angular: a, linear: Vec3::zeros() }
                DrivesCoordinate(#qa)
                Polynomial(vec![0.0, 1.0]))
            (Twist { angular: b, linear: Vec3::zeros() }
                DrivesCoordinate(#qb)
                Polynomial(vec![0.0, 1.0]))
        ]
    }
}

// ── Spawn helpers (non-BSN, for scripting / systems) ──

/// Spawn a hinge joint between two frames.
pub fn spawn_hinge(
    commands: &mut Commands,
    parent_frame: Entity,
    child_frame: Entity,
    axis: [f64; 3],
) -> Entity {
    let axis = Vec3::from(axis);
    let coord = commands.spawn(Coordinate::default()).id();
    let axis_ent = commands
        .spawn((
            Twist {
                angular: axis,
                linear: Vec3::zeros(),
            },
            DrivesCoordinate(coord),
            Polynomial(vec![0.0, 1.0]),
        ))
        .id();

    let joint = commands.spawn((Joint, ChildOf(parent_frame))).id();
    commands.entity(axis_ent).insert(ChildOf(joint));
    commands.entity(child_frame).insert(ChildOf(joint));
    joint
}

/// Spawn a custom joint with arbitrary axes.
///
/// Each entry in `axes` is (twist, default_value, polynomial_coeffs).
pub fn spawn_custom(
    commands: &mut Commands,
    parent_frame: Entity,
    child_frame: Entity,
    axes: Vec<(Twist, f64, Vec<f64>)>,
) -> Entity {
    let joint = commands.spawn((Joint, ChildOf(parent_frame))).id();
    for (twist, default_val, coeffs) in axes {
        let coord = commands
            .spawn(Coordinate {
                default_value: default_val,
                ..default()
            })
            .id();
        let axis_ent = commands
            .spawn((twist, DrivesCoordinate(coord), Polynomial(coeffs)))
            .id();
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
