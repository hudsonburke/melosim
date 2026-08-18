use bevy::prelude::*;
use nalgebra::{Isometry3, Translation, UnitQuaternion, Vector3};

use super::Function;

// nalgebra type aliases — all f64 for simulation precision
type Vec3 = Vector3<f64>;
type UQuat = UnitQuaternion<f64>;
type Iso3 = Isometry3<f64>;

/// Marker for a joint entity.
#[derive(Component, Clone, Debug, Default)]
pub struct Joint;

/// Links a twist axis to the coordinate that drives it.
#[derive(Component, Clone, Debug)]
pub struct DrivesCoordinate(pub Entity);

#[derive(Component, Clone, Debug, Default)]
#[require(CoordinateProperties, InitialConditions)]
pub struct Coordinate;

// ── Joint → Coordinates relationship ──

/// Ordered list of coordinate entities owned by this joint.
#[derive(Component, Clone, Debug)]
#[relationship_target(relationship = CoordinateOf)]
pub struct JointCoordinates(Vec<Entity>);

impl JointCoordinates {
    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }
}

/// Relationship: this coordinate is owned by a joint.
#[derive(Component, Clone, Debug, FromTemplate)]
#[relationship(relationship_target = JointCoordinates)]
pub struct CoordinateOf(pub Entity);

#[derive(Component, Clone, Debug)]
pub struct CoordinateProperties {
    pub range: (f64, f64),
    pub clamped: bool,
    pub locked: bool,
    pub stiffness: f64,
    pub damping: f64,
}

impl Default for CoordinateProperties {
    fn default() -> Self {
        Self {
            range: (-std::f64::consts::PI, std::f64::consts::PI),
            clamped: true,
            locked: false,
            stiffness: 0.0,
            damping: 0.0,
        }
    }
}

#[derive(Component, Clone, Debug, Default)]
pub struct InitialConditions {
    pub value: f64,
    pub velocity: f64,
}
/// Runtime state of a generalized coordinate (transient simulation data).
#[derive(Component, Clone, Debug, Default)]
pub struct CoordinateState {
    pub value: f64,
    pub velocity: f64,
}

/// A twist in se(3) — the Lie algebra of SE(3).
///
/// A pure rotation has angular = axis, linear = zero.
/// A pure translation has angular = zero, linear = direction.
/// A screw has both.
#[derive(Component, Clone, Debug, Default)]
pub struct Twist {
    /// ω — rotation axis (not necessarily unit)
    pub angular: Vec3,
    /// v — translation direction
    pub linear: Vec3,
}

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
/// Axes without a `DrivesCoordinate` are fixed offsets: their function
/// (typically a constant) is evaluated at q = 0.
pub fn evaluate_joint(
    children: &Children,
    twists: &Query<(&Twist, Option<&DrivesCoordinate>, Option<&Function>)>,
    states: &Query<&CoordinateState>,
) -> Iso3 {
    let mut transform = Iso3::identity();

    for child in children.iter() {
        let Ok((twist, drives, func)) = twists.get(child) else {
            continue;
        };

        let q = drives
            .and_then(|d| states.get(d.0).ok())
            .map(|s| s.value)
            .unwrap_or(0.0);
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

// ── Bevy sync system ─────────────────────────────────

/// Evaluate each joint's PoE transform (sim f64) and write it to the
/// joint entity's own `Transform`. Frames and bodies nested below the
/// joint follow via Bevy's transform propagation.
///
/// Runs in PostUpdate, before `TransformSystems::Propagate`.
pub fn sync_kinematics(
    mut joints: Query<(&Children, &mut Transform), With<Joint>>,
    twists: Query<(&Twist, Option<&DrivesCoordinate>, Option<&Function>)>,
    states: Query<&CoordinateState>,
) {
    for (children, mut transform) in &mut joints {
        *transform = to_bevy_transform(&evaluate_joint(children, &twists, &states));
    }
}
