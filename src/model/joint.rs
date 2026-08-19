use bevy::prelude::*;
use nalgebra::{Isometry3, Translation, UnitQuaternion, Vector3};

use super::Function;

// nalgebra type aliases — all f64 for simulation precision
type Vec3 = Vector3<f64>;
type UQuat = UnitQuaternion<f64>;
type Iso3 = Isometry3<f64>;

/// Marker for a joint entity.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform, Visibility)]
pub struct Joint;

/// Marker for a generalized coordinate entity.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(CoordinateProperties, InitialConditions)]
pub struct Coordinate;

// ── Joint → Coordinates relationship ──

/// Ordered list of coordinate entities owned by this joint.
#[derive(Component, Clone, Debug, Reflect)]
#[relationship_target(relationship = CoordinateOf)]
pub struct JointCoordinates(Vec<Entity>);

impl JointCoordinates {
    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }
}

/// Relationship: this coordinate is owned by a joint.
#[derive(Component, Clone, Debug, FromTemplate, Reflect)]
#[relationship(relationship_target = JointCoordinates)]
pub struct CoordinateOf(pub Entity);

#[derive(Component, Clone, Debug, Reflect)]
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

#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct InitialConditions {
    pub value: f64,
    pub velocity: f64,
}

/// Runtime state of a generalized coordinate.
#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct CoordinateState {
    pub value: f64,
    pub velocity: f64,
}

/// Ensure every `Coordinate` has a `CoordinateState` (seeded from its
/// `InitialConditions`), so the FK (`sync_kinematics`) can drive it and the
/// editor can articulate it. No-op once state exists.
pub fn ensure_coordinate_states(
    mut commands: Commands,
    coords: Query<(Entity, &InitialConditions), (With<Coordinate>, Without<CoordinateState>)>,
) {
    for (entity, ic) in &coords {
        commands.entity(entity).insert(CoordinateState {
            value: ic.value,
            velocity: ic.velocity,
        });
    }
}

/// A twist in se(3) — the Lie algebra of SE(3).
#[derive(Component, Clone, Debug, Default, Reflect)]
pub struct Twist {
    #[reflect(ignore)]
    pub angular: Vec3,
    #[reflect(ignore)]
    pub linear: Vec3,
}

/// The type of coupling between linked coordinates.
#[derive(Component, Clone, Debug, Reflect)]
pub struct Coupling {
    pub kind: CouplingKind,
}

#[derive(Clone, Debug, Reflect)]
pub enum CouplingKind {
    Equality(Function),
    Tendon,
    Gear { ratio: f64 },
}

// ── Twist → SE(3) via exponential map ─────────────────

impl Twist {
    pub fn exp(&self, theta: f64) -> Iso3 {
        if theta.abs() < 1e-12 {
            return Iso3::identity();
        }

        let w = self.angular;
        let v = self.linear;
        let w_norm = w.norm();

        if w_norm < 1e-12 {
            return Iso3::from_parts(Translation::from(v * theta), UQuat::identity());
        }

        let angle = w_norm * theta;
        let q = UQuat::from_scaled_axis(w * theta);

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

pub fn evaluate_joint(
    children: &Children,
    twists: &Query<(&Twist, Option<&Function>)>,
    states: &Query<&CoordinateState>,
) -> Iso3 {
    let mut transform = Iso3::identity();

    for child in children.iter() {
        let Ok((twist, func)) = twists.get(child) else {
            continue;
        };

        let q = states.get(child).map(|s| s.value).unwrap_or(0.0);
        let f = func.map(|f| f.evaluate(q)).unwrap_or(q);

        transform *= twist.exp(f);
    }

    transform
}

pub fn axis_jacobian(twist: &Twist, func: &Function, q: f64) -> (Vec3, Vec3) {
    let df = func.derivative().evaluate(q);
    (twist.angular * df, twist.linear * df)
}

// ── Rendering boundary: nalgebra → glam ───────────────

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

pub fn sync_kinematics(
    mut joints: Query<(&Children, &mut Transform), With<Joint>>,
    twists: Query<(&Twist, Option<&Function>)>,
    states: Query<&CoordinateState>,
) {
    for (children, mut transform) in &mut joints {
        *transform = to_bevy_transform(&evaluate_joint(children, &twists, &states));
    }
}
