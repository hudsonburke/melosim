use std::collections::HashSet;

use bevy::prelude::*;
use nalgebra::{Isometry3, Translation, UnitQuaternion, Vector3};

use super::Function;

// nalgebra type aliases — all f64 for simulation precision
type Vec3 = Vector3<f64>;
type UQuat = UnitQuaternion<f64>;
type Iso3 = Isometry3<f64>;

/// Marker for a joint entity.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(Transform, JointCoordinates)]
pub struct Joint;

/// Marker for a generalized coordinate entity.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(CoordinateProperties, InitialConditions)]
pub struct Coordinate;

// ── Joint → Coordinates relationship ──

/// Ordered list of coordinate entities owned by this joint.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[relationship_target(relationship = CoordinateOf)]
pub struct JointCoordinates(Vec<Entity>);

impl JointCoordinates {
    pub fn new(entities: Vec<Entity>) -> Self {
        Self(entities)
    }

    pub fn iter(&self) -> impl Iterator<Item = Entity> + '_ {
        self.0.iter().copied()
    }
}

/// Relationship: this coordinate is owned by a joint.
///
/// Injected by the relationship machinery from the owner's `JointCoordinates`
/// (like Bevy's own `ChildOf`), not authored in scene notation.
///
/// Note: an earlier "self-referential relationship" warning was *not* caused by
/// this component. It came from the myoarm model naming a coordinate identically
/// to its owning joint (`#pro_sup`), so scene name-resolution pointed the
/// relationship at the wrong (self) entity. Keep entity names distinct.
#[derive(Component, Clone, Debug, FromTemplate, Reflect)]
#[relationship(relationship_target = JointCoordinates)]
pub struct CoordinateOf(pub Entity);

/// Ordered reverse relationship for coordinates whose value is driven by
/// another coordinate. The coupling metadata lives on the source relationship
/// component, so each edge can have its own mapping.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[relationship_target(relationship = DrivenBy)]
pub struct DrivenCoordinates(Vec<Entity>);

/// Relationship: this coordinate derives its scalar value from another
/// coordinate, while retaining its own `Twist` axis.
///
/// The relationship and its mapping are intentionally one component: coupling
/// is metadata about this specific source → derived-coordinate edge.
#[derive(Component, Clone, Debug, FromTemplate, Reflect)]
#[relationship(relationship_target = DrivenCoordinates)]
pub struct DrivenBy {
    #[relationship]
    pub source: Entity,
    pub coupling: CouplingKind,
}

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
#[derive(Clone, Debug, Reflect)]
pub enum CouplingKind {
    /// The derived coordinate evaluates this function at its source value.
    Equality(Function),
    /// The derived coordinate equals `ratio * source`.
    Gear { ratio: f64 },
}

impl Default for CouplingKind {
    fn default() -> Self {
        Self::Equality(Function::identity())
    }
}

// ── Twist → SE(3) via exponential map ─────────────────

impl Twist {
    /// Pure rotation around an axis.
    pub fn rotation(axis: Vec3) -> Self {
        Self { angular: axis, linear: Vec3::zeros() }
    }

    /// Pure translation along an axis.
    pub fn translation(axis: Vec3) -> Self {
        Self { angular: Vec3::zeros(), linear: axis }
    }

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
    coords: impl Iterator<Item = Entity>,
    twists: &Query<&Twist>,
    values: &Query<
        (Option<&CoordinateState>, &InitialConditions, Option<&DrivenBy>),
        With<Coordinate>,
    >,
) -> Iso3 {
    let mut transform = Iso3::identity();

    for coord in coords {
        let Ok(twist) = twists.get(coord) else {
            continue;
        };

        let mut visiting = HashSet::new();
        let q = resolve_coordinate_value(coord, values, &mut visiting);
        transform *= twist.exp(q);
    }

    transform
}

/// Resolve the effective value of a coordinate, following any directed
/// coordinate coupling. Cycles are treated as zero at runtime; the model
/// validator reports them as structural issues before simulation/export.
fn resolve_coordinate_value(
    coordinate: Entity,
    values: &Query<
        (Option<&CoordinateState>, &InitialConditions, Option<&DrivenBy>),
        With<Coordinate>,
    >,
    visiting: &mut HashSet<Entity>,
) -> f64 {
    let Ok((state, initial, driven_by)) = values.get(coordinate) else {
        return 0.0;
    };

    let Some(source) = driven_by else {
        return state.map(|value| value.value).unwrap_or(initial.value);
    };

    if !visiting.insert(coordinate) {
        return 0.0;
    }
    let source_value = resolve_coordinate_value(source.source, values, visiting);
    visiting.remove(&coordinate);

    match &source.coupling {
        CouplingKind::Equality(function) => function.evaluate(source_value),
        CouplingKind::Gear { ratio } => ratio * source_value,
    }
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
    mut joints: Query<(&JointCoordinates, &mut Transform), With<Joint>>,
    twists: Query<&Twist>,
    values: Query<
        (Option<&CoordinateState>, &InitialConditions, Option<&DrivenBy>),
        With<Coordinate>,
    >,
) {
    for (coords, mut transform) in &mut joints {
        *transform = to_bevy_transform(&evaluate_joint(coords.iter(), &twists, &values));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Body, Frame};

    #[test]
    fn coupled_coordinate_drives_a_second_twist() {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            bevy::transform::TransformPlugin,
        ));
        app.add_systems(
            PostUpdate,
            sync_kinematics.before(bevy::transform::TransformSystems::Propagate),
        );

        let root = app.world_mut().spawn((Body, Transform::IDENTITY)).id();
        let source = app
            .world_mut()
            .spawn((
                Coordinate,
                CoordinateProperties::default(),
                InitialConditions::default(),
                CoordinateState {
                    value: 0.5,
                    velocity: 0.0,
                },
                Twist::rotation(Vector3::z()),
            ))
            .id();
        let derived = app
            .world_mut()
            .spawn((
                Coordinate,
                CoordinateProperties::default(),
                InitialConditions::default(),
                CoordinateState {
                    value: 99.0,
                    velocity: 0.0,
                },
                Twist::rotation(Vector3::x()),
                DrivenBy {
                    source,
                    coupling: CouplingKind::Equality(Function::identity()),
                },
            ))
            .id();
        let joint = app
            .world_mut()
            .spawn((
                Joint,
                JointCoordinates::new(vec![source, derived]),
                Transform::IDENTITY,
                ChildOf(root),
            ))
            .id();
        let child = app
            .world_mut()
            .spawn((Body, Transform::IDENTITY, ChildOf(joint)))
            .id();

        app.update();

        let actual = app.world().get::<GlobalTransform>(child).unwrap().rotation();
        let expected = bevy::math::Quat::from_rotation_z(0.5)
            * bevy::math::Quat::from_rotation_x(0.5);
        assert!(actual.angle_between(expected) < 1e-5);
    }

    #[test]
    fn joint_motion_preserves_fixed_parent_frame_offset() {
        let mut app = App::new();
        app.add_plugins((
            bevy::app::TaskPoolPlugin::default(),
            bevy::transform::TransformPlugin,
        ));
        app.add_systems(
            PostUpdate,
            sync_kinematics.before(bevy::transform::TransformSystems::Propagate),
        );

        let root = app.world_mut().spawn((Body, Transform::IDENTITY)).id();
        let frame = app
            .world_mut()
            .spawn((
                Frame,
                Transform::from_xyz(1.0, 0.0, 0.0),
                ChildOf(root),
            ))
            .id();
        let coordinate = app
            .world_mut()
            .spawn((
                Coordinate,
                CoordinateProperties::default(),
                InitialConditions::default(),
                CoordinateState::default(),
                Twist::rotation(Vector3::z()),
            ))
            .id();
        let joint = app
            .world_mut()
            .spawn((
                Joint,
                JointCoordinates::new(vec![coordinate]),
                Transform::IDENTITY,
                ChildOf(frame),
            ))
            .id();
        let child = app
            .world_mut()
            .spawn((Body, Transform::IDENTITY, ChildOf(joint)))
            .id();

        app.update();
        let initial = app.world().get::<GlobalTransform>(child).unwrap();
        assert!((initial.translation().x - 1.0).abs() < 1e-6);
        assert!(initial.translation().y.abs() < 1e-6);

        app.world_mut()
            .get_mut::<CoordinateState>(coordinate)
            .unwrap()
            .value = 0.5;
        app.update();

        let articulated = app.world().get::<GlobalTransform>(child).unwrap();
        assert!((articulated.translation().x - 1.0).abs() < 1e-6);
        assert!(articulated.translation().y.abs() < 1e-6);
        assert!(
            articulated
                .rotation()
                .angle_between(bevy::math::Quat::from_rotation_z(0.5))
                < 1e-5
        );
    }
}
