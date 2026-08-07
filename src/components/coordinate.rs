use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// A single degree of freedom (generalized coordinate).
///
/// Coordinates are separate entities referenced by Joints
/// and CoordinateEffects. This allows independent iteration
/// (e.g., "find all locked coordinates") without touching every joint.
///
/// In the hierarchy, a coordinate is a child of its joint entity via ChildOf.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointCoordinate {
    pub range_min: f64,
    pub range_max: f64,
    pub default_value: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub clamped: bool,
    pub locked: bool,
    pub prescribed_function: Option<JointFunction>,
}

/// Defines how a coordinate affects one component of a spatial transform.
///
/// A Joint's full spatial transform is the composition of all
/// CoordinateEffects on that joint's coordinates, evaluated at the
/// current coordinate values. Each effect drives one of the transform
/// components (rotation/translation about axes).
///
/// In the hierarchy, an effect is a child of its coordinate entity via ChildOf.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CoordinateEffect {
    /// Which spatial transform component this effect drives.
    pub component: TransformComponent,
    /// The function mapping coordinate value → transform value.
    pub function: JointFunction,
}

/// Which transform components a CoordinateEffect drives.
///
/// Includes the original 6 axis-aligned components plus
/// arbitrary-axis rotation/translation for general joints.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum TransformComponent {
    RotationX,
    RotationY,
    RotationZ,
    TranslationX,
    TranslationY,
    TranslationZ,
    /// Rotation about an arbitrary axis [x, y, z].
    RotationAboutAxis([f64; 3]),
    /// Translation along an arbitrary axis [x, y, z].
    TranslationAlongAxis([f64; 3]),
}

/// Functions that map coordinate values to spatial transform components.
///
/// OpenSim's CustomJoint uses these in its SpatialTransform to define
/// how each coordinate drives the joint's 6-DOF transform.
///
/// - `Constant`: fixed offset (e.g., a fixed translation that doesn't vary)
/// - `Linear`: slope * q + intercept (e.g., a simple gear ratio)
/// - `Polynomial`: c0 + c1*q + c2*q^2 + ... (coupled motion in knee joints)
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum JointFunction {
    /// f(q) = c
    Constant(f64),
    /// f(q) = slope * q + intercept
    Linear { slope: f64, intercept: f64 },
    /// f(q) = c0 + c1*q + c2*q^2 + c3*q^3 + ...
    Polynomial { coefficients: Vec<f64> },
}

// ── Validation ────────────────────────────────────────

use super::Validate;
use super::relationship::ChildOf;
use crate::systems::{System, validate_all};
use crate::world::World;

impl Validate for JointCoordinate {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        if self.clamped && self.range_min > self.range_max {
            let name = world
                .get::<super::Name>(entity)
                .map(|n| n.value.clone())
                .unwrap_or_default();
            vec![format!(
                "{:?} JointCoordinate '{}' has invalid range [{},{}]",
                entity.0, name, self.range_min, self.range_max
            )]
        } else {
            Vec::new()
        }
    }
}

impl Validate for CoordinateEffect {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        let mut errors = Vec::new();
        if let Some(co) = world.get::<ChildOf>(entity) {
            if world.get::<JointCoordinate>(co.parent).is_none() {
                errors.push(format!(
                    "{:?} CoordinateEffect parent {:?} is missing JointCoordinate",
                    entity.0, co.parent.0
                ));
            }
        } else {
            errors.push(format!(
                "{:?} CoordinateEffect is missing ChildOf component",
                entity.0
            ));
        }
        errors
    }
}

inventory::submit! { System::new("validate_coordinate", |w| validate_all::<JointCoordinate>(w)) }
inventory::submit! { System::new("validate_coordinate_effect", |w| validate_all::<CoordinateEffect>(w)) }
