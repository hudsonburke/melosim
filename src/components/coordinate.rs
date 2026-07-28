use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// A single degree of freedom (generalized coordinate).
///
/// Coordinates are separate entities referenced by CustomJoints
/// and CoordinateEffects. This allows independent iteration
/// (e.g., "find all locked coordinates") without touching every joint.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointCoordinate {
    pub name: String,
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
/// A CustomJoint's full spatial transform is the composition of all
/// CoordinateEffects on that joint's coordinates, evaluated at the
/// current coordinate values. Each effect drives one of the six
/// transform components (3 rotation, 3 translation).
///
/// Example: a knee CustomJoint where flexion (coord0) drives:
///   - RotationY → knee flexion angle (linear, slope=-1.0)
///   - TranslationX → coupled AP translation (polynomial)
///   - TranslationZ → coupled vertical translation (polynomial)
///
/// All three are separate CoordinateEffect entities referencing coord0.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CoordinateEffect {
    /// The coordinate this effect reads from.
    pub coordinate: EntityID,
    /// The joint this effect belongs to.
    pub joint: EntityID,
    /// Which spatial transform component this effect drives.
    pub component: TransformComponent,
    /// The function mapping coordinate value → transform value.
    pub function: JointFunction,
}

/// Which of the 6 spatial transform components a CoordinateEffect drives.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum TransformComponent {
    RotationX,
    RotationY,
    RotationZ,
    TranslationX,
    TranslationY,
    TranslationZ,
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

/// Groups the CoordinateEffects that define a CustomJoint's spatial transform.
///
/// A convenience grouping — the actual data lives in CoordinateEffect components.
/// OpenSim's CustomJoint spatial transform has exactly 6 transform components:
/// 3 rotations (X, Y, Z) and 3 translations (X, Y, Z), each of which can be
/// driven by zero or one coordinate.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SpatialTransform {
    /// The joint this transform belongs to.
    pub joint: EntityID,
    /// EntityIDs of the CoordinateEffect components making up this transform.
    pub effects: Vec<EntityID>,
}
