use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// A muscle entity — the identity component.
///
/// Every muscle in the model is an entity with at least a `Muscle` component.
/// Additional components define its path (`MusclePath`) and physiology
/// (`Millard2012Params`, `HillTypeMuscleParams`, etc.).
///
/// In Rajagopal 2015, all 80 muscles are `Millard2012EquilibriumMuscle`.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Muscle;

/// Millard 2012 equilibrium muscle model parameters.
///
/// This is the full parameter set for the Millard 2012 Hill-type muscle model
/// used by Rajagopal 2015. Defaults are applied by OpenSim's
/// `extendFinalizeFromProperties()` at model init for any fields not
/// explicitly set in the `.osim` file.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Millard2012Params {
    pub muscle: EntityID,
    pub max_isometric_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub pennation_angle_at_optimal: f64,
    pub max_contraction_velocity: f64,
    pub activation_time_constant: f64,
    pub deactivation_time_constant: f64,
    pub minimum_activation: f64,
    pub fiber_damping: f64,
    pub ignore_activation_dynamics: bool,
    pub ignore_tendon_compliance: bool,
}

/// Runtime muscle state (not persisted — computed during simulation).
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MuscleState {
    pub fiber_length: f64,
    pub fiber_velocity: f64,
    pub activation: f64,
}

/// Generic Hill-type muscle parameters (simpler models).
///
/// For Millard 2012 muscles (Rajagopal), use `Millard2012Params` instead.
/// This is kept for backward compatibility with simpler Hill-type models.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct HillTypeMuscleParams {
    pub max_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub pcsa: f64,
    pub pennation_angle: f64,
}

/// Placeholder for force-length curve data.
/// Will be expanded when custom curves need to be stored for round-trip.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ForceLengthCurve {}

/// Placeholder for force-velocity curve data.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ForceVelocityCurve {}
