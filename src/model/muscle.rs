use bevy::prelude::*;

/// A muscle entity.
#[derive(Component, Clone, Debug, Default)]
pub struct Muscle;

/// Generic Hill-type muscle parameters.
#[derive(Component, Clone, Debug, Default)]
pub struct HillTypeMuscleParams {
    pub max_isometric_force: f64,
    pub optimal_fiber_length: f64,
    pub tendon_slack_length: f64,
    pub pennation_angle_at_optimal: f64,
    pub minimum_activation: f64,
    pub fiber_damping: f64,
}

/// Millard 2012 equilibrium muscle model parameters.
#[derive(Component, Clone, Debug)]
pub struct Millard2012Params {
    pub pennation_angle_at_optimal: f64,
    pub max_contraction_velocity: f64,
    pub activation_time_constant: f64,
    pub deactivation_time_constant: f64,
    pub ignore_activation_dynamics: bool,
    pub ignore_tendon_compliance: bool,
}

impl Default for Millard2012Params {
    fn default() -> Self {
        Self {
            pennation_angle_at_optimal: 0.0,
            max_contraction_velocity: 10.0,
            activation_time_constant: 0.01,
            deactivation_time_constant: 0.04,
            ignore_activation_dynamics: false,
            ignore_tendon_compliance: false,
        }
    }
}

/// Runtime muscle state.
#[derive(Component, Clone, Debug)]
pub struct MuscleState {
    pub fiber_length: f64,
    pub fiber_velocity: f64,
    pub activation: f64,
}

impl Default for MuscleState {
    fn default() -> Self {
        Self {
            fiber_length: 1.0,
            fiber_velocity: 0.0,
            activation: 0.0,
        }
    }
}
