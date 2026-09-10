use bevy::prelude::*;

use super::PathEntities;

/// A user-authored routed cable, distinct from a biological muscle.
#[derive(Component, Clone, Debug, Default, Reflect)]
#[require(PathEntities, CableParameters)]
pub struct Cable;

/// Minimal cable parameters used by the editor and MuJoCo exporter.
#[derive(Component, Clone, Debug, Reflect)]
pub struct CableParameters {
    pub rest_length: f64,
    pub stiffness: f64,
    pub damping: f64,
    pub max_tension: f64,
    pub actuator_force: f64,
}

impl Default for CableParameters {
    fn default() -> Self {
        Self {
            rest_length: 0.0,
            stiffness: 1000.0,
            damping: 0.0,
            max_tension: 1000.0,
            actuator_force: 100.0,
        }
    }
}
