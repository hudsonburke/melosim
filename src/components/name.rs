use serde::{Deserialize, Serialize};

/// A human-readable name for any entity.
///
/// Names are metadata — used for import/export, logging, and debugging.
/// Solvers never access names during simulation.
///
/// Attach to any entity that needs identity:
/// ```ignore
/// let entity = world.spawn();
/// world.attach(entity, InertialProperties { mass: 11.78, ... });
/// world.attach(entity, Name { value: "pelvis".into() });
/// ```
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Name {
    pub value: String,
}
