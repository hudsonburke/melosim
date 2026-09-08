//! Import adapters for the canonical Bevy model.
//!
//! Format-specific parsers build the live ECS model through the shared
//! `ModelBuilder` construction API. There is no persistent importer IR.

#[derive(Debug)]
pub enum ImportError {
    Load(String),
}

#[cfg(feature = "mujoco")]
mod mujoco_live;

#[cfg(feature = "mujoco")]
pub use mujoco_live::import_mjcf;

#[cfg(feature = "opensim")]
mod opensim_live;

#[cfg(feature = "opensim")]
pub use opensim_live::import_osim;
