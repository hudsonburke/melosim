use bevy_ecs::prelude::*;
use crate::math::Transform;

/// A display/mesh geometry attached to a body for visualization.
///
/// OpenSim serializes these in each Body's `<VisibleObject>` element.
/// They're purely visual — not used in simulation.
#[derive(Component, Clone, Debug)]
pub struct DisplayGeometry {
    pub body: Entity,
    pub mesh_file: Option<String>,
    pub scale: [f64; 3],
    pub color: [f64; 3],
    pub opacity: f64,
    /// Maps raw mesh-file vertex coordinates into the body frame:
    ///   v_body = translation + rotation * (scale ⊙ v_file)
    /// Importers must bake any source-format mesh-frame quirks (e.g.
    /// MuJoCo's compile-time re-centering to CoM/principal axes) into
    /// this transform, so consumers never handle format-specific frames.
    pub transform: Transform,
}

/// A mesh geometry reference (file path).
/// Used for both display and collision geometry in some imports.
#[derive(Component, Clone, Debug)]
pub struct MeshGeometry {
    pub mesh: String,
}

/// Primitive geometry shapes.
/// These are kept as standalone structs for import/export convenience.
/// In the ECS, entities use `DisplayGeometry` for visualization.
#[derive(Component, Clone, Debug)]
pub struct Sphere {
    pub radius: f64,
}
#[derive(Component, Clone, Debug)]
pub struct Cylinder {
    pub radius: f64,
    pub length: f64,
}
#[derive(Component, Clone, Debug)]
pub struct Capsule {
    pub radius: f64,
    pub length: f64,
}
#[derive(Component, Clone, Debug)]
pub struct BoxGeom {
    pub half_extents: [f64; 3],
}
#[derive(Component, Clone, Debug)]
pub struct Plane;
