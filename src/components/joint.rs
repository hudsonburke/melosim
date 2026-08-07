use serde::{Deserialize, Serialize};
use crate::id::EntityID;

/// Common fields shared by all joint types.
/// Inlined into each type-specific joint component.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointLimits {
    pub lower: f64,
    pub upper: f64,
}

/// A joint entity connects two frames and defines degrees of freedom.
///
/// Frame connections are stored as `ParentFrame` and `ChildFrame`
/// components on the same entity. The joint type is emergent from
/// the `SpatialTransform`/`CoordinateEffect` configuration:
/// - Hinge: 1 coordinate driving RotationAboutAxis
/// - Ball: 3 coordinates driving 3 RotationAboutAxis
/// - Free: 6 coordinates (3 rotation + 3 translation)
/// - Weld: 0 coordinates
/// - Universal: 2 coordinates on orthogonal axes
/// - Custom: arbitrary coordinates defined by CoordinateEffects
///
/// Coordinates (DOFs) are separate entities referenced by `coordinates`.
/// CoordinateEffect components define how each coordinate drives the
/// spatial transform. SpatialTransform groups the effects for a joint.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Joint {
    pub limits: Option<JointLimits>,
    pub coordinates: Vec<EntityID>,
}

/// Infer the joint kind from the coordinate/effect configuration.
/// Returns a string matching the old joint_type values for compatibility
/// with exporter match patterns.
pub fn infer_joint_kind(world: &World, joint: &Joint) -> &'static str {
    let n_coords = joint.coordinates.len();
    match n_coords {
        0 => "WeldJoint",
        1 => {
            for coord_key in &joint.coordinates {
                for (_, effect) in world.iter::<super::coordinate::CoordinateEffect>() {
                    if effect.coordinate == *coord_key {
                        return match &effect.component {
                            super::coordinate::TransformComponent::RotationAboutAxis(_)
                            | super::coordinate::TransformComponent::RotationX
                            | super::coordinate::TransformComponent::RotationY
                            | super::coordinate::TransformComponent::RotationZ => "PinJoint",
                            super::coordinate::TransformComponent::TranslationAlongAxis(_)
                            | super::coordinate::TransformComponent::TranslationX
                            | super::coordinate::TransformComponent::TranslationY
                            | super::coordinate::TransformComponent::TranslationZ => "SlideJoint",
                        };
                    }
                }
            }
            "PinJoint"
        }
        2 => "UniversalJoint",
        3 => "BallJoint",
        6 => "FreeJoint",
        _ => "CustomJoint",
    }
}

// ── Validation ────────────────────────────────────────

use super::{Validate, ParentFrame, ChildFrame, JointCoordinate};
use crate::world::World;
use crate::systems::{System, validate_all, check_has};

impl Validate for Joint {
    fn validate(&self, entity: EntityID, world: &World) -> Vec<String> {
        let mut e: Vec<String> = Vec::new();

        // Validate that ParentFrame and ChildFrame components exist on this entity
        if world.get::<ParentFrame>(entity).is_none() {
            e.push(format!(
                "{:?} Joint is missing ParentFrame component",
                entity.0
            ));
        }
        if world.get::<ChildFrame>(entity).is_none() {
            e.push(format!(
                "{:?} Joint is missing ChildFrame component",
                entity.0
            ));
        }

        for (i, coord_key) in self.coordinates.iter().enumerate() {
            if let Some(err) = check_has::<JointCoordinate>(world, entity, &format!("coordinates[{i}]"), *coord_key) {
                e.push(err);
            }
        }
        e
    }
}

inventory::submit! { System::new("validate_joint", |w| validate_all::<Joint>(w)) }
