// ── FlatWorld: frozen simulation snapshot ─────────────
// Produced by World::freeze() after model construction and validation.
// Dense Vec<Option<T>> storage indexed by EntityID(u32).
// Zero hash lookups — named fields provide O(1) access.
//
// Custom types from downstream crates live in `extensions: AnyMap<Vec<Option<T>>>`.
// Their solvers access them via the same get() API — no core changes.

use crate::components::*;
use crate::id::EntityID;
use anymap2::AnyMap;

/// The frozen simulation snapshot. Immutable after freeze.
/// `inertials[id]` — single load, no hash lookup.
/// `&inertials` — `&[InertialProperties]` for cudaMemcpy to GPU.
pub struct FlatWorld {
    /// Dense component arrays. Indexed by EntityID.
    /// `None` means the entity does not have this component.
    pub inertials: Vec<Option<InertialProperties>>,
    pub frames: Vec<Option<Frame>>,
    pub sites: Vec<Option<Site>>,
    pub hinge_joints: Vec<Option<HingeJoint>>,
    pub slide_joints: Vec<Option<SlideJoint>>,
    pub ball_joints: Vec<Option<BallJoint>>,
    pub free_joints: Vec<Option<FreeJoint>>,
    pub fixed_joints: Vec<Option<FixedJoint>>,
    pub universal_joints: Vec<Option<UniversalJoint>>,
    pub custom_joints: Vec<Option<CustomJoint>>,
    pub coordinates: Vec<Option<JointCoordinate>>,
    pub coordinate_effects: Vec<Option<CoordinateEffect>>,
    pub spatial_transforms: Vec<Option<SpatialTransform>>,
    pub muscles: Vec<Option<Muscle>>,
    pub millard_params: Vec<Option<Millard2012Params>>,
    pub wraps: Vec<Option<WrapGeom>>,
    pub display_geoms: Vec<Option<DisplayGeometry>>,

    /// Custom types from downstream crates.
    /// Stored as Vec<Option<T>> indexed by the same EntityID.
    /// Access via `extensions.get::<Vec<Option<MyType>>>()`.
    pub extensions: AnyMap,

    /// Total entity capacity (max ID + 1).
    pub num_entities: u32,
}

impl FlatWorld {
    /// Get a component by EntityID.
    /// Built-in types are accessed directly via named fields.
    /// Custom types dispatch through `extensions`.
    pub fn get<T: 'static>(&self, id: EntityID) -> Option<&T> {
        let i = id.0 as usize;

        // Extension types (custom/downstream)
        if let Some(vec) = self.extensions.get::<Vec<Option<T>>>() {
            return vec.get(i)?.as_ref();
        }

        None
    }

    /// Iterate all entities that have component T.
    /// For extension types only (built-in iterators are direct field access).
    pub fn iter<T: 'static>(&self) -> Vec<(EntityID, &T)> {
        let mut results = Vec::new();
        if let Some(vec) = self.extensions.get::<Vec<Option<T>>>() {
            for (i, opt) in vec.iter().enumerate() {
                if let Some(c) = opt.as_ref() {
                    results.push((EntityID(i as u32), c));
                }
            }
        }
        results
    }

    /// Number of entities.
    pub fn len(&self) -> usize {
        self.num_entities as usize
    }

    /// Returns true if no entities.
    pub fn is_empty(&self) -> bool {
        self.num_entities == 0
    }
}

impl std::fmt::Debug for FlatWorld {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FlatWorld")
            .field("entities", &self.num_entities)
            .field("inertials", &self.inertials.iter().filter_map(|x| x.as_ref()).count())
            .field("frames", &self.frames.iter().filter_map(|x| x.as_ref()).count())
            .field("hinge_joints", &self.hinge_joints.iter().filter_map(|x| x.as_ref()).count())
            .field("slide_joints", &self.slide_joints.iter().filter_map(|x| x.as_ref()).count())
            .field("ball_joints", &self.ball_joints.iter().filter_map(|x| x.as_ref()).count())
            .field("free_joints", &self.free_joints.iter().filter_map(|x| x.as_ref()).count())
            .field("fixed_joints", &self.fixed_joints.iter().filter_map(|x| x.as_ref()).count())
            .field("universal_joints", &self.universal_joints.iter().filter_map(|x| x.as_ref()).count())
            .field("custom_joints", &self.custom_joints.iter().filter_map(|x| x.as_ref()).count())
            .field("coordinates", &self.coordinates.iter().filter_map(|x| x.as_ref()).count())
            .field("coordinate_effects", &self.coordinate_effects.iter().filter_map(|x| x.as_ref()).count())
            .field("spatial_transforms", &self.spatial_transforms.iter().filter_map(|x| x.as_ref()).count())
            .field("muscles", &self.muscles.iter().filter_map(|x| x.as_ref()).count())
            .field("wraps", &self.wraps.iter().filter_map(|x| x.as_ref()).count())
            .finish()
    }
}
