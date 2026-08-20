//! Editor model registry: the models the editor can load, at startup or from
//! the model menu in the toolbar.
//!
//! A model is a `&'static str` name plus a `spawn` function that injects its
//! entities into the world (via a BSN scene list). Models live in the crate —
//! see `crate::model::myoarm_skeleton` — because `.bsn` *asset files* aren't
//! loadable yet in Bevy 0.19 (the format isn't released); when they are, this
//! registry can hand out `AssetPath` handles instead of builder functions.

use bevy::prelude::*;

use crate::model::{Body, Coordinate, Frame, Joint, Muscle, Site};

/// Model builders live in the top-level `models/` directory (specific model
/// implementations, not part of the package proper). Each file is included here
/// at compile time and registered below; this becomes a runtime scan of the
/// directory once Bevy can load `.bsn` asset files.
#[path = "../../models/myoarm.rs"]
mod myoarm;

/// A spawnable model definition.
#[derive(Clone, Copy)]
pub struct ModelDef {
    pub name: &'static str,
    pub spawn: fn(&mut Commands),
}

/// All models the editor knows how to load (collected from `models/`).
#[derive(Resource)]
pub struct ModelRegistry(pub Vec<ModelDef>);

impl Default for ModelRegistry {
    fn default() -> Self {
        Self(vec![ModelDef {
            name: "MyoArm",
            spawn: |commands: &mut Commands| {
                commands.spawn_scene_list(myoarm::myoarm_skeleton());
            },
        }])
    }
}

/// Marks the root model entities, so the loaded model can be despawned on reload.
#[derive(Component)]
pub struct ModelRoot;

/// Which model in [`ModelRegistry`] to load next. When `Some`, the loader
/// despawns the current model and spawns the selected one. Initialised to the
/// default model's index (0) so the editor is usable immediately at startup.
#[derive(Resource, Default)]
pub struct SelectedModel(pub Option<usize>);

/// Despawns the previous model's whole subtree, then spawns the requested model
/// from the registry. Clears `SelectedModel` so this runs once per request.
pub fn spawn_selected_model(
    mut commands: Commands,
    registry: Option<Res<ModelRegistry>>,
    mut selection: ResMut<SelectedModel>,
    roots: Query<Entity, With<ModelRoot>>,
    children: Query<&Children>,
) {
    let Some(index) = selection.0 else {
        return;
    };
    let Some(reg) = registry else {
        return;
    };

    // Collect every entity under each model root (including mesh children) and
    // despawn them, so a reload cleanly removes the previous model.
    let mut to_despawn: Vec<Entity> = Vec::new();
    for root in roots.iter() {
        let mut stack: Vec<Entity> = vec![root];
        while let Some(entity) = stack.pop() {
            to_despawn.push(entity);
            if let Ok(children) = children.get(entity) {
                stack.extend(children.iter());
            }
        }
    }
    for entity in &to_despawn {
        commands.entity(*entity).despawn();
    }

    if let Some(def) = reg.0.get(index) {
        (def.spawn)(&mut commands);
    }

    selection.0 = None;
}

/// Tag the root model entities (model markers without a `ChildOf` parent) with
/// [`ModelRoot`] so a later load can despawn the whole model. Idempotent.
pub fn tag_model_roots(
    mut commands: Commands,
    roots: Query<
        (Entity, Option<&ModelRoot>),
        (
            Without<ChildOf>,
            Or<(
                With<Body>,
                With<Joint>,
                With<Muscle>,
                With<Site>,
                With<Frame>,
                With<Coordinate>,
            )>,
        ),
    >,
) {
    for (entity, root) in &roots {
        if root.is_none() {
            commands.entity(entity).insert(ModelRoot);
        }
    }
}
