//! Editor events for context menus and model mutations.
//!
//! This module provides lightweight event infrastructure for future undo/redo
//! and context menu support. Events are collected via `EditorEvents` (a Bevy
//! Resource) during the frame, then drained at the end of `Update`.
//!
//! - `EditorEvent::ContextMenu` — right-click on an entity in hierarchy/viewport.
//! - `EditorEvent::ModelMutation` — a structural change to the model graph.
//!
//! Because `editor_ui` is already at Bevy's 16-param system limit, events
//! created during the egui pass are staged in a thread-local
//! (`CONTEXT_MENU_STASH`) and flushed into `EditorEvents` by
//! `flush_context_menu_events`, which runs right after the egui pass.

use bevy::prelude::*;
use std::cell::RefCell;

/// An editor-level event, distinct from Bevy's `Event` trait.
///
/// These are collected into `EditorEvents` during the egui pass and can be
/// drained by systems that want to act on them (e.g., undo/redo, context menus).
#[derive(Debug, Clone)]
pub enum EditorEvent {
    /// The user right-clicked on an entity.
    ContextMenu {
        entity: Entity,
        /// Screen-space position of the click (for positioning the menu).
        screen_pos: [f32; 2],
    },

    /// A structural mutation to the model graph.
    ModelMutation {
        kind: MutationKind,
        /// The entity that was (or will be) created/modified/deleted.
        entity: Entity,
    },
}

/// Kinds of model mutations, for future undo/redo categorisation.
#[derive(Debug, Clone)]
pub enum MutationKind {
    /// A new body/joint/site/frame was spawned as a child.
    AddChild {
        parent: Entity,
        child: Entity,
        /// The model marker name (e.g. "Body", "Joint", "Site", "Frame").
        marker: &'static str,
    },
    /// A child was detached from its parent.
    RemoveChild {
        parent: Entity,
        child: Entity,
    },
    /// An entity was renamed.
    Rename {
        old_name: String,
        new_name: String,
    },
    /// An entity was deleted (despawned).
    Delete { entity: Entity },
}

/// Collects editor events during the frame. Systems push events here;
/// a drain system reads and clears them after the egui pass.
#[derive(Resource, Default, Debug)]
pub struct EditorEvents {
    pub events: Vec<EditorEvent>,
}

impl EditorEvents {
    /// Push an event.
    pub fn push(&mut self, event: EditorEvent) {
        self.events.push(event);
    }

    /// Drain all events, leaving the buffer empty.
    pub fn drain(&mut self) -> Vec<EditorEvent> {
        std::mem::take(&mut self.events)
    }

    /// Peek at pending events without consuming them.
    pub fn pending(&self) -> &[EditorEvent] {
        &self.events
    }
}

// ── Thread-local stash for egui pass ──────────────────────────
// editor_ui can't take a 17th param, so context-menu events are
// staged here during the egui pass and flushed by
// `flush_context_menu_events` right after.

thread_local! {
    static CONTEXT_MENU_STASH: RefCell<Vec<EditorEvent>> = const { RefCell::new(Vec::new()) };
}

/// Stage a context-menu event from inside an egui closure.
pub fn stash_context_menu_event(event: EditorEvent) {
    CONTEXT_MENU_STASH.with(|s| s.borrow_mut().push(event));
}

/// Drain the thread-local stash into `EditorEvents`. Run this system
/// immediately after the egui primary-context-pass systems.
pub fn flush_context_menu_events(mut events: ResMut<EditorEvents>) {
    CONTEXT_MENU_STASH.with(|s| {
        let drained = s.borrow_mut().drain(..).collect::<Vec<_>>();
        events.events.extend(drained);
    });
}
