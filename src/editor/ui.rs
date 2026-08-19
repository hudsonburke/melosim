//! egui-based editor shell, overlaid on the native Bevy 3D viewport.
//! `WorldInspectorPlugin` provides the entity hierarchy + editable Inspector;
//! this module holds the parts specific to melosim (toolbar / status).
//!
//! Panning note (egui 0.34 + bevy_egui): user systems run *before*
//! `Context::run()`, so `Panel`/`CentralPanel` `.show(ctx)` (which call
//! `available_rect()`) panic. We therefore build the shell from `egui::Window`s
//! (like `WorldInspectorPlugin`), which are safe pre-`run()`.

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::selection::Selection;

/// Top toolbar + selection status, drawn as an anchored floating window so it
/// stays out of the 3D viewport's way.
pub fn editor_toolbar(
    mut contexts: EguiContexts,
    selection: Res<Selection>,
    names: Query<&Name>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");

    egui::Window::new("melosim")
        .anchor(egui::Align2::LEFT_TOP, [0.0, 0.0])
        .title_bar(true)
        .resizable(false)
        .collapsible(false)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("melosim");
                ui.separator();

                match selection.primary() {
                    Some(e) => {
                        let name = names
                            .get(e)
                            .map(|n| n.as_str().to_owned())
                            .unwrap_or_else(|_| format!("{:?}", e));
                        ui.label(format!("Selected: {}", name));
                    }
                    None => {
                        ui.label("Nothing selected");
                    }
                }

                ui.separator();
                ui.weak("egui shell + Bevy 3D viewport");
            });
        });
}
