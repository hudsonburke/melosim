//! egui-based editor shell, overlaid on the native Bevy 3D viewport.
//! `WorldInspectorPlugin` provides the entity hierarchy + editable Inspector;
//! this module holds the parts specific to melosim (toolbar / status).

use bevy::prelude::*;
use bevy_inspector_egui::bevy_egui::egui;
use bevy_inspector_egui::bevy_egui::EguiContexts;

use super::selection::Selection;

/// Top toolbar + selection status for the editor shell.
///
/// NOTE: egui 0.34 deprecated showing panels over `&Context` (`Panel::show` /
/// `CentralPanel::show` → `show_inside(&mut Ui)`), but the root-`Ui` replacement
/// isn't surfaced through `bevy_egui` yet. These panels overlay a *transparent*
/// central frame so the Bevy 3D viewport shows through. Revisit when the egui
/// 0.34 panel API stabilises behind bevy_egui.
#[allow(deprecated)]
pub fn editor_toolbar(
    mut contexts: EguiContexts,
    selection: Res<Selection>,
    names: Query<&Name>,
) {
    let ctx = contexts.ctx_mut().expect("one primary egui context");

    // A transparent central panel keeps the Bevy 3D viewport visible; the toolbar
    // is a top panel drawn over it.
    egui::CentralPanel::default()
        .frame(egui::Frame::NONE)
        .show(ctx, |ui| {
            egui::Panel::top("melosim_toolbar").show_inside(ui, |ui| {
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
                    ui.weak("Bevy 3D viewport + egui editor shell");
                });
            });
        });
}
