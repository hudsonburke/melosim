use three_d::core::ClearState;
use three_d::prelude::*;
use three_d::window::{FrameOutput, Window, WindowSettings};

mod render;

fn main() {
    let window = Window::new(WindowSettings {
        title: "melosim".to_string(),
        initial_size: Some((1200, 800)),
        ..Default::default()
    })
    .expect("failed to create window");

    let context = window.gl();

    let mut gui = three_d::GUI::new(&context);

    window.render_loop(move |frame_input| {
        let mut sidebar_width = 0.0;
        gui.update(
            &mut frame_input.events.clone(),
            frame_input.accumulated_time,
            frame_input.viewport,
            frame_input.device_pixel_ratio,
            |ui| {
                three_d::egui::Panel::left("info").show(ui, |ui| {
                    ui.heading("melosim");
                    ui.label(format!(
                        "Screen: {}×{}",
                        frame_input.viewport.width, frame_input.viewport.height
                    ));
                    sidebar_width = ui.available_width();
                });
            },
        );

        frame_input
            .screen()
            .clear(ClearState::color_and_depth(0.1, 0.1, 0.1, 1.0, 1.0));

        // TODO: Draw your scene here
        // - Grid
        // - Meshes (from DisplayGeometry)
        // - Lines (joints)
        // - Points (sites)

        gui.render().expect("failed to render egui");

        FrameOutput {
            swap_buffers: true,
            wait_next_event: false,
            exit: false,
        }
    });
}
