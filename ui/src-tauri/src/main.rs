use serde::Serialize;

#[derive(Serialize)]
struct Body { id: String, name: String, position: [f32; 3], quaternion: [f32; 4] }
#[derive(Serialize)]
struct Snapshot { model: String, bodies: Vec<Body> }

#[tauri::command]
fn model_snapshot() -> Snapshot {
    Snapshot { model: "prototype".into(), bodies: vec![
        Body { id: "upper_arm".into(), name: "upper_arm".into(), position: [0.0, 0.35, 0.0], quaternion: [1.0, 0.0, 0.0, 0.0] },
        Body { id: "forearm".into(), name: "forearm".into(), position: [0.0, 0.70, 0.0], quaternion: [1.0, 0.0, 0.0, 0.0] },
    ] }
}

fn main() { tauri::Builder::default().invoke_handler(tauri::generate_handler![model_snapshot]).run(tauri::generate_context!()).expect("error while running melosim UI"); }
