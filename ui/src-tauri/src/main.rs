use melosim::web_editor::{Command, EditorClient, Snapshot};

#[tauri::command]
async fn editor_command(
    command: Command,
    state: tauri::State<'_, EditorClient>,
) -> Result<Snapshot, String> {
    let client = state.inner().clone();
    tauri::async_runtime::spawn_blocking(move || client.execute(command))
        .await
        .map_err(|e| e.to_string())?
}

#[tauri::command]
async fn choose_path(save: bool, mesh: bool) -> Option<String> {
    let dialog = rfd::AsyncFileDialog::new().add_filter(
        if mesh { "Mesh" } else { "MuJoCo model" },
        if mesh {
            &["stl", "obj"][..]
        } else {
            &["xml"][..]
        },
    );
    let file = if save {
        dialog.save_file().await
    } else {
        dialog.pick_file().await
    };
    file.map(|f| f.path().to_string_lossy().into_owned())
}

fn main() {
    tauri::Builder::default()
        .manage(EditorClient::start())
        .invoke_handler(tauri::generate_handler![editor_command, choose_path])
        .run(tauri::generate_context!())
        .expect("error while running melosim UI");
}
