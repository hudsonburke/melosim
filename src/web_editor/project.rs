//! Versioned, portable source-plus-edit-log project packages.
//! Replay is allow-listed and checked against the saved entity identities.
use super::*;
use crate::mjcf_document::MjcfDocument;

#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Project {
    format: String,
    version: u32,
    model: String,
    source: String,
    commands: Vec<Command>,
    identities: Vec<(String, String, String)>,
}

fn contained(directory: &Path, path: &str) -> Result<PathBuf, String> {
    let relative = Path::new(path);
    if relative.is_absolute()
        || relative
            .components()
            .any(|p| !matches!(p, std::path::Component::Normal(_)))
    {
        return Err("Project asset paths must stay inside the package".into());
    }
    let result = directory
        .join(relative)
        .canonicalize()
        .map_err(|e| e.to_string())?;
    if !result.starts_with(directory) {
        return Err("Project asset path escapes its package".into());
    }
    Ok(result)
}

impl Editor {
    pub(super) fn save_project(&mut self, destination: &Path) -> Result<(), String> {
        if destination.exists() {
            return Err(
                "Choose a new project folder; existing projects are not overwritten".into(),
            );
        }
        let document = self.root.and_then(|r| self.app.world().get::<MjcfDocument>(r)).cloned()
            .ok_or("This import could not retain a complete MuJoCo source; lossless saving is unavailable")?;
        let destination = std::path::absolute(destination).map_err(|e| e.to_string())?;
        let parent = destination.parent().ok_or("Invalid project path")?;
        if !parent.is_dir() {
            return Err("The parent folder must exist".into());
        }
        let stage = parent.join(format!(
            ".melosim-save-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir(&stage).map_err(|e| e.to_string())?;
        let result = (|| {
            document.package(&stage, "source.xml")?;
            let mut commands = self.journal.clone();
            let mut copied = HashMap::<String, String>::new();
            for command in &mut commands {
                if let Command::ImportPart { path, .. } = command {
                    let canonical = std::fs::canonicalize(&*path)
                        .map(|p| p.to_string_lossy().into_owned())
                        .unwrap_or_else(|_| path.clone());
                    let name = if let Some(name) = copied.get(&canonical) {
                        name.clone()
                    } else {
                        let bytes = self
                            .part_files
                            .get(&canonical)
                            .ok_or("Missing captured part asset")?;
                        let extension = Path::new(&canonical)
                            .extension()
                            .and_then(|e| e.to_str())
                            .unwrap_or("obj");
                        let name = format!("part_{}.{}", copied.len(), extension);
                        std::fs::write(stage.join(&name), bytes).map_err(|e| e.to_string())?;
                        copied.insert(canonical, name.clone());
                        name
                    };
                    *path = name;
                }
            }
            let snapshot = self.snapshot(false, None, "");
            let project = Project {
                format: "melosim-project".into(),
                version: 1,
                model: self.model.clone(),
                source: "source.xml".into(),
                commands,
                identities: snapshot
                    .nodes
                    .into_iter()
                    .map(|n| (n.id, n.name, n.kind))
                    .collect(),
            };
            let json = serde_json::to_vec_pretty(&project).map_err(|e| e.to_string())?;
            std::fs::write(stage.join("project.json"), json).map_err(|e| e.to_string())?;
            std::fs::rename(&stage, &destination).map_err(|e| e.to_string())
        })();
        if result.is_err() {
            let _ = std::fs::remove_dir_all(&stage);
        }
        result
    }

    pub(super) fn open_project(&mut self, path: &Path) -> Result<(), String> {
        let manifest = if path.is_dir() {
            path.join("project.json")
        } else {
            path.to_path_buf()
        };
        let directory = manifest
            .parent()
            .ok_or("Invalid project path")?
            .canonicalize()
            .map_err(|e| e.to_string())?;
        let project: Project =
            serde_json::from_slice(&std::fs::read(&manifest).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())?;
        if project.format != "melosim-project" || project.version != 1 {
            return Err("Unsupported project format or version".into());
        }
        if project.commands.iter().any(|c| !c.is_edit()) {
            return Err("Project contains a forbidden command".into());
        }
        let mut next = Editor::default();
        next.load(&contained(&directory, &project.source)?)?;
        next.model = project.model.clone();
        if let Some(root) = next.root {
            next.app
                .world_mut()
                .entity_mut(root)
                .insert(Name::new(project.model));
        }
        for mut command in project.commands {
            if let Command::ImportPart { path, .. } = &mut command {
                *path = contained(&directory, path)?.to_string_lossy().into_owned();
            }
            next.execute(command)?;
        }
        let identities: Vec<_> = next
            .snapshot(false, None, "")
            .nodes
            .into_iter()
            .map(|n| (n.id, n.name, n.kind))
            .collect();
        if identities != project.identities {
            return Err(
                "Project replay identity mismatch; the current workspace was not replaced".into(),
            );
        }
        next.epoch = self.epoch + 1;
        *self = next;
        Ok(())
    }
}
