//! Transport-independent editor commands. A single worker owns the ECS World.
mod project;
use crate::model::*;
use bevy::prelude::*;
use mujoco_rs::wrappers::SpecItem;
use serde::{Deserialize, Serialize};
use std::{
    collections::{HashMap, HashSet},
    path::{Path, PathBuf},
    sync::{Arc, mpsc},
};

#[derive(Clone, Serialize, Deserialize, Debug)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum Command {
    Snapshot,
    Import {
        path: String,
    },
    LoadArm,
    SaveProject {
        path: String,
    },
    OpenProject {
        path: String,
    },
    SetColor {
        id: String,
        rgba: [f32; 4],
    },
    SetCoordinate {
        id: String,
        value: f64,
    },
    SetTransform {
        id: String,
        position: [f32; 3],
        quaternion: [f32; 4],
    },
    Rename {
        id: String,
        name: String,
    },
    AddBody {
        parent: String,
        name: String,
        mass: f64,
        size: [f32; 3],
    },
    ImportPart {
        parent: String,
        path: String,
        name: String,
        mass: f64,
    },
    AddJoint {
        parent: String,
        child: String,
        name: String,
        axis: [f64; 3],
        kind: String,
    },
    AddCable {
        name: String,
    },
    AddSite {
        parent: String,
        cable: String,
        name: String,
        position: [f32; 3],
    },
    SetPath {
        id: String,
        sites: Vec<String>,
    },
    SetCable {
        id: String,
        rest_length: f64,
        stiffness: f64,
        damping: f64,
        max_tension: f64,
        actuator_force: f64,
    },
    Export {
        path: String,
    },
    ExportVisual {
        path: String,
    },
    Undo,
    Redo,
}

impl Command {
    fn is_edit(&self) -> bool {
        matches!(
            self,
            Self::SetColor { .. }
                | Self::SetCoordinate { .. }
                | Self::SetTransform { .. }
                | Self::Rename { .. }
                | Self::AddBody { .. }
                | Self::ImportPart { .. }
                | Self::AddJoint { .. }
                | Self::AddCable { .. }
                | Self::AddSite { .. }
                | Self::SetPath { .. }
                | Self::SetCable { .. }
                | Self::Undo
                | Self::Redo
        )
    }
}

#[derive(Serialize)]
pub struct MeshDto {
    id: String,
    positions: Vec<[f32; 3]>,
    indices: Vec<i32>,
}
#[derive(Serialize)]
pub struct Node {
    id: String,
    parent: Option<String>,
    name: String,
    kind: String,
    position: [f32; 3],
    quaternion: [f32; 4],
    matrix: [f32; 16],
    mesh: Option<String>,
    rgba: Option<[f32; 4]>,
    value: Option<f64>,
    range: Option<[f64; 2]>,
    driven: bool,
    mass: Option<f64>,
    path: Vec<String>,
    length: Option<f32>,
    cable: Option<[f64; 5]>,
    editable: bool,
}
#[derive(Serialize)]
pub struct Snapshot {
    pub model: String,
    pub epoch: u64,
    pub nodes: Vec<Node>,
    pub meshes: Vec<MeshDto>,
    pub issues: Vec<String>,
    pub selected: Option<String>,
    pub message: String,
    pub can_undo: bool,
    pub can_redo: bool,
}

fn app() -> App {
    let mut app = App::new();
    app.add_plugins((MinimalPlugins, bevy::transform::TransformPlugin));
    app.insert_resource(bevy::transform::StaticTransformOptimizations::Disabled);
    app.add_systems(
        PostUpdate,
        (ensure_coordinate_states, sync_kinematics)
            .chain()
            .before(bevy::transform::TransformSystems::Propagate),
    );
    app
}
fn id(entity: Entity) -> String {
    entity.to_bits().to_string()
}
fn finite(values: &[f64]) -> Result<(), String> {
    if values.iter().all(|v| v.is_finite()) {
        Ok(())
    } else {
        Err("Values must be finite".into())
    }
}

pub struct Editor {
    app: App,
    model: String,
    root: Option<Entity>,
    epoch: u64,
    undo: Vec<(Command, Command)>,
    redo: Vec<(Command, Command)>,
    journal: Vec<Command>,
    part_files: HashMap<String, Arc<[u8]>>,
}
impl Default for Editor {
    fn default() -> Self {
        Self {
            app: app(),
            model: "Untitled model".into(),
            root: None,
            epoch: 0,
            undo: vec![],
            redo: vec![],
            journal: vec![],
            part_files: HashMap::new(),
        }
    }
}
impl Editor {
    fn entity(&self, value: &str) -> Result<Entity, String> {
        let entity = value
            .parse::<u64>()
            .ok()
            .and_then(Entity::try_from_bits)
            .ok_or("Invalid entity identifier")?;
        self.app
            .world()
            .get_entity(entity)
            .map_err(|_| "Entity no longer exists")?;
        Ok(entity)
    }
    fn parent(&self, value: &str) -> Result<Entity, String> {
        let entity = self.entity(value)?;
        let w = self.app.world();
        if w.get::<Body>(entity).is_none() && w.get::<Frame>(entity).is_none() {
            return Err("Choose a body or fixed frame as the parent".into());
        }
        Ok(entity)
    }
    fn name(&mut self, name: &str, except: Option<Entity>) -> Result<String, String> {
        let name = name.trim();
        if name.is_empty() || name.len() > 120 || name.chars().any(|c| c.is_control()) {
            return Err("Enter a name of 1–120 characters".into());
        }
        if self
            .app
            .world_mut()
            .query::<(Entity, &Name)>()
            .iter(self.app.world())
            .any(|(e, n)| Some(e) != except && n.as_str() == name)
        {
            return Err(format!("The name '{name}' is already in use"));
        }
        Ok(name.into())
    }
    fn load(&mut self, path: &Path) -> Result<(), String> {
        let mut next = app();
        let root = crate::importer::import_mjcf(next.world_mut(), path)
            .map_err(|e| format!("Import failed: {e:?}"))?;
        next.update();
        self.app = next;
        self.root = Some(root);
        self.model = path
            .file_stem()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned();
        self.epoch += 1;
        self.undo.clear();
        self.redo.clear();
        self.journal.clear();
        self.part_files.clear();
        Ok(())
    }
    pub fn execute(&mut self, mut command: Command) -> Result<Snapshot, String> {
        if let Command::ImportPart { path, .. } = &mut command {
            *path = std::fs::canonicalize(&*path)
                .map_err(|e| e.to_string())?
                .to_string_lossy()
                .into_owned();
        }
        if matches!(command, Command::Undo | Command::Redo) {
            let undo = matches!(command, Command::Undo);
            let pair = if undo {
                self.undo.pop()
            } else {
                self.redo.pop()
            }
            .ok_or("No edit to undo or redo")?;
            self.apply(if undo { &pair.0 } else { &pair.1 })?;
            if undo {
                self.redo.push(pair);
            } else {
                self.undo.push(pair);
            }
            self.app.update();
            self.journal.push(command);
            return Ok(self.snapshot(
                false,
                None,
                if undo { "Edit undone" } else { "Edit redone" },
            ));
        }
        let inverse = self.inverse(&command)?;
        let (assets, selected, message) = self.apply(&command)?;
        if let Some(before) = inverse {
            self.undo.push((before, command.clone()));
            self.redo.clear();
        }
        self.app.update();
        if command.is_edit() {
            self.journal.push(command);
        }
        Ok(self.snapshot(assets, selected, &message))
    }
    fn inverse(&self, c: &Command) -> Result<Option<Command>, String> {
        let w = self.app.world();
        Ok(match c {
            Command::SetColor { id: key, .. } => Some(Command::SetColor {
                id: key.clone(),
                rgba: w
                    .get::<MeshGeometry>(self.entity(key)?)
                    .ok_or("Select a mesh")?
                    .rgba,
            }),
            Command::SetCoordinate { id: key, .. } => Some(Command::SetCoordinate {
                id: key.clone(),
                value: w
                    .get::<CoordinateState>(self.entity(key)?)
                    .ok_or("Select a coordinate")?
                    .value,
            }),
            Command::SetTransform { id: key, .. } => {
                let t = w
                    .get::<Transform>(self.entity(key)?)
                    .ok_or("No transform")?;
                Some(Command::SetTransform {
                    id: key.clone(),
                    position: t.translation.to_array(),
                    quaternion: t.rotation.to_array(),
                })
            }
            Command::Rename { id: key, .. } => Some(Command::Rename {
                id: key.clone(),
                name: w
                    .get::<Name>(self.entity(key)?)
                    .ok_or("No name")?
                    .as_str()
                    .into(),
            }),
            Command::SetPath { id: key, .. } => Some(Command::SetPath {
                id: key.clone(),
                sites: w
                    .get::<PathEntities>(self.entity(key)?)
                    .ok_or("Select a cable")?
                    .iter()
                    .map(id)
                    .collect(),
            }),
            Command::SetCable { id: key, .. } => {
                let p = w
                    .get::<CableParameters>(self.entity(key)?)
                    .ok_or("Select a cable")?;
                Some(Command::SetCable {
                    id: key.clone(),
                    rest_length: p.rest_length,
                    stiffness: p.stiffness,
                    damping: p.damping,
                    max_tension: p.max_tension,
                    actuator_force: p.actuator_force,
                })
            }
            _ => None,
        })
    }
    fn apply(&mut self, c: &Command) -> Result<(bool, Option<String>, String), String> {
        let mut assets = false;
        let mut selected = None;
        let mut message = "Model updated".to_string();
        match c {
            Command::SaveProject { path } => {
                self.save_project(Path::new(path))?;
                message = format!("Project saved: {path}");
            }
            Command::OpenProject { path } => {
                self.open_project(Path::new(path))?;
                assets = true;
                message = "Project opened".into();
            }
            Command::SetColor { id: key, rgba } => {
                finite(&rgba.map(f64::from))?;
                if rgba.iter().any(|v| !(0.0..=1.0).contains(v)) {
                    return Err("RGBA values must be between zero and one".into());
                }
                let entity = self.entity(key)?;
                self.app
                    .world_mut()
                    .get_mut::<MeshGeometry>(entity)
                    .ok_or("Select a mesh")?
                    .rgba = *rgba;
            }
            Command::Snapshot => {
                assets = true;
                message = "Connected".into();
            }
            Command::LoadArm => {
                self.load(
                    &Path::new(env!("CARGO_MANIFEST_DIR"))
                        .join("assets/myo_sim/myo_sim/models/arm/myoarm_r.xml"),
                )?;
                assets = true;
                message = "MyoArm imported".into();
            }
            Command::Import { path } => {
                self.load(Path::new(path))?;
                assets = true;
                message = "Model imported".into();
            }
            Command::SetCoordinate { id: key, value } => {
                finite(&[*value])?;
                let e = self.entity(key)?;
                let w = self.app.world_mut();
                if w.get::<DrivenBy>(e).is_some() {
                    return Err("This coordinate is coupled to another coordinate".into());
                }
                let p = w
                    .get::<CoordinateProperties>(e)
                    .ok_or("Select a coordinate")?;
                if p.locked || (p.clamped && (*value < p.range.0 || *value > p.range.1)) {
                    return Err("Value is outside the coordinate limits".into());
                }
                w.get_mut::<CoordinateState>(e)
                    .ok_or("Coordinate state is unavailable")?
                    .value = *value;
            }
            Command::SetTransform {
                id: key,
                position,
                quaternion,
            } => {
                finite(&position.map(f64::from))?;
                finite(&quaternion.map(f64::from))?;
                let q = Quat::from_array(*quaternion);
                if q.length() < 1e-6 {
                    return Err("Rotation must be a nonzero quaternion".into());
                }
                let e = self.entity(key)?;
                let w = self.app.world_mut();
                if w.get::<Joint>(e).is_some() {
                    return Err("Use coordinate controls to move a joint".into());
                }
                let mut t = w
                    .get_mut::<Transform>(e)
                    .ok_or("This item has no transform")?;
                t.translation = Vec3::from_array(*position);
                t.rotation = q.normalize();
            }
            Command::Rename { id: key, name } => {
                let e = self.entity(key)?;
                let name = self.name(name, Some(e))?;
                self.app.world_mut().entity_mut(e).insert(Name::new(name));
            }
            Command::AddBody {
                parent,
                name,
                mass,
                size,
            } => {
                let parent = self.parent(parent)?;
                let name = self.name(name, None)?;
                finite(&[*mass])?;
                finite(&size.map(f64::from))?;
                if *mass <= 0.0 || size.iter().any(|x| *x <= 0.0) {
                    return Err("Mass and dimensions must be positive".into());
                }
                let [x, y, z] = size.map(f64::from);
                let geometry_name = self.name(&format!("{name}_mesh"), None)?;
                let inertia = Inertia::diag(
                    mass * (y * y + z * z) / 12.0,
                    mass * (x * x + z * z) / 12.0,
                    mass * (x * x + y * y) / 12.0,
                );
                let body = self
                    .app
                    .world_mut()
                    .spawn((
                        Body,
                        Name::new(name),
                        ChildOf(parent),
                        Transform::IDENTITY,
                        InertialProperties::new(*mass, nalgebra::Vector3::zeros(), inertia),
                    ))
                    .id();
                let vertices = [
                    [-1., -1., -1.],
                    [1., -1., -1.],
                    [1., 1., -1.],
                    [-1., 1., -1.],
                    [-1., -1., 1.],
                    [1., -1., 1.],
                    [1., 1., 1.],
                    [-1., 1., 1.],
                ]
                .map(|v| {
                    [
                        v[0] * size[0] / 2.,
                        v[1] * size[1] / 2.,
                        v[2] * size[2] / 2.,
                    ]
                })
                .to_vec();
                let source = Arc::new(MeshSource {
                    file: None,
                    scale: [1.; 3],
                    refpos: [0.; 3],
                    refquat: [1., 0., 0., 0.],
                    vertices,
                    faces: vec![
                        [0, 2, 1],
                        [0, 3, 2],
                        [4, 5, 6],
                        [4, 6, 7],
                        [0, 1, 5],
                        [0, 5, 4],
                        [2, 3, 7],
                        [2, 7, 6],
                        [0, 4, 7],
                        [0, 7, 3],
                        [1, 2, 6],
                        [1, 6, 5],
                    ],
                    compiled_frame: Transform::IDENTITY,
                });
                self.app.world_mut().spawn((
                    Name::new(geometry_name),
                    ChildOf(body),
                    MeshGeometry {
                        source,
                        rgba: [0.22, 0.65, 0.72, 1.],
                        contype: 0,
                        conaffinity: 0,
                        condim: 3,
                        friction: [1., 0.005, 0.0001],
                        margin: 0.,
                        gap: 0.,
                        group: 0,
                    },
                ));
                selected = Some(id(body));
                assets = true;
            }
            Command::ImportPart {
                parent,
                path,
                name,
                mass,
            } => {
                let parent = self.parent(parent)?;
                let name = self.name(name, None)?;
                finite(&[*mass])?;
                if *mass <= 0. {
                    return Err("Mass must be positive".into());
                }
                let path = Path::new(path).canonicalize().map_err(|e| e.to_string())?;
                let part_bytes: Arc<[u8]> =
                    Arc::from(std::fs::read(&path).map_err(|e| e.to_string())?);
                let mut spec = mujoco_rs::wrappers::mj_editing::MjSpec::new();
                spec.add_mesh()
                    .with_name("part_mesh")
                    .set_file(&path.to_string_lossy());
                spec.world_body_mut()
                    .add_body()
                    .with_name("part_body")
                    .add_geom()
                    .with_type(mujoco_rs::wrappers::mj_model::MjtGeom::mjGEOM_MESH)
                    .with_meshname("part_mesh");
                spec.compile().map_err(|e| e.to_string())?;
                let dir = tempfile_path();
                std::fs::create_dir(&dir).map_err(|e| e.to_string())?;
                let xml = dir.join("part.xml");
                spec.save_xml(&xml).map_err(|e| e.to_string())?;
                let mut temporary = World::new();
                let result = crate::importer::import_mjcf(&mut temporary, &xml)
                    .map_err(|e| format!("{e:?}"));
                let _ = std::fs::remove_dir_all(&dir);
                result?;
                let geometry: Vec<_> = temporary
                    .query::<(&MeshGeometry, &Transform)>()
                    .iter(&temporary)
                    .map(|(g, t)| (g.clone(), *t))
                    .collect();
                let mut inertial = temporary
                    .query::<&InertialProperties>()
                    .iter(&temporary)
                    .next()
                    .cloned()
                    .ok_or("Mesh has no inertia")?;
                let ratio = mass / inertial.mass;
                inertial.mass = *mass;
                inertial.inertia.0.iter_mut().for_each(|x| *x *= ratio);
                let body = self
                    .app
                    .world_mut()
                    .spawn((
                        Body,
                        Name::new(name),
                        ChildOf(parent),
                        Transform::IDENTITY,
                        inertial,
                    ))
                    .id();
                for (index, (g, t)) in geometry.into_iter().enumerate() {
                    self.app.world_mut().spawn((
                        Name::new(format!("part_{}_mesh_{index}", body.index())),
                        ChildOf(body),
                        g,
                        t,
                    ));
                }
                selected = Some(id(body));
                assets = true;
                self.part_files
                    .insert(path.to_string_lossy().into_owned(), part_bytes);
            }
            Command::AddJoint {
                parent,
                child,
                name,
                axis,
                kind,
            } => {
                let parent = self.parent(parent)?;
                let child = self.parent(child)?;
                let name = self.name(name, None)?;
                if self.app.world().get::<Body>(child).is_none() {
                    return Err("Choose a body as the child".into());
                }
                let mut owner = self.app.world().get::<ChildOf>(child).map(ChildOf::parent);
                while let Some(e) = owner {
                    if self.app.world().get::<Joint>(e).is_some() {
                        return Err("The child already has a joint".into());
                    }
                    if self.app.world().get::<Body>(e).is_some() {
                        break;
                    }
                    owner = self.app.world().get::<ChildOf>(e).map(ChildOf::parent);
                }
                finite(axis)?;
                let axis = nalgebra::Vector3::from(*axis);
                if axis.norm() < 1e-8 || !["hinge", "slide", "fixed"].contains(&kind.as_str()) {
                    return Err("Choose a valid joint type and nonzero axis".into());
                }
                let mut ancestor = Some(parent);
                while let Some(e) = ancestor {
                    if e == child {
                        return Err("This would create a cycle".into());
                    }
                    ancestor = self.app.world().get::<ChildOf>(e).map(ChildOf::parent);
                }
                let pose = self
                    .app
                    .world()
                    .get::<GlobalTransform>(child)
                    .ok_or("Missing child pose")?
                    .reparented_to(
                        self.app
                            .world()
                            .get::<GlobalTransform>(parent)
                            .ok_or("Missing parent pose")?,
                    );
                let frame_name = self.name(&format!("{name}_frame"), None)?;
                let coord_name = self.name(&format!("{name}_q"), None)?;
                let coordinates = if kind == "fixed" {
                    vec![]
                } else {
                    vec![CoordinateSpec {
                        name: coord_name,
                        properties: CoordinateProperties {
                            range: if kind == "slide" {
                                (-0.1, 0.1)
                            } else {
                                (-std::f64::consts::PI, std::f64::consts::PI)
                            },
                            ..default()
                        },
                        initial: InitialConditions::default(),
                        twist: Some(if kind == "slide" {
                            Twist::translation(axis.normalize())
                        } else {
                            Twist::rotation(axis.normalize())
                        }),
                        driven_by: None,
                    }]
                };
                self.app.world_mut().entity_mut(child).remove::<ChildOf>();
                ModelBuilder::new(self.app.world_mut())
                    .joint_connection(
                        parent,
                        frame_name,
                        pose,
                        name,
                        child,
                        Transform::IDENTITY,
                        coordinates,
                    )
                    .map_err(|e| e.to_string())?;
                selected = Some(id(child));
            }
            Command::AddCable { name } => {
                let name = self.name(name, None)?;
                selected = Some(id(self
                    .app
                    .world_mut()
                    .spawn((Cable, Name::new(name)))
                    .id()));
            }
            Command::AddSite {
                parent,
                cable,
                name,
                position,
            } => {
                let parent = self.parent(parent)?;
                let cable = self.entity(cable)?;
                let name = self.name(name, None)?;
                finite(&position.map(f64::from))?;
                if self.app.world().get::<Cable>(cable).is_none() {
                    return Err("Choose a cable".into());
                }
                let site = self
                    .app
                    .world_mut()
                    .spawn((
                        Site,
                        Name::new(name),
                        ChildOf(parent),
                        Transform::from_translation(Vec3::from_array(*position)),
                        PathElement(cable),
                    ))
                    .id();
                selected = Some(id(site));
            }
            Command::SetPath { id: key, sites } => {
                let cable = self.entity(key)?;
                if self.app.world().get::<Cable>(cable).is_none() {
                    return Err("Only cable paths are editable here".into());
                }
                let sites = sites
                    .iter()
                    .map(|s| self.entity(s))
                    .collect::<Result<Vec<_>, _>>()?;
                if sites.iter().copied().collect::<HashSet<_>>().len() != sites.len() {
                    return Err("A site cannot appear twice in this path".into());
                }
                if sites
                    .iter()
                    .any(|s| self.app.world().get::<Site>(*s).is_none())
                {
                    return Err("Paths must reference sites".into());
                }
                self.app
                    .world_mut()
                    .get_mut::<PathEntities>(cable)
                    .unwrap()
                    .set(sites);
            }
            Command::SetCable {
                id: key,
                rest_length,
                stiffness,
                damping,
                max_tension,
                actuator_force,
            } => {
                let values = [
                    *rest_length,
                    *stiffness,
                    *damping,
                    *max_tension,
                    *actuator_force,
                ];
                finite(&values)?;
                if values.iter().any(|v| *v < 0.) {
                    return Err("Cable parameters cannot be negative".into());
                }
                let e = self.entity(key)?;
                let mut p = self
                    .app
                    .world_mut()
                    .get_mut::<CableParameters>(e)
                    .ok_or("Select a cable")?;
                *p = CableParameters {
                    rest_length: *rest_length,
                    stiffness: *stiffness,
                    damping: *damping,
                    max_tension: *max_tension,
                    actuator_force: *actuator_force,
                };
            }
            Command::Export { path } | Command::ExportVisual { path } => {
                let visual_only = matches!(c, Command::ExportVisual { .. });
                self.app.update();
                let issues = validate_kinematic_hierarchy(self.app.world_mut());
                if !issues.is_empty() {
                    return Err(format!(
                        "Resolve hierarchy errors before export: {}",
                        issues[0]
                    ));
                }
                for (name, path) in self
                    .app
                    .world_mut()
                    .query::<(&Name, &PathEntities)>()
                    .iter(self.app.world())
                {
                    if !visual_only && path.len() < 2 {
                        return Err(format!("{} needs at least two path sites", name.as_str()));
                    }
                }
                if Path::new(path).exists() {
                    return Err("That file already exists. Choose a new export filename.".into());
                }
                let save = if visual_only {
                    crate::exporter::save_mjcf_visual
                } else {
                    crate::exporter::save_mjcf
                };
                save(
                    self.app.world_mut(),
                    self.root.unwrap_or(Entity::PLACEHOLDER),
                    Path::new(path),
                )
                .map_err(|e| format!("Export failed: {e:?}"))?;
                message = format!("Exported {path} and mesh assets");
            }
            Command::Undo | Command::Redo => unreachable!(),
        }
        if matches!(
            c,
            Command::AddBody { .. }
                | Command::ImportPart { .. }
                | Command::AddJoint { .. }
                | Command::AddCable { .. }
                | Command::AddSite { .. }
        ) {
            self.undo.clear();
            self.redo.clear();
        }
        Ok((assets, selected, message))
    }
    fn snapshot(
        &mut self,
        include_meshes: bool,
        selected: Option<String>,
        message: &str,
    ) -> Snapshot {
        let mut meshes = vec![];
        let mut mesh_ids = HashMap::new();
        let mut nodes = vec![];
        let w = self.app.world_mut();
        for geometry in w.query::<&MeshGeometry>().iter(w) {
            let key = Arc::as_ptr(&geometry.source) as usize;
            if mesh_ids.contains_key(&key) {
                continue;
            }
            let mesh_id = format!("{}-{key}", self.epoch);
            mesh_ids.insert(key, mesh_id.clone());
            if include_meshes {
                meshes.push(MeshDto {
                    id: mesh_id,
                    positions: geometry.source.vertices.clone(),
                    indices: geometry.source.faces.iter().flatten().copied().collect(),
                });
            }
        }
        let entities = w
            .query::<(Entity, &Name)>()
            .iter(w)
            .map(|(e, _)| e)
            .collect::<Vec<_>>();
        for e in entities {
            let kind = if w.get::<MeshGeometry>(e).is_some() {
                "mesh"
            } else if w.get::<Body>(e).is_some() {
                "body"
            } else if w.get::<Joint>(e).is_some() {
                "joint"
            } else if w.get::<Coordinate>(e).is_some() {
                "coordinate"
            } else if w.get::<Site>(e).is_some() {
                "site"
            } else if w.get::<Cable>(e).is_some() {
                "cable"
            } else if w.get::<Muscle>(e).is_some() {
                "muscle"
            } else if w.get::<Frame>(e).is_some() {
                "frame"
            } else {
                continue;
            };
            let t = w.get::<Transform>(e).copied().unwrap_or_default();
            let geometry = w.get::<MeshGeometry>(e);
            let path = w
                .get::<PathEntities>(e)
                .map(|p| p.iter().collect::<Vec<_>>())
                .unwrap_or_default();
            let length = if path.is_empty() {
                None
            } else {
                Some(
                    path.windows(2)
                        .filter_map(|p| {
                            Some(
                                w.get::<GlobalTransform>(p[0])?
                                    .translation()
                                    .distance(w.get::<GlobalTransform>(p[1])?.translation()),
                            )
                        })
                        .sum(),
                )
            };
            let value = w.get::<CoordinateState>(e).map(|c| c.value);
            let range = w.get::<CoordinateProperties>(e).map(|p| {
                if p.clamped && p.range.0.abs() < 1e6 && p.range.1.abs() < 1e6 {
                    [p.range.0, p.range.1]
                } else {
                    [-std::f64::consts::PI, std::f64::consts::PI]
                }
            });
            let cable = w.get::<CableParameters>(e).map(|p| {
                [
                    p.rest_length,
                    p.stiffness,
                    p.damping,
                    p.max_tension,
                    p.actuator_force,
                ]
            });
            nodes.push(Node {
                id: id(e),
                parent: w
                    .get::<ChildOf>(e)
                    .map(|p| id(p.parent()))
                    .or_else(|| w.get::<CoordinateOf>(e).map(|p| id(p.0))),
                name: w.get::<Name>(e).unwrap().as_str().into(),
                kind: kind.into(),
                position: t.translation.to_array(),
                quaternion: t.rotation.to_array(),
                matrix: w
                    .get::<GlobalTransform>(e)
                    .copied()
                    .unwrap_or_default()
                    .to_matrix()
                    .to_cols_array(),
                mesh: geometry.map(|g| mesh_ids[&(Arc::as_ptr(&g.source) as usize)].clone()),
                rgba: geometry.map(|g| g.rgba),
                value,
                range,
                driven: w.get::<DrivenBy>(e).is_some(),
                mass: w.get::<InertialProperties>(e).map(|p| p.mass),
                path: path.into_iter().map(id).collect(),
                length,
                cable,
                editable: w.get::<Transform>(e).is_some() && kind != "joint",
            });
        }
        nodes.sort_by(|a, b| a.name.cmp(&b.name));
        let mut issues = validate_kinematic_hierarchy(w)
            .into_iter()
            .map(|i| i.to_string())
            .collect::<Vec<_>>();
        if let Some(warning) = self
            .root
            .and_then(|r| w.get::<crate::mjcf_document::MjcfDocument>(r))
            .and_then(|d| d.warning.as_ref())
        {
            issues.push(format!("Source simulation error: {warning}. Project saving retains the source; use visual-only export for appearance."));
        }
        for n in &nodes {
            if n.kind == "cable" && n.path.len() < 2 {
                issues.push(format!("{} needs at least two path sites", n.name));
            }
        }
        Snapshot {
            model: self.model.clone(),
            epoch: self.epoch,
            nodes,
            meshes,
            issues,
            selected,
            message: message.into(),
            can_undo: !self.undo.is_empty(),
            can_redo: !self.redo.is_empty(),
        }
    }
}
fn tempfile_path() -> PathBuf {
    std::env::temp_dir().join(format!(
        "melosim-part-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn saves_invalid_myoarm_source_without_discarding_it() {
        let mut editor = Editor::default();
        editor.execute(Command::LoadArm).unwrap();
        let destination = tempfile_path();
        editor.save_project(&destination).unwrap();
        let mut restored = Editor::default();
        restored.open_project(&destination).unwrap();
        let document = restored
            .app
            .world()
            .get::<crate::mjcf_document::MjcfDocument>(restored.root.unwrap())
            .unwrap();
        assert!(document.xml.contains("PECM2_PECM2-P3_r"));
        assert!(document.warning.is_some());
        assert_eq!(
            editor.snapshot(false, None, "").nodes.len(),
            restored.snapshot(false, None, "").nodes.len()
        );
        restored
            .execute(Command::ExportVisual {
                path: destination
                    .join("visual.xml")
                    .to_string_lossy()
                    .into_owned(),
            })
            .unwrap();
        std::fs::remove_dir_all(destination).unwrap();
    }

    #[test]
    fn editor_workflow_and_export() {
        let mut editor = Editor::default();
        let fixture = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures/rotated_arm.xml");
        let snapshot = editor
            .execute(Command::Import {
                path: fixture.to_string_lossy().into(),
            })
            .unwrap();
        let key = |name: &str, kind: &str| {
            snapshot
                .nodes
                .iter()
                .find(|n| n.name == name && n.kind == kind)
                .unwrap()
                .id
                .clone()
        };
        let upper = key("upper", "body");
        let forearm = key("forearm", "body");
        let elbow = key("elbow", "coordinate");
        let before = snapshot
            .nodes
            .iter()
            .find(|n| n.name == "insertion")
            .unwrap()
            .matrix;
        assert!(!snapshot.meshes.is_empty());
        let posed = editor
            .execute(Command::SetCoordinate {
                id: elbow.clone(),
                value: 0.7,
            })
            .unwrap();
        assert!(posed.meshes.is_empty());
        assert_ne!(
            before,
            posed
                .nodes
                .iter()
                .find(|n| n.name == "insertion")
                .unwrap()
                .matrix
        );
        let undone = editor.execute(Command::Undo).unwrap();
        assert_eq!(
            before,
            undone
                .nodes
                .iter()
                .find(|n| n.name == "insertion")
                .unwrap()
                .matrix
        );
        editor.execute(Command::Redo).unwrap();
        assert!(
            editor
                .execute(Command::SetCoordinate {
                    id: elbow,
                    value: 99.
                })
                .is_err()
        );
        assert!(
            editor
                .execute(Command::AddJoint {
                    parent: upper.clone(),
                    child: forearm.clone(),
                    name: "bad_joint".into(),
                    axis: [1., 0., 0.],
                    kind: "hinge".into()
                })
                .is_err()
        );
        let body = editor
            .execute(Command::AddBody {
                parent: upper.clone(),
                name: "cuff".into(),
                mass: 0.2,
                size: [0.04, 0.06, 0.03],
            })
            .unwrap()
            .selected
            .unwrap();
        editor
            .execute(Command::AddJoint {
                parent: upper.clone(),
                child: body,
                name: "exo_joint".into(),
                axis: [1., 0., 0.],
                kind: "hinge".into(),
            })
            .unwrap();
        let cable = editor
            .execute(Command::AddCable {
                name: "assist".into(),
            })
            .unwrap()
            .selected
            .unwrap();
        let destination = tempfile_path();
        std::fs::create_dir(&destination).unwrap();
        let path = destination.join("model.xml").to_string_lossy().into_owned();
        assert!(
            editor
                .execute(Command::Export { path: path.clone() })
                .is_err()
        );
        let first = editor
            .execute(Command::AddSite {
                parent: upper.clone(),
                cable: cable.clone(),
                name: "assist_origin".into(),
                position: [0., 0., 0.],
            })
            .unwrap()
            .selected
            .unwrap();
        let second = editor
            .execute(Command::AddSite {
                parent: forearm,
                cable: cable.clone(),
                name: "assist_end".into(),
                position: [0., -0.1, 0.],
            })
            .unwrap()
            .selected
            .unwrap();
        let routed = editor
            .execute(Command::SetPath {
                id: cable.clone(),
                sites: vec![second.clone(), first.clone()],
            })
            .unwrap();
        assert_eq!(
            routed.nodes.iter().find(|n| n.id == cable).unwrap().path,
            vec![second, first]
        );
        editor.execute(Command::Undo).unwrap();
        editor
            .execute(Command::ImportPart {
                parent: upper,
                path: Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("tests/fixtures/asymmetric.obj")
                    .to_string_lossy()
                    .into(),
                name: "custom_part".into(),
                mass: 0.3,
            })
            .unwrap();
        let mesh = editor
            .snapshot(false, None, "")
            .nodes
            .iter()
            .find(|n| n.kind == "mesh")
            .unwrap()
            .id
            .clone();
        editor
            .execute(Command::SetColor {
                id: mesh.clone(),
                rgba: [0.1, 0.8, 0.3, 0.6],
            })
            .unwrap();
        editor
            .execute(Command::SetCable {
                id: cable.clone(),
                rest_length: 0.3,
                stiffness: 250.,
                damping: 2.,
                max_tension: 80.,
                actuator_force: 20.,
            })
            .unwrap();
        let package = destination.join("edited.melosim");
        editor
            .execute(Command::SaveProject {
                path: package.to_string_lossy().into(),
            })
            .unwrap();
        let mut restored = Editor::default();
        let opened = restored
            .execute(Command::OpenProject {
                path: package.to_string_lossy().into(),
            })
            .unwrap();
        assert_eq!(
            opened.nodes.iter().find(|n| n.id == mesh).unwrap().rgba,
            Some([0.1, 0.8, 0.3, 0.6])
        );
        assert_eq!(
            opened.nodes.iter().find(|n| n.id == cable).unwrap().cable,
            Some([0.3, 250., 2., 80., 20.])
        );
        assert!(opened.can_undo);
        restored.execute(Command::Undo).unwrap();
        restored.execute(Command::Redo).unwrap();
        let mut manifest: serde_json::Value =
            serde_json::from_slice(&std::fs::read(package.join("project.json")).unwrap()).unwrap();
        manifest["commands"] =
            serde_json::json!([{"type":"export","path":"/tmp/should-not-write.xml"}]);
        std::fs::write(
            package.join("project.json"),
            serde_json::to_vec(&manifest).unwrap(),
        )
        .unwrap();
        assert!(
            restored
                .execute(Command::OpenProject {
                    path: package.to_string_lossy().into()
                })
                .is_err()
        );
        assert!(
            restored
                .snapshot(false, None, "")
                .nodes
                .iter()
                .any(|n| n.name == "custom_part")
        );
        editor
            .execute(Command::Export { path: path.clone() })
            .unwrap();
        assert!(
            editor
                .execute(Command::Export { path: path.clone() })
                .is_err()
        );
        let mut roundtrip = Editor::default();
        let result = roundtrip.execute(Command::Import { path }).unwrap();
        assert!(result.nodes.iter().any(|n| n.name == "custom_part"));
        assert!(
            result
                .nodes
                .iter()
                .any(|n| n.name == "assist_tendon" && n.path.len() == 2)
        );
        assert!(
            editor
                .execute(Command::Import {
                    path: "/missing/model.xml".into()
                })
                .is_err()
        );
        assert!(
            editor
                .execute(Command::Snapshot)
                .unwrap()
                .nodes
                .iter()
                .any(|n| n.name == "custom_part")
        );
        std::fs::remove_dir_all(destination).unwrap();
    }
}

#[derive(Clone)]
pub struct EditorClient(mpsc::Sender<(Command, mpsc::Sender<Result<Snapshot, String>>)>);
impl EditorClient {
    pub fn start() -> Self {
        let (tx, rx) = mpsc::channel::<(Command, mpsc::Sender<Result<Snapshot, String>>)>();
        std::thread::spawn(move || {
            let mut editor = Editor::default();
            for (command, reply) in rx {
                let _ = reply.send(editor.execute(command));
            }
        });
        Self(tx)
    }
    pub fn execute(&self, command: Command) -> Result<Snapshot, String> {
        let (tx, rx) = mpsc::channel();
        self.0.send((command, tx)).map_err(|_| "Editor stopped")?;
        rx.recv().map_err(|_| "Editor stopped")?
    }
}
