use std::sync::{Arc, Mutex};
use std::path::PathBuf;

use axum::{
    body::Bytes,
    extract::{DefaultBodyLimit, State, Path},
    http::StatusCode,
    response::Json,
    routing::{get, post},
    Router,
};
use melosim::components::*;
use melosim::math::{Quaternion, Vec3};
use melosim::world::World;
use serde::{Deserialize, Serialize};
use tower_http::cors::{Any, CorsLayer};

// ── Shared state ──────────────────────────────────────

struct SharedWorld {
    world: Mutex<World>,
    mesh_dir: Mutex<PathBuf>,
}

fn upload_root() -> PathBuf {
    std::env::temp_dir().join("melosim_uploads")
}

fn bad_request(msg: impl Into<String>) -> (StatusCode, Json<ErrorResponse>) {
    (StatusCode::BAD_REQUEST, Json(ErrorResponse { error: msg.into() }))
}
unsafe impl Send for SharedWorld {}
unsafe impl Sync for SharedWorld {}

type AppState = Arc<SharedWorld>;

// ── Serializable scene snapshot ───────────────────────

#[derive(Serialize)]
struct Scene {
    num_entities: u32,
    bodies: Vec<BodyInfo>,
    joints: Vec<JointInfo>,
    muscles: Vec<MuscleInfo>,
    muscle_paths: Vec<MusclePathInfo>,
    sites: Vec<SiteInfo>,
    meshes: Vec<MeshInfo>,
}

#[derive(Serialize)]
struct BodyInfo {
    id: u32,
    name: String,
    mass: f64,
    com: [f64; 3],
    parent_id: Option<u32>,
    transform: TransformInfo,
}

#[derive(Serialize)]
struct TransformInfo {
    translation: [f64; 3],
    rotation: [f64; 4],
}

#[derive(Serialize)]
struct JointInfo {
    id: u32,
    name: String,
    joint_type: String,
    body_a: u32,
    body_b: u32,
    axis: Option<[f64; 3]>,
    limits: Option<LimitInfo>,
}

#[derive(Serialize)]
struct LimitInfo {
    lower: f64,
    upper: f64,
}

#[derive(Serialize)]
struct MuscleInfo {
    id: u32,
    name: String,
    max_isometric_force: f64,
    optimal_fiber_length: f64,
    tendon_slack_length: f64,
}

#[derive(Serialize)]
struct SiteInfo {
    id: u32,
    name: String,
    parent: u32,
    offset: [f64; 3],
}

#[derive(Serialize)]
struct MusclePathInfo {
    muscle_id: u32,
    muscle_name: String,
    points: Vec<MusclePathPoint>,
}

#[derive(Serialize)]
struct MusclePathPoint {
    body: u32,
    location: [f64; 3],
}

#[derive(Serialize)]
struct MeshInfo {
    id: u32,
    name: String,
    parent: u32,
    path: String,
    offset: [f64; 3],
    rotation: [f64; 4],
    url: String,
    scale: [f64; 3],
    color: [f64; 3],
    opacity: f64,
}

/// Try to resolve a mesh name to a file in the mesh directory.
/// Tries common extensions: .stl, .obj, .vtk, .vtp
fn resolve_mesh_path(mesh_dir: &PathBuf, mesh_name: &str) -> Option<String> {
    let extensions = [".stl", ".obj", ".vtk", ".vtp", ".ply"];
    
    // Try the name as-is first
    let path = mesh_dir.join(mesh_name);
    if path.exists() {
        return Some(mesh_name.to_string());
    }
    
    // Try with extensions
    for ext in &extensions {
        let path_with_ext = mesh_dir.join(format!("{}{}", mesh_name, ext));
        if path_with_ext.exists() {
            return Some(format!("{}{}", mesh_name, ext));
        }
    }

    // Asset names may embed geom prefixes ("humerus_geom_1_humerus") while the
    // file is named by the plain bone ("humerus.stl"), and uploaded folder
    // drops nest meshes in subdirectories. Fall back to a recursive search
    // for a file whose stem equals the asset name or is a "_"-delimited
    // suffix of it; returns the path relative to mesh_dir.
    // ponytail: recursive scan per unresolved mesh, first match wins;
    // switch to an explicit asset->file map if collisions ever matter.
    find_mesh_file(mesh_dir, mesh_name)
        .and_then(|p| p.strip_prefix(mesh_dir).ok().map(|r| r.to_string_lossy().into_owned()))
}

fn find_mesh_file(dir: &std::path::Path, mesh_name: &str) -> Option<PathBuf> {
    let mut subdirs = Vec::new();
    for entry in std::fs::read_dir(dir).ok()?.flatten() {
        let p = entry.path();
        if p.is_dir() {
            subdirs.push(p);
            continue;
        }
        let fname = entry.file_name().to_string_lossy().into_owned();
        let stem = fname.rsplit_once('.').map(|(s, _)| s).unwrap_or(&fname);
        if stem == mesh_name || (mesh_name.len() > stem.len() && mesh_name.ends_with(&format!("_{stem}"))) {
            return Some(p);
        }
    }
    subdirs.into_iter().find_map(|d| find_mesh_file(&d, mesh_name))
}

fn world_to_scene(world: &World, mesh_base_url: &str, mesh_dir: &PathBuf) -> Scene {
    let mut bodies = Vec::new();
    let mut joints = Vec::new();
    let mut muscles = Vec::new();
    let mut sites = Vec::new();
    let mut meshes = Vec::new();

    // Bodies
    for (eid, inertial) in world.iter::<InertialProperties>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        let frame = world.get::<Frame>(eid);
        bodies.push(BodyInfo {
            id: eid.0,
            name,
            mass: inertial.mass,
            com: inertial.com,
            parent_id: frame.map(|f| f.parent.0),
            transform: TransformInfo {
                translation: frame.map(|f| [f.transform.translation.x, f.transform.translation.y, f.transform.translation.z])
                    .unwrap_or([0.0; 3]),
                rotation: frame.map(|f| [f.transform.rotation.w, f.transform.rotation.x, f.transform.rotation.y, f.transform.rotation.z])
                    .unwrap_or([1.0, 0.0, 0.0, 0.0]),
            },
        });
    }

    // Joints
    for (eid, hinge) in world.iter::<HingeJoint>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        joints.push(JointInfo {
            id: eid.0,
            name,
            joint_type: "hinge".into(),
            body_a: hinge.body_a.0,
            body_b: hinge.body_b.0,
            axis: Some(hinge.axis),
            limits: hinge.limits.as_ref().map(|l| LimitInfo { lower: l.lower, upper: l.upper }),
        });
    }
    for (eid, slide) in world.iter::<SlideJoint>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        joints.push(JointInfo {
            id: eid.0,
            name,
            joint_type: "slide".into(),
            body_a: slide.body_a.0,
            body_b: slide.body_b.0,
            axis: Some(slide.axis),
            limits: slide.limits.as_ref().map(|l| LimitInfo { lower: l.lower, upper: l.upper }),
        });
    }
    for (eid, ball) in world.iter::<BallJoint>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        joints.push(JointInfo {
            id: eid.0,
            name,
            joint_type: "ball".into(),
            body_a: ball.body_a.0,
            body_b: ball.body_b.0,
            axis: None,
            limits: ball.limits.as_ref().map(|l| LimitInfo { lower: l.lower, upper: l.upper }),
        });
    }
    for (eid, free) in world.iter::<FreeJoint>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        joints.push(JointInfo {
            id: eid.0,
            name,
            joint_type: "free".into(),
            body_a: free.body_a.0,
            body_b: free.body_b.0,
            axis: None,
            limits: None,
        });
    }

    // Muscles
    for (eid, _muscle) in world.iter::<Muscle>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        let params = world.get::<Millard2012Params>(eid);
        muscles.push(MuscleInfo {
            id: eid.0,
            name,
            max_isometric_force: params.map(|p| p.max_isometric_force).unwrap_or(0.0),
            optimal_fiber_length: params.map(|p| p.optimal_fiber_length).unwrap_or(0.0),
            tendon_slack_length: params.map(|p| p.tendon_slack_length).unwrap_or(0.0),
        });
    }

    // Muscle paths
    let mut muscle_paths = Vec::new();
    for (_eid, path) in world.iter::<MusclePath>() {
        let muscle_name = world.get::<Name>(path.muscle)
            .map(|n| n.value.clone())
            .unwrap_or_default();
        let points: Vec<MusclePathPoint> = path.points.iter().filter_map(|p| {
            match p {
                PathPoint::BodyFixed { body, location } => {
                    Some(MusclePathPoint { body: body.0, location: *location })
                }
                _ => None, // Skip Moving points for now
            }
        }).collect();
        if !points.is_empty() {
            muscle_paths.push(MusclePathInfo {
                muscle_id: path.muscle.0,
                muscle_name,
                points,
            });
        }
    }

    // Sites
    for (eid, site) in world.iter::<Site>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        sites.push(SiteInfo {
            id: eid.0,
            name,
            parent: site.parent.0,
            offset: [site.offset.x, site.offset.y, site.offset.z],
        });
    }

    // Mesh geometries from DisplayGeometry
    for (eid, geom) in world.iter::<DisplayGeometry>() {
        if let Some(ref mesh_name) = geom.mesh_file {
            // Try to resolve mesh name to actual file
            if let Some(resolved_path) = resolve_mesh_path(mesh_dir, mesh_name) {
                let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
                let url = format!("{}/{}", mesh_base_url, resolved_path);
                
                meshes.push(MeshInfo {
                    id: eid.0,
                    name,
                    parent: geom.body.0,
                    path: resolved_path,
                    offset: [
                        geom.transform.translation.x,
                        geom.transform.translation.y,
                        geom.transform.translation.z,
                    ],
                    rotation: [
                        geom.transform.rotation.w,
                        geom.transform.rotation.x,
                        geom.transform.rotation.y,
                        geom.transform.rotation.z,
                    ],
                    url,
                    scale: geom.scale,
                    color: geom.color,
                    opacity: geom.opacity,
                });
            }
        }
    }

    // Also include MeshGeometry components
    for (eid, mesh_geom) in world.iter::<MeshGeometry>() {
        let name = world.get::<Name>(eid).map(|n| n.value.clone()).unwrap_or_default();
        let frame = world.get::<Frame>(eid);
        let url = format!("{}/{}", mesh_base_url, mesh_geom.mesh);
        
        meshes.push(MeshInfo {
            id: eid.0,
            name,
            parent: frame.map(|f| f.parent.0).unwrap_or(0),
            path: mesh_geom.mesh.clone(),
            offset: frame.map(|f| [f.transform.translation.x, f.transform.translation.y, f.transform.translation.z])
                .unwrap_or([0.0; 3]),
            rotation: frame.map(|f| [f.transform.rotation.w, f.transform.rotation.x, f.transform.rotation.y, f.transform.rotation.z])
                .unwrap_or([1.0, 0.0, 0.0, 0.0]),
            url,
            scale: [1.0; 3],
            color: [0.5, 0.5, 0.5],
            opacity: 1.0,
        });
    }

    Scene {
        num_entities: world.next_id,
        bodies,
        joints,
        muscles,
        muscle_paths,
        sites,
        meshes,
    }
}

// ── Request / response types ──────────────────────────

#[derive(Deserialize)]
struct AttachMeshRequest {
    parent_id: u32,
    mesh_path: String,
    name: String,
    #[serde(default)]
    offset: [f64; 3],
}

#[derive(Deserialize)]
struct AttachBodyRequest {
    parent_id: u32,
    name: String,
    #[serde(default)]
    mass: f64,
    #[serde(default)]
    offset: [f64; 3],
}

#[derive(Deserialize)]
struct BodyBuilderRequest {
    parent_name: String,
    #[serde(default)]
    name: String,
    mesh: Option<String>,
    #[serde(default)]
    mass: f64,
    #[serde(default)]
    offset: [f64; 3],
    #[serde(default = "default_rotation")]
    rotation: [f64; 4],
    display_color: Option<[f64; 3]>,
    #[serde(default = "default_opacity")]
    display_opacity: f64,
}

fn default_rotation() -> [f64; 4] {
    [1.0, 0.0, 0.0, 0.0]
}

fn default_opacity() -> f64 {
    1.0
}

#[derive(Serialize)]
struct EntityResponse {
    entity_id: u32,
}

#[derive(Serialize)]
struct ErrorResponse {
    error: String,
}

// ── Handlers ──────────────────────────────────────────

async fn get_scene(State(state): State<AppState>) -> Json<Scene> {
    let world = state.world.lock().unwrap();
    let mesh_base_url = "/meshes";
    Json(world_to_scene(&world, mesh_base_url, &state.mesh_dir.lock().unwrap()))
}

async fn post_attach_mesh(
    State(state): State<AppState>,
    Json(req): Json<AttachMeshRequest>,
) -> Result<Json<EntityResponse>, (StatusCode, Json<ErrorResponse>)> {
    let mut world = state.world.lock().unwrap();
    let parent = melosim::id::EntityID(req.parent_id);
    let offset = Vec3::from(req.offset);
    let eid = world.attach_mesh(parent, &req.mesh_path, &req.name, offset);
    Ok(Json(EntityResponse { entity_id: eid.0 }))
}

async fn post_attach_body(
    State(state): State<AppState>,
    Json(req): Json<AttachBodyRequest>,
) -> Result<Json<EntityResponse>, (StatusCode, Json<ErrorResponse>)> {
    let mut world = state.world.lock().unwrap();
    let parent = melosim::id::EntityID(req.parent_id);
    let offset = Vec3::from(req.offset);
    let eid = world.attach_body(parent, &req.name, req.mass, offset);
    Ok(Json(EntityResponse { entity_id: eid.0 }))
}

async fn post_body_builder(
    State(state): State<AppState>,
    Json(req): Json<BodyBuilderRequest>,
) -> Result<Json<EntityResponse>, (StatusCode, Json<ErrorResponse>)> {
    let mut world = state.world.lock().unwrap();
    let mut builder = world.body_builder(&req.parent_name)
        .name(&req.name)
        .mass(req.mass)
        .offset(Vec3::from(req.offset))
        .rotation(Quaternion {
            w: req.rotation[0],
            x: req.rotation[1],
            y: req.rotation[2],
            z: req.rotation[3],
        })
        .opacity(req.display_opacity);

    if let Some(ref mesh) = req.mesh {
        builder = builder.mesh(mesh);
    }
    if let Some(color) = req.display_color {
        builder = builder.color(color);
    }

    match builder.build(&mut world) {
        Some(eid) => Ok(Json(EntityResponse { entity_id: eid.0 })),
        None => Err((
            StatusCode::NOT_FOUND,
            Json(ErrorResponse {
                error: format!("Parent '{}' not found", req.parent_name),
            }),
        )),
    }
}

#[derive(Deserialize)]
struct ImportRequest {
    path: String,
    format: String,
}

async fn post_import(
    State(state): State<AppState>,
    Json(req): Json<ImportRequest>,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<ErrorResponse>)> {
    let mut world = state.world.lock().unwrap();
    
    let imported = match req.format.as_str() {
        "mjcf" => melosim::importer::mujoco::import_mjcf(&req.path)
            .map_err(|e| {
                let mut msg = e;
                if msg.contains("Error opening file") {
                    msg.push_str(
                        " — a referenced file is missing on the server. MuJoCo resolves <include> \
                         and mesh paths relative to the .xml on disk, so drag in the whole model \
                         FOLDER (e.g. myo_sim/, not arm/ alone or loose files) to bring them along.",
                    );
                }
                bad_request(msg)
            })?
            .0,
        // No native .osim parser: extract via scripts/extract_opensim.py,
        // which needs the Python `opensim` package on this machine.
        "osim" => {
            let out_path = std::env::temp_dir().join(format!("melosim_extract_{}.json", std::process::id()));
            let output = std::process::Command::new("python3")
                .args(["scripts/extract_opensim.py", &req.path, &out_path.to_string_lossy()])
                .output()
                .map_err(|e| bad_request(format!("Failed to run extractor: {e}")))?;
            if !output.status.success() {
                let stderr: String = String::from_utf8_lossy(&output.stderr).chars().take(400).collect();
                return Err(bad_request(format!(
                    "OpenSim extraction failed (requires the Python `opensim` package). \
                     Run scripts/extract_opensim.py model.osim model.json and import the .json instead. {stderr}"
                )));
            }
            let json = std::fs::read_to_string(&out_path).map_err(|e| bad_request(e.to_string()))?;
            import_osim_json(&json).map_err(bad_request)?
        }
        // Extracted OpenSim JSON (from scripts/extract_opensim.py)
        "json" => import_osim_json(&std::fs::read_to_string(&req.path).map_err(|e| bad_request(e.to_string()))?)
            .map_err(bad_request)?,
        _ => return Err(bad_request(format!("Unsupported format: {}", req.format))),
    };

    *world = imported;

    // Uploaded folder drops carry their own assets — resolve meshes there.
    let upload_root = upload_root();
    if std::path::Path::new(&req.path).starts_with(&upload_root) {
        *state.mesh_dir.lock().unwrap() = upload_root;
    }

    Ok(Json(serde_json::json!({
        "status": "ok",
        "entities": world.next_id
    })))
}

fn import_osim_json(json: &str) -> Result<World, String> {
    let data: melosim::importer::opensim::OpenSimModelData =
        serde_json::from_str(json).map_err(|e| format!("Invalid OpenSim JSON: {e}"))?;
    let mut world = World::new();
    melosim::importer::opensim::import_opensim_model(&mut world, &data)
        .map_err(|errs| errs.join("; "))?;
    Ok(world)
}

// ── File upload (drag-and-drop) ───────────────────────

async fn post_upload(
    Path(rel): Path<String>,
    body: Bytes,
) -> Result<Json<serde_json::Value>, (StatusCode, Json<ErrorResponse>)> {
    let rel_path = std::path::Path::new(&rel);
    if rel_path.is_absolute()
        || rel_path.components().any(|c| matches!(c, std::path::Component::ParentDir))
    {
        return Err(bad_request("Invalid path"));
    }
    let dest = upload_root().join(rel_path);
    if let Some(parent) = dest.parent() {
        std::fs::create_dir_all(parent).map_err(|e| bad_request(e.to_string()))?;
    }
    std::fs::write(&dest, &body).map_err(|e| bad_request(e.to_string()))?;
    Ok(Json(serde_json::json!({ "path": dest.to_string_lossy() })))
}

// ── Mesh file serving ─────────────────────────────────

async fn serve_mesh(
    State(state): State<AppState>,
    Path(path): Path<String>,
) -> Result<Vec<u8>, (StatusCode, String)> {
    let mesh_dir = state.mesh_dir.lock().unwrap();
    let file_path = mesh_dir.join(&path);
    
    if path.contains("..") {
        return Err((StatusCode::BAD_REQUEST, "Invalid path".into()));
    }
    
    match std::fs::read(&file_path) {
        Ok(data) => Ok(data),
        Err(e) => Err((StatusCode::NOT_FOUND, format!("Mesh not found: {}", e))),
    }
}

// ── Main ──────────────────────────────────────────────

#[tokio::main]
async fn main() {
    let mesh_dir = std::env::var("MESH_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|_| PathBuf::from("meshes"));
    
    let world = World::new();
    let state: AppState = Arc::new(SharedWorld {
        world: Mutex::new(world),
        mesh_dir: Mutex::new(mesh_dir),
    });

    let cors = CorsLayer::new()
        .allow_origin(Any)
        .allow_methods(Any)
        .allow_headers(Any);

    let static_dir = std::env::var("STATIC_DIR").unwrap_or_else(|_| "static".to_string());

    let app = Router::new()
        .route("/scene", get(get_scene))
        .route("/attach_mesh", post(post_attach_mesh))
        .route("/attach_body", post(post_attach_body))
        .route("/body_builder", post(post_body_builder))
        .route("/import", post(post_import))
        .route("/upload/{*path}", post(post_upload).layer(DefaultBodyLimit::disable()))
        .route("/meshes/{*path}", get(serve_mesh))
        .fallback_service(tower_http::services::ServeDir::new(&static_dir))
        .layer(cors)
        .with_state(state);

    let port = std::env::var("PORT").unwrap_or_else(|_| "3000".to_string());
    let addr = format!("0.0.0.0:{port}");
    println!("melosim-server listening on {addr}");
    println!("Mesh directory: {:?}", std::env::var("MESH_DIR").unwrap_or_else(|_| "meshes".into()));

    let listener = tokio::net::TcpListener::bind(&addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}
