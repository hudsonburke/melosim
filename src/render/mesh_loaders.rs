//! A custom `Mesh` loader for STL — a format Bevy 0.19 does not bundle and for
//! which `bevy_stl` does not yet target 0.19 (it's on 0.18). OBJ is handled by
//! the external `bevy_obj` crate (0.19). Each loader turns a *file* into a
//! single Bevy `Mesh`; downstream (Body / mesh rendering) is format-agnostic.
//!
//! This parses both binary and ASCII STL into a triangle-list `Mesh` with
//! recomputed (per-triangle) normals when the file omits them.

use bevy::{
    asset::RenderAssetUsages,
    asset::{io::Reader, AssetLoader, LoadContext},
    reflect::TypePath,
    render::mesh::{Indices, Mesh, PrimitiveTopology},
    tasks::ConditionalSendFuture,
};

/// Loader for `.stl` files (STereoLithography).
#[derive(Default, TypePath)]
pub struct StlLoader;

#[derive(Debug)]
pub enum MeshLoadError {
    Io(std::io::Error),
    Stl(String),
}

impl std::fmt::Display for MeshLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MeshLoadError::Io(e) => write!(f, "io error: {e}"),
            MeshLoadError::Stl(e) => write!(f, "STL parse error: {e}"),
        }
    }
}
impl std::error::Error for MeshLoadError {}

/// Parse an STL file directly into a `Mesh` (synchronous, in-memory — no
/// `AssetServer` / filesystem watcher involved). Handles binary + ASCII.
pub fn mesh_from_stl_file(path: &std::path::Path) -> Result<Mesh, MeshLoadError> {
    let bytes = std::fs::read(path).map_err(MeshLoadError::Io)?;
    parse_stl(&bytes)
}

impl AssetLoader for StlLoader {
    type Asset = Mesh;
    type Settings = ();
    type Error = MeshLoadError;

    fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        _: &mut LoadContext,
    ) -> impl ConditionalSendFuture<Output = Result<Self::Asset, Self::Error>> {
        Box::pin(async move {
            let mut bytes = Vec::new();
            reader.read_to_end(&mut bytes).await.map_err(MeshLoadError::Io)?;
            parse_stl(&bytes)
        })
    }

    fn extensions(&self) -> &[&str] {
        &["stl"]
    }
}

fn build_mesh(
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    indices: Vec<u32>,
) -> Mesh {
    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::MAIN_WORLD | RenderAssetUsages::RENDER_WORLD,
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));
    mesh
}

fn parse_stl(bytes: &[u8]) -> Result<Mesh, MeshLoadError> {
    // Binary STL: 80-byte header + u32 triangle count + 50 bytes per triangle.
    let binary = bytes.len() >= 84
        && bytes.len()
            >= 84 + u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize * 50;

    let (positions, mut normals, mut indices) = if binary {
        parse_stl_binary(bytes)?
    } else {
        parse_stl_ascii(bytes)?
    };

    // STL is a triangle soup: emit an index per vertex.
    indices.extend(0..(positions.len() as u32));

    // Recompute normals if the file didn't provide them (all zero).
    if normals.iter().all(|n| n.iter().all(|c| *c == 0.0)) {
        normals.clear();
        for tri in positions.chunks_exact(3) {
            let a = tri[0];
            let b = tri[1];
            let c = tri[2];
            let mut n = cross(sub(b, a), sub(c, a));
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            if len > 0.0 {
                n = [n[0] / len, n[1] / len, n[2] / len];
            }
            normals.push(n);
            normals.push(n);
            normals.push(n);
        }
    }

    Ok(build_mesh(positions, normals, indices))
}

fn parse_stl_binary(
    bytes: &[u8],
) -> Result<(Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<u32>), MeshLoadError> {
    let count = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    let mut positions = Vec::with_capacity(count * 3);
    let mut normals = Vec::with_capacity(count * 3);
    for i in 0..count {
        let data = &bytes[84 + i * 50..84 + i * 50 + 48];
        let n = read_floats(&data[0..12]);
        let v1 = read_floats(&data[12..24]);
        let v2 = read_floats(&data[24..36]);
        let v3 = read_floats(&data[36..48]);
        positions.push(v1);
        positions.push(v2);
        positions.push(v3);
        normals.push(n);
        normals.push(n);
        normals.push(n);
    }
    Ok((positions, normals, Vec::new()))
}

fn parse_stl_ascii(
    bytes: &[u8],
) -> Result<(Vec<[f32; 3]>, Vec<[f32; 3]>, Vec<u32>), MeshLoadError> {
    let text = std::str::from_utf8(bytes)
        .map_err(|e| MeshLoadError::Stl(format!("ASCII STL not UTF-8: {e}")))?;
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut cur = [0.0f32; 3];
    for line in text.lines() {
        let words: Vec<&str> = line.trim().split_whitespace().collect();
        match words.first().copied() {
            Some("facet") if words.get(1) == Some(&"normal") => {
                cur = [
                    parse(words.get(2).copied()).unwrap_or(0.0),
                    parse(words.get(3).copied()).unwrap_or(0.0),
                    parse(words.get(4).copied()).unwrap_or(0.0),
                ];
            }
            Some("vertex") => {
                let v = [
                    parse(words.get(1).copied()).unwrap_or(0.0),
                    parse(words.get(2).copied()).unwrap_or(0.0),
                    parse(words.get(3).copied()).unwrap_or(0.0),
                ];
                positions.push(v);
                normals.push(cur);
            }
            _ => {}
        }
    }
    Ok((positions, normals, Vec::new()))
}

fn parse(s: Option<&str>) -> Option<f32> {
    s.and_then(|s| s.parse().ok())
}

fn read_floats(b: &[u8]) -> [f32; 3] {
    [
        f32::from_le_bytes(b[0..4].try_into().unwrap()),
        f32::from_le_bytes(b[4..8].try_into().unwrap()),
        f32::from_le_bytes(b[8..12].try_into().unwrap()),
    ]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
