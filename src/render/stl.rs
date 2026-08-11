use bevy::asset::io::Reader;
use bevy::asset::{AssetApp, AssetLoader, LoadContext, RenderAssetUsages};
use bevy::prelude::*;
use bevy::render::mesh::{Indices, PrimitiveTopology};
use std::fmt;

#[derive(Debug)]
pub enum StlError {
    Io(std::io::Error),
    Invalid(String),
}

impl fmt::Display for StlError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StlError::Io(e) => write!(f, "IO error: {e}"),
            StlError::Invalid(msg) => write!(f, "invalid STL: {msg}"),
        }
    }
}

impl std::error::Error for StlError {}

impl From<std::io::Error> for StlError {
    fn from(e: std::io::Error) -> Self {
        StlError::Io(e)
    }
}

/// STL asset loader — produces a Bevy `Mesh` directly.
///
/// Usage: `let handle: Handle<Mesh> = asset_server.load("bone.stl");`
#[derive(Default, TypePath)]
pub struct StlLoader;

impl AssetLoader for StlLoader {
    type Asset = Mesh;
    type Settings = ();
    type Error = StlError;

    async fn load(
        &self,
        reader: &mut dyn Reader,
        _settings: &Self::Settings,
        _load_context: &mut LoadContext<'_>,
    ) -> Result<Self::Asset, Self::Error> {
        let mut bytes = Vec::new();
        reader.read_to_end(&mut bytes).await?;

        if is_ascii_stl(&bytes) {
            parse_ascii_stl(&bytes)
        } else {
            parse_binary_stl(&bytes)
        }
    }

    fn extensions(&self) -> &[&str] {
        &["stl"]
    }
}

fn is_ascii_stl(bytes: &[u8]) -> bool {
    bytes.starts_with(b"solid")
        && bytes.len() > 80
        && bytes[6..].contains(&b'\n')
}

fn parse_binary_stl(bytes: &[u8]) -> Result<Mesh, StlError> {
    if bytes.len() < 84 {
        return Err(StlError::Invalid("too short for binary STL".into()));
    }

    let num_triangles = u32::from_le_bytes(bytes[80..84].try_into().unwrap()) as usize;
    let expected = 84 + num_triangles * 50;
    if bytes.len() < expected {
        return Err(StlError::Invalid(format!(
            "expected {} bytes, got {}",
            expected,
            bytes.len()
        )));
    }

    let mut positions = Vec::with_capacity(num_triangles * 3);
    let mut normals = Vec::with_capacity(num_triangles * 3);
    let mut indices = Vec::with_capacity(num_triangles * 3);

    for i in 0..num_triangles {
        let base = 84 + i * 50;

        let nx = f32::from_le_bytes(bytes[base..base + 4].try_into().unwrap());
        let ny = f32::from_le_bytes(bytes[base + 4..base + 8].try_into().unwrap());
        let nz = f32::from_le_bytes(bytes[base + 8..base + 12].try_into().unwrap());

        for v in 0..3 {
            let vbase = base + 12 + v * 12;
            let x = f32::from_le_bytes(bytes[vbase..vbase + 4].try_into().unwrap());
            let y = f32::from_le_bytes(bytes[vbase + 4..vbase + 8].try_into().unwrap());
            let z = f32::from_le_bytes(bytes[vbase + 8..vbase + 12].try_into().unwrap());
            positions.push([x, y, z]);
            normals.push([nx, ny, nz]);
        }

        let idx = (i * 3) as u32;
        indices.push(idx);
        indices.push(idx + 1);
        indices.push(idx + 2);
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));
    Ok(mesh)
}

fn parse_ascii_stl(bytes: &[u8]) -> Result<Mesh, StlError> {
    let text = std::str::from_utf8(bytes)
        .map_err(|e| StlError::Invalid(format!("not UTF-8: {e}")))?;

    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut indices = Vec::new();
    let mut current_normal = [0.0f32; 3];
    let mut vertex_count: u32 = 0;

    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("facet normal ") {
            let parts: Vec<f32> = rest
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                current_normal = [parts[0], parts[1], parts[2]];
            }
        } else if let Some(rest) = line.strip_prefix("vertex ") {
            let parts: Vec<f32> = rest
                .split_whitespace()
                .filter_map(|s| s.parse().ok())
                .collect();
            if parts.len() >= 3 {
                positions.push([parts[0], parts[1], parts[2]]);
                normals.push(current_normal);
                indices.push(vertex_count);
                vertex_count += 1;
            }
        }
    }

    if positions.is_empty() {
        return Err(StlError::Invalid("no vertices found".into()));
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, RenderAssetUsages::default());
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(Indices::U32(indices));
    Ok(mesh)
}

/// Plugin that registers the STL asset loader.
pub struct StlPlugin;

impl Plugin for StlPlugin {
    fn build(&self, app: &mut App) {
        app.register_asset_loader(StlLoader);
    }
}
