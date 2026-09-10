use bevy::prelude::*;
use std::{path::PathBuf, sync::Arc};

/// Shared source geometry. Render vertices are in MuJoCo's compiled mesh frame.
#[derive(Debug)]
pub struct MeshSource {
    pub file: Option<PathBuf>,
    pub scale: [f64; 3],
    pub refpos: [f64; 3],
    pub refquat: [f64; 4],
    pub vertices: Vec<[f32; 3]>,
    pub faces: Vec<[i32; 3]>,
    /// Compiled mesh frame in the authored geom frame; scale is already baked.
    pub compiled_frame: Transform,
}

/// A body-local instance of shared mesh geometry, independent of rendering.
#[derive(Component, Clone, Debug)]
#[require(Transform)]
pub struct MeshGeometry {
    pub source: Arc<MeshSource>,
    pub rgba: [f32; 4],
    pub contype: i32,
    pub conaffinity: i32,
    pub condim: i32,
    pub friction: [f64; 3],
    pub margin: f64,
    pub gap: f64,
    pub group: i32,
}
