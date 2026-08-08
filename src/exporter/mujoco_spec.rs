// ── MuJoCo MJCF Exporter (MjSpec-based) ──────────────

use crate::components::*;
use crate::world::World;
use super::mujoco::world_to_mjcf;
use crate::importer::mujoco_spec::StoredMjSpec;

pub fn world_to_mjcf_spec(world: &mut World, model_name: &str) -> Result<String, String> {
    if let Some(stored) = world.get_resource::<StoredMjSpec>() {
        let mut spec = stored.spec.clone();
        if spec.modelname() != model_name {
            spec.set_modelname(model_name);
        }
        let _model = spec.compile()
            .map_err(|e| format!("Failed to compile MjSpec: {}", e))?;
        let mut buf_size = 65536usize;
        loop {
            match spec.save_xml_string(buf_size) {
                Ok(xml) => return Ok(xml),
                Err(mujoco_rs::error::MjEditError::XmlBufferTooSmall { required_size, .. }) => {
                    buf_size = required_size + 1;
                }
                Err(e) => return Err(format!("Failed to save MJCF: {}", e)),
            }
        }
    }
    Ok(world_to_mjcf(world, model_name))
}

pub fn write_mjcf_spec(world: &mut World, path: &str, model_name: &str) -> Result<(), String> {
    if let Some(stored) = world.get_resource::<StoredMjSpec>() {
        let mut spec = stored.spec.clone();
        if spec.modelname() != model_name {
            spec.set_modelname(model_name);
        }
        let _model = spec.compile()
            .map_err(|e| format!("Failed to compile MjSpec: {}", e))?;
        spec.save_xml(path).map_err(|e| format!("Failed to save MJCF: {}", e))
    } else {
        super::mujoco::write_mjcf(world, path, model_name)
    }
}
