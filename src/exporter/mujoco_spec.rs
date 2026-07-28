// ── MuJoCo MJCF Exporter (MjSpec-based) ──────────────
//
// Instead of generating XML from scratch, this exporter modifies
// the original MjSpec (stored as a World resource during import)
// and saves it back to XML. This preserves all original attributes,
// default classes, mesh references, compiler options, and structure.
//
// For models imported via the MjSpec importer, this is lossless.
// For models built programmatically (no MjSpec resource), falls
// back to the XML-generating exporter.

use crate::components::*;
use crate::id::EntityID;
use crate::world::World;
use super::mujoco::world_to_mjcf;
use crate::importer::mujoco_spec::StoredMjSpec;

/// Export the World to MJCF XML.
///
/// If the World contains a StoredMjSpec resource (from MjSpec import),
/// uses it for lossless round-trip. Otherwise falls back to XML generation.
pub fn world_to_mjcf_spec(world: &World, model_name: &str) -> Result<String, String> {
    // Try lossless MjSpec round-trip first
    if let Some(stored) = world.get_resource::<StoredMjSpec>() {
        // Clone the spec so we can modify it
        let mut spec = stored.spec.clone();

        // Update model name if different
        if spec.modelname() != model_name {
            spec.set_modelname(model_name);
        }

        // MjSpec requires compilation before saving
        let _model = spec.compile()
            .map_err(|e| format!("Failed to compile MjSpec: {}", e))?;

        // Save to XML string
        // Start with a reasonable buffer size, retry if too small
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

    // Fallback to XML-generating exporter
    Ok(world_to_mjcf(world, model_name))
}

/// Write MJCF to a file using MjSpec if available.
pub fn write_mjcf_spec(world: &World, path: &str, model_name: &str) -> Result<(), String> {
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
