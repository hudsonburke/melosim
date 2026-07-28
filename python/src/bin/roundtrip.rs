// ── Round-trip binary ─────────────────────────────────
// CLI tool: import an .osim file via PyO3/OpenSim and export it back.
//
// Usage:
//   cargo run --bin roundtrip -- Rajagopal2015.osim [output.osim]   (needs OpenSim)
//   cargo run --bin roundtrip -- --from-json model.json [output.osim]    (from extracted JSON, no OpenSim)
//   qemu-x86_64 roundtrip Rajagopal2015.osim output.osim            (on aarch64 with QEMU)

use std::path::Path;
#[cfg(feature = "pyo3")]
use pyo3::types::PyAnyMethods;

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 2 {
        eprintln!("Usage:");
        eprintln!("  roundtrip <input.osim> [output.osim]        # PyO3 import (needs OpenSim)");
        eprintln!("  roundtrip --from-json <input.json> [output.osim]  # JSON import (no OpenSim)");
        std::process::exit(1);
    }

    // Detect --from-json mode
    let from_json = args[1] == "--from-json";
    let input_arg = if from_json { &args[2] } else { &args[1] };
    let output_path = if from_json {
        args.get(3).map(|s| s.as_str()).unwrap_or("roundtrip_output.osim")
    } else {
        args.get(2).map(|s| s.as_str()).unwrap_or("roundtrip_output.osim")
    };

    if !Path::new(input_arg).exists() {
        eprintln!("Error: input file not found: {}", input_arg);
        std::process::exit(1);
    }

    println!("=== melosim Round-Trip ===");
    println!("Input:  {}", input_arg);
    println!("Output: {}", output_path);
    println!();

    // Step 1: Import
    let world = if from_json {
        println!("[1/3] Importing model from JSON fixture...");
        match melosim::importer::opensim::load_opensim_json(input_arg) {
            Ok(model) => {
                let mut world = melosim::world::World::new();
                melosim::importer::opensim::import_opensim_model(&mut world, &model)
                    .unwrap_or_else(|errors| {
                        eprintln!("Import failed with {} errors:", errors.len());
                        for e in &errors {
                            eprintln!("  {}", e);
                        }
                        std::process::exit(1);
                    });
                world
            }
            Err(e) => {
                eprintln!("Failed to load JSON: {}", e);
                std::process::exit(1);
            }
        }
    } else {
        #[cfg(feature = "pyo3")]
        {
            println!("[1/3] Importing model via PyO3 (OpenSim)...");
            match import_via_pyo3(input_arg) {
                Ok(w) => w,
                Err(e) => {
                    eprintln!("Import failed: {}", e);
                    eprintln!("Tip: On aarch64, use '--from-json' with a pre-extracted JSON file instead.");
                    std::process::exit(1);
                }
            }
        }
        #[cfg(not(feature = "pyo3"))]
        {
            eprintln!("Error: PyO3 not available. Use '--from-json' mode instead.");
            eprintln!("  cargo run --bin roundtrip -- --from-json <file.json> [output.osim]");
            std::process::exit(1);
        }
    };
    println!("  Bodies: {}", world.count::<melosim::components::InertialProperties>());
    println!("  Joints: {}", count_all_joints(&world));
    println!("  Muscles: {}", world.count::<melosim::components::Muscle>());
    println!("  Markers: {}", world.count::<melosim::components::Site>());
    println!("  WrapGeoms: {}", world.count::<melosim::components::WrapGeom>());

    // Step 2: Validate
    println!("\n[2/3] Validating World...");
    let errors = world.validate();
    if errors.is_empty() {
        println!("  World is valid");
    } else {
        for e in &errors {
            println!("  ERROR: {}", e);
        }
    }

    // Step 3: Export
    println!("\n[3/3] Exporting to .osim XML...");
    let model_name = Path::new(input_arg)
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("model");
    match melosim::exporter::opensim::write_osim(&world, output_path, model_name) {
        Ok(()) => {
            let output_size = std::fs::metadata(output_path)
                .map(|m| m.len())
                .unwrap_or(0);
            println!("  Written: {} ({} bytes)", output_path, output_size);
        }
        Err(e) => {
            eprintln!("  Export failed: {}", e);
            std::process::exit(1);
        }
    }

    println!("\n=== Round-trip complete ===");
}

/// Import an .osim file via PyO3 (calls Python OpenSim API).
#[cfg(feature = "pyo3")]
fn import_via_pyo3(path: &str) -> Result<melosim::world::World, String> {
    pyo3::prepare_freethreaded_python();

    pyo3::Python::with_gil(|py| {
        // Import opensim module
        let opensim = py
            .import("opensim")
            .map_err(|e| format!("Failed to import opensim: {}. Make sure 'pip install opensim' is available.", e))?;

        // Load model
        let model = opensim
            .call_method1("Model", (path,))
            .map_err(|e| format!("Failed to load model: {}", e))?;

        model
            .call_method0("initSystem")
            .map_err(|e| format!("Failed to initSystem: {}", e))?;

        let mut world = melosim::world::World::new();
        let mut body_map: std::collections::HashMap<String, melosim::id::EntityID> =
            std::collections::HashMap::new();

        // ── Bodies ──
        let body_set = model.call_method0("getBodySet").map_err(|e| format!("getBodySet failed: {}", e))?;
        let num_bodies = body_set
            .call_method0("getSize")
            .map_err(|e| format!("getSize failed: {}", e))?
            .extract::<usize>()
            .map_err(|e| format!("extract usize failed: {}", e))?;

        for i in 0..num_bodies {
            let body = body_set.call_method1("get", (i,)).map_err(|e| format!("get body {i} failed: {e}"))?;
            let name: String = body
                .call_method0("getName")
                .map_err(|e| format!("getName failed: {e}"))?
                .extract()
                .map_err(|e| format!("extract name failed: {e}"))?;
            let mass: f64 = body
                .call_method0("getMass")
                .map_err(|e| format!("getMass failed: {e}"))?
                .extract()
                .map_err(|e| format!("extract mass failed: {e}"))?;
            let com = body.call_method0("getMassCenter").map_err(|e| format!("getMassCenter failed: {e}"))?;
            let inertia = body.call_method0("getInertia").map_err(|e| format!("getInertia failed: {e}"))?;

            let body_entity = world.spawn();
            world.attach(body_entity, melosim::components::InertialProperties {
                mass,
                com: [
                    com.get_item(0).map_err(|e| format!("com[0] failed: {e}"))?.extract().map_err(|e| format!("extract com[0] failed: {e}"))?,
                    com.get_item(1).map_err(|e| format!("com[1] failed: {e}"))?.extract().map_err(|e| format!("extract com[1] failed: {e}"))?,
                    com.get_item(2).map_err(|e| format!("com[2] failed: {e}"))?.extract().map_err(|e| format!("extract com[2] failed: {e}"))?,
                ],
                inertia: [
                    inertia.get_item(0).map_err(|e| format!("inertia[0] failed: {e}"))?.extract().map_err(|e| format!("extract inertia[0] failed: {e}"))?,
                    inertia.get_item(1).map_err(|e| format!("inertia[1] failed: {e}"))?.extract().map_err(|e| format!("extract inertia[1] failed: {e}"))?,
                    inertia.get_item(2).map_err(|e| format!("inertia[2] failed: {e}"))?.extract().map_err(|e| format!("extract inertia[2] failed: {e}"))?,
                    inertia.get_item(3).map_err(|e| format!("inertia[3] failed: {e}"))?.extract().map_err(|e| format!("extract inertia[3] failed: {e}"))?,
                    inertia.get_item(4).map_err(|e| format!("inertia[4] failed: {e}"))?.extract().map_err(|e| format!("extract inertia[4] failed: {e}"))?,
                    inertia.get_item(5).map_err(|e| format!("inertia[5] failed: {e}"))?.extract().map_err(|e| format!("extract inertia[5] failed: {e}"))?,
                ],
            });
            world.attach(body_entity, melosim::components::Name { value: name.clone() });
            // Frame is a separate entity referencing the body
            let frame_entity = world.spawn();
            world.attach(frame_entity, melosim::components::Frame {
                parent: body_entity,
                transform: melosim::math::Transform::default(),
            });
            body_map.insert(name, body_entity);
        }

        // Add ground if not present
        if !body_map.contains_key("ground") {
            let ground_entity = world.spawn();
            world.attach(ground_entity, melosim::components::InertialProperties {
                mass: 0.0,
                com: [0.0; 3],
                inertia: [0.0; 6],
            });
            world.attach(ground_entity, melosim::components::Name { value: "ground".to_string() });
            let ground_frame = world.spawn();
            world.attach(ground_frame, melosim::components::Frame {
                parent: ground_entity,
                transform: melosim::math::Transform::default(),
            });
            body_map.insert("ground".to_string(), ground_entity);
        }

        // ── Joints ──
        let joint_set = model
            .call_method0("getJointSet")
            .map_err(|e| format!("getJointSet failed: {e}"))?;
        let num_joints = joint_set
            .call_method0("getSize")
            .map_err(|e| format!("getSize failed: {e}"))?
            .extract::<usize>()
            .map_err(|e| format!("extract failed: {e}"))?;

        for i in 0..num_joints {
            let joint = joint_set.call_method1("get", (i,)).map_err(|e| format!("get joint {i} failed: {e}"))?;
            let joint_type: String = joint
                .call_method0("getConcreteClassName")
                .map_err(|e| format!("getConcreteClassName failed: {e}"))?
                .extract()
                .map_err(|e| format!("extract failed: {e}"))?;

            let parent_frame = joint
                .call_method0("getParentFrame")
                .map_err(|e| format!("getParentFrame failed: {e}"))?;
            let child_frame = joint
                .call_method0("getChildFrame")
                .map_err(|e| format!("getChildFrame failed: {e}"))?;
            let parent_name: String = parent_frame
                .call_method0("getName")
                .map_err(|e| format!("getName failed: {e}"))?
                .extract()
                .map_err(|e| format!("extract failed: {e}"))?;
            let child_name: String = child_frame
                .call_method0("getName")
                .map_err(|e| format!("getName failed: {e}"))?
                .extract()
                .map_err(|e| format!("extract failed: {e}"))?;

            let parent_key = *body_map
                .get(&parent_name)
                .ok_or_else(|| format!("Parent body '{}' not found for joint {}", parent_name, i))?;
            let child_key = *body_map
                .get(&child_name)
                .ok_or_else(|| format!("Child body '{}' not found for joint {}", child_name, i))?;

            match joint_type.as_str() {
                "PinJoint" => {
                    let coord = joint.call_method0("getCoordinate").map_err(|e| format!("getCoordinate failed: {e}"))?;
                    let axis_vec = coord.call_method0("getAxis").map_err(|e| format!("getAxis failed: {e}"))?;
                    let axis: [f64; 3] = [
                        axis_vec.get_item(0).map_err(|e| format!("axis[0] failed: {e}"))?.extract().map_err(|e| format!("extract axis[0] failed: {e}"))?,
                        axis_vec.get_item(1).map_err(|e| format!("axis[1] failed: {e}"))?.extract().map_err(|e| format!("extract axis[1] failed: {e}"))?,
                        axis_vec.get_item(2).map_err(|e| format!("axis[2] failed: {e}"))?.extract().map_err(|e| format!("extract axis[2] failed: {e}"))?,
                    ];
                    let joint_entity = world.spawn();
                    world.attach(joint_entity, melosim::components::HingeJoint {
                        body_a: parent_key,
                        body_b: child_key,
                        limits: None,
                        axis,
                    });
                }
                "FreeJoint" => {
                    let joint_entity = world.spawn();
                    world.attach(joint_entity, melosim::components::FreeJoint {
                        body_a: parent_key,
                        body_b: child_key,
                        limits: None,
                    });
                }
                "WeldJoint" => {
                    let joint_entity = world.spawn();
                    world.attach(joint_entity, melosim::components::FixedJoint {
                        body_a: parent_key,
                        body_b: child_key,
                        limits: None,
                    });
                }
                "BallJoint" => {
                    let joint_entity = world.spawn();
                    world.attach(joint_entity, melosim::components::BallJoint {
                        body_a: parent_key,
                        body_b: child_key,
                        limits: None,
                    });
                }
                "UniversalJoint" => {
                    let joint_entity = world.spawn();
                    world.attach(joint_entity, melosim::components::UniversalJoint {
                        body_a: parent_key,
                        body_b: child_key,
                        limits: None,
                        axis1: [1.0, 0.0, 0.0],
                        axis2: [0.0, 1.0, 0.0],
                    });
                }
                "CustomJoint" => {
                    let coord_set = joint.call_method0("getCoordinateSet").map_err(|e| format!("getCoordinateSet failed: {e}"))?;
                    let num_coords: usize = coord_set.call_method0("getSize").map_err(|e| format!("getSize failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;
                    let mut coord_keys = Vec::new();
                    for j in 0..num_coords {
                        let c = coord_set.call_method1("get", (j,)).map_err(|e| format!("get coord {j} failed: {e}"))?;
                        let cname: String = c.call_method0("getName").map_err(|e| format!("getName failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;
                        let range_min: f64 = c.call_method0("getRangeMin").map_err(|e| format!("getRangeMin failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;
                        let range_max: f64 = c.call_method0("getRangeMax").map_err(|e| format!("getRangeMax failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;
                        let clamped: bool = c.call_method0("getClamped").map_err(|e| format!("getClamped failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;
                        let locked: bool = c.call_method0("getLocked").map_err(|e| format!("getLocked failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;

                        let ck = world.spawn();
                        world.attach(ck, melosim::components::JointCoordinate {
                            range_min,
                            range_max,
                            default_value: 0.0,
                            stiffness: 0.0,
                            damping: 0.0,
                            clamped,
                            locked,
                            prescribed_function: None,
                        });
                        world.attach(ck, melosim::components::Name { value: cname });
                        coord_keys.push(ck);
                    }

                    let joint_entity = world.spawn();
                    world.attach(joint_entity, melosim::components::CustomJoint {
                        body_a: parent_key,
                        body_b: child_key,
                        limits: None,
                        coordinates: coord_keys,
                    });
                }
                _ => {
                    return Err(format!("Unknown joint type: {}", joint_type));
                }
            }
        }

        // ── Markers ──
        let marker_set = model
            .call_method0("getMarkerSet")
            .map_err(|e| format!("getMarkerSet failed: {e}"))?;
        let num_markers: usize = marker_set
            .call_method0("getSize")
            .map_err(|e| format!("getSize failed: {e}"))?
            .extract()
            .map_err(|e| format!("extract failed: {e}"))?;

        for i in 0..num_markers {
            let marker = marker_set.call_method1("get", (i,)).map_err(|e| format!("get marker {i} failed: {e}"))?;
            let name: String = marker.call_method0("getName").map_err(|e| format!("getName failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;
            let body_name: String = marker.call_method0("getBodyName").map_err(|e| format!("getBodyName failed: {e}"))?.extract().map_err(|e| format!("extract failed: {e}"))?;
            let loc = marker.call_method0("getLocation").map_err(|e| format!("getLocation failed: {e}"))?;

            if let Some(&body_key) = body_map.get(&body_name) {
                let site_entity = world.spawn();
                world.attach(site_entity, melosim::components::Site {
                    parent: body_key,
                    offset: melosim::math::Vec3::new(
                        loc.get_item(0).map_err(|e| format!("loc[0] failed: {e}"))?.extract().map_err(|e| format!("extract loc[0] failed: {e}"))?,
                        loc.get_item(1).map_err(|e| format!("loc[1] failed: {e}"))?.extract().map_err(|e| format!("extract loc[1] failed: {e}"))?,
                        loc.get_item(2).map_err(|e| format!("loc[2] failed: {e}"))?.extract().map_err(|e| format!("extract loc[2] failed: {e}"))?,
                    ),
                });
                world.attach(site_entity, melosim::components::Name { value: name });
            }
        }

        Ok(world)
    })
}

/// Count all joints in the world.
fn count_all_joints(world: &melosim::world::World) -> usize {
    world.count::<melosim::components::HingeJoint>()
        + world.count::<melosim::components::SlideJoint>()
        + world.count::<melosim::components::BallJoint>()
        + world.count::<melosim::components::FreeJoint>()
        + world.count::<melosim::components::FixedJoint>()
        + world.count::<melosim::components::UniversalJoint>()
        + world.count::<melosim::components::CustomJoint>()
}
