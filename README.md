# melosim

A neutral data model for neuromusculoskeletal simulation, written in Rust.

melosim is not a simulator (yet) — it's a common language for musculoskeletal models. Bodies, joints, coordinates, muscles, tendons, wrap surfaces, and actuators live as typed components in an ECS World; importers read `.osim` and MJCF into that World, exporters write it back out, and physics engines (OpenSim, MuJoCo, others) are services you compile *to* — not the source of truth.

## Why?

Neuromusculoskeletal modeling is fragmented across tools that each treat their own format as the center of the universe. [OpenSim](https://opensim.stanford.edu/) made biomechanics simulation accessible to the masses and is the standard for musculoskeletal dynamics but is a C++ monolith with a heavy API, and its `.osim` XML is the only serious representation of Hill-type muscle physiology, `CustomJoint` coupled motion (knee flexion driving tibial translation via polynomials), and wrap surfaces. [MuJoCo](https://mujoco.org/) is fast, contact-rich, and the backbone of modern motor-control research — but its MJCF models were historically hand-approximated from OpenSim ones. Converters exist ([MyoConverter](https://github.com/MyoHub/myoconverter), [O2MConverter](https://github.com/aikkala/O2MConverter)) but they are one-way, lossy pipelines: joints get approximated, muscle paths get re-fit by optimization, and the result can never go back. There is no tool you can point at a model and ask "edit this, validate it, and give it back in either format, structurally intact."

Meanwhile, the new generation of physics engines doesn't close that gap — it widens it. [Newton](https://github.com/newton-physics/newton) (NVIDIA/DeepMind/Disney, GPU, Warp-based) and [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp) are spectacular runtimes, but their model schemas are robotics-shaped: rigid bodies and standard joints, no muscles, no tendons, no wrapping, no OpenSim-style coupled coordinates. [Project Chrono](https://projectchrono.org/) has an OpenSim parser that silently drops every muscle type, and its one musculoskeletal user had to hand-roll 24 muscle models in C++. [SOFA](https://www.sofa-framework.org/) targets FEM soft tissue, not neuromusculoskeletal dynamics. [SCONE](https://scone.software/) and [Hyfydy](https://hyfydy.com/) get OpenSim-accurate muscle dynamics at 100x speed — but Hyfydy is closed-source and neither offers an interchange layer. [MyoSuite](https://github.com/MyoHub/myo_sim) ships excellent converted models, but as fixed artifacts, not a toolchain. Every one of these is a place you *simulate in*, not a thing you can *build models with* across engines.

melosim fills that gap: a white-box, strongly-typed, composable data model where the model itself is the product. Rust enforces invariants at compile time; the ECS pattern makes models editable and extensible without touching core code; and exporters compile the same World to whatever engine you want to run in — today OpenSim and MuJoCo (including lossless MJCF round-trips via MjSpec), tomorrow Newton or anything else, as just another exporter. If you want GPU dynamics later, the answer is a new backend, not a rewrite.

## What it does today

- **OpenSim round-trip** — imports `.osim` (bodies, all 7 joint types, coordinates + `SpatialTransform` coupled motion, Millard2012 muscles, wrap spheres/cylinders/ellipsoids, markers, display geometry, coordinate actuators) and exports valid `.osim` back. Verified on Rajagopal 2015 (23 bodies, 22 joints, 80 muscles) and MyoSuite models.
- **MJCF round-trip** — imports MJCF via MuJoCo's own C parser (handles defaults/includes), exports MJCF, and does **lossless** round-trips via MjSpec. Verified on myoHand (547 entities) and myoLeg (619 entities).
- **Extensible ECS core** — components are plain structs in `Vec<Option<T>>` storage; new component types, systems, and validation rules register via `inventory::submit!` with zero changes to core.
- **Web viewer** — an axum server (`server/`) serves the World as JSON to a React Three Fiber frontend for interactive 3D inspection.

See [DESIGN.md](DESIGN.md) for the architecture and [docs/](docs/) for the round-trip details and roadmap.

## Repository layout

- `src/` — the core crate: `components/` (typed data), `world.rs` (ECS storage), `systems.rs` (plugin registry), `importer/` + `exporter/` (format bridges)
- `server/` — axum HTTP server exposing the World as a scene graph
- `app/` — native app scaffold (in progress)
- `python/` — PyO3 bindings for OpenSim interop
- `docs/` — design notes, round-trip verification, roadmap

## Related work

[OpenSim](https://opensim.stanford.edu/) · [MuJoCo](https://mujoco.org/) · [MyoSuite](https://github.com/MyoHub/myo_sim) · [MyoConverter](https://github.com/MyoHub/myoconverter) · [SCONE](https://scone.software/) · [Hyfydy](https://hyfydy.com/) · [Newton](https://github.com/newton-physics/newton) · [MuJoCo Warp](https://github.com/google-deepmind/mujoco_warp) · [Project Chrono](https://projectchrono.org/) · [SOFA](https://www.sofa-framework.org/) · [OpenDiHu](https://github.com/opendihu/opendihu) · [AddBiomechanics](https://addbiomechanics.org/) · [mujoco-rs](https://github.com/davidhozic/mujoco-rs)
