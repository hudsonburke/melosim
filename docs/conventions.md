---
id: conventions
aliases: []
tags: []
---
# Conventions

The editor world uses Bevy's Y-up convention. Imported MuJoCo models keep
their authored body-local coordinates; a -90 degree X rotation on the model
container converts the MuJoCo Z-up world to the editor world. Export applies
the inverse conversion at the world boundary. Local offsets and axes must not
receive a second world-axis conversion.

Joint angles and coordinate limits use radians; lengths use metres.
The incoming connection is `parent body → fixed frame → joint → child body`.
The joint belongs to the child body during MuJoCo export. Inertia tensors are
expressed about the centre of mass in body-local axes, not principal axes.

MuJoCo import builds Bevy meshes directly from compiled vertices, face indices,
and per-corner normals, and places them at the compiled geom poses. Scale and
reference transforms are already baked into those vertices. No intermediate
GLTF files or filename-specific axis corrections are needed.

Each `MeshGeometry` instance shares a `MeshSource` containing the original
absolute asset path, authored scale/reference transforms, compiled vertices
and faces, and the compiler's rigid mesh-frame offset. Geometry and source
metadata survive headless import without Bevy render resources.

Export removes the compiled mesh-frame offset from file-backed geom poses and
references the original source with its authored scale/reference transforms.
Inline meshes export compiled vertices and faces with compiled geom poses.
The UI writes MJCF plus a fresh sibling mesh directory containing byte-for-byte
copies of referenced source files. Existing source files are never rewritten.
Standalone GLTF scenes remain available for visual part import.

Current limits: primitive/wrapping geometry, textures, standalone GLTF physical
geometry, cable parameters and muscle physiology are not preserved, and export uses the
current propagated body pose. General joint-frame rotations, nonzero joint
references, editor-applied geom scaling, and degree-authored MJCF import still
need dedicated round-trip coverage.
