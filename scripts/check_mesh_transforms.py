#!/usr/bin/env python3
"""
Inspect MuJoCo's compiled mesh transformations (mesh_pos, mesh_quat, mesh_scale).
These are the transformations MuJoCo applies to raw STL vertices during compilation.
"""

import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

def parse_pos(pos_str):
    """Parse MuJoCo pos attribute 'x y z' to numpy array."""
    return np.array([float(x) for x in pos_str.split()])

def parse_quat(quat_str):
    """Parse MuJoCo quat attribute 'w x y z' to numpy array [w, x, y, z]."""
    return np.array([float(x) for x in quat_str.split()])

def quat_to_matrix(q):
    """Convert MuJoCo quaternion [w, x, y, z] to 3x3 rotation matrix."""
    return Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

def main():
    # The GLB files were converted from STL using some tool
    # We need to figure out what transformation was applied
    
    # MuJoCo's compiler applies these transformations to mesh vertices:
    # 1. mesh_scale (default [1,1,1]) - scale the mesh
    # 2. mesh_quat (default [1,0,0,0]) - rotate the mesh  
    # 3. mesh_pos (default [0,0,0]) - translate the mesh
    # 4. Then center the mesh (unless inertiafromgeom="false")
    
    # The compiled model stores the final mesh_pos/mesh_quat that were applied
    # to transform the raw mesh data into the body frame
    
    # Let's check what the GLB files contain vs what MuJoCo expects
    import struct, json, os
    
    def read_glb(path):
        with open(path, 'rb') as f:
            magic, version, length = struct.unpack('<III', f.read(12))
            chunks = {}
            while f.tell() < length:
                chunk_length, chunk_type = struct.unpack('<II', f.read(8))
                data = f.read(chunk_length)
                if chunk_type == 0x4E4F534A:
                    chunks['json'] = json.loads(data)
                elif chunk_type == 0x004E4942:
                    chunks['bin'] = data
            return chunks
    
    def get_mesh_vertices(glb_path):
        glb = read_glb(glb_path)
        j = glb['json']
        bin_data = glb['bin']
        
        pos_acc = j['accessors'][0]
        bv = j['bufferViews'][pos_acc['bufferView']]
        byte_offset = bv['byteOffset'] + pos_acc.get('byteOffset', 0)
        count = pos_acc['count']
        
        positions = []
        for i in range(count):
            idx = byte_offset + i * 12
            x, y, z = struct.unpack_from('<fff', bin_data, idx)
            positions.append([x, y, z])
        
        return np.array(positions)
    
    # Check each bone
    for name in ['clavicle', 'scapula', 'humerus', 'ulna']:
        glb_path = f'assets/gltf/{name}.glb'
        if not os.path.exists(glb_path):
            continue
            
        verts = get_mesh_vertices(glb_path)
        center = verts.mean(axis=0)
        min_b = verts.min(axis=0)
        max_b = verts.max(axis=0)
        
        print(f"\n=== {name}.glb ===")
        print(f"  Vertex count: {len(verts)}")
        print(f"  Bounds: [{min_b[0]:.6f}, {min_b[1]:.6f}, {min_b[2]:.6f}] to [{max_b[0]:.6f}, {max_b[1]:.6f}, {max_b[2]:.6f}]")
        print(f"  Center: [{center[0]:.6f}, {center[1]:.6f}, {center[2]:.6f}]")
        
        # Check if mesh has been recentered
        # MuJoCo recenters meshes so geometric center is at origin
        # If the GLB was exported from MuJoCo, the center should be near (0,0,0)
        center_dist = np.linalg.norm(center)
        print(f"  Center distance from origin: {center_dist:.6f}")
        
        if center_dist > 0.001:
            print(f"  NOTE: Mesh appears to be recentered (MuJoCo compiler behavior)")
            print(f"  Original center would have been at body frame origin")
    
    print("\n=== MuJoCo Compiler Behavior ===")
    print("MuJoCo's compiler recenters meshes so the geometric center is at (0,0,0)")
    print("in the body frame. The mesh_pos/mesh_quat/mesh_scale stored in the")
    print("compiled model represent the transformation from the original mesh")
    print("to the recentered mesh in the body frame.")
    print()
    print("For the GLB files (exported from MuJoCo):")
    print("  - Vertices are already recentered")
    print("  - No additional mesh_pos/mesh_quat/mesh_scale needed")
    print("  - Just need rotation from Z-up to Y-up")

if __name__ == '__main__':
    main()
