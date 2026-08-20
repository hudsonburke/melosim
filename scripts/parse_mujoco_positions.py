#!/usr/bin/env python3
"""Parse MuJoCo myoarm XML and compute body positions at rest (all qpos=0)."""

import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation

def parse_pos(pos_str):
    """Parse MuJoCo pos attribute 'x y z' to numpy array."""
    return np.array([float(x) for x in pos_str.split()])

def parse_quat(quat_str):
    """Parse MuJoCo quat attribute 'w x y z' to numpy array [w, x, y, z]."""
    return np.array([float(x) for x in quat_str.split()])

def parse_euler(euler_str):
    """Parse MuJoCo euler attribute 'x y z' (radians) to rotation matrix."""
    xyz = [float(x) for x in euler_str.split()]
    return Rotation.from_euler('xyz', xyz).as_matrix()

def quat_to_matrix(q):
    """Convert MuJoCo quaternion [w, x, y, z] to 3x3 rotation matrix."""
    # scipy uses [x, y, z, w] format
    return Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix()

def compute_body_transform(body_elem, parent_pos=np.zeros(3), parent_mat=np.eye(3)):
    """Compute the global position and rotation of a body at rest (all qpos=0)."""
    # Get body position relative to parent
    pos = parse_pos(body_elem.get('pos', '0 0 0'))
    
    # Get body rotation (if specified)
    quat = body_elem.get('quat')
    euler = body_elem.get('euler')
    
    local_mat = np.eye(3)
    if quat:
        local_mat = quat_to_matrix(parse_quat(quat))
    elif euler:
        local_mat = parse_euler(euler)
    
    # Compute global position and rotation
    global_pos = parent_pos + parent_mat @ pos
    global_mat = parent_mat @ local_mat
    
    return global_pos, global_mat

def walk_body(body_elem, parent_pos=np.zeros(3), parent_mat=np.eye(3), depth=0):
    """Walk the body tree and print positions."""
    name = body_elem.get('name', 'unnamed')
    pos, mat = compute_body_transform(body_elem, parent_pos, parent_mat)
    
    indent = '  ' * depth
    print(f'{indent}{name}: pos=({pos[0]:.6f}, {pos[1]:.6f}, {pos[2]:.6f})')
    
    # Process child bodies
    for child in body_elem.findall('body'):
        walk_body(child, pos, mat, depth + 1)

def main():
    xml_path = 'tests/fixtures/myo_sim/myo_sim/models/arm/assets/myoarm_r_chain.xml'
    
    print(f'Parsing: {xml_path}')
    print()
    
    tree = ET.parse(xml_path)
    root = tree.getroot()
    
    # Find the top-level body (first body element)
    top_body = root.find('.//body')
    if top_body is None:
        print('No body found!')
        return
    
    print('=== Body Hierarchy (MuJoCo Z-up, all qpos=0) ===')
    walk_body(top_body)
    
    print()
    print('=== Coordinate Conversion ===')
    print('MuJoCo (x, y, z) -> Bevy (x, z, -y)')
    print()
    
    # Collect body positions
    bodies = []
    def collect_bodies(body_elem, parent_pos=np.zeros(3), parent_mat=np.eye(3)):
        name = body_elem.get('name', 'unnamed')
        pos, mat = compute_body_transform(body_elem, parent_pos, parent_mat)
        bodies.append((name, pos))
        for child in body_elem.findall('body'):
            collect_bodies(child, pos, mat)
    
    collect_bodies(top_body)
    
    print('Body positions in Bevy Y-up:')
    print(f'{"Body Name":<25} {"MuJoCo (x,y,z)":<40} {"Bevy (x,z,-y)":<40}')
    print('-' * 105)
    
    for name, pos in bodies:
        bevy_x = pos[0]
        bevy_y = pos[2]
        bevy_z = -pos[1]
        print(f'{name:<25} ({pos[0]:+.6f}, {pos[1]:+.6f}, {pos[2]:+.6f})   ({bevy_x:+.6f}, {bevy_y:+.6f}, {bevy_z:+.6f})')

if __name__ == '__main__':
    main()
