#!/usr/bin/env python3
"""
MuJoCo/Bevy Model Inspector

Usage:
    python scripts/mujoco_model_inspector.py [command]

Commands:
    positions   - Show body positions at rest (all qpos=0)
    compare     - Compare with Bevy debug output
    sites       - Show site positions
    joints      - Show joint info
    
Coordinate convention:
    MuJoCo: Z-up, right-handed
    Bevy:   Y-up, right-handed
    Conversion: MuJoCo (x, y, z) -> Bevy (x, z, -y)
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

def mujoco_to_bevy(pos):
    """Convert MuJoCo position to Bevy coordinates."""
    return np.array([pos[0], pos[2], -pos[1]])

def compute_body_transform(body_elem, parent_pos=np.zeros(3), parent_mat=np.eye(3)):
    """Compute the global position and rotation of a body at rest."""
    pos = parse_pos(body_elem.get('pos', '0 0 0'))
    quat = body_elem.get('quat')
    
    local_mat = quat_to_matrix(parse_quat(quat)) if quat else np.eye(3)
    
    global_pos = parent_pos + parent_mat @ pos
    global_mat = parent_mat @ local_mat
    
    return global_pos, global_mat

def collect_bodies(body_elem, parent_pos=np.zeros(3), parent_mat=np.eye(3)):
    """Collect all body positions."""
    name = body_elem.get('name', 'unnamed')
    pos, mat = compute_body_transform(body_elem, parent_pos, parent_mat)
    result = [(name, pos)]
    for child in body_elem.findall('body'):
        result.extend(collect_bodies(child, pos, mat))
    return result

def collect_sites(body_elem, parent_pos=np.zeros(3), parent_mat=np.eye(3)):
    """Collect all site positions."""
    name = body_elem.get('name', 'unnamed')
    pos, mat = compute_body_transform(body_elem, parent_pos, parent_mat)
    result = []
    for site in body_elem.findall('site'):
        site_name = site.get('name', 'unnamed')
        site_pos = parse_pos(site.get('pos', '0 0 0'))
        global_site_pos = pos + mat @ site_pos
        result.append((site_name, global_site_pos))
    for child in body_elem.findall('body'):
        result.extend(collect_sites(child, pos, mat))
    return result

def main():
    import sys
    
    xml_path = 'tests/fixtures/myo_sim/myo_sim/models/arm/assets/myoarm_r_chain.xml'
    tree = ET.parse(xml_path)
    root = tree.getroot()
    top_body = root.find('.//body')
    
    if len(sys.argv) < 2 or sys.argv[1] == 'positions':
        print('=== Body Positions at Rest (all qpos=0) ===')
        print()
        bodies = collect_bodies(top_body)
        
        print(f'{"Body":<20} {"MuJoCo Z-up":<35} {"Bevy Y-up":<35}')
        print('-' * 90)
        for name, pos in bodies[:10]:  # Just show main bodies
            bevy = mujoco_to_bevy(pos)
            print(f'{name:<20} ({pos[0]:+.4f}, {pos[1]:+.4f}, {pos[2]:+.4f})   ({bevy[0]:+.4f}, {bevy[1]:+.4f}, {bevy[2]:+.4f})')
    
    elif sys.argv[1] == 'compare':
        print('=== Expected vs Actual Bevy Positions ===')
        print()
        
        bodies = collect_bodies(top_body)
        expected = {name: mujoco_to_bevy(pos) for name, pos in bodies[:10]}
        
        # Read actual from stdin or use known values
        actual = {
            'clavicle_r': np.array([0.0000, 0.0000, 0.0000]),
            'scapula_r': np.array([-0.0143, 0.1355, -0.0201]),
            'humerus_r': np.array([-0.0239, 0.1445, 0.0139]),
            'ulna_r': np.array([-0.0178, 0.1322, 0.3043]),
        }
        
        print(f'{"Body":<20} {"Expected":<30} {"Actual":<30} {"Match?":<10}')
        print('-' * 90)
        
        for name in ['clavicle_r', 'scapula_r', 'humerus_r', 'ulna_r']:
            exp = expected.get(name, np.zeros(3))
            act = actual.get(name, np.zeros(3))
            match = np.allclose(exp, act, atol=0.001)
            print(f'{name:<20} ({exp[0]:+.4f}, {exp[1]:+.4f}, {exp[2]:+.4f})   ({act[0]:+.4f}, {act[1]:+.4f}, {act[2]:+.4f})   {"✓" if match else "✗"}')
    
    elif sys.argv[1] == 'sites':
        print('=== Site Positions (Bevy Y-up) ===')
        print()
        sites = collect_sites(top_body)
        for name, pos in sites[:20]:  # Show first 20 sites
            bevy = mujoco_to_bevy(pos)
            print(f'{name:<40} ({bevy[0]:+.4f}, {bevy[1]:+.4f}, {bevy[2]:+.4f})')
    
    else:
        print(__doc__)

if __name__ == '__main__':
    main()
