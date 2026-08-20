#!/usr/bin/env python3
"""Inspect the MuJoCo myoarm model - body positions, mesh info, etc."""

import mujoco
import numpy as np
import tempfile
import os

def main():
    # Read the assets file
    with open('tests/fixtures/myo_sim/myo_sim/models/arm/assets/myoarm_r_assets.xml') as f:
        assets_content = f.read()
    
    # Read the chain file
    with open('tests/fixtures/myo_sim/myo_sim/models/arm/assets/myoarm_r_chain.xml') as f:
        chain_content = f.read()
    
    # Strip the mujocoinclude wrapper tags
    assets_content = assets_content.replace('<mujocoinclude model="MyoArm_v0.01">', '').replace('</mujocoinclude>', '')
    chain_content = chain_content.replace('<mujocoinclude model="MyoArmRight_v0.01">', '').replace('</mujocoinclude>', '')
    
    # Create a standalone model
    wrapper = f'''<?xml version="1.0" ?>
<mujoco model="myoarm_right">
    {assets_content}
    <worldbody>
        {chain_content}
    </worldbody>
</mujoco>
'''
    
    # Write to temp file in the assets directory so relative paths work
    temp_dir = 'tests/fixtures/myo_sim/myo_sim/models/arm/assets'
    with tempfile.NamedTemporaryFile(mode='w', suffix='.xml', delete=False, dir=temp_dir) as f:
        f.write(wrapper)
        temp_path = f.name
    
    try:
        model = mujoco.MjModel.from_xml_path(temp_path)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        
        print('Model loaded successfully!')
        print(f'Number of bodies: {model.nbody}')
        print(f'Number of geoms: {model.ngeom}')
        print(f'Number of joints: {model.njnt}')
        print(f'Number of sites: {model.nsite}')
        
        print('\n=== Body Positions (MuJoCo Z-up) ===')
        print(f'{"Body Name":<25} {"Position (x,y,z)":<35}')
        print('-' * 60)
        
        for i in range(model.nbody):
            name = model.body(i).name
            pos = data.xpos[i]
            print(f'{name:<25} ({pos[0]:+.6f}, {pos[1]:+.6f}, {pos[2]:+.6f})')
        
        print('\n=== Coordinate Conversion ===')
        print('MuJoCo (x, y, z) -> Bevy (x, z, -y)')
        print('\nBody positions in Bevy Y-up:')
        for i in range(model.nbody):
            name = model.body(i).name
            pos = data.xpos[i]
            # Convert to Bevy coordinates
            bevy_x = pos[0]
            bevy_y = pos[2]
            bevy_z = -pos[1]
            print(f'{name:<25} ({bevy_x:+.6f}, {bevy_y:+.6f}, {bevy_z:+.6f})')
        
        print('\n=== Mesh Geom Info ===')
        for i in range(model.ngeom):
            name = model.geom(i).name
            gtype = model.geom(i).type
            if gtype == mujoco.mjtGeom.mjGEOM_MESH:
                pos = data.geom_xpos[i]
                print(f'{name:<25} pos=({pos[0]:+.6f}, {pos[1]:+.6f}, {pos[2]:+.6f})')
        
    finally:
        os.unlink(temp_path)

if __name__ == "__main__":
    main()
