import json, sys

with open(sys.argv[1]) as f:
    d = json.load(f)

print(f"Bodies: {len(d['bodies'])}")
print(f"Joints: {len(d['joints'])}")
print(f"Markers: {len(d['markers'])}")
print(f"Muscles: {len(d['muscles'])}")
print(f"Wrap objects: {len(d['wrap_objects'])}")
print(f"Display geom: {len(d['display_geometries'])}")

for b in d['bodies'][:5]:
    print(f"  Body: {b['name']} mass={b['mass']} inertia={b['inertia']}")
for j in d['joints'][:5]:
    print(f"  Joint: {j['name']} type={j['joint_type']} parent={j['parent_body']} child={j['child_body']}")
for m in d['muscles'][:3]:
    print(f"  Muscle: {m['name']} type={m['muscle_type']}")

zero_inertia = sum(1 for b in d['bodies'] if b['inertia'] == [0.0]*6)
print(f"  Zero inertia bodies: {zero_inertia}/{len(d['bodies'])}")

custom_joints = [j for j in d['joints'] if j['coordinates']]
for j in custom_joints:
    st = j['spatial_transform']
    if st:
        populated = sum(1 for v in st.values() if v is not None)
    else:
        populated = 0
    print(f"  CustomJoint {j['name']}: {len(j['coordinates'])} coords, ST_populated={populated}/6")

for j in d['joints']:
    if j['axis']:
        print(f"  Joint {j['name']} axis: {j['axis']}")
