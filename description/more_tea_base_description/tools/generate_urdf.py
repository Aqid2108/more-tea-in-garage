#!/usr/bin/env python3
"""Generate the More Tea navigation-base URDF from a Fusion 360 export.

Source design : "MoreTea's Body v5"  (Fusion 360, MORE-TEA / Navigation)
Inputs        : data/fusion_dump.json, data/occurrence_transforms.json   (written by the Fusion scripts)
                meshes/raw/*.stl  (Fusion STL export: millimetres, component-local coordinates)
Outputs       : urdf/more_tea.urdf, meshes/visual/*.stl (metres), docs/JOINT_TABLE.md, data/joint_table.csv

Frame convention (REP-103):  x forward = Fusion +X,  y left = Fusion -Z,  z up = Fusion +Y.
base_link sits at the centre of the four wheel axles; base_footprint is the ground projection below it.

Usage:  python tools/generate_urdf.py && python tools/validate_urdf.py
"""
import csv, hashlib, json, math, os, re, struct, sys
import numpy as np

PKG = 'more_tea_base_description'
ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
DATA, RAW = os.path.join(ROOT, 'data'), os.path.join(ROOT, 'meshes', 'raw')
VIS, URDF_DIR, DOCS = os.path.join(ROOT, 'meshes', 'visual'), os.path.join(ROOT, 'urdf'), os.path.join(ROOT, 'docs')

# Materials and weighed masses live in tools/materials.py (edit there, then re-run this script).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from materials import mass_kg, source_of  # noqa: E402


R_F2R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)   # Fusion (Y-up) -> ROS (Z-up, x forward)
E3 = np.eye(3)


def read_stl(fn):
    with open(fn, 'rb') as f:
        f.read(80)
        n = int(np.frombuffer(f.read(4), dtype='<u4')[0])
        dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        return np.frombuffer(f.read(n * 50), dtype=dt)['v'].astype(np.float64)


def write_stl(fn, tris):
    tris = tris.astype('<f4')
    nrm = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    l = np.linalg.norm(nrm, axis=1, keepdims=True)
    nrm = np.divide(nrm, l, out=np.zeros_like(nrm), where=l > 0)
    dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    arr = np.zeros(len(tris), dtype=dt)
    arr['n'], arr['v'] = nrm, tris
    with open(fn, 'wb') as f:
        f.write(b'more_tea_base_description binary stl (metres)'.ljust(80, b' '))
        f.write(struct.pack('<I', len(tris)))
        f.write(arr.tobytes())


def rpy_from_R(R):
    """Rotation matrix -> URDF roll/pitch/yaw (R = Rz(yaw) Ry(pitch) Rx(roll)); verified by reconstruction."""
    s = -R[2, 0]
    pitch = math.asin(max(-1.0, min(1.0, s)))
    if abs(s) < 1 - 1e-9:
        roll, yaw = math.atan2(R[2, 1], R[2, 2]), math.atan2(R[1, 0], R[0, 0])
    else:
        roll, yaw = math.atan2(s * R[0, 1], R[1, 1]), 0.0
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]); Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    assert np.allclose(Rz @ Ry @ Rx, R, atol=1e-7), 'rpy reconstruction failed'
    return roll, pitch, yaw


def f6(x):
    s = f'{float(x):.6f}'
    return '0.000000' if s == '-0.000000' else s


def vec(v): return ' '.join(f6(x) for x in v)
def slug(s): return re.sub(r'[^a-z0-9]+', '_', s.lower()).strip('_')
def mesh_group_name(comp): return slug(re.sub(r'\s*\(\d+\)', '', re.sub(r'\(Mirror\)', '', comp)))


dump = json.load(open(os.path.join(DATA, 'fusion_dump.json')))
xf = json.load(open(os.path.join(DATA, 'occurrence_transforms.json')))['occurrences']
P = {p['name']: p for p in dump['parts']}
by_path = {p['fusion_path']: p for p in dump['parts']}
assert len(dump['rigid_groups']) == 1, 'expected exactly one rigid group'
RG = dump['rigid_groups'][0]
rigid = [by_path[x]['name'] for x in RG['members'] if by_path[x]['stl']]
rigid_empty = [by_path[x]['name'] for x in RG['members'] if not by_path[x]['stl']]
REV = [dict(name=j['name'], wheel=by_path[j['occurrenceOne_child']]['name'], motor=by_path[j['occurrenceTwo_parent']]['name'])
       for j in dump['revolute_joints']]
wheels = [r['wheel'] for r in REV]
assert len(wheels) == 4 and not (set(wheels) & set(rigid)), 'unexpected joint/rigid-group layout'
for r in REV:
    assert r['motor'] in rigid, f"{r['name']}: parent {r['motor']} is not in the rigid group"

# base_link origin: centre of the four wheel axles (Fusion cm)
O_B = np.mean([P[w]['com_cm'] for w in wheels], axis=0)


def to_base(pF_cm):
    """Fusion world point (cm, Y-up) -> base_link frame (m, ROS axes)."""
    return R_F2R @ (np.asarray(pF_cm, float) - O_B) * 0.01


def inertial(p):
    """(mass kg, 3x3 inertia about the part COM in ROS axes, kg m^2).  Fusion reports the tensor about the WORLD ORIGIN in kg cm^2."""
    m0 = p['mass_kg']; m = mass_kg(p); k = m / m0
    c = np.array(p['com_cm']); I = p['I_about_world_origin_kg_cm2']
    IO = np.array([[I['ixx'], I['ixy'], I['ixz']], [I['ixy'], I['iyy'], I['iyz']], [I['ixz'], I['iyz'], I['izz']]])
    Ic = IO - m0 * ((c @ c) * E3 - np.outer(c, c))               # parallel-axis theorem back to the COM
    return m, R_F2R @ Ic @ R_F2R.T * 1e-4 * k


# aggregate rigid-body inertial for base_link (whole Fusion rigid group)
ms, cs, Is = [], [], []
for n in rigid:
    m, I = inertial(P[n]); ms.append(m); cs.append(to_base(P[n]['com_cm'])); Is.append(I)
M = sum(ms); C = sum(m * c for m, c in zip(ms, cs)) / M
Ibase = sum(I + m * ((d @ d) * E3 - np.outer(d, d)) for m, c, I in zip(ms, cs, Is) for d in [c - C])

# collision box over the chassis (frame + panels + lidar deck)
FRAME = [n for n in rigid if n.startswith(('rail_', 'post_', 'lr_side', 'fb_side', 'bottom_panel', 'middle_panel', 'lidar_holder'))]
bmin = np.min([P[n]['bbmin_cm'] for n in FRAME], axis=0); bmax = np.max([P[n]['bbmax_cm'] for n in FRAME], axis=0)
box_c = to_base((bmin + bmax) / 2); box_s = np.abs(R_F2R @ (bmax - bmin)) * 0.01

# wheel radius / width from the exported mesh (axis = the shortest extent)
wt = read_stl(os.path.join(RAW, wheels[0] + '.stl')).reshape(-1, 3) * 0.001
ext = wt.max(0) - wt.min(0)
W_R, W_W = round(0.5 * float(np.sort(ext)[-1]), 4), round(float(ext.min()), 4)

# meshes -> metres (kept in component-local frame), de-duplicated by content
os.makedirs(VIS, exist_ok=True)
mesh_of, groups = {}, {}
for n in rigid + wheels:
    t = read_stl(os.path.join(RAW, n + '.stl')) * 0.001
    groups.setdefault(hashlib.sha1(t.astype('<f4').tobytes()).hexdigest(), []).append((n, t))
used_names = {}
for h, lst in sorted(groups.items(), key=lambda kv: sorted(x[0] for x in kv[1])[0]):      # deterministic order
    lst.sort(key=lambda x: x[0])
    base = lst[0][0] if len(lst) == 1 else 'shared_' + mesh_group_name(P[lst[0][0]]['component'])
    k = used_names.get(base, 0); used_names[base] = k + 1
    name = base if k == 0 else f'{base}_{k + 1}'          # mirrored variants share a family name but not geometry
    write_stl(os.path.join(VIS, name + '.stl'), lst[0][1])
    for n, _ in lst:
        mesh_of[n] = name


def visual_pose(p, link_origin):
    """Pose of the component-local mesh frame expressed in the (ROS-aligned) link frame."""
    Mx = np.array(xf[p['fusion_path']]['matrix_row_major']).reshape(4, 4)
    assert abs(np.linalg.det(Mx[:3, :3]) - 1) < 1e-6, 'improper rotation in ' + p['name']
    Rt = R_F2R @ Mx[:3, :3]
    return to_base(Mx[:3, 3]) - link_origin, rpy_from_R(Rt), Rt


def material(n):
    if n.startswith(('rail_', 'post_')): return 'aluminium'
    if n.endswith('wheel'): return 'rubber'
    if n.endswith('motor'): return 'motor'
    if n == 'rplidar': return 'lidar'
    return 'panel'


def inertial_xml(m, I, ind='    '):
    return (f'{ind}<inertial>\n{ind}  <origin xyz="0 0 0" rpy="0 0 0"/>\n{ind}  <mass value="{m:.6f}"/>\n'
            f'{ind}  <inertia ixx="{I[0,0]:.6e}" ixy="{I[0,1]:.6e}" ixz="{I[0,2]:.6e}" iyy="{I[1,1]:.6e}" iyz="{I[1,2]:.6e}" izz="{I[2,2]:.6e}"/>\n{ind}</inertial>\n')


def visual_xml(n, link_origin, ind='    '):
    xyz, rpy, _ = visual_pose(P[n], link_origin)
    return (f'{ind}<visual>\n{ind}  <origin xyz="{vec(xyz)}" rpy="{vec(rpy)}"/>\n{ind}  <geometry><mesh filename="package://{PKG}/meshes/visual/{mesh_of[n]}.stl"/></geometry>\n'
            f'{ind}  <material name="{material(n)}"/>\n{ind}</visual>\n')


L = []
a = L.append
a('<?xml version="1.0"?>\n')
a('<!-- GENERATED by tools/generate_urdf.py from Fusion 360 "MoreTea\'s Body v5" (MORE-TEA / Navigation). Do not hand-edit; edit the generator or the Fusion model.\n'
  '     Frames (REP-103): x forward = Fusion +X, y left = Fusion -Z, z up = Fusion +Y.  Units: metres, kg, radians.\n'
  '     Fusion "Rigid Group 2" members -> fixed joints on base_link.  Fusion "Revolute 3..6" (wheel <-> motor mount) -> continuous joints. -->\n')
a('<robot name="more_tea">\n')
for nm, rgba in [('aluminium', '0.78 0.79 0.81 1'), ('panel', '0.82 0.83 0.88 1'), ('rubber', '0.12 0.12 0.12 1'), ('motor', '0.35 0.35 0.40 1'), ('lidar', '0.05 0.05 0.05 1')]:
    a(f'  <material name="{nm}"><color rgba="{rgba}"/></material>\n')
a('\n  <!-- ground projection (REP-105) -->\n  <link name="base_footprint"/>\n')
a(f'  <joint name="base_footprint_joint" type="fixed">\n    <parent link="base_footprint"/>\n    <child link="base_link"/>\n    <origin xyz="0 0 {W_R:.4f}" rpy="0 0 0"/>\n  </joint>\n\n')
a('  <!-- chassis: the whole Fusion rigid group as ONE rigid body (aggregate inertial + box collision); the part links below are visual-only frames -->\n')
a('  <link name="base_link">\n')
a(f'    <inertial>\n      <origin xyz="{vec(C)}" rpy="0 0 0"/>\n      <mass value="{M:.6f}"/>\n'
  f'      <inertia ixx="{Ibase[0,0]:.6e}" ixy="{Ibase[0,1]:.6e}" ixz="{Ibase[0,2]:.6e}" iyy="{Ibase[1,1]:.6e}" iyz="{Ibase[1,2]:.6e}" izz="{Ibase[2,2]:.6e}"/>\n    </inertial>\n')
a(f'    <collision>\n      <origin xyz="{vec(box_c)}" rpy="0 0 0"/>\n      <geometry><box size="{vec(box_s)}"/></geometry>\n    </collision>\n  </link>\n\n')

rows = []
a('  <!-- ===== Fusion Rigid Group members : fixed joints ===== -->\n')
for n in sorted(rigid):
    o = to_base(P[n]['com_cm'])
    a(f'  <link name="{n}_link">\n{visual_xml(n, o)}  </link>\n')
    a(f'  <joint name="{n}_joint" type="fixed">\n    <parent link="base_link"/>\n    <child link="{n}_link"/>\n    <origin xyz="{vec(o)}" rpy="0 0 0"/>\n  </joint>\n')
    rows.append(dict(part=n, fusion=P[n]['fusion_path'], relation=RG['name'], jtype='fixed', parent='base_link', xyz=o, rpy=(0, 0, 0), axis='', base_xyz=o, fusion_cm=P[n]['com_cm'], mass=inertial(P[n])[0]))
a('\n  <!-- ===== Fusion Revolute joints -> continuous : wheel (child) on motor mount (parent). Axis = lateral (ROS y); +rotation drives the robot forward ===== -->\n')
for r in sorted(REV, key=lambda r: r['wheel']):
    w, mo = r['wheel'], r['motor']
    ow, om = to_base(P[w]['com_cm']), to_base(P[mo]['com_cm'])
    m, I = inertial(P[w])
    a(f'  <link name="{w}_link">\n{inertial_xml(m, I)}{visual_xml(w, ow)}'
      f'    <collision>\n      <origin xyz="0 0 0" rpy="{math.pi/2:.7f} 0 0"/>\n      <geometry><cylinder radius="{W_R:.4f}" length="{W_W:.4f}"/></geometry>\n    </collision>\n  </link>\n')
    a(f'  <joint name="{w}_joint" type="continuous">\n    <parent link="{mo}_link"/>\n    <child link="{w}_link"/>\n    <origin xyz="{vec(ow - om)}" rpy="0 0 0"/>\n    <axis xyz="0 1 0"/>\n    <dynamics damping="0.0" friction="0.0"/>\n  </joint>\n')
    rows.append(dict(part=w, fusion=P[w]['fusion_path'], relation=r['name'], jtype='continuous', parent=mo + '_link', xyz=ow - om, rpy=(0, 0, 0), axis='0 1 0', base_xyz=ow, fusion_cm=P[w]['com_cm'], mass=m))
    _, _, Rt = visual_pose(P[w], ow)
    assert abs(abs((Rt @ np.array([0, 1, 0]))[1]) - 1) < 1e-6, 'wheel mesh axis is not lateral for ' + w

# lidar scan frame: on the rplidar axis, mid-height of the unit (approximate scan plane)
lb0, lb1 = np.array(P['rplidar']['bbmin_cm']), np.array(P['rplidar']['bbmax_cm'])
laser_base = to_base((lb0 + lb1) / 2)
a('\n  <!-- 2D LiDAR scan frame (x = RPLIDAR zero-angle direction, assumed = robot forward). Scan-plane height is approximate (mid-height of the unit). -->\n')
a(f'  <link name="laser"/>\n  <joint name="laser_joint" type="fixed">\n    <parent link="rplidar_link"/>\n    <child link="laser"/>\n    <origin xyz="{vec(laser_base - to_base(P["rplidar"]["com_cm"]))}" rpy="0 0 0"/>\n  </joint>\n')
a('</robot>\n')

for d in (URDF_DIR, DOCS):
    os.makedirs(d, exist_ok=True)
open(os.path.join(URDF_DIR, 'more_tea.urdf'), 'w').write(''.join(L))

# joint table ------------------------------------------------------------------------------------
rows_all = [dict(part='base_footprint -> base_link', fusion='(virtual)', relation='REP-105 ground frame', jtype='fixed', parent='base_footprint', xyz=np.array([0, 0, W_R]), rpy=(0, 0, 0), axis='', base_xyz=np.array([0, 0, 0]), fusion_cm=O_B.tolist(), mass=0.0)]
rows_all += sorted([r for r in rows if r['jtype'] == 'fixed'], key=lambda r: r['part']) + sorted([r for r in rows if r['jtype'] != 'fixed'], key=lambda r: r['part'])
rows_all.append(dict(part='laser', fusion='(virtual, on RP LIDAR)', relation='sensor frame', jtype='fixed', parent='rplidar_link', xyz=laser_base - to_base(P['rplidar']['com_cm']), rpy=(0, 0, 0), axis='', base_xyz=laser_base, fusion_cm=((lb0 + lb1) / 2).tolist(), mass=0.0))


def link_name(r): return r['part'] if (' ' in r['part'] or r['part'] == 'laser') else r['part'] + '_link'


with open(os.path.join(DATA, 'joint_table.csv'), 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['child_link', 'fusion_component', 'fusion_relationship', 'urdf_joint_type', 'parent_link', 'joint_xyz_in_parent_m', 'joint_rpy', 'axis', 'position_in_base_link_m', 'fusion_world_cm_YUp', 'mass_kg'])
    for r in rows_all:
        w.writerow([link_name(r), r['fusion'], r['relation'], r['jtype'], r['parent'], vec(r['xyz']), vec(r['rpy']), r['axis'], vec(r['base_xyz']), ' '.join(f'{x:.3f}' for x in r['fusion_cm']), f"{r['mass']:.4f}"])

md = ['# More Tea body - Fusion relationship -> URDF joint table\n',
      f'Source: Fusion 360 **{dump["document"]}** (MORE-TEA / Navigation). Generated by `tools/generate_urdf.py`; do not hand-edit.\n',
      f'Frames: REP-103 - x forward (Fusion +X), y left (Fusion -Z), z up (Fusion +Y). `base_link` = centre of the four wheel axles; the ground is `{W_R} m` below it.\n',
      f'Fusion origin of `base_link` (cm, Y-up): `{" ".join(f"{x:.5f}" for x in O_B)}`\n',
      '## Revolute joints (Fusion) -> `continuous`\n',
      '| Fusion joint | child (wheel) | parent (motor mount) | URDF joint | xyz in parent (m) | axis | wheel centre in base_link (m) |', '|---|---|---|---|---|---|---|']
for r in sorted(REV, key=lambda r: r['name']):
    rr = next(x for x in rows if x['part'] == r['wheel'])
    md.append(f"| {r['name']} | {r['wheel']}_link | {r['motor']}_link | {r['wheel']}_joint | {vec(rr['xyz'])} | 0 1 0 | {vec(rr['base_xyz'])} |")
md += ['', f'## {RG["name"]} (Fusion rigid group, {len(rigid)} members with geometry) -> `fixed` joints on `base_link`\n',
       '| child link | Fusion component | URDF joint | xyz in base_link (m) | Fusion world COM (cm, Y-up) | material | mass as modelled (kg) |', '|---|---|---|---|---|---|---|']
for r in rows_all:
    if r['relation'] == RG['name']:
        md.append(f"| {r['part']}_link | {r['fusion'].replace(':1', '')} | {r['part']}_joint | {vec(r['xyz'])} | {' '.join(f'{x:.3f}' for x in r['fusion_cm'])} | {source_of(P[r['part']])} | {r['mass']:.3f} |")
md += ['', '## Virtual frames\n', '| link | parent | xyz in parent (m) |', '|---|---|---|', f'| base_link | base_footprint | 0 0 {W_R:.4f} |', f"| laser | rplidar_link | {vec(rows_all[-1]['xyz'])} |"]
if rigid_empty:
    md += ['', '## Skipped\n', f'- `{", ".join(rigid_empty)}` - member of the rigid group but an **empty component** (no bodies). Not exported; safe to delete in Fusion.']
open(os.path.join(DOCS, 'JOINT_TABLE.md'), 'w').write('\n'.join(md) + '\n')

shared = {}
for n, m_ in mesh_of.items():
    shared.setdefault(m_, []).append(n)
print(f'links: {len(rigid) + len(wheels) + 3}  fixed parts: {len(rigid)}  continuous: {len(wheels)}  mesh files: {len(groups)}')
print(f'wheel radius {W_R} m  width {W_W} m   base_link origin in Fusion (cm) = {np.round(O_B, 5)}')
print(f'base_link mass {M:.3f} kg (as modelled)   total incl. wheels {M + sum(inertial(P[w])[0] for w in wheels):.3f} kg')
print('base_link COM (m, base frame):', np.round(C, 4), ' inertia eigenvalues:', np.round(np.linalg.eigvalsh(Ibase), 5))
print('collision box centre', np.round(box_c, 4), 'size', np.round(box_s, 4))
print('shared meshes:', {k: len(v) for k, v in shared.items() if len(v) > 1})
print('wrote urdf/more_tea.urdf, docs/JOINT_TABLE.md, data/joint_table.csv')
