#!/usr/bin/env python3
"""Validate urdf/more_tea.urdf against the Fusion data it was generated from.

Checks: unique names, single-root tree, mesh files, inertia sanity, Fusion relationship mapping
(rigid group -> fixed, revolute -> continuous), and a round trip: every mesh is placed with forward
kinematics at zero joint angles and its bounding box is compared to Fusion's own bounding box.
Also re-derives the whole-robot mass / COM / inertia straight from Fusion's numbers and compares.

Usage:  python tools/validate_urdf.py            (exit code 1 if anything fails)
"""
import json, math, os, sys
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from materials import mass_kg  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
URDF = os.path.join(ROOT, 'urdf', 'more_tea.urdf')
R_F2R = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float)
E3 = np.eye(3)
TOL_CM = 0.01          # 0.1 mm bounding-box tolerance
fails = []


def check(ok, msg):
    print(('PASS  ' if ok else 'FAIL  ') + msg)
    if not ok:
        fails.append(msg)


def read_stl(fn):
    with open(fn, 'rb') as f:
        f.read(80)
        n = int(np.frombuffer(f.read(4), dtype='<u4')[0])
        dt = np.dtype([('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
        return np.frombuffer(f.read(n * 50), dtype=dt)['v'].reshape(-1, 3).astype(np.float64)


def floats(s, n=3, default=0.0):
    return np.array([float(x) for x in s.split()]) if s else np.full(n, default)


def rot_rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]) @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def T_origin(el):
    T = np.eye(4)
    if el is not None:
        T[:3, 3] = floats(el.get('xyz'))
        T[:3, :3] = rot_rpy(*floats(el.get('rpy')))
    return T


tree = ET.parse(URDF).getroot()
links_l, joints_l = tree.findall('link'), tree.findall('joint')
links = {l.get('name'): l for l in links_l}
joints = {j.get('name'): j for j in joints_l}
check(len(links) == len(links_l) and len(joints) == len(joints_l), f'unique names ({len(links)} links, {len(joints)} joints)')

child_of, kids = {}, {}
for j in joints_l:
    p, c = j.find('parent').get('link'), j.find('child').get('link')
    check(p in links and c in links, f'joint {j.get("name")}: parent/child exist') if (p not in links or c not in links) else None
    child_of[c] = (p, j)
    kids.setdefault(p, []).append((c, j))
roots = [n for n in links if n not in child_of]
check(len(roots) == 1, f'single root link: {roots}')
T, seen, stack = {roots[0]: np.eye(4)}, set(), [roots[0]]
while stack:
    n = stack.pop(); seen.add(n)
    for c, j in kids.get(n, []):
        T[c] = T[n] @ T_origin(j.find('origin')); stack.append(c)
check(len(seen) == len(links), f'tree is connected and acyclic ({len(seen)}/{len(links)} links reachable)')
check(all(j.get('type') in ('fixed', 'continuous', 'revolute', 'prismatic') for j in joints_l), 'joint types are valid')
for j in joints_l:
    if j.get('type') == 'continuous':
        ax = floats(j.find('axis').get('xyz'))
        check(abs(np.linalg.norm(ax) - 1) < 1e-6, f'{j.get("name")}: axis is a unit vector {ax}')

# meshes ----------------------------------------------------------------------------------------
meshes = {}
for l in links.values():
    for tag in ('visual', 'collision'):
        for v in l.findall(tag):
            m = v.find('geometry/mesh')
            if m is not None:
                fn = m.get('filename'); pre = 'package://more_tea_base_description/'
                path = os.path.join(ROOT, fn[len(pre):]) if fn.startswith(pre) else fn
                check(os.path.isfile(path), f'mesh exists: {os.path.basename(path)} (for {l.get("name")})') if not os.path.isfile(path) else None
                meshes[(l.get('name'), tag)] = path
check(all(os.path.isfile(p) for p in meshes.values()), f'all {len(set(meshes.values()))} mesh files resolve')

# inertials -------------------------------------------------------------------------------------
inert = {}
for l in links.values():
    i = l.find('inertial')
    if i is None:
        continue
    m = float(i.find('mass').get('value')); a = i.find('inertia')
    I = np.array([[float(a.get('ixx')), float(a.get('ixy')), float(a.get('ixz'))], [float(a.get('ixy')), float(a.get('iyy')), float(a.get('iyz'))], [float(a.get('ixz')), float(a.get('iyz')), float(a.get('izz'))]])
    ev = np.linalg.eigvalsh(I)
    ok = m > 0 and ev.min() > 0 and ev[0] + ev[1] >= ev[2] * (1 - 1e-9)
    check(ok, f'{l.get("name")}: mass {m:.3f} kg, inertia positive-definite + triangle inequality (eig {np.round(ev, 5)})')
    inert[l.get('name')] = (m, I, T_origin(i.find('origin')))

# Fusion data -----------------------------------------------------------------------------------
dump = json.load(open(os.path.join(ROOT, 'data', 'fusion_dump.json')))
by_path = {p['fusion_path']: p for p in dump['parts']}
P = {p['name']: p for p in dump['parts']}
rigid = [by_path[x]['name'] for x in dump['rigid_groups'][0]['members'] if by_path[x]['stl']]
rev = [(j['name'], by_path[j['occurrenceOne_child']]['name'], by_path[j['occurrenceTwo_parent']]['name']) for j in dump['revolute_joints']]
wheels = [w for _, w, _ in rev]
O_B = np.mean([P[w]['com_cm'] for w in wheels], axis=0)

for n in rigid:
    j = joints.get(n + '_joint')
    check(j is not None and j.get('type') == 'fixed' and j.find('parent').get('link') == 'base_link' and j.find('child').get('link') == n + '_link',
          f'rigid group member {n}: fixed joint on base_link') if not (j is not None and j.get('type') == 'fixed') else None
check(all(joints.get(n + '_joint') is not None and joints[n + '_joint'].get('type') == 'fixed' for n in rigid), f'{len(rigid)} rigid-group members -> fixed joints on base_link')
for fname, w, mo in rev:
    j = joints.get(w + '_joint')
    check(j is not None and j.get('type') == 'continuous' and j.find('parent').get('link') == mo + '_link' and j.find('child').get('link') == w + '_link',
          f'Fusion "{fname}": {w}_link spins on {mo}_link (continuous)')

# round trip: mesh bbox vs Fusion bbox -----------------------------------------------------------
TB = np.linalg.inv(T['base_link'])
worst = 0.0
for n in rigid + wheels:
    l = links[n + '_link']; v = l.find('visual')
    verts = read_stl(meshes[(n + '_link', 'visual')])
    Tv = TB @ T[n + '_link'] @ T_origin(v.find('origin'))
    pb = (Tv[:3, :3] @ verts.T).T + Tv[:3, 3]                  # base frame, metres
    pF = (R_F2R.T @ pb.T).T * 100.0 + O_B                       # Fusion world, cm
    d = max(np.abs(pF.min(0) - np.array(P[n]['bbmin_cm'])).max(), np.abs(pF.max(0) - np.array(P[n]['bbmax_cm'])).max())
    worst = max(worst, d)
    check(d <= TOL_CM, f'{n}: placed mesh bbox matches Fusion (max deviation {d * 10:.3f} mm)') if d > TOL_CM else None
check(worst <= TOL_CM, f'round trip: all {len(rigid) + len(wheels)} meshes land on Fusion geometry (worst deviation {worst * 10:.3f} mm)')

# whole-robot mass / COM / inertia, URDF vs Fusion ------------------------------------------------
names = rigid + wheels
# expected values = Fusion's tensors rescaled per part to the modelled (material / weighed) mass;
# a uniform-density rescale keeps each part's COM and scales its inertia by the same factor
kf = {n: mass_kg(P[n]) / P[n]['mass_kg'] for n in names}
Mf = sum(P[n]['mass_kg'] * kf[n] for n in names)
Cf = sum(P[n]['mass_kg'] * kf[n] * np.array(P[n]['com_cm']) for n in names) / Mf
IOf = sum(kf[n] * np.array([[P[n]['I_about_world_origin_kg_cm2'][k] for k in r] for r in (('ixx', 'ixy', 'ixz'), ('ixy', 'iyy', 'iyz'), ('ixz', 'iyz', 'izz'))]) for n in names)
Itf = R_F2R @ (IOf - Mf * ((Cf @ Cf) * E3 - np.outer(Cf, Cf))) @ R_F2R.T * 1e-4
Cf_b = R_F2R @ (Cf - O_B) * 0.01
Mu = sum(m for m, _, _ in inert.values())
pos = {n: (TB @ T[n] @ iv[2])[:3, 3] for n, iv in inert.items()}
Cu = sum(inert[n][0] * pos[n] for n in inert) / Mu
Iu = sum((TB @ T[n])[:3, :3] @ inert[n][2][:3, :3] @ inert[n][1] @ inert[n][2][:3, :3].T @ (TB @ T[n])[:3, :3].T + inert[n][0] * (((pos[n] - Cu) @ (pos[n] - Cu)) * E3 - np.outer(pos[n] - Cu, pos[n] - Cu)) for n in inert)
check(abs(Mu - Mf) < 1e-4, f'total mass {Mu:.3f} kg == Fusion volumes x materials {Mf:.3f} kg')
check(np.abs(Cu - Cf_b).max() < 2e-5, f'whole-robot COM {np.round(Cu, 4)} m == Fusion-derived {np.round(Cf_b, 4)} m')
check(np.abs(Iu - Itf).max() < 1e-3 * np.abs(Itf).max() + 1e-7, f'whole-robot inertia about COM agrees with Fusion tensors (max diff {np.abs(Iu - Itf).max():.2e} kg m^2)')
print(f'INFO  total mass {Mu:.3f} kg, COM height above ground {Cu[2] + float(joints["base_footprint_joint"].find("origin").get("xyz").split()[2]):.3f} m')

print()
print('ALL CHECKS PASSED' if not fails else f'{len(fails)} CHECK(S) FAILED')
sys.exit(1 if fails else 0)
