"""Fusion 360 script  (Utilities > Add-Ins > Scripts and Add-Ins > Create > Python, paste, Run).

Exports, for the active design ("MoreTea's Body"):
  <PKG>/meshes/raw/<name>.stl         one STL per component (Fusion exports MILLIMETRES, component-local coordinates)
  <PKG>/data/fusion_dump.json         masses, COMs, inertia, bounding boxes, rigid groups and revolute joints
Then run 2_dump_transforms.py, then   python tools/generate_urdf.py && python tools/validate_urdf.py

NOTE: the design is non-parametric (direct modelling), so Fusion's API cannot return joint origins/axes/limits -
the generator derives wheel axes from the wheel geometry. Convert to a parametric design if you ever need them.
Edit NAMES below if components are added/renamed.
"""
import adsk.core, adsk.fusion, json, os, traceback, datetime

BASE = os.path.join(os.path.expanduser('~'), 'more-tea-in-garage', 'description', 'more_tea_base_description')     # <- change to where you cloned the repo

NAMES = {   # Fusion occurrence path -> URDF part name (front = Fusion +X, left = Fusion -Z)
 '2020 300mm v1 (7):1': 'rail_bottom_front',
 '2020 300mm v1:1': 'rail_bottom_rear',
 '2020 300mm v1 (9):1': 'rail_bottom_left_outer',
 '2020 300mm v1 (8):1': 'rail_bottom_right_outer',
 '2020 300mm v1 (6):1': 'rail_bottom_left_inner',
 '2020 300mm v1 (3):1': 'rail_bottom_right_inner',
 '2020 300mm v1 (1):1': 'rail_top_left',
 '2020 300mm v1 (4):1': 'rail_top_right',
 '2020 300mm v1 (5):1': 'rail_top_rear',
 '2020 300mm v1 (2):1': 'rail_top_front',
 '2020 200mm Base v2 (1):1': 'post_front_right',
 '2020 200mm Base v2 (2):1': 'post_front_left',
 '2020 200mm Base v2 (3):1': 'post_rear_left',
 '2020 200mm Base v2:1': 'post_rear_right',
 'Bottom Panel v4:1': 'bottom_panel',
 'Middle Panel v5:1': 'middle_panel',
 'LR Side Panel v5:1': 'lr_side_panel_right',
 'LR Side Panel v5 (1):1': 'lr_side_panel_left',
 'FB Side Panel v3:1': 'fb_side_panel',
 'Lidar Holder (Assembly) v4:1': 'lidar_holder_assembly_EMPTY',
 'Lidar Holder v5:1': 'lidar_holder',
 'RP LIDAR v1 v3:1': 'rplidar',
 'MOTEUR (1) v8 v2:1': 'front_right_motor',
 'MOTEUR (1) v8 v2(Mirror):1': 'front_left_motor',
 'MOTEUR (1) v8 v2(Mirror) (1):1': 'rear_right_motor',
 'MOTEUR (1) v8 v2(Mirror)(Mirror):1': 'rear_left_motor',
 'Mecanum Wheels v2:1': 'front_right_wheel',
 'Mecanum Wheels v2(Mirror):1': 'front_left_wheel',
 'Mecanum Wheels v2(Mirror) (1):1': 'rear_right_wheel',
 'Mecanum Wheels v2(Mirror)(Mirror):1': 'rear_left_wheel',
}


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        app = adsk.core.Application.get()
        design = adsk.fusion.Design.cast(app.activeProduct)
        root = design.rootComponent
        rawdir, datadir = os.path.join(BASE, 'meshes', 'raw'), os.path.join(BASE, 'data')
        os.makedirs(rawdir, exist_ok=True); os.makedirs(datadir, exist_ok=True)
        em = design.exportManager
        dump = {'document': app.activeDocument.name, 'exported_at': datetime.datetime.now().isoformat(),
                'fusion_internal_length_unit': 'cm', 'design_default_length_unit': design.unitsManager.defaultLengthUnits,
                'up_axis': 'Y', 'parts': [], 'unmapped': []}
        failed = []
        for o in root.allOccurrences:
            p = o.fullPathName
            if p not in NAMES:
                dump['unmapped'].append(p); continue
            name = NAMES[p]; bb = o.boundingBox
            rec = {'fusion_path': p, 'component': o.component.name, 'name': name,
                   'bbmin_cm': [bb.minPoint.x, bb.minPoint.y, bb.minPoint.z], 'bbmax_cm': [bb.maxPoint.x, bb.maxPoint.y, bb.maxPoint.z],
                   'materials': [b.material.name for b in o.component.bRepBodies], 'n_bodies': o.component.bRepBodies.count}
            pp = o.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
            c = pp.centerOfMass
            ok, ixx, iyy, izz, ixy, iyz, ixz = pp.getXYZMomentsOfInertia()      # about the WORLD ORIGIN, kg cm^2
            rec.update({'mass_kg': pp.mass, 'volume_cm3': pp.volume, 'com_cm': [c.x, c.y, c.z],
                        'I_about_world_origin_kg_cm2': {'ixx': ixx, 'iyy': iyy, 'izz': izz, 'ixy': ixy, 'iyz': iyz, 'ixz': ixz}})
            if o.component.bRepBodies.count == 0:
                rec['stl'] = None
            else:
                opts = em.createSTLExportOptions(o, os.path.join(rawdir, name + '.stl'))
                opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementMedium
                opts.isBinaryFormat = True
                if em.execute(opts):
                    rec['stl'] = 'meshes/raw/' + name + '.stl'
                else:
                    rec['stl'] = None; failed.append(name)
            dump['parts'].append(rec)
        dump['rigid_groups'] = [{'name': r.name, 'members': [x.fullPathName for x in r.occurrences]} for r in root.rigidGroups]
        dump['revolute_joints'] = [{'name': j.name, 'occurrenceOne_child': j.occurrenceOne.fullPathName,
                                    'occurrenceTwo_parent': j.occurrenceTwo.fullPathName, 'jointType': int(j.jointMotion.jointType)} for j in root.joints]
        with open(os.path.join(datadir, 'fusion_dump.json'), 'w') as f:
            json.dump(dump, f, indent=1)
        msg = f'Exported {len(dump["parts"])} parts to {BASE}'
        if dump['unmapped']: msg += f'\nNOT in NAMES (ignored): {dump["unmapped"]}'
        if failed: msg += f'\nSTL export failed: {failed}'
        ui.messageBox(msg)
    except Exception:
        ui.messageBox('Failed:\n' + traceback.format_exc())
