"""Fusion 360 script - run after 1_export_meshes_and_dump.py.
Writes data/occurrence_transforms.json: each occurrence's component-local(cm) -> world(cm) matrix, which the
generator needs because Fusion exports every STL in its component-local frame."""
import adsk.core, adsk.fusion, json, os, traceback

BASE = os.path.join(os.path.expanduser('~'), 'more-tea-in-garage', 'description', 'more_tea_base_description', 'data')     # <- keep in sync with script 1


def run(context):
    ui = adsk.core.Application.get().userInterface
    try:
        design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
        res = {'note': 'row-major 4x4, component-local(cm) -> world(cm), Fusion Y-up', 'occurrences': {}}
        for o in design.rootComponent.allOccurrences:
            m = o.transform2.asArray()
            res['occurrences'][o.fullPathName] = {'matrix_row_major': [m[i] for i in range(16)],
                                                  'component': o.component.name, 'isReferenced': o.isReferencedComponent}
        os.makedirs(BASE, exist_ok=True)
        with open(os.path.join(BASE, 'occurrence_transforms.json'), 'w') as f:
            json.dump(res, f, indent=1)
        ui.messageBox(f'Wrote {len(res["occurrences"])} transforms to {BASE}')
    except Exception:
        ui.messageBox('Failed:\n' + traceback.format_exc())
