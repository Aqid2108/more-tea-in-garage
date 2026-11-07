"""Real materials for the More Tea body.  Shared by generate_urdf.py and validate_urdf.py.

Fusion has every body on its default "Steel" (7.85 g/cm^3).  Here each part gets its real
material; mass = Fusion volume x density (x print fill for printed parts).  Because each part's
density is uniform, its COM is unchanged and its inertia scales by the same factor.

Priority: MASS_OVERRIDES_KG (a weighed part) > material below > Fusion's Steel value.
"""

DENSITY_G_CM3 = {
    'aluminium_6063': 2.70,   # 2020 aluminium extrusion
    'petg':           1.27,   # solid PETG
    'steel':          7.85,   # Fusion default; still used for parts not yet assigned
}

# Fraction of solid that a printed part really is.  The panels are 3 mm plates, which print as
# nearly all walls/top/bottom layers, so 1.0 is close.  Better: put slicer or scale weights in
# MASS_OVERRIDES_KG.
PRINT_FILL = 1.0


def material_of(name):
    if name.startswith(('rail_', 'post_')):
        return 'aluminium_6063'
    if name.endswith('_panel') or name.startswith(('lr_side_panel', 'fb_side_panel')) or name == 'lidar_holder':
        return 'petg'
    return 'steel'          # motors, wheels, rplidar: weigh these and put them below


# Weighed / datasheet masses (kg) per URDF part name.  These win over the material density.
MASS_OVERRIDES_KG = {
    # JGB37-520, 12 V, 178 rpm no-load (1:56 on the 10000 rpm motor): 158 g supplier figure, motor only
    'front_left_motor': 0.158, 'front_right_motor': 0.158,
    'rear_left_motor': 0.158, 'rear_right_motor': 0.158,
    # 'rplidar': 0.17,
    # 'front_left_wheel': 0.15,
}


def mass_kg(p):
    """Modelled mass for a fusion_dump part dict."""
    n = p['name']
    if n in MASS_OVERRIDES_KG:
        return MASS_OVERRIDES_KG[n]
    mat = material_of(n)
    fill = PRINT_FILL if mat == 'petg' else 1.0
    return p['volume_cm3'] * DENSITY_G_CM3[mat] * fill / 1000.0


def source_of(p):
    n = p['name']
    if n in MASS_OVERRIDES_KG:
        return 'weighed'
    return material_of(n)
