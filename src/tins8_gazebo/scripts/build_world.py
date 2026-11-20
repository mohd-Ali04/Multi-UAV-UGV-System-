#!/usr/bin/env python3
"""Build the native Gazebo Fieldlab world (and the minimal stage-2 world).

Source of truth for the layout: data/fieldlab_scene_seed42.json, exported from the
submitted Fieldlab generator (fieldlab_core.config.scene, Mission() defaults: seed 42,
terrain proving_ground, population 18, traffic 10).  Colours and procedural
dimensions follow the browser scene code (web/World.tsx).  See docs/world_conversion.md
for the component-by-component table.

Outputs: worlds/fieldlab.sdf, worlds/minimal_px4.sdf
"""
import copy
import json
import math
import xml.etree.ElementTree as E
from pathlib import Path

import yaml

PKG = Path(__file__).resolve().parent.parent
SCENE = json.loads((PKG / 'data/fieldlab_scene_seed42.json').read_text())
CFG = yaml.safe_load((PKG / 'config/scenario.yaml').read_text())
UGV = json.loads((PKG / 'data/ugv_model_summary.json').read_text())
QAV_CG_Z = 0.025          # model origin above the QAV250 bottom datum (original generator)
DOCK_CLEARANCE = 0.002


def hexrgb(h, a=1.0):
    h = h.lstrip('#')
    return ' '.join(f'{int(h[i:i + 2], 16) / 255:.3f}' for i in (0, 2, 4)) + f' {a}'


def el(parent, tag, text=None, **attrs):
    e = E.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        e.text = str(text)
    return e


def material(parent, color, emissive=None):
    m = el(parent, 'material')
    el(m, 'ambient', color)
    el(m, 'diffuse', color)
    el(m, 'specular', '0.05 0.05 0.05 1')
    if emissive:
        el(m, 'emissive', emissive)
    return m


def box(link, name, size, pose, color, collide=True):
    v = el(link, 'visual', name=name)
    el(v, 'pose', ' '.join(f'{p:.4g}' for p in pose))
    el(el(el(v, 'geometry'), 'box'), 'size', ' '.join(f'{s:.4g}' for s in size))
    material(v, color)
    if collide:
        c = el(link, 'collision', name=name + '_col')
        el(c, 'pose', ' '.join(f'{p:.4g}' for p in pose))
        el(el(el(c, 'geometry'), 'box'), 'size', ' '.join(f'{s:.4g}' for s in size))
    return v


def plugin(parent, filename, name, **kv):
    p = el(parent, 'plugin', filename=filename, name=name)
    for k, v in kv.items():
        el(p, k, v)
    return p


def world_header(name):
    root = E.Element('sdf', version='1.9')
    w = el(root, 'world', name=name)
    ph = el(w, 'physics', name='px4_step', type='dart')
    el(ph, 'max_step_size', 0.004)
    el(ph, 'real_time_factor', 1.0)
    el(ph, 'real_time_update_rate', 250)
    for file, cls in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                      ('scene-broadcaster', 'SceneBroadcaster'), ('imu', 'Imu'),
                      ('magnetometer', 'Magnetometer'), ('navsat', 'NavSat'),
                      ('air-pressure', 'AirPressure')]:
        plugin(w, f'gz-sim-{file}-system', f'gz::sim::systems::{cls}')
    plugin(w, 'gz-sim-sensors-system', 'gz::sim::systems::Sensors', render_engine='ogre2')
    el(w, 'gravity', '0 0 -9.80665')
    el(w, 'magnetic_field', '0.0000055645 0.0000228758 -0.0000423884')
    el(w, 'atmosphere', type='adiabatic')
    sc = el(w, 'spherical_coordinates')
    origin = CFG['world_origin']
    for k, v in [('surface_model', 'EARTH_WGS84'), ('world_frame_orientation', 'ENU'),
                 ('latitude_deg', origin['lat']), ('longitude_deg', origin['lon']),
                 ('elevation', origin['alt']), ('heading_deg', 0)]:
        el(sc, k, v)
    scene = el(w, 'scene')
    el(scene, 'ambient', '0.75 0.77 0.72 1')
    el(scene, 'background', hexrgb('#dce4d9'))
    el(scene, 'shadows', 'true')
    el(scene, 'grid', 'false')
    # Directional sun from World.tsx directionalLight position [-25,-35,65] towards origin.
    sun = el(w, 'light', name='sun', type='directional')
    el(sun, 'cast_shadows', 'true')
    el(sun, 'pose', '-25 -35 65 0 0 0')
    el(sun, 'diffuse', '0.95 0.95 0.92 1')
    el(sun, 'specular', '0.2 0.2 0.2 1')
    d = [25, 35, -65]
    n = math.sqrt(sum(x * x for x in d))
    el(sun, 'direction', ' '.join(f'{x / n:.4f}' for x in d))
    return root, w


def ground(w, size, color):
    m = el(w, 'model', name='ground')
    el(m, 'static', 'true')
    link = el(m, 'link', name='surface')
    c = el(link, 'collision', name='plane')
    pl = el(el(c, 'geometry'), 'plane')
    el(pl, 'normal', '0 0 1')
    el(pl, 'size', f'{size} {size}')
    fr = el(el(el(c, 'surface'), 'friction'), 'ode')
    el(fr, 'mu', 1.0)
    el(fr, 'mu2', 1.0)
    v = el(link, 'visual', name='plane')
    pv = el(el(v, 'geometry'), 'plane')
    el(pv, 'normal', '0 0 1')
    el(pv, 'size', f'{size} {size}')
    material(v, color)


def include(w, uri, name, pose, static=True):
    inc = el(w, 'include')
    el(inc, 'uri', uri)
    el(inc, 'name', name)
    el(inc, 'pose', ' '.join(f'{p:.5g}' for p in pose))
    if static:
        el(inc, 'static', 'true')
    return inc


def walker_model(w, obj, path):
    """A person whose pose is driven by Gazebo's VelocityControl system (no gravity, no collision).

    Visual meshes are the same CC0 human_mesh_N files used for static people."""
    variant = f"human_mesh_{obj['variant']}"
    src = E.parse(PKG / 'models' / variant / 'model.sdf').getroot().find('model/link')
    m = el(w, 'model', name=obj['id'])
    el(m, 'pose', f"{obj['x']:.4f} {obj['y']:.4f} 0 0 0 {obj['yaw']:.4f}")
    link = el(m, 'link', name='body')
    el(link, 'gravity', 'false')
    ine = el(link, 'inertial')
    el(ine, 'mass', 70)
    it = el(ine, 'inertia')
    for k, v in [('ixx', 12), ('iyy', 12), ('izz', 2), ('ixy', 0), ('ixz', 0), ('iyz', 0)]:
        el(it, k, v)
    for vis in src.findall('visual'):
        vis = copy.deepcopy(vis)
        uri = vis.find('geometry/mesh/uri')
        uri.text = f'model://{variant}/{uri.text}'
        link.append(vis)
    plugin(m, 'gz-sim-velocity-control-system', 'gz::sim::systems::VelocityControl',
           topic=f"/model/{obj['id']}/cmd_vel")


def roads(w, scene):
    m = el(w, 'model', name='fieldlab_roads')
    el(m, 'static', 'true')
    link = el(m, 'link', name='surface')
    for r in scene['roads']:
        box(link, r['id'] + '_shoulder', (r['sx'] + 3.5, r['sy'] + 3.5, 0.01), (r['x'], r['y'], 0.012, 0, 0, 0), hexrgb('#bbc0b1'), False)
        box(link, r['id'], (r['sx'], r['sy'], 0.01), (r['x'], r['y'], 0.016, 0, 0, 0), hexrgb('#788480'), False)
        horizontal = r['sx'] > r['sy']
        for i in range(45):
            off = (i - 22) * 5
            if horizontal:
                box(link, f"{r['id']}_dash{i}", (2.4, 0.12, 0.01), (r['x'] + off, r['y'], 0.021, 0, 0, 0), hexrgb('#e0e2c9'), False)
            else:
                box(link, f"{r['id']}_dash{i}", (0.12, 2.4, 0.01), (r['x'], r['y'] + off, 0.021, 0, 0, 0), hexrgb('#e0e2c9'), False)


def building(w, o):
    m = el(w, 'model', name=o['id'])
    el(m, 'static', 'true')
    el(m, 'pose', f"{o['x']} {o['y']} 0 0 0 {o.get('yaw', 0)}")
    link = el(m, 'link', name='body')
    sx, sy, sz = o['sx'], o['sy'], o['sz']
    colors = ['#b9b8a5', '#a2b1a7', '#bdac98', '#c1c3b5']
    box(link, 'walls', (sx, sy, sz), (0, 0, sz / 2, 0, 0, 0), hexrgb(colors[o.get('variant', 0)]))
    box(link, 'roof_cap', (sx + 0.4, sy + 0.4, 0.3), (0, 0, sz + 0.1, 0, 0, 0), hexrgb('#929d92'))
    box(link, 'roof_unit', (sx * 0.45, sy * 0.4, 0.7), (0, 0, sz + 0.6, 0, 0, 0), hexrgb('#a3a99b'))
    for i in range(max(1, int(sx // 2.5))):
        x = -sx / 2 + 1.6 + i * 2.4
        box(link, f'win_{i}_a', (1.15, 0.04, 1.45), (x, -sy / 2 - 0.03, 2.7, 0, 0, 0), hexrgb('#70898a'), False)
        if sz > 6:
            box(link, f'win_{i}_b', (1.15, 0.04, 1.45), (x, -sy / 2 - 0.03, 5.6, 0, 0, 0), hexrgb('#70898a'), False)


VEHICLE_SIZE = {'car': (4.45, 1.85, 1.65), 'truck': (7.7, 2.55, 3.4), 'bus': (8.35, 2.95, 3.2)}


def vehicle(w, o):
    """Static parked vehicle: CC0 Kenney-derived visuals, collision box sized from city.py
    (the copied model files carry a generic 4 x 1.8 x 1.6 m box)."""
    name = f"{o['kind']}_mesh_0"
    src = E.parse(PKG / 'models' / name / 'model.sdf').getroot().find('model/link')
    m = el(w, 'model', name=o['id'])
    el(m, 'static', 'true')
    el(m, 'pose', f"{o['x']} {o['y']} 0 0 0 {o['yaw']}")
    link = el(m, 'link', name='body')
    for vis in src.findall('visual'):
        vis = copy.deepcopy(vis)
        uri = vis.find('geometry/mesh/uri')
        uri.text = f'model://{name}/{uri.text}'
        link.append(vis)
    sx, sy, sz = VEHICLE_SIZE[o['kind']]
    c = el(link, 'collision', name='body')
    el(c, 'pose', f'0 0 {sz / 2} 0 0 0')
    el(el(el(c, 'geometry'), 'box'), 'size', f'{sx} {sy} {sz}')


def markers(w, scene_defaults):
    """Route line and stop rings drawn by the browser (visual only, no collision)."""
    m = el(w, 'model', name='fieldlab_route_markers')
    el(m, 'static', 'true')
    link = el(m, 'link', name='marks')
    pts = [scene_defaults['start']] + [[s['x'], s['y']] for s in scene_defaults['stops']]
    for i, (a, b) in enumerate(zip(pts, pts[1:])):
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        box(link, f'route_{i}', (L, 0.15, 0.01), ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0.03, 0, 0, math.atan2(dy, dx)), hexrgb('#dcedad'), False)
    for s in scene_defaults['stops']:
        v = el(link, 'visual', name=f"stop_{s['id']}")
        el(v, 'pose', f"{s['x']} {s['y']} 0.032 0 0 0")
        cy = el(el(v, 'geometry'), 'cylinder')
        el(cy, 'radius', 1.52)
        el(cy, 'length', 0.01)
        material(v, hexrgb('#dce5c2'))
    for name, p in CFG['observation_points'].items():
        v = el(link, 'visual', name=f'op_{name}')
        el(v, 'pose', f"{p['x']} {p['y']} 0.034 0 0 0")
        cy = el(el(v, 'geometry'), 'cylinder')
        el(cy, 'radius', 0.8)
        el(cy, 'length', 0.01)
        material(v, hexrgb('#6fa8dc'))
    for name, p in CFG['inspection_waypoints'].items():
        v = el(link, 'visual', name=f'wp_{name}')
        el(v, 'pose', f"{p['x']} {p['y']} 0.034 0 0 0")
        el(el(el(v, 'geometry'), 'box'), 'size', '1.2 1.2 0.01')
        material(v, hexrgb('#e69138'))
    for name, p in CFG.get('landing_pads', {}).items():
        v = el(link, 'visual', name=f'pad_{name}')
        el(v, 'pose', f"{p['x']} {p['y']} 0.034 0 0 0")
        el(el(el(v, 'geometry'), 'box'), 'size', '2.0 2.0 0.01')
        material(v, hexrgb('#cccccc'))
        v = el(link, 'visual', name=f'pad_{name}_h')
        el(v, 'pose', f"{p['x']} {p['y']} 0.036 0 0 0")
        el(el(el(v, 'geometry'), 'box'), 'size', '0.2 1.2 0.01')
        material(v, hexrgb('#333333'))


def qav250(w, name, pose, camera=True):
    """Independent PX4 QAV250 model, from the original project's generated qav250 SDF.

    Changes from the reference: one forward camera tilted down (fixed, no gimbal) replaces
    the FPV + nadir pair; prop contact sensors removed; topics namespaced by aircraft."""
    ref = PKG / 'models/qav250_reference'
    base = E.parse(ref / 'qav250_base.sdf').getroot().find('model')
    plugs = E.parse(ref / 'qav250.sdf').getroot().find('model').findall('plugin')
    m = el(w, 'model', name=name)
    el(m, 'pose', ' '.join(f'{p:.6g}' for p in pose))
    for node in base:
        if node.tag in ('static', 'self_collide'):
            continue
        m.append(copy.deepcopy(node))
    link = m.find("link[@name='base_link']")
    for s in list(link.findall('sensor')):
        if s.get('type') == 'camera':
            link.remove(s)
    for rl in m.findall('link'):
        for s in list(rl.findall('sensor')):
            if s.get('type') == 'contact':
                rl.remove(s)
    cam_cfg = CFG['camera']
    if camera:
        s = el(link, 'sensor', name='camera', type='camera')
        el(s, 'pose', f"{cam_cfg['x']} 0 {cam_cfg['z']} 0 {math.radians(cam_cfg['tilt_down_deg']):.6f} 0")
        el(s, 'always_on', 'true')
        el(s, 'update_rate', cam_cfg['rate_hz'])
        el(s, 'topic', f'/{name}/camera/image')
        el(s, 'gz_frame_id', f'{name}/camera_link')
        c = el(s, 'camera')
        el(c, 'camera_info_topic', f'/{name}/camera/camera_info')
        el(c, 'horizontal_fov', cam_cfg['hfov_rad'])
        im = el(c, 'image')
        el(im, 'width', cam_cfg['width'])
        el(im, 'height', cam_cfg['height'])
        el(im, 'format', 'R8G8B8')
        cl = el(c, 'clip')
        el(cl, 'near', 0.05)
        el(cl, 'far', 200)
    for p in plugs:
        m.append(copy.deepcopy(p))
    return m


def write(root, path):
    E.indent(root)
    path.parent.mkdir(parents=True, exist_ok=True)
    E.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def fieldlab():
    scene = SCENE['scene']
    defaults = SCENE['mission_defaults']
    root, w = world_header('fieldlab')
    ground(w, scene['size'] + 10, hexrgb('#abb093'))
    roads(w, scene)
    markers(w, defaults)
    walkers = {k: v for k, v in CFG['walkers'].items()}
    for o in scene['objects']:
        if o['kind'] == 'building':
            building(w, o)
        elif o['kind'] == 'person':
            if o['id'] in walkers:
                walker_model(w, o, walkers[o['id']])
            else:
                include(w, f"model://human_mesh_{o['variant']}", o['id'], (o['x'], o['y'], 0, 0, 0, o['yaw']))
        else:
            vehicle(w, o)
    # UGV at the Fieldlab start, facing its first leg (yaw 0 towards stop alpha).
    ugv = CFG['ugv']
    include(w, 'model://tins_ugv', 'tins_ugv', (ugv['x'], ugv['y'], ugv['z'], 0, 0, ugv['yaw']), static=False)
    c, s = math.cos(ugv['yaw']), math.sin(ugv['yaw'])
    for uav in ('uav01', 'uav02'):
        dx, dy, dz = UGV['dock_positions_model_frame_m'][uav]
        qav250(w, uav, (ugv['x'] + c * dx - s * dy, ugv['y'] + s * dx + c * dy,
                        ugv['z'] + dz + QAV_CG_Z + DOCK_CLEARANCE, 0, 0, ugv['yaw']))
    write(root, PKG / 'worlds/fieldlab.sdf')


def minimal():
    root, w = world_header('minimal')
    ground(w, 100, hexrgb('#abb093'))
    m = el(w, 'model', name='launch_pad')
    el(m, 'static', 'true')
    link = el(m, 'link', name='pad')
    box(link, 'pad', (1.0, 1.0, 0.1), (0, 0, 0.05, 0, 0, 0), hexrgb('#888888'))
    qav250(w, 'uav01', (0, 0, 0.1 + QAV_CG_Z + DOCK_CLEARANCE, 0, 0, 0))
    write(root, PKG / 'worlds/minimal_px4.sdf')


if __name__ == '__main__':
    fieldlab()
    minimal()
    print('wrote worlds/fieldlab.sdf and worlds/minimal_px4.sdf')
