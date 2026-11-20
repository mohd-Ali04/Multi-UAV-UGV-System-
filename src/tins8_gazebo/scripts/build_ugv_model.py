#!/usr/bin/env python3
"""Convert the submitted TinS-12E URDF into a Gazebo Harmonic SDF model.

Input : src/tins_description/urdf/tins_ugv_carousel.urdf  (aircraft_mode:=none, unchanged copy)
        src/tins_description/meshes/collision/collision_geometry.json
Output: src/tins8_gazebo/models/tins_ugv/{model.sdf,model.config}

What the conversion does (and why):
  * The 100-link URDF is grouped into five rigid SDF links: chassis, tray_a_x_carriage,
    tray_a, tray_b_x_carriage and tray_b.  Fixed joints and the 24 visual-only continuous
    running-gear joints are merged into their parent group (they carry no drive in the
    submitted design; 15 of them attach massless links, which Gazebo cannot simulate).
  * The four prismatic slide joints keep their axes, effort and velocity.  Their limits get a
    1 mm simulation-only margin (see LIMIT_MARGIN_M) so DART does not pin a tray at q=0.
  * Inertials are combined per group with the parallel-axis theorem (total mass preserved).
  * Collisions are the submitted primitive boxes/cylinders.  The six "contact wheel"
    cylinders become hidden drive wheels driven by Gazebo's DiffDrive system, as the
    collision README prescribes ("hidden wheels carry track contact").  4 kg per wheel is
    moved out of the chassis so the 385.119 kg total is unchanged.
  * Six stowed QAV250s (UAV03..UAV08) are massless visuals on their tray links, exactly like
    the URDF static_visual mode.  UAV01 and UAV02 are NOT drawn here: they are independent
    PX4 models held on their docks by DetachableJoint latches, so each active aircraft
    appears exactly once.
Label: SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED.
"""
import json
import math
import sys
import xml.etree.ElementTree as E
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
PKG = HERE.parent
SRC = PKG.parent
DESC = SRC / 'tins_description'
GROUP_ROOTS = ['base_link', 'tray_a_x_carriage', 'tray_a', 'tray_b_x_carriage', 'tray_b']
SDF_NAME = {'base_link': 'chassis'}
ACTIVE = {'uav01': 'tray_a', 'uav02': 'tray_b'}          # independent PX4 aircraft
STOWED = {'uav03': 'tray_a', 'uav05': 'tray_a', 'uav07': 'tray_a',
          'uav04': 'tray_b', 'uav06': 'tray_b', 'uav08': 'tray_b'}
WHEEL_MASS = 4.0
LIMIT_MARGIN_M = 0.001
TRAY_JOINTS = ['tray_a_x_joint', 'tray_a_z_joint', 'tray_b_x_joint', 'tray_b_z_joint']


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def tf(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def origin_tf(node):
    o = node.find('origin') if node is not None else None
    if o is None:
        return np.eye(4)
    xyz = [float(v) for v in o.get('xyz', '0 0 0').split()]
    rpy = [float(v) for v in o.get('rpy', '0 0 0').split()]
    return tf(xyz, rpy)


def pose_str(t):
    r = t[:3, :3]
    pitch = math.asin(max(-1.0, min(1.0, -r[2, 0])))
    roll = math.atan2(r[2, 1], r[2, 2])
    yaw = math.atan2(r[1, 0], r[0, 0])
    v = list(t[:3, 3]) + [roll, pitch, yaw]
    return ' '.join(f'{x:.6g}' if abs(x) > 1e-12 else '0' for x in v)


def el(parent, tag, text=None, **attrs):
    e = E.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        e.text = str(text)
    return e


def main():
    urdf = E.parse(DESC / 'urdf/tins_ugv_carousel.urdf').getroot()
    collisions = json.loads((DESC / 'meshes/collision/collision_geometry.json').read_text())['geometry']
    links = {l.get('name'): l for l in urdf.findall('link')}
    joints = urdf.findall('joint')
    children = {}
    for j in joints:
        children.setdefault(j.find('parent').get('link'), []).append(j)

    # Walk the tree: group membership and link pose relative to its group root (q = 0).
    group_of, t_in_group, t_model = {}, {}, {}
    joint_models = {}

    def walk(link, group, t_g, t_m):
        group_of[link], t_in_group[link], t_model[link] = group, t_g, t_m
        for j in children.get(link, []):
            child = j.find('child').get('link')
            t_j = origin_tf(j)
            if j.get('type') == 'prismatic':
                joint_models[j.get('name')] = j
                walk(child, child, np.eye(4), t_m @ t_j)
            else:
                walk(child, group, t_g @ t_j, t_m @ t_j)

    walk('base_link', 'base_link', np.eye(4), np.eye(4))
    assert set(joint_models) == set(TRAY_JOINTS), joint_models.keys()

    sdf = E.Element('sdf', version='1.9')
    model = el(sdf, 'model', name='tins_ugv')
    el(model, 'self_collide', 'false')
    wheel_names = [k for k in collisions if 'contact_wheel' in k]
    totals = {}
    for g in GROUP_ROOTS:
        link = el(model, 'link', name=SDF_NAME.get(g, g))
        el(link, 'pose', pose_str(t_model[g]))
        # combined inertial
        m_tot, c_acc, items = 0.0, np.zeros(3), []
        for name, l in links.items():
            if group_of.get(name) != g or l.find('inertial') is None:
                continue
            ine = l.find('inertial')
            m = float(ine.find('mass').get('value'))
            t = t_in_group[name] @ origin_tf(ine)
            i = ine.find('inertia')
            ii = {k: float(i.get(k)) for k in ['ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz']}
            I = np.array([[ii['ixx'], ii['ixy'], ii['ixz']], [ii['ixy'], ii['iyy'], ii['iyz']], [ii['ixz'], ii['iyz'], ii['izz']]])
            R = t[:3, :3]
            items.append((m, t[:3, 3], R @ I @ R.T))
            m_tot += m
            c_acc += m * t[:3, 3]
        if g == 'base_link':  # wheel masses become separate links
            m_removed = WHEEL_MASS * len(wheel_names)
        else:
            m_removed = 0.0
        com = c_acc / m_tot
        I_tot = np.zeros((3, 3))
        for m, c, I in items:
            d = c - com
            I_tot += I + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
        scale = (m_tot - m_removed) / m_tot
        totals[g] = m_tot - m_removed
        ine = el(link, 'inertial')
        el(ine, 'pose', ' '.join(f'{v:.6g}' for v in com) + ' 0 0 0')
        el(ine, 'mass', f'{m_tot - m_removed:.6f}')
        it = el(ine, 'inertia')
        for k, (a, b) in {'ixx': (0, 0), 'ixy': (0, 1), 'ixz': (0, 2), 'iyy': (1, 1), 'iyz': (1, 2), 'izz': (2, 2)}.items():
            el(it, k, f'{I_tot[a, b] * scale:.6g}')
        # visuals and collisions of every merged URDF link
        for name, l in links.items():
            if group_of.get(name) != g:
                continue
            for vi, v in enumerate(l.findall('visual')):
                mesh = v.find('geometry/mesh')
                if mesh is None:
                    continue
                vis = el(link, 'visual', name=f'{name}_v{vi}')
                el(vis, 'pose', pose_str(t_in_group[name] @ origin_tf(v)))
                gm = el(el(vis, 'geometry'), 'mesh')
                el(gm, 'uri', mesh.get('filename').replace('package://', 'model://'))
                rgba = v.find('material/color')
                if rgba is not None:
                    mat = el(vis, 'material')
                    c = rgba.get('rgba')
                    el(mat, 'ambient', c)
                    el(mat, 'diffuse', c)
                    el(mat, 'specular', '0.1 0.1 0.1 1')
            if name in wheel_names:
                continue
            for ci, c in enumerate(collisions.get(name, [])):
                col = el(link, 'collision', name=f'{name}_c{ci}')
                el(col, 'pose', pose_str(t_in_group[name] @ tf(c['xyz_m'], c['rpy_rad'])))
                geo = el(col, 'geometry')
                if c['type'] == 'box':
                    el(el(geo, 'box'), 'size', ' '.join(f'{v:.6g}' for v in c['size_m']))
                else:
                    cy = el(geo, 'cylinder')
                    el(cy, 'radius', c['radius_m'])
                    el(cy, 'length', c['length_m'])
        # stowed aircraft (massless visuals, URDF static_visual semantics)
        for uav, tray in STOWED.items():
            if tray != g:
                continue
            vis = el(link, 'visual', name=f'{uav}_stowed')
            el(vis, 'pose', pose_str(t_in_group[f'{uav}_dock']))
            gm = el(el(vis, 'geometry'), 'mesh')
            el(gm, 'uri', 'model://tins_description/meshes/aircraft/qav250_visual.stl')
            mat = el(vis, 'material')
            el(mat, 'ambient', '0.18 0.19 0.2 1')
            el(mat, 'diffuse', '0.18 0.19 0.2 1')

    # hidden drive wheels (track contact proxies from the submitted collision register)
    left, right = [], []
    for w in sorted(wheel_names):
        c = collisions[w][0]
        link = el(model, 'link', name=w)
        el(link, 'pose', pose_str(t_model[w]))
        ine = el(link, 'inertial')
        el(ine, 'mass', WHEEL_MASS)
        r, h = c['radius_m'], c['length_m']
        it = el(ine, 'inertia')
        ia = WHEEL_MASS * (3 * r * r + h * h) / 12
        for k, v in {'ixx': ia, 'iyy': WHEEL_MASS * r * r / 2, 'izz': ia, 'ixy': 0, 'ixz': 0, 'iyz': 0}.items():
            el(it, k, f'{v:.6g}')
        col = el(link, 'collision', name='track_contact')
        el(col, 'pose', pose_str(tf(c['xyz_m'], c['rpy_rad'])))
        cy = el(el(col, 'geometry'), 'cylinder')
        el(cy, 'radius', r)
        el(cy, 'length', h)
        surf = el(el(col, 'surface'), 'friction')
        ode = el(surf, 'ode')
        el(ode, 'mu', 1.0)
        el(ode, 'mu2', 0.6)
        j = el(model, 'joint', name=f'{w}_drive', type='revolute')
        el(j, 'parent', 'chassis')
        el(j, 'child', w)
        el(el(j, 'axis'), 'xyz', '0 1 0')
        (left if w.startswith('left') else right).append(f'{w}_drive')

    for jn in TRAY_JOINTS:
        j = joint_models[jn]
        sj = el(model, 'joint', name=jn, type='prismatic')
        parent_group = group_of[j.find('parent').get('link')]
        el(sj, 'parent', SDF_NAME.get(parent_group, parent_group))
        el(sj, 'child', j.find('child').get('link'))
        ax = el(sj, 'axis')
        el(ax, 'xyz', j.find('axis').get('xyz'))
        lim = j.find('limit')
        sl = el(ax, 'limit')
        # Simulation-only margin of LIMIT_MARGIN_M on each slide limit.  With DART, a joint that
        # sits exactly on a limit (q=0 is both the park pose and a limit for every tray joint) can
        # start a hair outside it (-1e-13 m); the limit constraint then overrides the velocity
        # servo and a tray that must lift against gravity never moves.  The supervisor still
        # commands only the register's waypoints, so the margin is never used in normal operation.
        el(sl, 'lower', f"{float(lim.get('lower')) - LIMIT_MARGIN_M:.6g}")
        el(sl, 'upper', f"{float(lim.get('upper')) + LIMIT_MARGIN_M:.6g}")
        for k in ['effort', 'velocity']:
            el(sl, k, lim.get(k))
        dyn = el(ax, 'dynamics')
        el(dyn, 'damping', 50)

    def plugin(fn, name, **kv):
        p = el(model, 'plugin', filename=fn, name=name)
        for k, v in kv.items():
            for item in (v if isinstance(v, list) else [v]):
                el(p, k, item)
        return p

    plugin('gz-sim-diff-drive-system', 'gz::sim::systems::DiffDrive',
           left_joint=left, right_joint=right, wheel_separation=0.65, wheel_radius=0.13,
           topic='/ugv/cmd_vel', odom_topic='/ugv/odom', tf_topic='/ugv/tf',
           frame_id='odom', child_frame_id='base_link', odom_publish_frequency=20,
           max_linear_velocity=1.2, min_linear_velocity=-1.2, max_angular_velocity=0.8, min_angular_velocity=-0.8,
           max_linear_acceleration=0.8, min_linear_acceleration=-0.8)
    plugin('gz-sim-joint-state-publisher-system', 'gz::sim::systems::JointStatePublisher',
           joint_name=TRAY_JOINTS, topic='/ugv/joint_states', update_rate=50)
    for jn in TRAY_JOINTS:
        # Velocity-command mode: the commanded slide speed is bounded by the joint's
        # submitted velocity limit (0.02 m/s).  Completion is judged by the supervisor
        # from the measured joint state, never by elapsed time.
        plugin('gz-sim-joint-position-controller-system', 'gz::sim::systems::JointPositionController',
               joint_name=jn, topic=f'/ugv/carousel/{jn}/cmd_pos', p_gain=4.0, i_gain=0.0, d_gain=0.0,
               use_velocity_commands='true', cmd_max=0.02, cmd_min=-0.02, initial_position=0.0)
    for uav, tray in ACTIVE.items():
        plugin('gz-sim-detachable-joint-system', 'gz::sim::systems::DetachableJoint',
               parent_link=tray, child_model=uav, child_link='base_link',
               detach_topic=f'/ugv/latch/{uav}/detach', attach_topic=f'/ugv/latch/{uav}/attach',
               output_topic=f'/ugv/latch/{uav}/state')

    # Dock poses of the active aircraft in the model frame (used by the world builder).
    docks = {uav: [float(v) for v in t_model[f'{uav}_dock'][:3, 3]] for uav in list(ACTIVE) + list(STOWED)}
    out = PKG / 'models/tins_ugv'
    out.mkdir(parents=True, exist_ok=True)
    E.indent(sdf)
    E.ElementTree(sdf).write(out / 'model.sdf', encoding='utf-8', xml_declaration=True)
    cfg = E.Element('model')
    el(cfg, 'name', 'tins_ugv')
    el(cfg, 'version', '1.0')
    el(cfg, 'sdf', 'model.sdf', version='1.9')
    el(cfg, 'description', 'TinS-12E crawler with two-tray guarded-exchange carousel, converted from the '
       'submitted URDF. SIMULATION_GRADE_NOT_PHYSICALLY_QUALIFIED.')
    E.indent(cfg)
    E.ElementTree(cfg).write(out / 'model.config', encoding='utf-8', xml_declaration=True)
    summary = {'group_mass_kg': totals, 'wheel_mass_kg': WHEEL_MASS * len(wheel_names),
               'total_mass_kg': sum(totals.values()) + WHEEL_MASS * len(wheel_names),
               'dock_positions_model_frame_m': docks}
    (PKG / 'data/ugv_model_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    sys.exit(main())
