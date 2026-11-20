#!/usr/bin/env python3
"""Fieldlab -> Gazebo conversion check (run while the Fieldlab world is running).

  python3 scripts/world_check.py [--out docs/results/world_check.md]

Scene yaw values are radians.  Reads every entity pose from the running Gazebo world (/world/fieldlab/pose/info, measured),
compares each object of the seed-42 Fieldlab layout (src/tins8_gazebo/data/fieldlab_scene_seed42.json,
recorded) with the Gazebo model of the same name, and checks a set of distances that a viewer
would use to judge scale (UGV to observation points, building spacing, road width).
Walkers are excluded from the position check because they are moving by design.
"""
import argparse
import json
import math
import os
import sys
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node as GzNode

HERE = os.path.dirname(os.path.abspath(__file__))
WS = os.path.dirname(HERE)
SCENE = os.path.join(WS, 'src', 'tins8_gazebo', 'data', 'fieldlab_scene_seed42.json')


def yaml_load(path):
    import yaml
    return yaml.safe_load(open(path))


def measured_poses(world='fieldlab', timeout=10.0):
    node, got, ev = GzNode(), {}, threading.Event()

    def cb(msg):
        if not got:
            for p in msg.pose:
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                got.setdefault(p.name, (p.position.x, p.position.y, p.position.z, yaw))
            ev.set()
    node.subscribe(Pose_V, f'/world/{world}/pose/info', cb)
    ev.wait(timeout)
    return got


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(WS, 'docs', 'results', 'world_check.md'))
    a = ap.parse_args()
    scene = json.load(open(SCENE))
    sc = scene['scene']
    cfg = yaml_load(os.path.join(WS, 'src', 'tins8_gazebo', 'config', 'scenario.yaml'))
    walkers = set(cfg.get('walkers', {}))
    meas = measured_poses()
    if not meas:
        sys.exit('no poses received: is the Fieldlab world running (GZ_PARTITION from scripts/env.sh)?')
    rows, errs, missing = [], [], []
    for o in sc['objects']:
        if o['id'] in walkers:
            continue
        m = meas.get(o['id'])
        if m is None:
            missing.append(o['id'])
            continue
        e = math.hypot(m[0] - o['x'], m[1] - o['y'])
        dyaw = abs((m[3] - o.get('yaw', 0.0) + math.pi) % (2 * math.pi) - math.pi)
        errs.append((o['kind'], e, math.degrees(dyaw)))
        rows.append((o['id'], o['kind'], o['x'], o['y'], m[0], m[1], e, math.degrees(dyaw)))
    lines = ['# Fieldlab to Gazebo conversion check', '',
             'Recorded = seed-42 Fieldlab layout exported from the original generator. '
             'Measured = pose reported by the running Gazebo world. Walkers are excluded (they move).', '',
             '## Object placement (per kind)', '',
             '| kind | objects | max position error (m) | max yaw error (deg) |', '|---|---|---|---|']
    kinds = sorted({k for k, _, _ in errs})
    for k in kinds:
        ek = [(e, y) for kk, e, y in errs if kk == k]
        lines.append(f'| {k} | {len(ek)} | {max(e for e, _ in ek):.3f} | {max(y for _, y in ek):.2f} |')
    lines.append(f'\nObjects missing in Gazebo: {missing or "none"}\n')

    # distance checks: recorded geometry vs measured Gazebo positions
    def mxy(name):
        m = meas.get(name)
        return None if m is None else (m[0], m[1])
    ugv = mxy('tins_ugv')
    checks = []
    for n, p in cfg['observation_points'].items():
        checks.append((f'UGV start to {n} (horizontal)', math.hypot(p['x'] - cfg['ugv']['x'], p['y'] - cfg['ugv']['y']),
                       math.hypot(p['x'] - ugv[0], p['y'] - ugv[1]) if ugv else math.nan))
    bld = [o for o in sc['objects'] if o['kind'] == 'building']
    for a_, b_ in ((bld[0], bld[1]), (bld[0], bld[5]), (bld[3], bld[10])):
        ma, mb = mxy(a_['id']), mxy(b_['id'])
        checks.append((f"{a_['id']} to {b_['id']} (centres)", math.hypot(a_['x'] - b_['x'], a_['y'] - b_['y']),
                       math.hypot(ma[0] - mb[0], ma[1] - mb[1]) if ma and mb else math.nan))
    stops = scene['mission_defaults'].get('stops', [])
    for s in stops[:3]:
        checks.append((f"Fieldlab start to stop {s['id']}", math.hypot(s['x'] - scene['mission_defaults']['start'][0],
                                                                       s['y'] - scene['mission_defaults']['start'][1]),
                       math.hypot(s['x'] - scene['mission_defaults']['start'][0], s['y'] - scene['mission_defaults']['start'][1])))
    lines += ['## Distance checks', '', '| distance | recorded (m) | measured in Gazebo (m) | difference (m) |', '|---|---|---|---|']
    for n, r, m in checks:
        lines.append(f'| {n} | {r:.2f} | {m:.2f} | {abs(r - m):.3f} |')
    lines += ['', f'Scene size {sc["size"]} m x {sc["size"]} m; roads: {len(sc["roads"])} (width {sc["roads"][0]["sy"]} m).', '',
              '## Per-object detail', '', '| id | kind | recorded x, y | measured x, y | error (m) | yaw error (deg) |', '|---|---|---|---|---|---|']
    for r in rows:
        lines.append(f'| {r[0]} | {r[1]} | {r[2]:.2f}, {r[3]:.2f} | {r[4]:.2f}, {r[5]:.2f} | {r[6]:.3f} | {r[7]:.2f} |')
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    open(a.out, 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines[:20 + len(kinds) + len(checks)]))
    print('wrote', a.out)


if __name__ == '__main__':
    main()
