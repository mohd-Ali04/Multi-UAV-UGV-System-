#!/usr/bin/env python3
"""Offline analysis of one evaluation run (logs written by the evaluator and pursuit_node).

  python3 scripts/analyze_run.py log/runs/<run_name> [--out docs/results/<run_name>]

Writes results.md (tables) and PNG plots.  Ground truth is used here for EVALUATION only:
camera projection of the scripted pedestrians checks that the selected YOLOX track really is
the intended person and that the tracker did not switch to the other one.
"""
import argparse
import csv
import glob
import json
import math
import os

import numpy as np

HFOV, W, H, TILT = 1.74, 640, 480, math.radians(20.0)
CAM_X = 0.09


def load(path):
    with open(path) as f:
        return list(csv.DictReader(f))


def fnum(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def qmat(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def project(uav_pose, point):
    """Project a world point into uav01's camera using simulator ground truth (evaluation only)."""
    p, q = np.array(uav_pose[:3]), uav_pose[3:]
    R = qmat(*q)                                   # body (FLU) -> world
    c, s = math.cos(TILT), math.sin(TILT)
    Rc = R @ np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    cam_pos = p + R @ np.array([CAM_X, 0, 0])
    v = Rc.T @ (np.array(point) - cam_pos)          # camera FLU
    if v[0] <= 0.1:
        return None
    f = (W / 2) / math.tan(HFOV / 2)
    return W / 2 - f * v[1] / v[0], H / 2 - f * v[2] / v[0]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run')
    ap.add_argument('--out', default='')
    ap.add_argument('--target', default='person_12', help='walker the operator meant to select')
    a = ap.parse_args()
    out = a.out or os.path.join(a.run, 'analysis')
    os.makedirs(out, exist_ok=True)
    tel = load(os.path.join(a.run, 'telemetry.csv'))
    cmds = load(os.path.join(a.run, 'commands.csv'))
    walkers = sorted({k[3:-2] for k in tel[0] if k.startswith('gt_person') and k.endswith('_x')})
    t = np.array([fnum(r['t_sim']) for r in tel])
    lines = [f'# Run analysis: {os.path.basename(os.path.normpath(a.run))}', '',
             'Estimated = PX4 estimate used by the controllers. Ground truth (GT) = Gazebo pose, used here only for evaluation.', '']

    # ---- hold performance per aircraft
    lines += ['## Hold at observation points', '', '| UAV | target | samples | est. error mean / max (m) | GT error mean / max (m) | telemetry age max (s) |', '|---|---|---|---|---|---|']
    for u in ('uav01', 'uav02'):
        rows = [r for r in tel if r.get(f'{u}_phase') == 'HOLD' and r.get(f'{u}_target', '').startswith(('op', 'wp'))]
        if not rows:
            lines.append(f'| {u} | - | 0 | - | - | - |')
            continue
        est = np.array([fnum(r[f'{u}_hold_err']) for r in rows])
        gt = np.array([math.dist([fnum(r[f'gt_{u}_x']), fnum(r[f'gt_{u}_y']), fnum(r[f'gt_{u}_z'])],
                                 [fnum(r[f'{u}_hold_x']), fnum(r[f'{u}_hold_y']), fnum(r[f'{u}_hold_z'])]) for r in rows])
        age = np.array([fnum(r[f'{u}_age_s']) for r in rows])
        targets = sorted({r[f'{u}_target'] for r in rows})
        lines.append(f'| {u} | {", ".join(targets)} | {len(rows)} | {np.nanmean(est):.2f} / {np.nanmax(est):.2f} | '
                     f'{np.nanmean(gt):.2f} / {np.nanmax(gt):.2f} | {np.nanmax(age):.2f} |')

    # ---- tracking test
    trk = [r for r in tel if r.get('trk_state') in ('ACQUIRING', 'TRACKING', 'LOST')]
    if trk:
        sel_ids = sorted({r['trk_id'] for r in trk})
        lat = [fnum(r['trk_latency']) for r in trk if fnum(r['trk_latency']) >= 0]
        hold = np.array([[fnum(r['uav01_hold_x']), fnum(r['uav01_hold_y']), fnum(r['uav01_hold_z'])] for r in trk])
        drift = np.nanmax(np.linalg.norm(hold - hold[0], axis=1))
        match, total, other = 0, 0, 0
        errs = []
        for r in trk:
            if r['trk_state'] != 'TRACKING' or not r['trk_x1']:
                continue
            pose = [fnum(r[f'gt_uav01_{k}']) for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
            if any(math.isnan(v) for v in pose):
                continue
            cx, cy = (fnum(r['trk_x1']) + fnum(r['trk_x2'])) / 2, (fnum(r['trk_y1']) + fnum(r['trk_y2'])) / 2
            best, bd = None, 1e9
            for w in walkers:
                g = [fnum(r[f'gt_{w}_x']), fnum(r[f'gt_{w}_y']), 0.9]
                pr = project(pose, g)
                if pr is None:
                    continue
                d = math.hypot(pr[0] - cx, pr[1] - cy)
                if d < bd:
                    best, bd = w, d
            if best is None:
                continue
            total += 1
            match += best == a.target
            other += best != a.target
            errs.append(math.hypot(fnum(r['trk_err_x']), fnum(r['trk_err_y'])))
        lost_rows = [r for r in trk if r['trk_state'] == 'LOST']
        lines += ['', '## Inset tracking test (uav01 holding at its observation point)', '',
                  f'- selected YOLOX IDs during the test: {sel_ids}',
                  f'- selection to first centred inset: {lat[0]:.2f} s' if lat else '- selection to first centred inset: not reached',
                  f'- TRACKING samples where the box matches {a.target} by GT projection: {match}/{total}'
                  + (f' ({100 * match / total:.1f} %)' if total else ''),
                  f'- TRACKING samples matching another pedestrian: {other}',
                  f'- inset centring error while tracking: median {np.median(errs):.1f} px, 95th pct {np.percentile(errs, 95):.1f} px (inset pixels)' if errs else '',
                  f'- LOST samples: {len(lost_rows)}; selected ID while LOST: {sorted({r["trk_id"] for r in lost_rows})}',
                  f'- uav01 commanded hold position drift during the test: {drift:.3f} m']

    # ---- command isolation
    lines += ['', '## Command log (terminal and GUI share it)', '', '| t (s) | target | command | argument | state | detail |', '|---|---|---|---|---|---|']
    for c in cmds:
        lines.append(f"| {fnum(c['t_sim']):.1f} | {c['target']} | {c['command']} | {c['argument']} | {c['state']} | {c['detail'][:90]} |")
    iso = []
    for c in cmds:
        if c['target'] in ('uav01', 'uav02') and c['state'] == 'ACCEPTED':
            other = 'uav02' if c['target'] == 'uav01' else 'uav01'
            tc = fnum(c['t_sim'])
            before = [r for r in tel if tc - 1.0 <= fnum(r['t_sim']) < tc]
            after = [r for r in tel if tc < fnum(r['t_sim']) <= tc + 5.0]
            if before and after:
                same = {(r[f'{other}_phase'], r[f'{other}_target'], r[f'{other}_last_cmd']) for r in before + after}
                iso.append((tc, c['target'], c['command'], c['argument'], other, len(same) == 1, sorted(same)[0]))
    lines += ['', '## Per-aircraft command isolation', '', '| t (s) | commanded | command | other aircraft | other unchanged for 5 s | other state |', '|---|---|---|---|---|---|']
    for tc, u, cmd, arg, o, same, st in iso:
        lines.append(f'| {tc:.1f} | {u} | {cmd} {arg} | {o} | {"yes" if same else "NO"} | {st[0]} {st[1]} |')

    # ---- pursuit summaries (logs written by pursuit_node, copied into <run>/pursuit/)
    pursuits = sorted(glob.glob(os.path.join(a.run, 'pursuit', '*.json')))
    for js in pursuits:
        m = json.load(open(js))
        lines += ['', f'## Pursuit {os.path.basename(js)}', '',
                  'From the pursuit log (estimated quantities, what the controller saw):', '',
                  '| metric | value |', '|---|---|']
        for k in ('track_id', 'outcome', 'time_to_intercept_s', 'terminal_range_est_m', 'peak_closing_speed_est_mps',
                  'peak_ground_speed_mps', 'mean_abs_err_u_px', 'p95_abs_err_u_px', 'fov_containment', 'min_alt_m',
                  'max_pitch_deg', 'lost_events', 'duration_s'):
            v = m.get(k)
            lines.append(f'| {k} | {v:.3f} |' if isinstance(v, float) else f'| {k} | {v} |')
        # Ground truth during the engagement: separation to the walker the operator selected.
        t0 = m.get('engaged_sim_time_s')
        rows = [r for r in tel if r.get('uav01_phase') == 'PURSUIT' and f'gt_{a.target}_x' in r]
        if t0 is not None and rows:
            sep = np.array([[fnum(r['t_sim']) - t0,
                             math.hypot(fnum(r['gt_uav01_x']) - fnum(r[f'gt_{a.target}_x']),
                                        fnum(r['gt_uav01_y']) - fnum(r[f'gt_{a.target}_y'])),
                             fnum(r['gt_uav01_z'])] for r in rows])
            first = lambda d: next((f'{t:.2f} s' for t, s_, _ in sep if s_ <= d), 'not reached')
            lines += ['', f'Ground truth (Gazebo poses, evaluation only), separation uav01 to {a.target}:', '',
                      '| metric | value |', '|---|---|',
                      f'| initial horizontal separation | {sep[0, 1]:.2f} m |',
                      f'| time to close to 5 m / 3 m / 2 m | {first(5.0)} / {first(3.0)} / {first(2.0)} |',
                      f'| minimum horizontal separation | {np.nanmin(sep[:, 1]):.2f} m |',
                      f'| mean separation after first reaching 3 m | '
                      f'{np.nanmean(sep[np.argmax(sep[:, 1] <= 3.0):, 1]):.2f} m |' if (sep[:, 1] <= 3.0).any() else
                      '| mean separation after first reaching 3 m | not reached |',
                      f'| minimum altitude (GT) | {np.nanmin(sep[:, 2]):.2f} m |']
            try:
                import matplotlib
                matplotlib.use('Agg')
                import matplotlib.pyplot as plt
                c = list(csv.DictReader(open(js.replace('.json', '.csv'))))
                fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
                ax[0].plot(sep[:, 0], sep[:, 1], color='tab:purple', label=f'horizontal separation to {a.target} (GT)')
                ax[0].axhline(1.5, color='0.5', ls='--', lw=1, label='1.5 m standoff')
                ax[0].set_ylabel('m')
                ax[1].plot([fnum(r['t_s']) for r in c], [fnum(r['speed_mps']) for r in c], color='tab:orange', label='ground speed (est.)')
                ax[1].plot(sep[:, 0], sep[:, 2], color='tab:green', label='altitude (GT)')
                ax[1].axhline(1.2, color='tab:red', ls=':', lw=1, label='1.2 m altitude floor')
                ax[1].set_ylabel('m/s, m')
                ax[2].plot([fnum(r['t_s']) for r in c], [fnum(r['err_u_px']) for r in c], '.', ms=2, color='tab:blue', label='horizontal box error (px)')
                ax[2].set_ylabel('px')
                ax[2].set_xlabel('time since engage (s)')
                for x in ax:
                    x.grid(alpha=.3)
                    x.legend(fontsize=8, loc='upper right')
                fig.suptitle(f'Pursuit of track {m.get("track_id")}: {m.get("outcome")}')
                fig.savefig(os.path.join(out, os.path.basename(js).replace('.json', '.png')), dpi=120, bbox_inches='tight')
                plt.close(fig)
            except ImportError:
                pass
    open(os.path.join(out, 'results.md'), 'w').write('\n'.join(lines) + '\n')

    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(7, 6))
        for u, col in (('uav01', 'tab:orange'), ('uav02', 'tab:blue')):
            ax.plot([fnum(r[f'gt_{u}_x']) for r in tel], [fnum(r[f'gt_{u}_y']) for r in tel], color=col, lw=1.6, label=f'{u} (GT)')
        for w, col in zip(walkers, ('tab:purple', 'tab:green', 'tab:brown')):
            ax.plot([fnum(r[f'gt_{w}_x']) for r in tel], [fnum(r[f'gt_{w}_y']) for r in tel], '--', color=col, lw=1, label=f'{w} (GT)')
        ax.plot([fnum(r['gt_tins_ugv_x']) for r in tel], [fnum(r['gt_tins_ugv_y']) for r in tel], 'k-', lw=3)
        ax.plot([fnum(tel[0]['gt_tins_ugv_x'])], [fnum(tel[0]['gt_tins_ugv_y'])], 'ks', ms=8, label='UGV (GT)')
        try:
            import yaml
            cfg = yaml.safe_load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'tins8_gazebo', 'config', 'scenario.yaml')))
            for n, p in cfg['observation_points'].items():
                ax.plot(p['x'], p['y'], 'o', mfc='none', mec='tab:cyan', ms=10)
                ax.annotate(n, (p['x'], p['y']), xytext=(4, 4), textcoords='offset points', fontsize=8)
            for n, p in cfg['inspection_waypoints'].items():
                ax.plot(p['x'], p['y'], 's', mfc='none', mec='tab:olive', ms=8)
                ax.annotate(n, (p['x'], p['y']), xytext=(4, -10), textcoords='offset points', fontsize=7)
        except (OSError, ImportError):
            pass
        ax.set_aspect('equal')
        ax.set_xlabel('x east (m)')
        ax.set_ylabel('y north (m)')
        ax.legend(fontsize=8)
        ax.grid(alpha=.3)
        ax.set_title('Top view of the run (simulator ground truth)')
        fig.savefig(os.path.join(out, 'top_view.png'), dpi=130, bbox_inches='tight')
        fig, ax = plt.subplots(figsize=(8, 3))
        for u in ('uav01', 'uav02'):
            ax.plot(t, [fnum(r[f'{u}_hold_err']) for r in tel], label=f'{u} distance to commanded target (est.)')
        ax.set_xlabel('sim time (s)')
        ax.set_ylabel('m')
        ax.set_ylim(0, 10)
        ax.legend()
        ax.grid(alpha=.3)
        fig.savefig(os.path.join(out, 'hold_error.png'), dpi=130, bbox_inches='tight')
    except ImportError:
        pass
    print(open(os.path.join(out, 'results.md')).read())


if __name__ == '__main__':
    main()
