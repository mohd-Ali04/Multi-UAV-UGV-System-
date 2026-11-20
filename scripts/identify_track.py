#!/usr/bin/env python3
"""TEST HELPER: which YOLOX track ID on uav01's camera is a given scripted pedestrian?

  python3 scripts/identify_track.py person_12

In a demonstration the operator reads the ID off the annotated camera view and types
'tins8ctl select <ID>'.  For repeatable recorded tests this helper makes the same choice
automatically by projecting the pedestrian's simulator ground-truth position into the camera
and picking the nearest detection box.  It is never used by the tracker or the controllers.
"""
import math
import sys
import time

import rclpy
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection2DArray

sys.path.insert(0, __file__.rsplit('/', 1)[0])
from analyze_run import project  # noqa: E402  (same camera model as the offline analysis)


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else 'person_12'
    rclpy.init()
    node = rclpy.create_node('identify_track')
    st = {'gt': {}, 'det': None}

    def on_gt(m):
        for t in m.transforms:
            p, q = t.transform.translation, t.transform.rotation
            st['gt'][t.child_frame_id] = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    node.create_subscription(TFMessage, '/gz/dynamic_poses', on_gt, 10)
    node.create_subscription(Detection2DArray, '/uav01/detections', lambda m: st.__setitem__('det', m), 10)
    t0 = time.time()
    votes = {}
    while time.time() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if st['det'] is None or 'uav01' not in st['gt'] or target not in st['gt']:
            continue
        g = st['gt'][target]
        pr = project(st['gt']['uav01'], [g[0], g[1], 0.9])
        st_det, st['det'] = st['det'], None
        if pr is None:
            continue
        best, bd = None, 1e9
        for d in st_det.detections:
            c = d.bbox.center.position
            dist = math.hypot(c.x - pr[0], c.y - pr[1])
            if dist < bd:
                best, bd = d.id, dist
        if best is not None and bd < 60:
            votes[best] = votes.get(best, 0) + 1
    node.destroy_node()
    rclpy.shutdown()
    if not votes:
        print('NONE')
        sys.exit(1)
    print(max(votes, key=votes.get))


if __name__ == '__main__':
    main()
