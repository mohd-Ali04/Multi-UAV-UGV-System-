#!/usr/bin/env python3
"""Save the current frame of one or more ROS image topics as PNG files (evidence screenshots).

  python3 scripts/save_frames.py OUT_PREFIX [topic ...]
  e.g. python3 scripts/save_frames.py docs/figures/op1 /uav01/camera/annotated /uav01/camera/inset

Writes OUT_PREFIX_<topic-with-underscores>.png for every topic that delivers a frame within 10 s.
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def to_bgr(m):
    a = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)
    ch = m.step // m.width
    a = a[:, :m.width * ch].reshape(m.height, m.width, ch)
    if m.encoding in ('rgb8',):
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    if m.encoding in ('mono8',):
        return a[:, :, 0]
    return a.copy()


def main():
    prefix = sys.argv[1]
    topics = sys.argv[2:] or ['/uav01/camera/annotated', '/uav01/camera/inset']
    rclpy.init()
    node = rclpy.create_node('save_frames')
    got = {}
    for t in topics:
        node.create_subscription(Image, t, lambda m, t=t: got.setdefault(t, m), qos_profile_sensor_data)
    t0 = time.time()
    while len(got) < len(topics) and time.time() - t0 < 10.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    for t, m in got.items():
        path = f"{prefix}_{t.strip('/').replace('/', '_')}.png"
        cv2.imwrite(path, to_bgr(m))
        print('saved', path)
    for t in topics:
        if t not in got:
            print('no frame on', t)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
