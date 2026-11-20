"""YOLOX person detector + IoU tracker for one aircraft camera.

Detection works on camera images only.  Simulator ground truth is never read here; the
evaluator node uses ground truth separately to score the result.

  sub /<uav>/camera/image        sensor_msgs/Image (rgb8, bridged from Gazebo)
  sub /<uav>/camera/camera_info  sensor_msgs/CameraInfo (intrinsics for ego-motion compensation)
  sub /<uav>/fmu/out/vehicle_attitude  px4_msgs/VehicleAttitude (camera rotation between frames)
  pub /<uav>/detections          vision_msgs/Detection2DArray (class 'person', Detection2D.id = track ID)
  pub /<uav>/detector/latency_s  std_msgs/Float32 (image stamp -> publish, sim time)
"""
import os
import threading

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from px4_msgs.msg import VehicleAttitude
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from .common import PX4_OUT_QOS, AttitudeHistory, px4_topic, scenario
from .geo import rotation_warp
from .tracking import IouTracker
from .yolox import YoloX


def image_to_bgr(msg):
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = buf.reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1].copy() if msg.encoding == 'rgb8' else img.copy()


class YoloxDetector(Node):
    def __init__(self):
        super().__init__('yolox_detector')
        self.declare_parameter('uav_id', 'uav01')
        self.declare_parameter('model_path', '')
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('max_rate_hz', 8.0)
        self.declare_parameter('threads', 6)
        uav = self.get_parameter('uav_id').value
        path = self.get_parameter('model_path').value or os.path.join(
            get_package_share_directory('tins8_mission'), 'models', 'yolox.onnx')
        self.net = YoloX(path, conf=self.get_parameter('confidence').value, threads=self.get_parameter('threads').value)
        self.tracker = IouTracker()
        self.tilt = np.radians(scenario()['camera']['tilt_down_deg'])
        self.k = None
        self.att = AttitudeHistory()
        self.prev_q = None
        self.create_subscription(CameraInfo, f'/{uav}/camera/camera_info', lambda m: setattr(self, 'k', list(m.k)), 2)
        self.create_subscription(VehicleAttitude, px4_topic(f'/{uav}', 'out', 'vehicle_attitude', VehicleAttitude),
                                 lambda m: self.att.add(self.get_clock().now().nanoseconds * 1e-9, m.q), PX4_OUT_QOS)
        self.latest = None
        self.cv = threading.Condition()
        self.pub = self.create_publisher(Detection2DArray, f'/{uav}/detections', 10)
        self.lat_pub = self.create_publisher(Float32, f'/{uav}/detector/latency_s', 10)
        self.create_subscription(Image, f'/{uav}/camera/image', self.on_image, 2)
        threading.Thread(target=self.worker, daemon=True).start()
        self.get_logger().info(f'YOLOX-s loaded from {path}')

    def on_image(self, msg):
        with self.cv:
            self.latest = msg
            self.cv.notify()

    def worker(self):
        period = 1.0 / self.get_parameter('max_rate_hz').value
        import time
        while rclpy.ok():
            with self.cv:
                while self.latest is None and rclpy.ok():
                    self.cv.wait(0.5)
                msg, self.latest = self.latest, None
            if msg is None:
                continue
            t0 = time.monotonic()
            boxes = self.net(image_to_bgr(msg))
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q = self.att.at(stamp)
            warp = None
            if q is not None and self.prev_q is not None and self.k is not None:
                k = self.k
                warp = rotation_warp(k[0], k[4], k[2], k[5], self.tilt, self.prev_q, q)
            self.prev_q = q
            tracks = self.tracker.update([[b.x1, b.y1, b.x2, b.y2] for b in boxes], [b.score for b in boxes], stamp, warp)
            out = Detection2DArray()
            out.header = msg.header
            for tr in tracks:
                d = Detection2D()
                d.header = msg.header
                d.id = str(tr.tid)
                x1, y1, x2, y2 = (float(v) for v in tr.box)
                d.bbox.center.position.x, d.bbox.center.position.y = (x1 + x2) / 2, (y1 + y2) / 2
                d.bbox.size_x, d.bbox.size_y = x2 - x1, y2 - y1
                h = ObjectHypothesisWithPose()
                h.hypothesis.class_id = 'person'
                h.hypothesis.score = float(tr.score)
                d.results.append(h)
                out.detections.append(d)
            self.pub.publish(out)
            now = self.get_clock().now().nanoseconds * 1e-9
            self.lat_pub.publish(Float32(data=float(now - stamp)))
            spare = period - (time.monotonic() - t0)
            if spare > 0:
                time.sleep(spare)


def main():
    rclpy.init()
    node = YoloxDetector()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
