"""Operator target selection and the magnified inset, driven only by camera detections.

  sub /<uav>/camera/image, /<uav>/detections
  srv /<uav>/tracker/select           tins8_interfaces/SelectTarget (track ID, -1 clears)
  pub /<uav>/track/status             tins8_interfaces/TrackStatus
  pub /<uav>/camera/annotated         sensor_msgs/Image  full frame with IDs (selected one highlighted)
  pub /<uav>/camera/inset             sensor_msgs/Image  magnified view centred on the selected person

Track loss: if the selected ID is absent for more than lost_after_s the state becomes LOST,
the inset is cleared with a "TRACK LOST" banner, and the tracker NEVER switches to another
pedestrian by itself.  The operator must select a new ID.
"""
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

from tins8_interfaces.msg import TrackStatus
from tins8_interfaces.srv import SelectTarget

from .common import CommandLog, new_request_id
from .yolox_detector import image_to_bgr


def bgr_to_msg(img, header):
    m = Image()
    m.header = header
    m.height, m.width = img.shape[:2]
    m.encoding = 'bgr8'
    m.step = m.width * 3
    m.data = img.tobytes()
    return m


class TargetTracker(Node):
    def __init__(self):
        super().__init__('target_tracker')
        self.declare_parameter('uav_id', 'uav01')
        self.declare_parameter('lost_after_s', 0.5)
        self.declare_parameter('inset_size', 320)
        self.declare_parameter('zoom', 3.0)
        self.declare_parameter('follow_gain', 0.6)        # fraction of the remaining error removed per frame
        self.declare_parameter('centered_px', 12.0)       # inset-pixel radius that counts as "centred"
        self.uav = self.get_parameter('uav_id').value
        self.log = CommandLog(self, 'tracker')
        self.selected = -1
        self.select_t = None
        self.select_req = None
        self.latency = -1.0
        self.state = 'IDLE'
        self.last_seen = None
        self.last_box = None
        self.view_c = None          # inset window centre in full-frame pixels (smoothed)
        self.dets = []
        self.visible = []
        self.pub_status = self.create_publisher(TrackStatus, f'/{self.uav}/track/status', 10)
        self.pub_ann = self.create_publisher(Image, f'/{self.uav}/camera/annotated', 2)
        self.pub_inset = self.create_publisher(Image, f'/{self.uav}/camera/inset', 2)
        self.create_subscription(Detection2DArray, f'/{self.uav}/detections', self.on_dets, 10)
        self.create_subscription(Image, f'/{self.uav}/camera/image', self.on_image, 2)
        self.create_service(SelectTarget, f'/{self.uav}/tracker/select', self.on_select)
        self.create_timer(0.1, self.check_loss)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_select(self, req, res):
        res.request_id = new_request_id('trk')
        self.log.emit(res.request_id, 'SELECT', 'REQUESTED', argument=req.track_id)
        if req.track_id < 0:
            self.selected, self.state, self.view_c = -1, 'IDLE', None
            res.accepted, res.message = True, 'selection cleared'
            self.log.emit(res.request_id, 'SELECT', 'COMPLETED', 'selection cleared', req.track_id)
            return res
        if self.select_req:
            self.log.emit(self.select_req, 'SELECT', 'CANCELED', 'superseded by a new selection', self.selected)
        self.selected = int(req.track_id)
        self.select_t = self.now_s()
        self.select_req = res.request_id
        self.latency = -1.0
        self.view_c = None
        self.last_seen = None
        visible = str(req.track_id) in [d.id for d in self.dets]
        self.state = 'TRACKING' if visible else 'ACQUIRING'
        res.accepted = True
        res.message = 'selected; visible now' if visible else 'selected; waiting for this ID to be detected'
        self.log.emit(res.request_id, 'SELECT', 'ACCEPTED', res.message, req.track_id)
        return res

    def on_dets(self, msg):
        self.dets = [d for d in msg.detections if d.results and d.results[0].hypothesis.class_id == 'person']
        self.visible = sorted(int(d.id) for d in self.dets)
        for d in self.dets:
            if int(d.id) == self.selected:
                b = d.bbox
                self.last_box = (b.center.position.x - b.size_x / 2, b.center.position.y - b.size_y / 2,
                                 b.center.position.x + b.size_x / 2, b.center.position.y + b.size_y / 2,
                                 d.results[0].hypothesis.score)
                self.last_seen = self.now_s()
                if self.state in ('ACQUIRING', 'LOST'):
                    if self.state == 'LOST':
                        self.log.emit('-', 'TRACK', 'COMPLETED', 'selected ID re-detected', self.selected)
                    self.state = 'TRACKING'

    def check_loss(self):
        if self.selected < 0 or self.state == 'LOST':
            self.publish_status()
            return
        ref = self.last_seen if self.last_seen is not None else self.select_t
        if self.state == 'TRACKING' and ref is not None and self.now_s() - ref > self.get_parameter('lost_after_s').value:
            self.state = 'LOST'
            self.view_c = None
            self.log.emit(self.select_req or '-', 'TRACK', 'FAILED', 'TRACK LOST: selected ID not detected', self.selected)
        self.publish_status()

    def on_image(self, msg):
        img = image_to_bgr(msg)
        h, w = img.shape[:2]
        ann = img.copy()
        for d in self.dets:
            b = d.bbox
            x1, y1 = int(b.center.position.x - b.size_x / 2), int(b.center.position.y - b.size_y / 2)
            x2, y2 = int(b.center.position.x + b.size_x / 2), int(b.center.position.y + b.size_y / 2)
            sel = int(d.id) == self.selected
            color = (40, 220, 255) if sel else (90, 200, 90)
            cv2.rectangle(ann, (x1, y1), (x2, y2), color, 3 if sel else 1)
            cv2.putText(ann, f'ID {d.id}', (x1, max(12, y1 - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
        size = self.get_parameter('inset_size').value
        inset = np.zeros((size, size, 3), np.uint8)
        if self.state == 'TRACKING' and self.last_box is not None:
            cx, cy = (self.last_box[0] + self.last_box[2]) / 2, (self.last_box[1] + self.last_box[3]) / 2
            if self.view_c is None:
                self.view_c = np.array([cx, cy])
            else:
                g = self.get_parameter('follow_gain').value
                self.view_c += g * (np.array([cx, cy]) - self.view_c)
            zoom = self.get_parameter('zoom').value
            half = size / (2 * zoom)
            vx = float(np.clip(self.view_c[0], half, w - half))
            vy = float(np.clip(self.view_c[1], half, h - half))
            crop = img[int(vy - half):int(vy + half), int(vx - half):int(vx + half)]
            if crop.size:
                inset = cv2.resize(crop, (size, size), interpolation=cv2.INTER_LINEAR)
            err = (np.array([cx, cy]) - self.view_c) * zoom
            self.err = err
            if self.latency < 0 and math.hypot(*err) <= self.get_parameter('centered_px').value:
                self.latency = self.now_s() - self.select_t
                self.log.emit(self.select_req or '-', 'SELECT', 'COMPLETED',
                              f'inset centred {self.latency:.2f} s after selection', self.selected)
            cv2.rectangle(ann, (int(vx - half), int(vy - half)), (int(vx + half), int(vy + half)), (255, 160, 40), 1)
            cv2.putText(inset, f'ID {self.selected}  TRACKING', (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (40, 220, 255), 1, cv2.LINE_AA)
        elif self.state == 'LOST':
            cv2.putText(inset, 'TRACK LOST', (int(size * 0.18), size // 2), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (40, 40, 255), 2, cv2.LINE_AA)
            cv2.putText(ann, f'TRACK LOST (ID {self.selected})', (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (40, 40, 255), 2, cv2.LINE_AA)
        elif self.state == 'ACQUIRING':
            cv2.putText(inset, f'waiting for ID {self.selected}', (10, size // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1, cv2.LINE_AA)
        else:
            cv2.putText(inset, 'no target selected', (10, size // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (160, 160, 160), 1, cv2.LINE_AA)
        self.pub_ann.publish(bgr_to_msg(ann, msg.header))
        self.pub_inset.publish(bgr_to_msg(inset, msg.header))

    def publish_status(self):
        m = TrackStatus()
        m.stamp = self.get_clock().now().to_msg()
        m.camera_uav = self.uav
        m.state = self.state
        m.selected_id = self.selected
        if self.last_box is not None and self.state == 'TRACKING':
            m.bbox_xyxy = [float(v) for v in self.last_box[:4]]
            m.confidence = float(self.last_box[4])
            e = getattr(self, 'err', (0.0, 0.0))
            m.center_error_px_x, m.center_error_px_y = float(e[0]), float(e[1])
        m.seconds_since_seen = float(self.now_s() - self.last_seen) if self.last_seen else -1.0
        m.selection_to_centered_s = float(self.latency)
        m.visible_person_ids = list(self.visible)
        m.visible_person_ids_count = len(self.visible)
        self.pub_status.publish(m)


def main():
    rclpy.init()
    rclpy.spin(TargetTracker())


if __name__ == '__main__':
    main()
