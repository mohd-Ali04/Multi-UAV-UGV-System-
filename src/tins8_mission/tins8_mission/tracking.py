"""Image-space multi-object tracker that gives YOLOX person boxes stable integer IDs.

Greedy IoU association with a constant-velocity prediction (a light SORT variant) and
optional camera-rotation compensation between frames.
An ID is never reassigned to a different track: once a track dies its ID is retired.
That property is what lets the inset report "track lost" instead of silently
jumping to another pedestrian.
"""
from dataclasses import dataclass, field

import numpy as np


def iou(a, b):
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    inter = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    if inter <= 0:
        return 0.0
    area = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / area


@dataclass
class Track:
    tid: int
    box: np.ndarray            # x1 y1 x2 y2
    score: float
    last_seen: float
    hits: int = 1
    vel: np.ndarray = field(default_factory=lambda: np.zeros(4))

    def predict(self, t):
        return self.box + self.vel * max(0.0, t - self.last_seen)


class IouTracker:
    def __init__(self, iou_min=0.2, max_age_s=1.0, min_hits=2, centre_gate_px=60.0):
        self.iou_min, self.max_age, self.min_hits, self.gate = iou_min, max_age_s, min_hits, centre_gate_px
        self.tracks = []
        self.next_id = 1

    def update(self, boxes, scores, t, warp=None):
        """boxes: list of [x1,y1,x2,y2]; returns confirmed tracks seen at time t.

        warp: optional pixel map (u, v) -> (u, v) for the camera rotation since the previous
        frame (ego-motion compensation).  Existing tracks are moved by it before association,
        so a fast yaw of the aircraft does not break the IoU match and spawn new IDs.
        """
        boxes = [np.asarray(b, dtype=float) for b in boxes]
        if warp is not None:
            for tr in self.tracks:
                cu, cv = (tr.box[0] + tr.box[2]) / 2, (tr.box[1] + tr.box[3]) / 2
                nu, nv = warp(cu, cv)
                tr.box = tr.box + np.array([nu - cu, nv - cv, nu - cu, nv - cv])
        preds = [tr.predict(t) for tr in self.tracks]
        pairs = []
        for i, p in enumerate(preds):
            for j, b in enumerate(boxes):
                ov = iou(p, b)
                if ov < self.iou_min:
                    pc, bc = (p[:2] + p[2:]) / 2, (b[:2] + b[2:]) / 2
                    if np.linalg.norm(pc - bc) < self.gate * 0.5:
                        ov = self.iou_min       # small fast boxes: centre gate fallback
                if ov >= self.iou_min:
                    pairs.append((ov, i, j))
        pairs.sort(reverse=True)
        used_t, used_b = set(), set()
        for ov, i, j in pairs:
            if i in used_t or j in used_b:
                continue
            tr = self.tracks[i]
            dt = max(1e-3, t - tr.last_seen)
            new_vel = (boxes[j] - tr.box) / dt
            tr.vel = 0.6 * tr.vel + 0.4 * new_vel
            tr.box, tr.score, tr.last_seen = boxes[j], scores[j], t
            tr.hits += 1
            used_t.add(i)
            used_b.add(j)
        for j, b in enumerate(boxes):
            if j not in used_b:
                self.tracks.append(Track(self.next_id, b, scores[j], t))
                self.next_id += 1
        self.tracks = [tr for tr in self.tracks if t - tr.last_seen <= self.max_age]
        return [tr for tr in self.tracks if tr.last_seen == t and tr.hits >= self.min_hits]
