"""YOLOX-s ONNX inference with OpenCV DNN (no PyTorch needed).

Model: opencv_zoo object_detection_yolox_2022nov.onnx (Apache-2.0), the same pinned file the
original Fieldlab project used.  80 COCO classes; this project keeps class 0 (person).
Pre/post-processing matches the original vision.py: top-left letterbox with value 114,
grid/stride decode, objectness x class score, NMS 0.45.
"""
from dataclasses import dataclass

import cv2
import numpy as np

INPUT = 640
STRIDES = (8, 16, 32)
PERSON = 0
MODEL_SHA256 = 'c5c2d13e59ae883e6af3b45daea64af4833a4951c92d116ec270d9ddbe998063'


@dataclass
class Box:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    cls: int

    @property
    def cx(self):
        return (self.x1 + self.x2) / 2

    @property
    def cy(self):
        return (self.y1 + self.y2) / 2

    @property
    def h(self):
        return self.y2 - self.y1


def _grids():
    grid, strides = [], []
    for s in STRIDES:
        n = INPUT // s
        yy, xx = np.meshgrid(np.arange(n), np.arange(n), indexing='ij')
        grid.append(np.stack((xx, yy), -1).reshape(-1, 2))
        strides.append(np.full((n * n, 1), s))
    return np.concatenate(grid).astype(np.float32), np.concatenate(strides).astype(np.float32)


class YoloX:
    def __init__(self, model_path, conf=0.35, nms=0.45, classes=(PERSON,), threads=4):
        self.net = cv2.dnn.readNet(str(model_path))
        cv2.setNumThreads(threads)
        self.conf, self.nms, self.classes = conf, nms, set(classes)
        self.grid, self.strides = _grids()

    def preprocess(self, bgr):
        h, w = bgr.shape[:2]
        ratio = min(INPUT / w, INPUT / h)
        resized = cv2.resize(bgr, (int(w * ratio), int(h * ratio)))
        padded = np.full((INPUT, INPUT, 3), 114, dtype=np.uint8)
        padded[:resized.shape[0], :resized.shape[1]] = resized
        return cv2.dnn.blobFromImage(padded, 1.0, (INPUT, INPUT), swapRB=False), ratio

    def decode(self, rows, ratio, width, height):
        rows = rows.copy()
        rows[:, :2] = (rows[:, :2] + self.grid) * self.strides
        rows[:, 2:4] = np.exp(np.minimum(rows[:, 2:4], 10)) * self.strides
        scores = rows[:, 4, None] * rows[:, 5:]
        cls = np.argmax(scores, axis=1)
        conf = np.max(scores, axis=1)
        idx = np.where((conf >= self.conf) & np.isin(cls, list(self.classes)))[0]
        xywh = rows[idx, :4].copy()
        xywh[:, :2] -= xywh[:, 2:] / 2
        out = []
        for c in self.classes:
            members = np.where(cls[idx] == c)[0]
            if not len(members):
                continue
            keep = cv2.dnn.NMSBoxes(xywh[members].tolist(), conf[idx[members]].tolist(), self.conf, self.nms)
            for k in np.asarray(keep).reshape(-1):
                m = members[k]
                x, y, w, h = (xywh[m] / ratio).tolist()
                x1, y1 = max(0.0, x), max(0.0, y)
                x2, y2 = min(float(width), x + w), min(float(height), y + h)
                if x2 - x1 > 2 and y2 - y1 > 2:
                    out.append(Box(x1, y1, x2, y2, float(conf[idx[m]]), int(c)))
        return out

    def __call__(self, bgr):
        blob, ratio = self.preprocess(bgr)
        self.net.setInput(blob)
        rows = self.net.forward()[0]
        return self.decode(rows, ratio, bgr.shape[1], bgr.shape[0])
