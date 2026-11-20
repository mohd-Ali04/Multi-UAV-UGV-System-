#!/usr/bin/env python3
"""TEST HELPER: press buttons of the operator GUI without a mouse (recorded GUI tests).

  python3 scripts/gui_click.py [--stay SECONDS] [--set WIDGET=VALUE ...] "Button text" [...]
  e.g. python3 scripts/gui_click.py --stay 150 "Start mission"
       python3 scripts/gui_click.py --set track=12 "Select ID"
       python3 scripts/gui_click.py --group UAV02 "Hold"

It builds the same tins8_mission.gui.Window (offscreen, a second GUI instance) and clicks the
named QPushButton, so the request goes through exactly the GUI code path and ROS interfaces.
The visible GUI and 'tins8ctl watch' show the resulting REQUESTED / ACCEPTED / COMPLETED states.
--set track=N sets the track-ID spin box before clicking.  --group restricts the search to the
group box with that title (the UAV01 and UAV02 rows have buttons with the same text).
"""
import argparse
import os
import sys
import threading
import time

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import rclpy  # noqa: E402
from PyQt5 import QtWidgets  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402

from tins8_mission.gui import RosSide, Window  # noqa: E402


def find_button(win, text, group):
    scope = win
    if group:
        boxes = [g for g in win.findChildren(QtWidgets.QGroupBox) if g.title() == group]
        labels = [lb for lb in win.findChildren(QtWidgets.QLabel) if lb.text() == group]
        if boxes:
            scope = boxes[0]
        elif labels:                         # section label followed by its row widget
            scope = labels[0].parentWidget()
    hits = [b for b in scope.findChildren(QtWidgets.QPushButton) if b.text() == text]
    if group and not boxes and labels:
        # choose the button closest below the group label
        y0 = labels[0].mapTo(win, labels[0].rect().topLeft()).y()
        hits = sorted((b for b in hits if b.mapTo(win, b.rect().topLeft()).y() > y0),
                      key=lambda b: b.mapTo(win, b.rect().topLeft()).y())
    return hits[0] if hits else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('buttons', nargs='+')
    ap.add_argument('--stay', type=float, default=4.0, help='seconds to keep the GUI node alive after clicking')
    ap.add_argument('--set', action='append', default=[])
    ap.add_argument('--group', default='')
    a = ap.parse_args()
    rclpy.init()
    ros = RosSide()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(ros)
    threading.Thread(target=ex.spin, daemon=True).start()
    app = QtWidgets.QApplication(sys.argv)
    win = Window(ros)
    win.show()
    t0 = time.time()
    while time.time() - t0 < 3.0:              # discovery of services and the action server
        app.processEvents()
        time.sleep(0.05)
    for s in a.set:
        k, v = s.split('=', 1)
        if k == 'track':
            win.findChildren(QtWidgets.QSpinBox)[0].setValue(int(v))
    for text in a.buttons:
        b = find_button(win, text, a.group)
        if b is None:
            print(f'no button {text!r}')
            continue
        b.click()
        print(f'clicked {text!r}' + (f' in {a.group}' if a.group else ''))
        app.processEvents()
        time.sleep(0.5)
    t0 = time.time()
    while time.time() - t0 < a.stay:
        app.processEvents()
        time.sleep(0.05)
    ex.shutdown()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
