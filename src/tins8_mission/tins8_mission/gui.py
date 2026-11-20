"""tins8_gui: operator GUI (PyQt5).  Uses exactly the same ROS topics, services and action
as tins8ctl, so terminal and GUI always show the same states."""
import sys
import threading

import numpy as np
import rclpy
from action_msgs.srv import CancelGoal
from PyQt5 import QtCore, QtGui, QtWidgets
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from tins8_interfaces.action import RunMission
from tins8_interfaces.msg import CarouselState, CommandStatus, FleetState, TrackStatus, UavStatus
from tins8_interfaces.srv import ActorCommand, CarouselCommand, PursuitCommand, SelectTarget, UavCommand

from .common import LATCHED_QOS, LOG_QOS, UAVS, scenario

STATE_COLOR = {'REQUESTED': '#9aa5b1', 'ACCEPTED': '#3b82c4', 'COMPLETED': '#2e8b57', 'REJECTED': '#c0392b',
               'CANCELED': '#d68910', 'FAILED': '#c0392b'}


class RosSide(Node):
    def __init__(self):
        super().__init__('tins8_gui')
        self.data = {'fleet': None, 'carousel': None, 'track': None, 'uav': {}, 'img': None, 'inset': None,
                     'log': [], 'mission': ('-', 0.0, ''), 'mission_result': ''}
        self.lock = threading.Lock()
        self.create_subscription(FleetState, '/fleet/state', lambda m: self._put('fleet', m), LATCHED_QOS)
        self.create_subscription(CarouselState, '/carousel/state', lambda m: self._put('carousel', m), LATCHED_QOS)
        self.create_subscription(TrackStatus, '/uav01/track/status', lambda m: self._put('track', m), 10)
        for u in UAVS:
            self.create_subscription(UavStatus, f'/{u}/status', lambda m, u=u: self.data['uav'].__setitem__(u, m), 10)
        self.create_subscription(Image, '/uav01/camera/annotated', lambda m: self._put('img', m), 2)
        self.create_subscription(Image, '/uav01/camera/inset', lambda m: self._put('inset', m), 2)
        self.create_subscription(CommandStatus, '/tins8/command_log', lambda m: self.data['log'].append(m), LOG_QOS)
        self.car = self.create_client(CarouselCommand, '/carousel/command')
        self.uavc = {u: self.create_client(UavCommand, f'/{u}/command') for u in UAVS}
        self.sel = self.create_client(SelectTarget, '/uav01/tracker/select')
        self.pur = {u: self.create_client(PursuitCommand, f'/{u}/pursuit/command') for u in UAVS}
        self.act = self.create_client(ActorCommand, '/scenario/actor')
        self.cancel = self.create_client(CancelGoal, '/mission/run/_action/cancel_goal')
        self.mission = ActionClient(self, RunMission, '/mission/run')
        self.goal_handle = None

    def _put(self, k, v):
        with self.lock:
            self.data[k] = v

    def call(self, client, req, note):
        if not client.service_is_ready():
            self.data['log'].append(CommandStatus(target='gui', command=note, state='REJECTED', detail='service not available'))
            return
        client.call_async(req)

    def start_mission(self, p1, p2, track):
        if not self.mission.server_is_ready():
            self.data['mission'] = ('unavailable', 0.0, 'mission server not running')
            return
        fut = self.mission.send_goal_async(RunMission.Goal(uav1_point=p1, uav2_point=p2, track_id=track),
                                           feedback_callback=self._fb)
        fut.add_done_callback(self._goal_resp)

    def _fb(self, f):
        self.data['mission'] = (f.feedback.phase, f.feedback.progress, f.feedback.detail)

    def _goal_resp(self, fut):
        h = fut.result()
        if not h.accepted:
            self.data['mission_result'] = 'REJECTED'
            return
        self.goal_handle = h
        self.data['mission_result'] = 'ACCEPTED'
        h.get_result_async().add_done_callback(lambda r: self.data.__setitem__(
            'mission_result', f'{r.result().result.outcome}: {r.result().result.summary}'))


def to_qimage(msg):
    if msg is None:
        return None
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)
    fmt = QtGui.QImage.Format_BGR888 if msg.encoding == 'bgr8' else QtGui.QImage.Format_RGB888
    return QtGui.QImage(arr.data, msg.width, msg.height, msg.step, fmt).copy()


class Window(QtWidgets.QMainWindow):
    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        cfg = scenario()
        self.obs = list(cfg['observation_points'])
        self.wps = list(cfg['inspection_waypoints'])
        self.walkers = list(cfg.get('walkers', {}))
        self.setWindowTitle('TinS-12E 8-UAV course build - operator GUI')
        central = QtWidgets.QWidget()
        scroll = QtWidgets.QScrollArea()          # fits small screens; scrolls instead of clipping
        scroll.setWidgetResizable(True)
        scroll.setWidget(central)
        self.setCentralWidget(scroll)
        grid = QtWidgets.QGridLayout(central)

        # camera column
        cam = QtWidgets.QGroupBox('UAV01 camera (YOLOX detections) and magnified inset')
        cl = QtWidgets.QGridLayout(cam)
        self.img = QtWidgets.QLabel('waiting for /uav01/camera/annotated')
        self.img.setFixedSize(560, 420)
        self.img.setStyleSheet('background:#222;color:#aaa')
        self.inset = QtWidgets.QLabel('inset')
        self.inset.setFixedSize(280, 280)
        self.inset.setStyleSheet('background:#111;color:#aaa')
        cl.addWidget(self.img, 0, 0, 3, 1)
        cl.addWidget(self.inset, 0, 1)
        self.trk = QtWidgets.QLabel('-')
        self.trk.setWordWrap(True)
        cl.addWidget(self.trk, 1, 1)
        row = QtWidgets.QHBoxLayout()
        self.track_id = QtWidgets.QSpinBox()
        self.track_id.setRange(0, 9999)
        b_sel = QtWidgets.QPushButton('Select ID')
        b_sel.clicked.connect(lambda: ros.call(ros.sel, SelectTarget.Request(track_id=self.track_id.value()), 'SELECT'))
        b_clr = QtWidgets.QPushButton('Clear')
        b_clr.clicked.connect(lambda: ros.call(ros.sel, SelectTarget.Request(track_id=-1), 'SELECT'))
        b_ps = QtWidgets.QPushButton('Pursuit start')
        b_ps.clicked.connect(lambda: ros.call(ros.pur['uav01'], PursuitCommand.Request(engage=True, track_id=self.track_id.value()), 'PURSUIT'))
        b_pt = QtWidgets.QPushButton('Pursuit stop')
        b_pt.clicked.connect(lambda: ros.call(ros.pur['uav01'], PursuitCommand.Request(engage=False, track_id=-1), 'PURSUIT'))
        for w in (QtWidgets.QLabel('track ID'), self.track_id, b_sel, b_clr, b_ps, b_pt):
            row.addWidget(w)
        rw = QtWidgets.QWidget()
        rw.setLayout(row)
        cl.addWidget(rw, 3, 0, 1, 2)
        arow = QtWidgets.QHBoxLayout()
        self.actor = QtWidgets.QComboBox()
        self.actor.addItems(self.walkers)
        arow.addWidget(QtWidgets.QLabel('pedestrian'))
        arow.addWidget(self.actor)
        for a in ('WALK', 'STOP', 'LEAVE', 'RETURN'):
            b = QtWidgets.QPushButton(a.title())
            b.clicked.connect(lambda _, a=a: ros.call(ros.act, ActorCommand.Request(actor=self.actor.currentText(), command=a), 'ACTOR'))
            arow.addWidget(b)
        aw = QtWidgets.QWidget()
        aw.setLayout(arow)
        cl.addWidget(aw, 4, 0, 1, 2)
        grid.addWidget(cam, 0, 0, 2, 1)

        # fleet + carousel
        fc = QtWidgets.QGroupBox('Fleet (8 positions) and carousel')
        fl = QtWidgets.QVBoxLayout(fc)
        self.fleet = QtWidgets.QTableWidget(8, 4)
        self.fleet.setHorizontalHeaderLabels(['UAV', 'tray / level', 'state', 'model'])
        self.fleet.verticalHeader().setVisible(False)
        self.fleet.verticalHeader().setDefaultSectionSize(22)
        self.fleet.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        self.fleet.setFixedHeight(8 * 22 + 30)
        fl.addWidget(self.fleet)
        self.fleet_sum = QtWidgets.QLabel('-')
        fl.addWidget(self.fleet_sum)
        self.car = QtWidgets.QLabel('-')
        self.car.setWordWrap(True)
        fl.addWidget(self.car)
        crow = QtWidgets.QHBoxLayout()
        for label, cmd, arg in (('Release UAV01', 'RELEASE', 'uav01'), ('Release UAV02', 'RELEASE', 'uav02'),
                                ('Index to B upper', 'INDEX_TO_B_UPPER', ''), ('Index to A upper', 'INDEX_TO_A_UPPER', ''),
                                ('Stop', 'STOP', '')):
            b = QtWidgets.QPushButton(label)
            b.clicked.connect(lambda _, c=cmd, a=arg: ros.call(ros.car, CarouselCommand.Request(command=c, argument=a), c))
            crow.addWidget(b)
        cw = QtWidgets.QWidget()
        cw.setLayout(crow)
        fl.addWidget(cw)
        grid.addWidget(fc, 0, 1)

        # UAV panels + mission
        side = QtWidgets.QGroupBox('Aircraft and mission')
        sl = QtWidgets.QVBoxLayout(side)
        self.uav_lbl, self.combo, self.land = {}, {}, {}
        for u in UAVS:
            box = QtWidgets.QGroupBox(u.upper())
            bl = QtWidgets.QVBoxLayout(box)
            self.uav_lbl[u] = QtWidgets.QLabel('-')
            self.uav_lbl[u].setWordWrap(True)
            self.uav_lbl[u].setMinimumHeight(64)
            bl.addWidget(self.uav_lbl[u])
            r = QtWidgets.QHBoxLayout()
            self.combo[u] = QtWidgets.QComboBox()
            self.combo[u].addItems([f'obs:{n}' for n in self.obs] + [f'wp:{n}' for n in self.wps])
            go = QtWidgets.QPushButton('Go')
            go.clicked.connect(lambda _, u=u: self.go(u))
            r.addWidget(self.combo[u])
            r.addWidget(go)
            for label, cmd in (('Take off', 'TAKEOFF_HOLD'), ('Hold', 'HOLD')):
                b = QtWidgets.QPushButton(label)
                b.clicked.connect(lambda _, u=u, c=cmd: ros.call(ros.uavc[u], UavCommand.Request(uav_id=u, command=c), c))
                r.addWidget(b)
            # Landing target: ground (here / a pad) is implemented; ugv and restow are rejected with
            # the reason, so the three landing cases stay distinct in the GUI as in the terminal.
            self.land[u] = QtWidgets.QComboBox()
            self.land[u].addItems(['here'] + list(scenario().get('landing_pads', {})) + ['ugv', 'restow'])
            b = QtWidgets.QPushButton('Land')
            b.clicked.connect(lambda _, u=u: ros.call(ros.uavc[u], UavCommand.Request(
                uav_id=u, command='LAND', argument=self.land[u].currentText()), 'LAND'))
            r.addWidget(self.land[u])
            r.addWidget(b)
            w = QtWidgets.QWidget()
            w.setLayout(r)
            bl.addWidget(w)
            sl.addWidget(box)
        mrow = QtWidgets.QHBoxLayout()
        self.m1, self.m2 = QtWidgets.QComboBox(), QtWidgets.QComboBox()
        self.m1.addItems(self.obs)
        self.m2.addItems(self.obs)
        if len(self.obs) > 1:
            self.m2.setCurrentIndex(1)
        self.mtrack = QtWidgets.QSpinBox()
        self.mtrack.setRange(-1, 9999)
        self.mtrack.setValue(-1)
        bs = QtWidgets.QPushButton('Start mission')
        bs.clicked.connect(lambda: ros.start_mission(self.m1.currentText(), self.m2.currentText(), self.mtrack.value()))
        bc = QtWidgets.QPushButton('Cancel mission')
        bc.clicked.connect(lambda: ros.call(ros.cancel, CancelGoal.Request(), 'CANCEL'))
        for w in (QtWidgets.QLabel('UAV01'), self.m1, QtWidgets.QLabel('UAV02'), self.m2, QtWidgets.QLabel('track'), self.mtrack, bs, bc):
            mrow.addWidget(w)
        mw = QtWidgets.QWidget()
        mw.setLayout(mrow)
        sl.addWidget(mw)
        self.mlabel = QtWidgets.QLabel('mission: -')
        self.mlabel.setWordWrap(True)
        self.mprog = QtWidgets.QProgressBar()
        sl.addWidget(self.mprog)
        sl.addWidget(self.mlabel)
        grid.addWidget(side, 1, 1)

        self.log = QtWidgets.QTableWidget(0, 6)
        self.log.setHorizontalHeaderLabels(['time', 'target', 'command', 'argument', 'state', 'detail'])
        self.log.horizontalHeader().setStretchLastSection(True)
        self.log.setFixedHeight(150)
        grid.addWidget(self.log, 2, 0, 1, 2)
        self.log_seen = 0
        t = QtCore.QTimer(self)
        t.timeout.connect(self.refresh)
        t.start(100)

    def go(self, u):
        kind, name = self.combo[u].currentText().split(':', 1)
        cmd = 'GOTO_OBSERVATION' if kind == 'obs' else 'GOTO_WAYPOINT'
        self.ros.call(self.ros.uavc[u], UavCommand.Request(uav_id=u, command=cmd, argument=name), cmd)

    def refresh(self):
        d = self.ros.data
        with self.ros.lock:
            img, inset, fleet, car, trk = d['img'], d['inset'], d['fleet'], d['carousel'], d['track']
        q = to_qimage(img)
        if q is not None:
            self.img.setPixmap(QtGui.QPixmap.fromImage(q).scaled(560, 420, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        q = to_qimage(inset)
        if q is not None:
            self.inset.setPixmap(QtGui.QPixmap.fromImage(q).scaled(280, 280, QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation))
        if trk is not None:
            lost = trk.state == 'LOST'
            self.trk.setText(f"<b style='color:{'#c0392b' if lost else '#2e8b57'}'>{'TRACK LOST' if lost else trk.state}</b>"
                             f'<br>selected ID {trk.selected_id}<br>visible IDs {list(trk.visible_person_ids)}'
                             f'<br>centre error ({trk.center_error_px_x:.0f}, {trk.center_error_px_y:.0f}) px'
                             f'<br>selection to centred: {trk.selection_to_centered_s:.2f} s')
        if fleet is not None:
            for i, s in enumerate(fleet.slots):
                for j, v in enumerate((s.uav_id.upper(), f'{s.tray} / {s.level}', s.state,
                                       'PX4 aircraft' if s.independent_model else 'stowed visual')):
                    self.fleet.setItem(i, j, QtWidgets.QTableWidgetItem(v))
            self.fleet_sum.setText(f'stowed {fleet.stowed}   active docked {fleet.active_docked}   airborne {fleet.airborne}')
        if car is not None:
            mm = ', '.join(f'{n.replace("_joint", "")} {v * 1000:+.1f} mm' for n, v in zip(car.joint_names, car.measured_position_m))
            self.car.setText(f'<b>{car.station}</b>  activity {car.activity}  waypoint {car.waypoint_index}->{car.target_index}'
                             f'  fresh {car.measured_fresh}<br>measured: {mm}<br>latched: {list(car.latched)}  accessible: {list(car.accessible_docks)}')
        for u in UAVS:
            s = d['uav'].get(u)
            if s is None:
                continue
            p = s.position_enu
            color = '#2e8b57' if s.connected else '#c0392b'
            self.uav_lbl[u].setText(
                f"<b>{s.phase}</b> owner {s.control_owner}  armed {s.armed}  nav {s.nav_state}"
                f"  <span style='color:{color}'>telemetry age {s.telemetry_age_s:.2f} s</span><br>"
                f'pos ({p.x:.1f}, {p.y:.1f}, {p.z:.1f})  target {s.target_name or "-"}  error {s.hold_error_m:.2f} m<br>'
                f'last command: {s.last_command or "-"} <b>{s.last_command_state}</b>')
        phase, prog, detail = d['mission']
        self.mprog.setValue(int(prog * 100))
        self.mlabel.setText(f'mission phase {phase}: {detail}<br>{d["mission_result"]}')
        logs = d['log']
        for m in logs[self.log_seen:]:
            r = self.log.rowCount()
            self.log.insertRow(r)
            for j, v in enumerate((f'{m.stamp.sec}.{m.stamp.nanosec // 10**8}', m.target, m.command, m.argument, m.state, m.detail)):
                it = QtWidgets.QTableWidgetItem(v)
                if j == 4:
                    it.setForeground(QtGui.QColor(STATE_COLOR.get(m.state, '#000')))
                self.log.setItem(r, j, it)
            self.log.scrollToBottom()
        self.log_seen = len(logs)


def main():
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    ros = RosSide()
    ex = MultiThreadedExecutor(num_threads=2)
    ex.add_node(ros)
    threading.Thread(target=ex.spin, daemon=True).start()
    app = QtWidgets.QApplication(sys.argv)
    # Ctrl+C in the tab and SIGTERM from scripts/stop_all.sh close the window cleanly.  The timer
    # gives the Python interpreter a chance to run the handler while Qt's event loop is running.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    tick = QtCore.QTimer()
    tick.timeout.connect(lambda: None)
    tick.start(200)
    w = Window(ros)
    avail = app.primaryScreen().availableGeometry()
    w.resize(min(1560, avail.width() - 40), min(1000, avail.height() - 60))
    w.show()
    rc = app.exec_()
    ex.shutdown()
    rclpy.try_shutdown()
    return rc


if __name__ == '__main__':
    sys.exit(main())
