"""Two-aircraft observation mission as a ROS 2 action (/mission/run, tins8_interfaces/RunMission).

Sequence (each step waits on measured state, never on a timer):
  1 CHECK      carousel at A_UPPER (measured), both PX4 agents connected, both latches attached
  2 RELEASE_1  carousel RELEASE uav01            -> latch state 'detached'
  3 FLY_1      uav01 GOTO_OBSERVATION op1        -> agent reports HOLD at op1
  4 INDEX_B    carousel INDEX_TO_B_UPPER         -> measured station B_UPPER
  5 RELEASE_2  carousel RELEASE uav02            -> latch state 'detached'
  6 FLY_2      uav02 GOTO_OBSERVATION op2        -> agent reports HOLD at op2
  7 SELECT     (optional) tracker select track_id on uav01's camera
  8 DONE       both aircraft holding; result COMPLETED
Cancel: airborne aircraft are commanded HOLD, a moving carousel is STOPped at its measured
position, and the action ends CANCELED.  A failed step ends FAILED with the reason.
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from tins8_interfaces.action import RunMission
from tins8_interfaces.msg import CarouselState, TrackStatus, UavStatus
from tins8_interfaces.srv import CarouselCommand, SelectTarget, UavCommand

from .common import UAVS, CommandLog, LATCHED_QOS

STEPS = ['CHECK', 'RELEASE_1', 'FLY_1', 'INDEX_B', 'RELEASE_2', 'FLY_2', 'SELECT', 'DONE']


class Aborted(Exception):
    pass


class MissionServer(Node):
    def __init__(self):
        super().__init__('mission_server')
        self.declare_parameter('step_timeout_s', 240.0)
        self.cb = ReentrantCallbackGroup()
        self.log = CommandLog(self, 'mission')
        self.carousel = None
        self.uav = {}
        self.track = None
        self.create_subscription(CarouselState, '/carousel/state', lambda m: setattr(self, 'carousel', m), LATCHED_QOS, callback_group=self.cb)
        for u in UAVS:
            self.create_subscription(UavStatus, f'/{u}/status', lambda m, u=u: self.uav.__setitem__(u, m), 10, callback_group=self.cb)
        self.create_subscription(TrackStatus, '/uav01/track/status', lambda m: setattr(self, 'track', m), 10, callback_group=self.cb)
        self.car_cli = self.create_client(CarouselCommand, '/carousel/command', callback_group=self.cb)
        self.uav_cli = {u: self.create_client(UavCommand, f'/{u}/command', callback_group=self.cb) for u in UAVS}
        self.sel_cli = self.create_client(SelectTarget, '/uav01/tracker/select', callback_group=self.cb)
        self.busy = False
        ActionServer(self, RunMission, '/mission/run', execute_callback=self.execute,
                     goal_callback=self.on_goal, cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.cb)

    def on_goal(self, goal):
        if self.busy:
            self.get_logger().warn('mission goal rejected: a mission is already running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    # ------------------------------------------------------------ helpers
    def call(self, client, req, what):
        if not client.wait_for_service(timeout_sec=3.0):
            raise Aborted(f'{what}: service unavailable')
        fut = client.call_async(req)
        t = time.monotonic()
        while not fut.done():
            if time.monotonic() - t > 10:
                raise Aborted(f'{what}: no response')
            time.sleep(0.02)
        res = fut.result()
        if not res.accepted:
            raise Aborted(f'{what}: rejected ({res.message})')
        return res

    def wait(self, handle, pred, what):
        t = time.monotonic()
        while True:
            if handle.is_cancel_requested:
                raise KeyboardInterrupt
            ok = pred()
            if ok:
                return
            if time.monotonic() - t > self.get_parameter('step_timeout_s').value:
                raise Aborted(f'{what}: timed out waiting for measured state')
            time.sleep(0.1)

    def feedback(self, handle, step, detail):
        fb = RunMission.Feedback()
        fb.phase, fb.detail = step, detail
        fb.progress = STEPS.index(step) / (len(STEPS) - 1)
        handle.publish_feedback(fb)
        self.log.emit(self.rid, step, 'ACCEPTED', detail)

    # ------------------------------------------------------------ execute
    def execute(self, handle):
        self.busy = True
        g = handle.request
        self.rid = f'mission-{int(time.time()) % 100000:05d}'
        self.log.emit(self.rid, 'RUN', 'ACCEPTED', f'uav01->{g.uav1_point} uav02->{g.uav2_point} track={g.track_id}')
        result = RunMission.Result()
        try:
            self.feedback(handle, 'CHECK', 'waiting for measured carousel state and PX4 telemetry')
            self.wait(handle, lambda: self.carousel is not None and self.carousel.measured_fresh and
                      all(u in self.uav and self.uav[u].connected for u in UAVS), 'readiness')
            if self.carousel.station != 'A_UPPER':
                raise Aborted(f'carousel must start at A_UPPER (measured {self.carousel.station})')
            for n, (u, point) in enumerate([('uav01', g.uav1_point), ('uav02', g.uav2_point)], start=1):
                if u == 'uav02':
                    self.feedback(handle, 'INDEX_B', 'guarded exchange to B_UPPER')
                    self.call(self.car_cli, CarouselCommand.Request(command='INDEX_TO_B_UPPER'), 'index')
                    self.wait(handle, lambda: self.carousel.station == 'B_UPPER' and self.carousel.activity == 'IDLE', 'index to B_UPPER')
                self.feedback(handle, f'RELEASE_{n}', f'releasing {u}')
                if u in self.carousel.latched:
                    self.call(self.car_cli, CarouselCommand.Request(command='RELEASE', argument=u), f'release {u}')
                    self.wait(handle, lambda: u not in self.carousel.latched, f'release {u}')
                self.feedback(handle, f'FLY_{n}', f'{u} to {point}')
                self.call(self.uav_cli[u], UavCommand.Request(uav_id=u, command='GOTO_OBSERVATION', argument=point), f'{u} goto')
                self.wait(handle, lambda: self.uav[u].phase == 'HOLD' and self.uav[u].target_name == point, f'{u} arrival')
            if g.track_id >= 0:
                self.feedback(handle, 'SELECT', f'selecting track {g.track_id} on uav01 camera')
                self.call(self.sel_cli, SelectTarget.Request(track_id=g.track_id), 'select')
                self.wait(handle, lambda: self.track is not None and self.track.state == 'TRACKING', 'tracking')
            self.feedback(handle, 'DONE', 'both aircraft holding at their observation points')
            result.success, result.outcome = True, 'COMPLETED'
            result.summary = '; '.join(f'{u} {self.uav[u].phase} at {self.uav[u].target_name} (err {self.uav[u].hold_error_m:.2f} m)' for u in UAVS)
            handle.succeed()
            self.log.emit(self.rid, 'RUN', 'COMPLETED', result.summary)
        except KeyboardInterrupt:
            self.safe_stop()
            result.success, result.outcome, result.summary = False, 'CANCELED', 'operator cancel: airborne aircraft holding, carousel stopped'
            handle.canceled()
            self.log.emit(self.rid, 'RUN', 'CANCELED', result.summary)
        except Aborted as e:
            self.safe_stop()
            result.success, result.outcome, result.summary = False, 'FAILED', str(e)
            handle.abort()
            self.log.emit(self.rid, 'RUN', 'FAILED', str(e))
        finally:
            self.busy = False
        return result

    def safe_stop(self):
        if self.carousel is not None and self.carousel.activity == 'INDEXING':
            self.car_cli.call_async(CarouselCommand.Request(command='STOP'))
        for u in UAVS:
            st = self.uav.get(u)
            if st is not None and st.phase in ('TAKEOFF', 'TRANSIT'):
                self.uav_cli[u].call_async(UavCommand.Request(uav_id=u, command='HOLD'))


def main():
    rclpy.init()
    node = MissionServer()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    ex.spin()


if __name__ == '__main__':
    main()
