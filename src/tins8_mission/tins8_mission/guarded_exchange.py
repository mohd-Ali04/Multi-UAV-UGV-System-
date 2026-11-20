"""Guarded slide-exchange sequencing for the TinS-12E two-tray carousel (pure logic).

Source register: tins_description/config/guarded_exchange_waypoints.yaml (26 states,
units m).  Rules carried over from the register:
  * Follow the guarded segments in order; independent joint limits alone admit collision.
  * Advance to the next waypoint only when the MEASURED joint positions reach the current
    one within tolerance and the measurement is fresh.  Elapsed time is never completion
    evidence.
  * Only upper, stationary trays offer accessible docks.
"""
from dataclasses import dataclass

import yaml

JOINTS = ('tray_a_x_joint', 'tray_a_z_joint', 'tray_b_x_joint', 'tray_b_z_joint')
A_UPPER_INDEX = 0
B_UPPER_INDEX = 12
A_RETURN_INDEX = 24


@dataclass
class Waypoint:
    index: int
    name: str
    q: tuple
    accessible: tuple


def load_waypoints(path):
    d = yaml.safe_load(open(path))
    out = []
    for w in d['waypoints']:
        q = tuple(float(w['joint_positions_m'][j]) for j in JOINTS)
        out.append(Waypoint(int(w['index']), w['name'], q, tuple(w.get('accessible_stations') or ())))
    return out


def route(start, goal):
    """Waypoint indices to visit.  A_UPPER -> B_UPPER is 0..12, B_UPPER -> A_UPPER is 12..24."""
    if start == goal:
        return []
    if start == A_UPPER_INDEX and goal == B_UPPER_INDEX:
        return list(range(1, 13))
    if start == B_UPPER_INDEX and goal == A_UPPER_INDEX:
        return list(range(13, 25))
    raise ValueError(f'no guarded route from {start} to {goal}')


def reached(measured, target, tol):
    return all(abs(m - t) <= tol for m, t in zip(measured, target))


def station_of(measured, waypoints, tol):
    """Which accessible station (if any) the measured joints are at."""
    if reached(measured, waypoints[A_UPPER_INDEX].q, tol):
        return 'A_UPPER', A_UPPER_INDEX
    if reached(measured, waypoints[B_UPPER_INDEX].q, tol):
        return 'B_UPPER', B_UPPER_INDEX
    return 'IN_TRANSIT', -1


class Sequencer:
    """Steps through a route; the caller feeds measured positions and sends the target."""

    def __init__(self, waypoints, tol=0.0015):
        self.w = waypoints
        self.tol = tol
        self.queue = []
        self.current = None

    def start(self, indices):
        self.queue = list(indices)
        self.current = self.queue.pop(0) if self.queue else None
        return self.target()

    def target(self):
        return None if self.current is None else self.w[self.current].q

    def update(self, measured, fresh):
        """Returns (target, reached_index_or_None, done)."""
        if self.current is None:
            return None, None, True
        if not fresh or not reached(measured, self.w[self.current].q, self.tol):
            return self.target(), None, False
        done_index = self.current
        self.current = self.queue.pop(0) if self.queue else None
        return self.target(), done_index, self.current is None

    def stop(self):
        self.queue, self.current = [], None
