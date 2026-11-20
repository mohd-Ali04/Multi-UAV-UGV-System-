# Run analysis: minimal_flight

Estimated = PX4 estimate used by the controllers. Ground truth (GT) = Gazebo pose, used here only for evaluation.

## Hold at observation points

| UAV | target | samples | est. error mean / max (m) | GT error mean / max (m) | telemetry age max (s) |
|---|---|---|---|---|---|
| uav01 | wp_road_east | 372 | 0.03 / 0.10 | 0.04 / 0.16 | 0.02 |
| uav02 | - | 0 | - | - | - |

## Command log (terminal and GUI share it)

| t (s) | target | command | argument | state | detail |
|---|---|---|---|---|---|
| 33.0 | uav01 | TAKEOFF_HOLD |  | REQUESTED |  |
| 33.0 | uav01 | TAKEOFF_HOLD |  | ACCEPTED | take off and hold at z=3.1 m |
| 41.7 | uav01 | TAKEOFF_HOLD |  | COMPLETED | arrived at takeoff_hold, error 0.17 m; holding |
| 44.5 | uav01 | GOTO_WAYPOINT | wp_road_east | REQUESTED |  |
| 44.5 | uav01 | GOTO_WAYPOINT | wp_road_east | ACCEPTED | flying to wp_road_east (0.0, -22.0, 6.0) |
| 55.1 | uav01 | GOTO_WAYPOINT | wp_road_east | COMPLETED | arrived at wp_road_east, error 0.10 m; holding |
| 92.3 | uav01 | LAND |  | REQUESTED |  |
| 92.3 | uav01 | LAND |  | ACCEPTED | PX4 AUTO_LAND at the current horizontal position (ground, not the UGV) |
| 104.5 | uav01 | LAND |  | COMPLETED | disarmed on the ground after AUTO_LAND |

## Per-aircraft command isolation

| t (s) | commanded | command | other aircraft | other unchanged for 5 s | other state |
|---|---|---|---|---|---|
| 33.0 | uav01 | TAKEOFF_HOLD  | uav02 | yes |   |
| 44.5 | uav01 | GOTO_WAYPOINT wp_road_east | uav02 | yes |   |
| 92.3 | uav01 | LAND  | uav02 | yes |   |
