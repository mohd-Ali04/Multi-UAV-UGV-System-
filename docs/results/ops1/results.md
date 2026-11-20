# Run analysis: ops1

Estimated = PX4 estimate used by the controllers. Ground truth (GT) = Gazebo pose, used here only for evaluation.

## Hold at observation points

| UAV | target | samples | est. error mean / max (m) | GT error mean / max (m) | telemetry age max (s) |
|---|---|---|---|---|---|
| uav01 | op1 | 1045 | 0.03 / 0.18 | 0.06 / 0.28 | 0.04 |
| uav02 | op2 | 281 | 0.04 / 0.23 | 0.11 / 0.32 | 0.02 |

## Command log (terminal and GUI share it)

| t (s) | target | command | argument | state | detail |
|---|---|---|---|---|---|
| 17.3 | mission | RUN |  | ACCEPTED | uav01->op1 uav02->op2 track=-1 |
| 17.4 | mission | CHECK |  | ACCEPTED | waiting for measured carousel state and PX4 telemetry |
| 17.4 | mission | RELEASE_1 |  | ACCEPTED | releasing uav01 |
| 17.4 | carousel | RELEASE | uav01 | REQUESTED |  |
| 17.4 | carousel | RELEASE | uav01 | ACCEPTED | detach sent; waiting for measured latch state |
| 17.4 | carousel | RELEASE | uav01 | COMPLETED | latch state measured detached |
| 17.5 | mission | FLY_1 |  | ACCEPTED | uav01 to op1 |
| 17.5 | uav01 | GOTO_OBSERVATION | op1 | REQUESTED |  |
| 17.5 | uav01 | GOTO_OBSERVATION | op1 | ACCEPTED | flying to op1 (-12.0, -19.0, 8.0) |
| 32.9 | uav01 | GOTO_OBSERVATION | op1 | COMPLETED | arrived at op1, error 0.20 m; holding |
| 33.0 | mission | INDEX_B |  | ACCEPTED | guarded exchange to B_UPPER |
| 33.0 | carousel | INDEX_TO_B_UPPER |  | REQUESTED |  |
| 33.0 | carousel | INDEX_TO_B_UPPER |  | ACCEPTED | guarded route through waypoints 1..12 |
| 44.4 | mission | RUN |  | CANCELED | operator cancel: airborne aircraft holding, carousel stopped |
| 44.4 | carousel | STOP |  | REQUESTED |  |
| 44.4 | carousel | INDEX |  | CANCELED | stopped by operator at measured position |
| 44.4 | carousel | STOP |  | ACCEPTED | holding measured position |
| 62.9 | carousel | INDEX_TO_B_UPPER |  | REQUESTED |  |
| 62.9 | carousel | INDEX_TO_B_UPPER |  | ACCEPTED | guarded route through waypoints 3..12 |
| 100.5 | carousel | INDEX |  | COMPLETED | measured station B_UPPER |
| 104.7 | carousel | RELEASE | uav02 | REQUESTED |  |
| 104.7 | carousel | RELEASE | uav02 | ACCEPTED | detach sent; waiting for measured latch state |
| 104.7 | carousel | RELEASE | uav02 | COMPLETED | latch state measured detached |
| 107.1 | uav02 | GOTO_OBSERVATION | op2 | REQUESTED |  |
| 107.2 | uav02 | GOTO_OBSERVATION | op2 | ACCEPTED | flying to op2 (16.0, -18.0, 10.0) |
| 131.7 | uav02 | GOTO_OBSERVATION | op2 | COMPLETED | arrived at op2, error 0.23 m; holding |
| 133.9 | uav01 | LAND | ugv | REQUESTED |  |
| 133.9 | uav01 | LAND | ugv | REJECTED | landing on the UGV is not implemented in this build: it needs precision landing on the dec |
| 136.7 | uav01 | LAND | restow | REQUESTED |  |
| 136.7 | uav01 | LAND | restow | REJECTED | re-stowage into the carousel is not implemented in this build: it needs dock alignment, re |
| 137.7 | uav01 | LAND | pad_west | REQUESTED |  |
| 137.7 | uav01 | LAND | pad_west | ACCEPTED | flying to ground pad pad_west (-22.0, -14.0), then PX4 AUTO_LAND |
| 158.9 | uav01 | LAND | pad_west | COMPLETED | disarmed on the ground after AUTO_LAND |
| 160.1 | uav02 | LAND |  | REQUESTED |  |
| 160.1 | uav02 | LAND |  | ACCEPTED | PX4 AUTO_LAND on the ground below (ground case; not the UGV) |
| 175.6 | uav02 | LAND |  | COMPLETED | disarmed on the ground after AUTO_LAND |

## Per-aircraft command isolation

| t (s) | commanded | command | other aircraft | other unchanged for 5 s | other state |
|---|---|---|---|---|---|
| 17.5 | uav01 | GOTO_OBSERVATION op1 | uav02 | yes | DOCKED  |
| 107.2 | uav02 | GOTO_OBSERVATION op2 | uav01 | yes | HOLD op1 |
| 137.7 | uav01 | LAND pad_west | uav02 | yes | HOLD op2 |
| 160.1 | uav02 | LAND  | uav01 | yes | LANDED pad_west |
