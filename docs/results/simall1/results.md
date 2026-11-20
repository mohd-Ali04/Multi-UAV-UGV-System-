# Run analysis: simall1

Estimated = PX4 estimate used by the controllers. Ground truth (GT) = Gazebo pose, used here only for evaluation.

## Hold at observation points

| UAV | target | samples | est. error mean / max (m) | GT error mean / max (m) | telemetry age max (s) |
|---|---|---|---|---|---|
| uav01 | op1 | 949 | 0.03 / 0.17 | 0.09 / 5.12 | 4.28 |
| uav02 | op2 | 211 | 0.05 / 0.21 | 3.76 / 18.90 | 8.59 |

## Command log (terminal and GUI share it)

| t (s) | target | command | argument | state | detail |
|---|---|---|---|---|---|
| 8.7 | mission | RUN |  | ACCEPTED | uav01->op1 uav02->op2 track=-1 |
| 8.7 | mission | CHECK |  | ACCEPTED | waiting for measured carousel state and PX4 telemetry |
| 8.7 | mission | RELEASE_1 |  | ACCEPTED | releasing uav01 |
| 8.7 | carousel | RELEASE | uav01 | REQUESTED |  |
| 8.7 | carousel | RELEASE | uav01 | ACCEPTED | detach sent; waiting for measured latch state |
| 8.8 | carousel | RELEASE | uav01 | COMPLETED | latch state measured detached |
| 8.9 | mission | FLY_1 |  | ACCEPTED | uav01 to op1 |
| 8.9 | uav01 | GOTO_OBSERVATION | op1 | REQUESTED |  |
| 8.9 | uav01 | GOTO_OBSERVATION | op1 | ACCEPTED | flying to op1 (-12.0, -19.0, 8.0) |
| 24.3 | uav01 | GOTO_OBSERVATION | op1 | COMPLETED | arrived at op1, error 0.18 m; holding |
| 24.4 | mission | INDEX_B |  | ACCEPTED | guarded exchange to B_UPPER |
| 24.4 | carousel | INDEX_TO_B_UPPER |  | REQUESTED |  |
| 24.4 | carousel | INDEX_TO_B_UPPER |  | ACCEPTED | guarded route through waypoints 1..12 |
| 73.3 | carousel | INDEX |  | COMPLETED | measured station B_UPPER |
| 73.3 | mission | RELEASE_2 |  | ACCEPTED | releasing uav02 |
| 73.3 | carousel | RELEASE | uav02 | REQUESTED |  |
| 73.3 | carousel | RELEASE | uav02 | ACCEPTED | detach sent; waiting for measured latch state |
| 73.4 | carousel | RELEASE | uav02 | COMPLETED | latch state measured detached |
| 73.4 | mission | FLY_2 |  | ACCEPTED | uav02 to op2 |
| 73.4 | uav02 | GOTO_OBSERVATION | op2 | REQUESTED |  |
| 73.5 | uav02 | GOTO_OBSERVATION | op2 | ACCEPTED | flying to op2 (16.0, -18.0, 10.0) |
| 98.1 | uav02 | GOTO_OBSERVATION | op2 | COMPLETED | arrived at op2, error 0.22 m; holding |
| 98.3 | mission | DONE |  | ACCEPTED | both aircraft holding at their observation points |
| 98.3 | mission | RUN |  | COMPLETED | uav01 HOLD at op1 (err 0.01 m); uav02 HOLD at op2 (err 0.21 m) |

## Per-aircraft command isolation

| t (s) | commanded | command | other aircraft | other unchanged for 5 s | other state |
|---|---|---|---|---|---|
| 8.9 | uav01 | GOTO_OBSERVATION op1 | uav02 | yes | DOCKED  |
| 73.5 | uav02 | GOTO_OBSERVATION op2 | uav01 | yes | HOLD op1 |
