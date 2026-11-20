# Run analysis: pursuit7

Estimated = PX4 estimate used by the controllers. Ground truth (GT) = Gazebo pose, used here only for evaluation.

## Hold at observation points

| UAV | target | samples | est. error mean / max (m) | GT error mean / max (m) | telemetry age max (s) |
|---|---|---|---|---|---|
| uav01 | op1 | 868 | 0.03 / 0.17 | 0.06 / 0.27 | 0.02 |
| uav02 | op2 | 1978 | 0.03 / 0.25 | 0.14 / 0.37 | 0.17 |

## Inset tracking test (uav01 holding at its observation point)

- selected YOLOX IDs during the test: ['8']
- selection to first centred inset: 0.28 s
- TRACKING samples where the box matches person_12 by GT projection: 876/876 (100.0 %)
- TRACKING samples matching another pedestrian: 0
- inset centring error while tracking: median 4.2 px, 95th pct 38.6 px (inset pixels)
- LOST samples: 1027; selected ID while LOST: ['8']
- uav01 commanded hold position drift during the test: 17.039 m

## Command log (terminal and GUI share it)

| t (s) | target | command | argument | state | detail |
|---|---|---|---|---|---|
| 13.9 | mission | RUN |  | ACCEPTED | uav01->op1 uav02->op2 track=-1 |
| 13.9 | mission | CHECK |  | ACCEPTED | waiting for measured carousel state and PX4 telemetry |
| 13.9 | mission | RELEASE_1 |  | ACCEPTED | releasing uav01 |
| 13.9 | carousel | RELEASE | uav01 | REQUESTED |  |
| 13.9 | carousel | RELEASE | uav01 | ACCEPTED | detach sent; waiting for measured latch state |
| 13.9 | carousel | RELEASE | uav01 | COMPLETED | latch state measured detached |
| 14.0 | mission | FLY_1 |  | ACCEPTED | uav01 to op1 |
| 14.0 | uav01 | GOTO_OBSERVATION | op1 | REQUESTED |  |
| 14.1 | uav01 | GOTO_OBSERVATION | op1 | ACCEPTED | flying to op1 (-12.0, -19.0, 8.0) |
| 31.5 | uav01 | GOTO_OBSERVATION | op1 | COMPLETED | arrived at op1, error 0.18 m; holding |
| 31.6 | mission | INDEX_B |  | ACCEPTED | guarded exchange to B_UPPER |
| 31.6 | carousel | INDEX_TO_B_UPPER |  | REQUESTED |  |
| 31.6 | carousel | INDEX_TO_B_UPPER |  | ACCEPTED | guarded route through waypoints 1..12 |
| 80.5 | carousel | INDEX |  | COMPLETED | measured station B_UPPER |
| 80.5 | mission | RELEASE_2 |  | ACCEPTED | releasing uav02 |
| 80.5 | carousel | RELEASE | uav02 | REQUESTED |  |
| 80.5 | carousel | RELEASE | uav02 | ACCEPTED | detach sent; waiting for measured latch state |
| 80.5 | carousel | RELEASE | uav02 | COMPLETED | latch state measured detached |
| 80.6 | mission | FLY_2 |  | ACCEPTED | uav02 to op2 |
| 80.6 | uav02 | GOTO_OBSERVATION | op2 | REQUESTED |  |
| 80.7 | uav02 | GOTO_OBSERVATION | op2 | ACCEPTED | flying to op2 (16.0, -18.0, 10.0) |
| 105.6 | uav02 | GOTO_OBSERVATION | op2 | COMPLETED | arrived at op2, error 0.25 m; holding |
| 105.7 | mission | DONE |  | ACCEPTED | both aircraft holding at their observation points |
| 105.7 | mission | RUN |  | COMPLETED | uav01 HOLD at op1 (err 0.03 m); uav02 HOLD at op2 (err 0.25 m) |
| 112.8 | tracker | SELECT | 8 | REQUESTED |  |
| 112.9 | tracker | SELECT | 8 | ACCEPTED | selected; visible now |
| 113.1 | tracker | SELECT | 8 | COMPLETED | inset centred 0.28 s after selection |
| 118.2 | uav01/pursuit | ENGAGE | 8 | REQUESTED |  |
| 118.3 | uav01/pursuit | ENGAGE | 8 | ACCEPTED | pursuing track 8 |
| 143.1 | uav01/pursuit | ENGAGE | 8 | COMPLETED | standoff reached in 24.86 s (est. range 1.42 m); terminal hover |
| 195.9 | uav01/pursuit | DISENGAGE | -1 | REQUESTED |  |
| 195.9 | uav01/pursuit | DISENGAGE | -1 | ACCEPTED | stopping pursuit; hover then position hold |
| 195.9 | uav01/pursuit | ENGAGE | 8 | CANCELED | stopped by operator |
| 200.8 | tracker | TRACK | 8 | FAILED | TRACK LOST: selected ID not detected |

## Per-aircraft command isolation

| t (s) | commanded | command | other aircraft | other unchanged for 5 s | other state |
|---|---|---|---|---|---|
| 14.1 | uav01 | GOTO_OBSERVATION op1 | uav02 | yes | DOCKED  |
| 80.7 | uav02 | GOTO_OBSERVATION op2 | uav01 | yes | HOLD op1 |

## Pursuit uav01-track8-20261120-145307.json

From the pursuit log (estimated quantities, what the controller saw):

| metric | value |
|---|---|
| track_id | 8 |
| outcome | CANCELED |
| time_to_intercept_s | 24.864 |
| terminal_range_est_m | 1.418 |
| peak_closing_speed_est_mps | 4.928 |
| peak_ground_speed_mps | 4.137 |
| mean_abs_err_u_px | 79.738 |
| p95_abs_err_u_px | 203.472 |
| fov_containment | 0.993 |
| min_alt_m | 1.456 |
| max_pitch_deg | 22.972 |
| lost_events | 0 |
| duration_s | 77.724 |

Ground truth (Gazebo poses, evaluation only), separation uav01 to person_12:

| metric | value |
|---|---|
| initial horizontal separation | 15.06 m |
| time to close to 5 m / 3 m / 2 m | 3.30 s / 9.40 s / 14.29 s |
| minimum horizontal separation | 1.47 m |
| mean separation after first reaching 3 m | 2.33 m |
| minimum altitude (GT) | 1.45 m |
