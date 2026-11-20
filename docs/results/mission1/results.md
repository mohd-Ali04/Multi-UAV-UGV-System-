# Run analysis: mission1

Estimated = PX4 estimate used by the controllers. Ground truth (GT) = Gazebo pose, used here only for evaluation.

## Hold at observation points

| UAV | target | samples | est. error mean / max (m) | GT error mean / max (m) | telemetry age max (s) |
|---|---|---|---|---|---|
| uav01 | op1 | 2966 | 0.02 / 0.18 | 0.06 / 0.26 | 0.33 |
| uav02 | op2, wp_bus_stop | 4904 | 0.02 / 0.19 | 0.15 / 3.99 | 0.33 |

## Inset tracking test (uav01 holding at its observation point)

- selected YOLOX IDs during the test: ['45', '53']
- selection to first centred inset: 0.25 s
- TRACKING samples where the box matches person_12 by GT projection: 872/872 (100.0 %)
- TRACKING samples matching another pedestrian: 0
- inset centring error while tracking: median 0.5 px, 95th pct 9.5 px (inset pixels)
- LOST samples: 3999; selected ID while LOST: ['45', '53']
- uav01 commanded hold position drift during the test: 3.297 m

## Command log (terminal and GUI share it)

| t (s) | target | command | argument | state | detail |
|---|---|---|---|---|---|
| 35.2 | mission | RUN |  | ACCEPTED | uav01->op1 uav02->op2 track=-1 |
| 35.2 | mission | CHECK |  | ACCEPTED | waiting for measured carousel state and PX4 telemetry |
| 35.2 | mission | RELEASE_1 |  | ACCEPTED | releasing uav01 |
| 35.2 | carousel | RELEASE | uav01 | REQUESTED |  |
| 35.2 | carousel | RELEASE | uav01 | ACCEPTED | detach sent; waiting for measured latch state |
| 35.3 | carousel | RELEASE | uav01 | COMPLETED | latch state measured detached |
| 35.3 | mission | FLY_1 |  | ACCEPTED | uav01 to op1 |
| 35.3 | uav01 | GOTO_OBSERVATION | op1 | REQUESTED |  |
| 35.4 | uav01 | GOTO_OBSERVATION | op1 | ACCEPTED | flying to op1 (-12.0, -19.0, 8.0) |
| 51.7 | uav01 | GOTO_OBSERVATION | op1 | COMPLETED | arrived at op1, error 0.20 m; holding |
| 51.9 | mission | INDEX_B |  | ACCEPTED | guarded exchange to B_UPPER |
| 51.9 | carousel | INDEX_TO_B_UPPER |  | REQUESTED |  |
| 51.9 | carousel | INDEX_TO_B_UPPER |  | ACCEPTED | guarded route through waypoints 1..12 |
| 101.5 | carousel | INDEX |  | COMPLETED | measured station B_UPPER |
| 101.6 | mission | RELEASE_2 |  | ACCEPTED | releasing uav02 |
| 101.6 | carousel | RELEASE | uav02 | REQUESTED |  |
| 101.6 | carousel | RELEASE | uav02 | ACCEPTED | detach sent; waiting for measured latch state |
| 101.7 | carousel | RELEASE | uav02 | COMPLETED | latch state measured detached |
| 101.7 | mission | FLY_2 |  | ACCEPTED | uav02 to op2 |
| 101.7 | uav02 | GOTO_OBSERVATION | op2 | REQUESTED |  |
| 101.8 | uav02 | GOTO_OBSERVATION | op2 | ACCEPTED | flying to op2 (16.0, -18.0, 10.0) |
| 127.0 | uav02 | GOTO_OBSERVATION | op2 | COMPLETED | arrived at op2, error 0.21 m; holding |
| 127.2 | mission | DONE |  | ACCEPTED | both aircraft holding at their observation points |
| 127.3 | mission | RUN |  | COMPLETED | uav01 HOLD at op1 (err 0.03 m); uav02 HOLD at op2 (err 0.19 m) |
| 146.2 | tracker | SELECT | 45 | REQUESTED |  |
| 146.2 | tracker | SELECT | 45 | ACCEPTED | selected; visible now |
| 146.5 | tracker | SELECT | 45 | COMPLETED | inset centred 0.25 s after selection |
| 151.4 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 151.7 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 171.1 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 171.5 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 178.5 | uav02 | GOTO_WAYPOINT | wp_bus_stop | REQUESTED |  |
| 178.6 | uav02 | GOTO_WAYPOINT | wp_bus_stop | ACCEPTED | flying to wp_bus_stop (16.0, -32.0, 7.0) |
| 186.5 | uav02 | GOTO_WAYPOINT | wp_bus_stop | COMPLETED | arrived at wp_bus_stop, error 0.16 m; holding |
| 191.1 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 191.4 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 194.4 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 194.7 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 201.7 | uav02 | GOTO_OBSERVATION | op2 | REQUESTED |  |
| 201.7 | uav02 | GOTO_OBSERVATION | op2 | ACCEPTED | flying to op2 (16.0, -18.0, 10.0) |
| 202.2 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 202.4 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 209.7 | uav02 | GOTO_OBSERVATION | op2 | COMPLETED | arrived at op2, error 0.13 m; holding |
| 211.2 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 211.6 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 219.6 | scenario | LEAVE | person_12 | REQUESTED |  |
| 219.6 | scenario | LEAVE | person_12 | COMPLETED | leaving view along [(-2.0, -30.8), (14.0, -30.8), (26.0, -30.8)] |
| 225.7 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 226.4 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 232.8 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 232.9 | tracker | TRACK | 45 | COMPLETED | selected ID re-detected |
| 233.9 | tracker | TRACK | 45 | FAILED | TRACK LOST: selected ID not detected |
| 302.6 | scenario | RETURN | person_12 | REQUESTED |  |
| 302.6 | scenario | RETURN | person_12 | COMPLETED | returning to its loop |
| 347.3 | tracker | SELECT | 53 | REQUESTED |  |
| 347.3 | tracker | SELECT | 45 | CANCELED | superseded by a new selection |
| 347.3 | tracker | SELECT | 53 | ACCEPTED | selected; visible now |
| 347.4 | tracker | SELECT | 53 | COMPLETED | inset centred 0.02 s after selection |
| 348.7 | uav01/pursuit | ENGAGE | 53 | REQUESTED |  |
| 348.7 | uav01/pursuit | ENGAGE | 53 | ACCEPTED | pursuing track 53 |
| 350.1 | uav01/pursuit | TRACK | 53 | FAILED | selected ID lost > 0.5 s: hover in place |
| 350.1 | tracker | TRACK | 53 | FAILED | TRACK LOST: selected ID not detected |
| 365.1 | uav01/pursuit | ENGAGE | 53 | FAILED | target not reacquired; position hold |

## Per-aircraft command isolation

| t (s) | commanded | command | other aircraft | other unchanged for 5 s | other state |
|---|---|---|---|---|---|
| 35.4 | uav01 | GOTO_OBSERVATION op1 | uav02 | yes | DOCKED  |
| 101.8 | uav02 | GOTO_OBSERVATION op2 | uav01 | yes | HOLD op1 |
| 178.6 | uav02 | GOTO_WAYPOINT wp_bus_stop | uav01 | yes | HOLD op1 |
| 201.7 | uav02 | GOTO_OBSERVATION op2 | uav01 | yes | HOLD op1 |

## Pursuit uav01-track53-20261120-141837.json

From the pursuit log (estimated quantities, what the controller saw):

| metric | value |
|---|---|
| track_id | 53 |
| outcome | FAILED |
| time_to_intercept_s | None |
| terminal_range_est_m | None |
| peak_closing_speed_est_mps | 0.826 |
| peak_ground_speed_mps | 2.251 |
| mean_abs_err_u_px | 161.100 |
| p95_abs_err_u_px | 165.312 |
| fov_containment | 1.000 |
| min_alt_m | 6.546 |
| max_pitch_deg | 12.604 |
| lost_events | 1 |
| duration_s | 16.392 |

Ground truth (Gazebo poses, evaluation only), separation uav01 to person_12:

| metric | value |
|---|---|
| initial horizontal separation | 17.73 m |
| time to close to 5 m / 3 m / 2 m | not reached / not reached / not reached |
| minimum horizontal separation | 12.53 m |
| mean separation after first reaching 3 m | not reached |
| minimum altitude (GT) | 6.58 m |
