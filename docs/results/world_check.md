# Fieldlab to Gazebo conversion check

Recorded = seed-42 Fieldlab layout exported from the original generator. Measured = pose reported by the running Gazebo world. Walkers are excluded (they move).

## Object placement (per kind)

| kind | objects | max position error (m) | max yaw error (deg) |
|---|---|---|---|
| building | 25 | 0.000 | 0.00 |
| bus | 3 | 0.000 | 0.00 |
| car | 4 | 0.000 | 0.00 |
| person | 16 | 0.001 | 0.00 |
| truck | 3 | 0.000 | 0.00 |

Objects missing in Gazebo: none

## Distance checks

| distance | recorded (m) | measured in Gazebo (m) | difference (m) |
|---|---|---|---|
| UGV start to op1 (horizontal) | 16.28 | 16.28 | 0.000 |
| UGV start to op2 (horizontal) | 44.18 | 44.18 | 0.000 |
| building_0 to building_1 (centres) | 48.00 | 48.00 | 0.000 |
| building_0 to building_5 (centres) | 50.00 | 50.00 | 0.000 |
| building_3 to building_10 (centres) | 183.05 | 183.05 | 0.000 |
| Fieldlab start to stop alpha | 16.00 | 16.00 | 0.000 |
| Fieldlab start to stop bravo | 44.00 | 44.00 | 0.000 |
| Fieldlab start to stop charlie | 63.16 | 63.16 | 0.000 |

Scene size 240 m x 240 m; roads: 8 (width 10 m).

## Per-object detail

| id | kind | recorded x, y | measured x, y | error (m) | yaw error (deg) |
|---|---|---|---|---|---|
| building_0 | building | -96.00, -96.00 | -96.00, -96.00 | 0.000 | 0.00 |
| building_1 | building | -48.00, -96.00 | -48.00, -96.00 | 0.000 | 0.00 |
| building_2 | building | 4.00, -96.00 | 4.00, -96.00 | 0.000 | 0.00 |
| building_3 | building | 56.00, -96.00 | 56.00, -96.00 | 0.000 | 0.00 |
| building_4 | building | 101.00, -96.00 | 101.00, -96.00 | 0.000 | 0.00 |
| building_5 | building | -96.00, -46.00 | -96.00, -46.00 | 0.000 | 0.00 |
| building_6 | building | -48.00, -46.00 | -48.00, -46.00 | 0.000 | 0.00 |
| building_7 | building | 4.00, -46.00 | 4.00, -46.00 | 0.000 | 0.00 |
| building_8 | building | 56.00, -46.00 | 56.00, -46.00 | 0.000 | 0.00 |
| building_9 | building | 101.00, -46.00 | 101.00, -46.00 | 0.000 | 0.00 |
| building_10 | building | -96.00, 6.00 | -96.00, 6.00 | 0.000 | 0.00 |
| building_11 | building | -48.00, 6.00 | -48.00, 6.00 | 0.000 | 0.00 |
| building_12 | building | 4.00, 6.00 | 4.00, 6.00 | 0.000 | 0.00 |
| building_13 | building | 56.00, 6.00 | 56.00, 6.00 | 0.000 | 0.00 |
| building_14 | building | 101.00, 6.00 | 101.00, 6.00 | 0.000 | 0.00 |
| building_15 | building | -96.00, 60.00 | -96.00, 60.00 | 0.000 | 0.00 |
| building_16 | building | -48.00, 60.00 | -48.00, 60.00 | 0.000 | 0.00 |
| building_17 | building | 4.00, 60.00 | 4.00, 60.00 | 0.000 | 0.00 |
| building_18 | building | 56.00, 60.00 | 56.00, 60.00 | 0.000 | 0.00 |
| building_19 | building | 101.00, 60.00 | 101.00, 60.00 | 0.000 | 0.00 |
| building_20 | building | -96.00, 103.00 | -96.00, 103.00 | 0.000 | 0.00 |
| building_21 | building | -48.00, 103.00 | -48.00, 103.00 | 0.000 | 0.00 |
| building_22 | building | 4.00, 103.00 | 4.00, 103.00 | 0.000 | 0.00 |
| building_23 | building | 56.00, 103.00 | 56.00, 103.00 | 0.000 | 0.00 |
| building_24 | building | 101.00, 103.00 | 101.00, 103.00 | 0.000 | 0.00 |
| person_0 | person | -6.78, -15.74 | -6.78, -15.74 | 0.000 | 0.00 |
| person_1 | person | 13.92, -34.24 | 13.92, -34.24 | 0.001 | 0.00 |
| person_2 | person | 23.36, 8.34 | 23.36, 8.34 | 0.000 | 0.00 |
| person_3 | person | -15.76, -33.29 | -15.76, -33.29 | 0.000 | 0.00 |
| person_4 | person | 13.07, -14.51 | 13.07, -14.52 | 0.000 | 0.00 |
| person_5 | person | 19.26, -4.47 | 19.26, -4.47 | 0.000 | 0.00 |
| person_6 | person | -3.83, -29.82 | -3.83, -29.82 | 0.000 | 0.00 |
| person_7 | person | 18.08, -30.22 | 18.08, -30.22 | 0.000 | 0.00 |
| person_8 | person | 22.21, -2.73 | 22.21, -2.73 | 0.000 | 0.00 |
| person_10 | person | 10.12, -32.89 | 10.12, -32.89 | 0.000 | 0.00 |
| person_11 | person | 40.13, -6.58 | 40.13, -6.58 | 0.000 | 0.00 |
| person_13 | person | 20.72, -32.96 | 20.72, -32.96 | 0.000 | 0.00 |
| person_14 | person | 20.76, 8.50 | 20.76, 8.50 | 0.000 | 0.00 |
| person_15 | person | -12.24, -33.37 | -12.24, -33.37 | 0.000 | 0.00 |
| person_16 | person | 6.92, -29.48 | 6.92, -29.48 | 0.000 | 0.00 |
| person_17 | person | 37.40, -0.94 | 37.40, -0.94 | 0.000 | 0.00 |
| vehicle_0 | car | -61.50, -13.00 | -61.50, -13.00 | 0.000 | 0.00 |
| vehicle_1 | truck | -52.00, 44.50 | -52.00, 44.50 | 0.000 | 0.00 |
| vehicle_2 | bus | -52.00, -11.50 | -52.00, -11.50 | 0.000 | 0.00 |
| vehicle_3 | car | 90.50, 39.00 | 90.50, 39.00 | 0.000 | 0.00 |
| vehicle_4 | truck | -104.00, 23.50 | -104.00, 23.50 | 0.000 | 0.00 |
| vehicle_5 | bus | 19.50, -39.00 | 19.50, -39.00 | 0.000 | 0.00 |
| vehicle_6 | car | 69.50, -13.00 | 69.50, -13.00 | 0.000 | 0.00 |
| vehicle_7 | truck | -32.50, -52.00 | -32.50, -52.00 | 0.000 | 0.00 |
| vehicle_8 | bus | 40.50, 65.00 | 40.50, 65.00 | 0.000 | 0.00 |
| vehicle_9 | car | -91.00, -80.50 | -91.00, -80.50 | 0.000 | 0.00 |
