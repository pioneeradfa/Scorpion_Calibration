# Right legs (RF, RM, RR) - Step 1

Same method as `docs/RF_leg_step1.md`, applied to each right-side leg. Pure math: nothing is sent to the servos.

Files: `right_legs_step1.py` (calculations, `--plot`, `--doc`), `docs/right_legs_diagram.png`, `docs/right_legs_swing.png`.

## Geometry (mm)

Coxa 64.5, femur 64.5, tibia 130.59. Body frame: +X forward, +Y left, +Z up. z is measured from the coxa axis.

| Leg | Coxa axis (x, y) | Calibration coxa / femur / tibia |
|---|---|---|
| RF | (115, -60) | 98 / 88 / 94 |
| RM | (0, -60) | 90 / 92 / 84 |
| RR | (-115, -60) | 103 / 87 / 90 |

## Equations (same for every right leg)

```
A  v = x - mx        u = -(y - my)        z = z
B  yaw = atan2(v, u)                     coxa  = cal_coxa  + yaw[deg]
C  r = hypot(u, v)   L = r - 64.5   d = hypot(L, z)
D  |64.5 - 130.59| <= d <= 195.09        (reachability)
E  alpha = atan2(z, L)   beta = acos((F^2 + d^2 - T^2) / (2 F d))
   femur = cal_femur + (alpha + beta)[deg]
F  knee = acos((F^2 + T^2 - d^2) / (2 F T))        (180 = straight)
G  tibia = cal_tibia + (-1) * (180 - knee[deg])
```

Here mx, my is the coxa axis: RF (115, -60), RM (0, -60), RR (-115, -60).

## RF

| Pose | Raw [coxa, femur, tibia] | Foot x | Foot y | Foot z | Femur elev | Knee | Tibia dir |
|---|---|---|---|---|---|---|---|
| stand | [102, 110, 35] | 130.95 | -288.04 | -54.43 | +22.00° | 121.00° | -37.00° |
| sit | [102, 80, 60] | 130.72 | -284.87 | -96.36 | -8.00° | 146.00° | -42.00° |

IK worked example, stand foot:

| Step | Result |
|---|---|
| A | v = 15.946, u = 228.040, z = -54.429 |
| B | yaw = 4.000° -> coxa = 102.00 |
| C | r = 228.597, L = 164.097, d = 172.888 |
| D | 66.09 <= d <= 195.09: reachable = True |
| E | alpha = -18.350°, beta = 40.350°, elevation = 22.000° -> femur = 110.00 |
| F | knee = 121.000° |
| G | tibia = 35.00 |

Round trip: stand max error 0.0000°, sit max error 0.0000°. Matches `ik_module.py` (diff 0.0e+00).

Dry-run swing (30 mm forward, 20 mm lift, 11 waypoints):

| i | Foot x | Foot z | Coxa | Femur | Tibia | In range |
|---|---|---|---|---|---|---|
| 0 | 130.95 | -54.43 | 102.00 | 110.00 | 35.00 | True |
| 1 | 131.68 | -48.25 | 102.18 | 113.71 | 32.58 | True |
| 2 | 133.81 | -42.67 | 102.72 | 116.78 | 30.84 | True |
| 3 | 137.13 | -38.25 | 103.54 | 118.98 | 29.85 | True |
| 4 | 141.31 | -35.41 | 104.58 | 120.13 | 29.61 | True |
| 5 | 145.95 | -34.43 | 105.73 | 120.15 | 30.08 | True |
| 6 | 150.58 | -35.41 | 106.87 | 119.08 | 31.18 | True |
| 7 | 154.76 | -38.25 | 107.89 | 116.98 | 32.86 | True |
| 8 | 158.08 | -42.67 | 108.70 | 114.03 | 35.05 | True |
| 9 | 160.21 | -48.25 | 109.21 | 110.45 | 37.63 | True |
| 10 | 160.95 | -54.43 | 109.39 | 106.52 | 40.47 | True |

## RM

| Pose | Raw [coxa, femur, tibia] | Foot x | Foot y | Foot z | Femur elev | Knee | Tibia dir |
|---|---|---|---|---|---|---|---|
| stand | [90, 110, 35] | 0.00 | -297.78 | -47.33 | +18.00° | 131.00° | -31.00° |
| sit | [90, 80, 60] | 0.00 | -293.24 | -90.17 | -12.00° | 156.00° | -36.00° |

IK worked example, stand foot:

| Step | Result |
|---|---|
| A | v = 0.000, u = 237.781, z = -47.327 |
| B | yaw = 0.000° -> coxa = 90.00 |
| C | r = 237.781, L = 173.281, d = 179.628 |
| D | 66.09 <= d <= 195.09: reachable = True |
| E | alpha = -15.276°, beta = 33.276°, elevation = 18.000° -> femur = 110.00 |
| F | knee = 131.000° |
| G | tibia = 35.00 |

Round trip: stand max error 0.0000°, sit max error 0.0000°. Matches `ik_module.py` (diff 0.0e+00).

Dry-run swing (30 mm forward, 20 mm lift, 11 waypoints):

| i | Foot x | Foot z | Coxa | Femur | Tibia | In range |
|---|---|---|---|---|---|---|
| 0 | 0.00 | -47.33 | 90.00 | 110.00 | 35.00 | True |
| 1 | 0.73 | -41.15 | 90.18 | 113.62 | 32.58 | True |
| 2 | 2.86 | -35.57 | 90.69 | 116.65 | 30.77 | True |
| 3 | 6.18 | -31.15 | 91.49 | 118.88 | 29.62 | True |
| 4 | 10.36 | -28.31 | 92.50 | 120.15 | 29.12 | True |
| 5 | 15.00 | -27.33 | 93.61 | 120.39 | 29.25 | True |
| 6 | 19.64 | -28.31 | 94.72 | 119.58 | 29.97 | True |
| 7 | 23.82 | -31.15 | 95.72 | 117.79 | 31.25 | True |
| 8 | 27.14 | -35.57 | 96.51 | 115.14 | 33.06 | True |
| 9 | 29.27 | -41.15 | 97.02 | 111.80 | 35.35 | True |
| 10 | 30.00 | -47.33 | 97.19 | 108.04 | 38.03 | True |

## RR

| Pose | Raw [coxa, femur, tibia] | Foot x | Foot y | Foot z | Femur elev | Knee | Tibia dir |
|---|---|---|---|---|---|---|---|
| stand | [103, 105, 35] | -115.00 | -290.14 | -58.66 | +18.00° | 125.00° | -37.00° |
| sit | [101, 75, 60] | -122.84 | -284.50 | -100.79 | -12.00° | 150.00° | -42.00° |

IK worked example, stand foot:

| Step | Result |
|---|---|
| A | v = 0.000, u = 230.137, z = -58.659 |
| B | yaw = 0.000° -> coxa = 103.00 |
| C | r = 230.137, L = 165.637, d = 175.717 |
| D | 66.09 <= d <= 195.09: reachable = True |
| E | alpha = -19.501°, beta = 37.501°, elevation = 18.000° -> femur = 105.00 |
| F | knee = 125.000° |
| G | tibia = 35.00 |

Round trip: stand max error 0.0000°, sit max error 0.0000°. Matches `ik_module.py` (diff 0.0e+00).

Dry-run swing (30 mm forward, 20 mm lift, 11 waypoints):

| i | Foot x | Foot z | Coxa | Femur | Tibia | In range |
|---|---|---|---|---|---|---|
| 0 | -115.00 | -58.66 | 103.00 | 105.00 | 35.00 | True |
| 1 | -114.27 | -52.48 | 103.18 | 108.92 | 32.20 | True |
| 2 | -112.14 | -46.90 | 103.71 | 112.25 | 30.02 | True |
| 3 | -108.82 | -42.48 | 104.54 | 114.73 | 28.56 | True |
| 4 | -104.64 | -39.64 | 105.58 | 116.17 | 27.85 | True |
| 5 | -100.00 | -38.66 | 106.73 | 116.49 | 27.88 | True |
| 6 | -95.36 | -39.64 | 107.88 | 115.66 | 28.62 | True |
| 7 | -91.18 | -42.48 | 108.91 | 113.75 | 30.05 | True |
| 8 | -87.86 | -46.90 | 109.72 | 110.90 | 32.10 | True |
| 9 | -85.73 | -52.48 | 110.25 | 107.31 | 34.71 | True |
| 10 | -85.00 | -58.66 | 110.43 | 103.27 | 37.74 | True |

## Open items

- Tibia sign: the comment in `ik_module.py` disagrees with its formula (see `docs/RF_leg_step1.md`). Confirm on hardware.
- Stand foot height is assumed to be ground level for every leg.
- Servo safe ranges are not yet confirmed. Nothing here has been run on the robot.
