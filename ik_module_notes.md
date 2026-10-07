# Scorpion Hexapod — Inverse Kinematics (as of 2026-10-07)

## Confirmed geometry (mm)
- Coxa length: 64.5
- Femur length: 64.5
- Tibia length: 130.59
- Right/left coxa-axis spacing: 120 (±60mm from centerline)
- Front/mid/rear coxa-axis spacing: 115mm each
- Mount angle: all 6 coxa axes point straight out (90°) from body centerline
- Confirmed via CAD drawing on 2026-10-07 (115.28/115.04/115.05/114.97 row spacing, 119.96/120.12/120.03 width spacing)

## Confirmed joint directions
- Femur: increasing raw servo angle = leg lifts up (consistent all legs)
- Tibia: increasing raw servo angle = leg lifts/curls up (consistent all legs)
- Coxa: increasing raw angle = CCW rotation (same physical direction on every servo). On RIGHT legs, CCW = forward. On LEFT legs, CCW = backward (mirrored mounting).

## IK module
`ik_module.py` (delivered to user) — 3-DOF per-leg IK (coxa yaw + femur/tibia 2-link planar solve), loads `leg_calibration.json` (from `servo_calibration_gui.py`) as the per-servo zero reference.

Self-test passes: reproduces exact calibration angles at full leg extension (0.02° error, floating point only).

**Not yet hardware-verified**: femur/tibia sign convention (FEMUR_SIGN, TIBIA_SIGN in the module) was derived from the user's verbal description of joint direction, not a live test of full IK output. Must verify on the Pi with one leg before trusting on a full gait — lift a single foot via `set_foot_position_body()` and confirm it moves the expected direction.

**Noted limitation**: tibia (130.59mm) is ~2x femur (64.5mm) length. This ratio requires wide femur sweep angles (~150°+) to reach practical standing heights, and tibia angle can approach near 0° — close to likely servo mechanical limits. Worth checking standing/walking foot-target heights don't push joints past safe range.

## Calibration data
See `leg_calibration_belly_down.json` in this project — belly-down, legs-stretched pose, manually verified per leg on hardware.
