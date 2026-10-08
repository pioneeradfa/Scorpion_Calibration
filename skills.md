# Scorpion Hexapod — RF (Right Front) Leg: Geometry & Gait Reference

Scope: this document covers **only the right-front (RF) leg**, as the first leg to get walking
smoothly before extending the same approach to the other five. Everything here is
derived from measurements already confirmed on the physical robot, plus a numerical
scan (done in code, not guessed) of which foot positions keep all three RF servos
inside a safe angle range.

---

## 1. Hardware identity

| Item | Value |
|---|---|
| Leg name | RF (Right Front) |
| Servo pins (coxa, femur, tibia) | 29, 30, 31 |
| Servo type | 20kg digital servo (per hardware label) |
| Controller | Raspberry Pi 5 + 24-channel PWM servo controller |
| Joint order along the leg | coxa (body mount) → femur → tibia → foot |

## 2. Link lengths (measured on hardware)

| Link | Length |
|---|---|
| Coxa | 64.5 mm |
| Femur | 64.5 mm |
| Tibia | 130.59 mm |
| Max straight-line reach (coxa+femur+tibia) | 259.59 mm |

Note: tibia is ~2x the femur length. This is an unusual ratio for a hexapod leg
and is the reason the safe operating envelope (section 5) sits toward the
leg's outer reach rather than near the body — closer-in foot positions force
the femur/tibia servos toward their mechanical limits (confirmed by the scan
below, not assumed).

## 3. Body mount geometry

Body frame: +X = forward (nose direction), +Y = left, +Z = up, origin at body center,
at coxa-axis height.

| Item | Value |
|---|---|
| RF coxa axis position (body frame) | x = +115 mm, y = −60 mm |
| Right/left coxa-axis spacing (confirmed via CAD) | 120 mm (→ ±60mm from centerline) |
| Front/mid/rear coxa-axis spacing (confirmed via CAD) | 115 mm |
| RF coxa mount angle | 90° — straight out, perpendicular to body centerline (no radial splay) |

## 4. Calibration reference pose (`leg_calibration.json`)

Confirmed description (from hardware, manually verified): belly down, foot tip
just touching the ground, femur and tibia both horizontal and collinear (leg
fully straight, no knee bend), coxa zeroed so the leg points perpendicular to
the body's forward axis.

| Joint | Raw servo angle at this pose |
|---|---|
| Coxa | 98° |
| Femur | 88° |
| Tibia | 94° |

This is the IK module's reference: feeding it a foot target at full leg
extension, zero height offset, reproduces these three numbers exactly
(verified: 0.02° error, floating-point only).

## 5. Confirmed joint direction conventions

| Joint | Increasing raw servo angle means... |
|---|---|
| Coxa | CCW rotation (same physical direction as all other legs). On RF (right side), CCW = leg swings **forward**. |
| Femur | Leg lifts **up**. |
| Tibia | Leg lifts/curls **up**. |

## 6. Safe operating envelope (numerically scanned, not assumed)

Using the IK module (`ik_module.py`), every foot position in a grid was solved
and checked against a conservative 15°–165° band on all three servos (well
inside their 0°–180° mechanical range, to leave margin for calibration drift
and dynamic overshoot).

**Finding:** a wide, flat safe region exists for outward reach (`u`, the
leg-local horizontal distance from the coxa axis) between **220mm and 255mm**,
across a height (`z`, relative to the coxa axis) range of **−100mm to +18mm**.
Positions closer to the body (u < 200mm) quickly push the tibia toward its
lower limit and were excluded.

### Chosen default stance (tune later, as agreed)

| Parameter | Value | Why |
|---|---|---|
| Outward reach (u) | 230 mm | Inside the safe plateau, with margin on both sides |
| Stance height (z) | −35 mm | Foot below coxa-axis height, giving ground clearance while standing |
| Swing lift height (z) | up to +15 mm (relative lift of ~50mm from stance) | Confirmed safe at u=230 |
| Stride (v, forward/back) | ±60 mm | Confirmed all 3 joints stay in the 15°–165° band across this whole range at the chosen stance height |

At this stance, across the full ±60mm stride and the full lift range, computed
joint angles stay within roughly **25°–140°** — comfortably inside the safe
band, confirmed by direct computation (see `rf_leg_gait.py` self-test).

## 7. What this does NOT cover yet

- The other 5 legs (same link lengths and conventions apply, but each has its
  own calibration zero and mount position — not yet scanned for a safe
  envelope).
- Coordinating multiple legs into a tripod/ripple gait (this document and the
  accompanying script move RF alone, in place, to validate the motion is
  smooth before combining legs).
- Actual body translation — RF stepping alone does not move the robot; it
  only exercises the leg's swing/stance cycle for visual confirmation.
