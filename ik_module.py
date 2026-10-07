#!/usr/bin/env python3
"""
Scorpion Hexapod - Inverse Kinematics Module
==============================================
Converts a desired foot position (in millimeters, body-centered frame) into
raw servo angles for each leg, using your measured link lengths and your
per-servo calibration (leg_calibration.json, produced by
servo_calibration_gui.py).

COORDINATE FRAMES
------------------
Body frame (robot-centric, looking down from above):
  +X = forward (nose direction)
  +Y = left
  +Z = up
  Origin = center of the body, at the height of the coxa axes.

Leg-local frame (per leg, used internally by the IK solver):
  u = horizontal distance outward from that leg's coxa axis, along the
      direction the leg points at its calibrated "straight extended" pose.
  v = horizontal forward/backward offset from that straight-out direction.
  z = height relative to the coxa axis (positive = up), same as body Z.

MEASURED GEOMETRY (as given, mm)
----------------------------------
  Coxa link length:  64.5
  Femur link length: 64.5
  Tibia link length: 130.59
  Right/left coxa-axis spacing: 120   -> each side's coxa axis sits at
                                          y = +/-60mm from body centerline
  Front/mid/rear coxa-axis spacing: 115 (front-to-mid and mid-to-rear)
                                       -> front x=+115, mid x=0, rear x=-115
  Mount angle: all 6 coxa axes point straight out (90 deg) from the body's
               forward centerline (confirmed, not a radial splay).

JOINT DIRECTION CONVENTIONS (as you described, confirmed by testing)
----------------------------------------------------------------------
  Femur: increasing raw servo angle lifts the leg up.      (same for all legs)
  Tibia: increasing raw servo angle lifts/curls the leg up. (same for all legs)
  Coxa:  increasing raw servo angle rotates CCW (same physical direction on
         every servo). On RIGHT-side legs, CCW = leg swings forward.
         On LEFT-side legs, CCW = leg swings backward (mirrored mounting).

IMPORTANT - VERIFY BEFORE TRUSTING THIS ON HARDWARE
------------------------------------------------------
The femur/tibia sign mapping below is derived from your description of
"increase = up" via the standard 2-link planar IK triangle, not from a
live test of the full IK output. Before running any gait on real
hardware:
  1. Run self_test() (below) - it's a pure-math check with no hardware
     calls, confirming the solver reproduces your exact calibrated
     angles at the fully-extended reference pose.
  2. On the Pi, with the robot propped up so legs can move freely and
     power supply current-limited/attended, call set_foot_position_body()
     for ONE leg with a small z lift (e.g. +10mm) and confirm the foot
     actually moves up, not down. If it's inverted, flip FEMUR_SIGN or
     TIBIA_SIGN below for that leg type and retest.
"""

import json
import math
import os

try:
    import ServoControl
    HAVE_HARDWARE = True
except ImportError:
    HAVE_HARDWARE = False

# ============================================================
# MEASURED GEOMETRY
# ============================================================
COXA_LEN = 64.5     # mm
FEMUR_LEN = 64.5    # mm
TIBIA_LEN = 130.59  # mm

HALF_WIDTH = 120 / 2.0     # 60mm, right/left coxa-axis offset from centerline
ROW_SPACING = 115.0        # mm between front/mid/rear coxa axes

# Coxa-axis mount position in body frame (x=forward, y=left), mm
MOUNT_POS = {
    "RF": (ROW_SPACING, -HALF_WIDTH),
    "RM": (0.0,         -HALF_WIDTH),
    "RR": (-ROW_SPACING, -HALF_WIDTH),
    "LF": (ROW_SPACING,  HALF_WIDTH),
    "LM": (0.0,          HALF_WIDTH),
    "LR": (-ROW_SPACING, HALF_WIDTH),
}

LEG_MAP = {
    "RF": [29, 30, 31],
    "RM": [3, 4, 5],
    "RR": [9, 1, 2],
    "LF": [24, 23, 22],
    "LM": [13, 14, 15],
    "LR": [10, 11, 12],
}
LEG_ORDER = ["RF", "RM", "RR", "LR", "LM", "LF"]
JOINT_NAMES = ["coxa", "femur", "tibia"]

# Sign conventions (see docstring). Flip these per-leg-type during hardware
# verification if a joint moves opposite to what's expected.
FEMUR_SIGN = +1   # +1: increasing IK femur angle -> increasing raw angle (up)
TIBIA_SIGN = -1   # -1: knee bending (knee_angle below 180) -> raw increases (up/curl)
COXA_SIGN = {"R": +1, "L": -1}  # right legs: raw increases with +yaw (forward)
                                 # left legs:  raw increases with -yaw (backward)

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "leg_calibration.json"
)

ANGLE_MIN, ANGLE_MAX = 0, 180

# ============================================================
# CALIBRATION LOADING
# ============================================================
def load_calibration(path=CALIBRATION_FILE):
    """Load per-servo zero-reference angles saved by servo_calibration_gui.py.
    Falls back to the known-good values you already measured if the file
    isn't present (useful for running self_test() off the Pi)."""
    if os.path.exists(path):
        with open(path, "r") as f:
            data = json.load(f)
        cal = {}
        for leg in LEG_ORDER:
            cal[leg] = {j: data[leg][j]["angle"] for j in JOINT_NAMES}
        return cal

    # Fallback: your manually-verified belly-down stretched-leg calibration
    return {
        "RF": {"coxa": 98, "femur": 88, "tibia": 94},
        "RM": {"coxa": 90, "femur": 92, "tibia": 84},
        "RR": {"coxa": 103, "femur": 87, "tibia": 90},
        "LR": {"coxa": 87, "femur": 90, "tibia": 91},
        "LM": {"coxa": 90, "femur": 90, "tibia": 84},
        "LF": {"coxa": 115, "femur": 93, "tibia": 101},
    }

CALIBRATION = load_calibration()

# ============================================================
# COORDINATE TRANSFORM: body frame -> leg-local frame
# ============================================================
def body_to_leg_local(leg, body_x, body_y, body_z):
    mx, my = MOUNT_POS[leg]
    dx = body_x - mx
    dy = body_y - my
    side = 1 if leg.startswith("R") else -1
    u = -side * dy   # outward distance from that leg's coxa axis
    v = dx            # forward/backward offset
    z = body_z
    return u, v, z

# ============================================================
# PER-LEG INVERSE KINEMATICS
# ============================================================
def clamp(val, lo, hi):
    return max(lo, min(hi, val))

def leg_ik(leg, u, v, z):
    """Solve 3-DOF IK for one leg given a target foot position in that leg's
    local frame (u=outward mm, v=forward mm, z=up mm, origin at coxa axis).
    Returns raw servo angles (coxa_deg, femur_deg, tibia_deg)."""

    coxa_yaw = math.atan2(v, u)
    r = math.hypot(u, v)
    L = r - COXA_LEN                 # horizontal reach from femur axis to foot
    d = math.hypot(L, z)             # straight-line femur-axis -> foot distance

    # Clamp to reachable range so acos() never gets an invalid argument.
    # Margin is tiny (not a safety margin) - just enough to dodge float
    # rounding at the exact boundary.
    d_min = abs(FEMUR_LEN - TIBIA_LEN) + 1e-6
    d_max = FEMUR_LEN + TIBIA_LEN - 1e-6
    d = clamp(d, d_min, d_max)

    alpha = math.atan2(z, L)
    beta = math.acos(clamp((FEMUR_LEN**2 + d**2 - TIBIA_LEN**2) / (2 * FEMUR_LEN * d), -1, 1))
    femur_angle = alpha + beta       # 0 = fully horizontal extension

    # gamma is the interior angle at the knee joint (law of cosines). It is
    # already 180deg when the leg is fully straight and decreases as the
    # knee bends, so it IS the knee angle directly (no pi-gamma needed).
    gamma = math.acos(clamp((FEMUR_LEN**2 + TIBIA_LEN**2 - d**2) / (2 * FEMUR_LEN * TIBIA_LEN), -1, 1))
    knee_angle = gamma               # 180deg (pi rad) = leg fully straight

    cal = CALIBRATION[leg]
    side = leg[0]  # 'R' or 'L'

    coxa_deg = cal["coxa"] + COXA_SIGN[side] * math.degrees(coxa_yaw)
    femur_deg = cal["femur"] + FEMUR_SIGN * math.degrees(femur_angle)
    tibia_deg = cal["tibia"] + TIBIA_SIGN * (180.0 - math.degrees(knee_angle))

    return (
        clamp(coxa_deg, ANGLE_MIN, ANGLE_MAX),
        clamp(femur_deg, ANGLE_MIN, ANGLE_MAX),
        clamp(tibia_deg, ANGLE_MIN, ANGLE_MAX),
    )

def leg_ik_body(leg, body_x, body_y, body_z):
    """Solve IK for one leg given a target foot position in body frame (mm)."""
    u, v, z = body_to_leg_local(leg, body_x, body_y, body_z)
    return leg_ik(leg, u, v, z)

# ============================================================
# HARDWARE OUTPUT
# ============================================================
def angle_to_pulse(angle):
    return int(500 + (angle / 180.0) * 2000)

def set_foot_position_body(leg, body_x, body_y, body_z, duration=400):
    if not HAVE_HARDWARE:
        raise RuntimeError("ServoControl not available - run this on the Raspberry Pi.")
    coxa_deg, femur_deg, tibia_deg = leg_ik_body(leg, body_x, body_y, body_z)
    for pin, angle in zip(LEG_MAP[leg], (coxa_deg, femur_deg, tibia_deg)):
        ServoControl.setPWMServoMove(pin, angle_to_pulse(angle), duration)
    return coxa_deg, femur_deg, tibia_deg

# ============================================================
# SELF TEST (pure math, no hardware, safe to run anywhere)
# ============================================================
def self_test():
    """Verifies the IK math alone: at full horizontal extension
    (u = coxa+femur+tibia, v=0, z=0), the solver should reproduce the exact
    calibrated angles for every leg, since that is what the calibration
    angles represent (the belly-down, legs-stretched-straight pose)."""
    print("Self-test: full extension should match calibration exactly\n")
    max_err = 0.0
    full_reach = COXA_LEN + FEMUR_LEN + TIBIA_LEN

    for leg in LEG_ORDER:
        coxa_deg, femur_deg, tibia_deg = leg_ik(leg, full_reach, 0, 0)
        cal = CALIBRATION[leg]
        err = (
            abs(coxa_deg - cal["coxa"])
            + abs(femur_deg - cal["femur"])
            + abs(tibia_deg - cal["tibia"])
        )
        max_err = max(max_err, err)
        print(
            f"  {leg}: computed coxa={coxa_deg:.2f} femur={femur_deg:.2f} tibia={tibia_deg:.2f}"
            f"   | calibrated coxa={cal['coxa']} femur={cal['femur']} tibia={cal['tibia']}"
            f"   | total err={err:.4f} deg"
        )

    print(f"\nMax total error across all legs: {max_err:.4f} deg")
    print("PASS" if max_err < 0.1 else "FAIL - check geometry/signs")

    # Use a realistic standing reach (not full extension) so there's actual
    # slack left in the leg to lift - testing the lift at 100% stretch would
    # correctly saturate at max reach and look like a false failure.
    standing_u = full_reach * 0.8
    print(f"\nSanity check: lifting RF foot by +20mm (z) at {standing_u:.1f}mm reach"
          f" (80% of max {full_reach:.1f}mm) - femur/tibia raw angle should INCREASE")
    base = leg_ik(  "RF", standing_u, 0, 0)
    lifted = leg_ik("RF", standing_u, 0, 20)
    print(f"  z=0:   coxa={base[0]:.2f} femur={base[1]:.2f} tibia={base[2]:.2f}")
    print(f"  z=+20: coxa={lifted[0]:.2f} femur={lifted[1]:.2f} tibia={lifted[2]:.2f}")
    print(f"  femur delta: {lifted[1]-base[1]:+.2f} deg (expect positive)")
    print(f"  tibia delta: {lifted[2]-base[2]:+.2f} deg (expect positive)")


if __name__ == "__main__":
    self_test()
