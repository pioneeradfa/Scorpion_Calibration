#!/usr/bin/env python3
"""
Scorpion Hexapod - Inverse Kinematics (corrected for 270-degree servos).

WHAT CHANGED FROM THE PREVIOUS VERSION
--------------------------------------
1. TIBIA_SIGN is -1, and it took three attempts to settle because no one had
   touched a servo.  History: the original file shipped -1 with a comment that
   described +1; the first correction flipped it to +1 to match that comment;
   the hardware evidence then flipped it back to -1 for good.  The evidence is
   the hand-calibrated STAND pose (POSITIONS["stand"] in the legacy driver,
   RF raw 153.0/165.0/52.5 physical), which is the pose in the bench photo:
   the calibration pose is the leg DEAD STRAIGHT, so a standing knee bend of
   88.5 deg has to be reached by MOVING AWAY from cal_tibia = 141.0, and stand
   sits 88.5 deg BELOW it.  A hinge only bends one way, therefore decreasing
   raw tibia bends the knee and increasing it straightens the knee.  With +1
   the same stand pose solves to a foot 146 mm ABOVE the body plane, and the
   photo shows the knee servo as the highest joint on the leg with the body
   deck ~70 mm up, matching the -1 solution (72.5 mm) to within measurement
   error.  See RF_LEG_IK.md section 2 for the full table.

2. All angles are TRUE PHYSICAL SHAFT DEGREES (0-270), not the legacy /180
   command units.  Pulse conversion lives in servo_lsc24.py.  Calibration
   values recorded through the old mapping are converted on load by
   LEGACY_CALIBRATION_SCALE = 1.5.

3. ANGLE_MIN/ANGLE_MAX were 0/180.  On a 270-degree servo that threw away a
   third of the travel -- and LF's coxa calibrates to 172.5 physical degrees,
   only 7.5 degrees from the old clamp, so any yaw over 7 degrees saturated it
   silently.

4. Unreachable targets used to be silently clamped.  leg_ik() now raises
   WorkspaceError unless you explicitly pass allow_out_of_range=True.  Silent
   clamping is what made the earlier "lift test" results uninterpretable.

5. The old self_test() evaluated the solver at full leg extension, which is the
   SINGULAR point: there beta = 0.008 deg and the knee bend phi = 0.012 deg, so
   the elbow branch (alpha+beta vs alpha-beta) and TIBIA_SIGN were both
   multiplied by ~zero.  The test could not fail and could not detect a sign or
   branch error.  It also never called body_to_leg_local(), so MOUNT_POS and
   the COXA_SIGN mirroring had zero coverage.  The new self_test() uses
   bent-knee poses and cross-checks against forward kinematics written
   independently from the geometry.

CONFIRMED GEOMETRY (mm)
    coxa 64.5   femur 64.5   tibia 130.59
    coxa axes +/-60mm from the centreline, rows at x = +115 / 0 / -115
    all six coxa axes point straight out (90 deg), no radial splay

JOINT CONVENTIONS
    Femur: increasing raw angle lifts the leg up.          FEMUR_SIGN = +1
    Tibia: increasing raw angle STRAIGHTENS the knee;
           decreasing it curls the knee up under the body. TIBIA_SIGN = -1
    Coxa : increasing raw angle rotates CCW on every servo, which is FORWARD
           on right legs and BACKWARD on left legs.        COXA_SIGN R=+1 L=-1

ELBOW BRANCH
    The solver uses femur_angle = alpha + beta (knee ABOVE the coxa plane,
    tibia reaching down).  This was verified by elimination, not from the
    calibration file: the belly-down calibration pose has the leg straight,
    which is the singular point where both branches coincide, so it carries no
    information about the branch.  The alternative branch (alpha - beta) drives
    the femur raw angle NEGATIVE for every plausible standing target (-7 to -57
    degrees), which a real servo cannot represent.  Only alpha + beta yields
    in-range angles.

    STILL WORTH A PHYSICAL CHECK: with the robot propped up, command a bent
    pose and confirm the knee ends up ABOVE the coxa plane, not below.
"""

from __future__ import annotations

import json
import math
import os
from typing import Dict, Optional, Tuple

from servo_lsc24 import (
    DEGREE_SAFE_MAX,
    DEGREE_SAFE_MIN,
    JOINT_NAMES,
    LEG_CHANNELS,
    LEG_ORDER,
    ServoBus,
)

# ============================================================
# MEASURED GEOMETRY
# ============================================================
COXA_LEN = 64.5     # mm
FEMUR_LEN = 64.5    # mm
TIBIA_LEN = 130.59  # mm

HALF_WIDTH = 120 / 2.0     # 60mm, right/left coxa-axis offset from centreline
ROW_SPACING = 115.0        # mm between front/mid/rear coxa axes

PLANAR_MAX = FEMUR_LEN + TIBIA_LEN           # 195.09 mm, femur pivot -> foot
PLANAR_MIN = abs(TIBIA_LEN - FEMUR_LEN)      #  66.09 mm, fully folded
FULL_REACH = COXA_LEN + PLANAR_MAX           # 259.59 mm, coxa axis -> foot

# Body frame: +X forward, +Y left, +Z up, origin at body centre at coxa height.
# Nominal values, confirmed against the body-plate CAD photo: as-built column
# gaps 114.97-115.28 and lateral gaps 119.96-120.12, i.e. within 0.28 mm of
# nominal, which moves a solved joint angle by at most 0.26 deg -- inside the
# servo's own 0.3 deg accuracy.  See RF_LEG_IK.md section 1 for the full table.
MOUNT_POS: Dict[str, Tuple[float, float]] = {
    "RF": (ROW_SPACING,  -HALF_WIDTH),
    "RM": (0.0,          -HALF_WIDTH),
    "RR": (-ROW_SPACING, -HALF_WIDTH),
    "LF": (ROW_SPACING,   HALF_WIDTH),
    "LM": (0.0,           HALF_WIDTH),
    "LR": (-ROW_SPACING,  HALF_WIDTH),
}

# Joint signs -- flip an individual entry only if a physical test contradicts it.
FEMUR_SIGN = +1
TIBIA_SIGN = -1        # hardware-settled: raw down = knee bends.  See docstring item 1.
COXA_SIGN = {"R": +1, "L": -1}

# The recorded calibration was captured through the legacy /180 pulse mapping,
# so its numbers are 1/1.5 of true shaft degrees.  Set this to 1.0 once you
# re-calibrate with servo_lsc24.degree_to_pulse() in place.
LEGACY_CALIBRATION_SCALE = 1.5

CALIBRATION_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "leg_calibration.json"
)

# Fallback matches leg_calibration.json exactly.  Kept so self_test() runs on a
# machine with no data file, but it is a duplicate -- if you re-calibrate, edit
# the JSON and this table together or delete this table.
_CALIBRATION_LEGACY = {
    "RF": {"coxa": 98,  "femur": 88, "tibia": 94},
    "RM": {"coxa": 90,  "femur": 92, "tibia": 84},
    "RR": {"coxa": 103, "femur": 87, "tibia": 90},
    "LR": {"coxa": 87,  "femur": 90, "tibia": 91},
    "LM": {"coxa": 90,  "femur": 90, "tibia": 84},
    "LF": {"coxa": 115, "femur": 93, "tibia": 101},
}


class WorkspaceError(ValueError):
    """Raised when a foot target cannot be reached, or lands outside servo travel."""


# ============================================================
# CALIBRATION
# ============================================================
def load_calibration(path: str = CALIBRATION_FILE,
                     scale: float = LEGACY_CALIBRATION_SCALE) -> Dict[str, Dict[str, float]]:
    """Load per-servo zero references and convert them to true physical degrees.

    The stored values are the raw numbers recorded while the leg sat in the
    belly-down, legs-stretched-straight pose.  They are multiplied by `scale`
    because they were captured through the legacy /180 pulse mapping.
    """
    if os.path.exists(path):
        with open(path, "r") as f:
            data = json.load(f)
        src = {leg: {j: data[leg][j]["angle"] for j in JOINT_NAMES} for leg in LEG_ORDER}
    else:
        src = {leg: dict(_CALIBRATION_LEGACY[leg]) for leg in LEG_ORDER}
    return {leg: {j: src[leg][j] * scale for j in JOINT_NAMES} for leg in LEG_ORDER}


CALIBRATION = load_calibration()


# ============================================================
# WORKSPACE
# ============================================================
def leg_reach_limits(leg: str) -> Tuple[float, float]:
    """(d_min, d_max) reachable from the femur pivot, limited by tibia travel.

    The knee can only bend one way, and raw_tibia = cal_tibia + TIBIA_SIGN*phi
    must stay inside the servo's safe band.  With TIBIA_SIGN = -1 that bounds
    phi from above by cal_tibia - DEGREE_SAFE_MIN.
    """
    cal_t = CALIBRATION[leg]["tibia"]
    if TIBIA_SIGN > 0:
        phi_max = DEGREE_SAFE_MAX - cal_t
    else:
        phi_max = cal_t - DEGREE_SAFE_MIN
    if phi_max <= 0:
        raise WorkspaceError(
            f"{leg}: calibrated tibia {cal_t:.1f} deg leaves no room to bend the knee"
        )
    gamma_min = 180.0 - phi_max
    d_min = math.sqrt(
        FEMUR_LEN ** 2 + TIBIA_LEN ** 2
        - 2 * FEMUR_LEN * TIBIA_LEN * math.cos(math.radians(gamma_min))
    )
    return max(PLANAR_MIN, d_min), PLANAR_MAX


def femur_travel(leg: str) -> Tuple[float, float]:
    """(min, max) femur elevation in degrees allowed by the servo safe band."""
    cal_f = CALIBRATION[leg]["femur"]
    return DEGREE_SAFE_MIN - cal_f, DEGREE_SAFE_MAX - cal_f


# ============================================================
# COORDINATE TRANSFORM
# ============================================================
def body_to_leg_local(leg: str, body_x: float, body_y: float, body_z: float
                      ) -> Tuple[float, float, float]:
    """Body frame -> leg-local (u = outboard, v = forward, z = up), origin at
    that leg's coxa axis."""
    if leg not in MOUNT_POS:
        raise KeyError(f"unknown leg {leg!r}; expected one of {sorted(MOUNT_POS)}")
    mx, my = MOUNT_POS[leg]
    dx = body_x - mx
    dy = body_y - my
    side = 1.0 if leg.startswith("R") else -1.0
    u = -side * dy
    v = dx
    return u, v, body_z


def leg_local_to_body(leg: str, u: float, v: float, z: float
                      ) -> Tuple[float, float, float]:
    """Inverse of body_to_leg_local()."""
    mx, my = MOUNT_POS[leg]
    side = 1.0 if leg.startswith("R") else -1.0
    return mx + v, my - side * u, z


# ============================================================
# INVERSE KINEMATICS
# ============================================================
def leg_ik(leg: str, u: float, v: float, z: float,
           allow_out_of_range: bool = False) -> Tuple[float, float, float]:
    """3-DOF IK for one leg from a leg-local foot target.

    u = outboard mm from the coxa axis, v = forward mm, z = up mm.
    Returns TRUE physical servo degrees (coxa, femur, tibia), each in 0-270.

    Raises WorkspaceError if the target is unreachable or would drive a joint
    outside its safe band, unless allow_out_of_range=True (in which case the
    angles are clamped and the caller is on its own).
    """
    cal = CALIBRATION[leg]
    side = leg[0]

    if u <= 0.0:
        raise WorkspaceError(
            f"{leg}: foot target is inboard of the coxa axis (u={u:.1f} mm). "
            f"The coxa cannot swing that far; hypot(u,v) would silently mirror it outward."
        )

    coxa_yaw = math.atan2(v, u)
    r = math.hypot(u, v)
    L = r - COXA_LEN               # horizontal, femur pivot -> foot
    d = math.hypot(L, z)           # straight-line, femur pivot -> foot

    d_min, d_max = leg_reach_limits(leg)
    problems = []
    if d > PLANAR_MAX:
        problems.append(f"d={d:.2f} exceeds full stretch {PLANAR_MAX:.2f}")
    elif d < d_min:
        problems.append(f"d={d:.2f} below the {d_min:.2f} floor set by tibia travel")

    d_clamped = max(PLANAR_MIN, min(PLANAR_MAX - 1e-9, d))
    alpha = math.atan2(z, L)
    beta = math.acos(max(-1.0, min(1.0,
        (FEMUR_LEN ** 2 + d_clamped ** 2 - TIBIA_LEN ** 2) / (2 * FEMUR_LEN * d_clamped))))
    femur_angle = alpha + beta                      # knee-up branch, see docstring

    gamma = math.acos(max(-1.0, min(1.0,
        (FEMUR_LEN ** 2 + TIBIA_LEN ** 2 - d_clamped ** 2) / (2 * FEMUR_LEN * TIBIA_LEN))))
    phi = 180.0 - math.degrees(gamma)               # knee bend away from straight

    coxa_deg = cal["coxa"] + COXA_SIGN[side] * math.degrees(coxa_yaw)
    femur_deg = cal["femur"] + FEMUR_SIGN * math.degrees(femur_angle)
    tibia_deg = cal["tibia"] + TIBIA_SIGN * phi

    for name, val in (("coxa", coxa_deg), ("femur", femur_deg), ("tibia", tibia_deg)):
        if not (DEGREE_SAFE_MIN <= val <= DEGREE_SAFE_MAX):
            problems.append(f"{name} {val:.2f} deg outside safe band "
                            f"[{DEGREE_SAFE_MIN}, {DEGREE_SAFE_MAX}]")

    if problems and not allow_out_of_range:
        raise WorkspaceError(
            f"{leg} cannot reach (u={u:.2f}, v={v:.2f}, z={z:.2f}): " + "; ".join(problems)
        )

    clamp = (lambda x: max(DEGREE_SAFE_MIN, min(DEGREE_SAFE_MAX, x))) if allow_out_of_range \
        else (lambda x: x)
    return clamp(coxa_deg), clamp(femur_deg), clamp(tibia_deg)


def leg_ik_body(leg: str, body_x: float, body_y: float, body_z: float,
                allow_out_of_range: bool = False) -> Tuple[float, float, float]:
    """IK from a body-frame foot target (mm)."""
    u, v, z = body_to_leg_local(leg, body_x, body_y, body_z)
    return leg_ik(leg, u, v, z, allow_out_of_range=allow_out_of_range)


# ============================================================
# FORWARD KINEMATICS  (written from the geometry, not by inverting leg_ik)
# ============================================================
def leg_fk(leg: str, coxa_deg: float, femur_deg: float, tibia_deg: float
           ) -> Tuple[float, float, float]:
    """True physical servo degrees -> leg-local foot position (u, v, z).

    Used by self_test() to verify IK independently.  At the calibration angles
    this must return the belly-down pose: u = FULL_REACH, v = 0, z = 0.
    """
    cal = CALIBRATION[leg]
    side = leg[0]

    yaw = math.radians(COXA_SIGN[side] * (coxa_deg - cal["coxa"]))
    elev = math.radians(FEMUR_SIGN * (femur_deg - cal["femur"]))
    phi = TIBIA_SIGN * (tibia_deg - cal["tibia"])
    tib_abs = elev - math.radians(phi)

    cu, cv = math.cos(yaw), math.sin(yaw)
    horiz = COXA_LEN + FEMUR_LEN * math.cos(elev) + TIBIA_LEN * math.cos(tib_abs)
    z = FEMUR_LEN * math.sin(elev) + TIBIA_LEN * math.sin(tib_abs)
    return horiz * cu, horiz * cv, z


def leg_fk_body(leg: str, coxa_deg: float, femur_deg: float, tibia_deg: float
                ) -> Tuple[float, float, float]:
    u, v, z = leg_fk(leg, coxa_deg, femur_deg, tibia_deg)
    return leg_local_to_body(leg, u, v, z)


# ============================================================
# HARDWARE OUTPUT
# ============================================================
_BUS: Optional[ServoBus] = None


def get_bus(dry_run: bool = False) -> ServoBus:
    global _BUS
    if _BUS is None:
        _BUS = ServoBus(dry_run=dry_run)
    return _BUS


def leg_pose(leg: str, angles: Tuple[float, float, float]) -> Dict[int, float]:
    """{channel: degrees} for one leg, ready for ServoBus.send_pose()."""
    return dict(zip(LEG_CHANNELS[leg], angles))


def all_legs_pose(angles_by_leg: Dict[str, Tuple[float, float, float]]) -> Dict[int, float]:
    pose: Dict[int, float] = {}
    for leg, ang in angles_by_leg.items():
        pose.update(leg_pose(leg, ang))
    return pose


def set_foot_position_body(leg: str, body_x: float, body_y: float, body_z: float,
                           duration: int = 400, bus: Optional[ServoBus] = None
                           ) -> Tuple[float, float, float]:
    """Move one foot to a body-frame target.  Returns the commanded angles."""
    angles = leg_ik_body(leg, body_x, body_y, body_z)
    (bus or get_bus()).send_pose(leg_pose(leg, angles), duration)
    return angles


# ============================================================
# SELF TEST
# ============================================================
def _fmt(t: Tuple[float, float, float]) -> str:
    return f"coxa={t[0]:7.2f} femur={t[1]:7.2f} tibia={t[2]:7.2f}"


def self_test(verbose: bool = True) -> bool:
    """Bent-knee verification.  Nothing here is evaluated at full extension,
    because full extension is the singular point where the branch and tibia-sign
    errors cancel out and the test degenerates to a tautology."""

    def say(s: str = "") -> None:
        if verbose:
            print(s)

    ok = True

    # ---- 1. calibration pose must be the straight, level, belly-down leg ----
    say("1. FK at the calibrated angles -> legs straight out and level")
    for leg in LEG_ORDER:
        c = CALIBRATION[leg]
        u, v, z = leg_fk(leg, c["coxa"], c["femur"], c["tibia"])
        err = abs(u - FULL_REACH) + abs(v) + abs(z)
        flag = "ok" if err < 0.05 else "FAIL"
        ok &= err < 0.05
        say(f"   {leg}: u={u:7.2f} (want {FULL_REACH:.2f})  v={v:+6.3f}  z={z:+6.3f}   {flag}")

    # ---- 2. IK -> FK round trip at bent-knee poses ----
    # Targets span the gait envelope.  Not every target is reachable by every
    # leg -- LF has the tightest tibia margin, so its d floor is the highest --
    # and a target outside a leg's band MUST raise rather than clamp.  Both
    # paths are checked.
    say("\n2. IK -> FK round trip at BENT poses (non-singular), plus rejection checks")
    targets = [
        (144.5,   0.0, -155.0),   # nominal standing foot
        (144.5, -25.0, -155.0),   # end of stance sweep
        (144.5,  25.0, -155.0),   # start of stance sweep
        (144.5,   0.0, -130.0),   # mid swing, foot lifted
        (144.5,   0.0, -170.0),   # deeper, straighter leg
        (160.0,   0.0, -140.0),   # further outboard
        (144.5, -25.0, -170.0),   # corner of the envelope
        (120.0,  15.0,  -90.0),   # tight: out of range for some legs on purpose
    ]
    worst = 0.0
    solved = rejected = 0
    for leg in LEG_ORDER:
        d_lo, d_hi = leg_reach_limits(leg)
        for (tu, tv, tz) in targets:
            L = math.hypot(tu, tv) - COXA_LEN
            d = math.hypot(L, tz)
            reachable = d_lo <= d <= d_hi
            try:
                ang = leg_ik(leg, tu, tv, tz)
            except WorkspaceError:
                if reachable:
                    say(f"   {leg}: ({tu},{tv},{tz}) d={d:.1f} in [{d_lo:.1f},{d_hi:.1f}] "
                        f"but was REJECTED  FAIL")
                    ok = False
                else:
                    rejected += 1
                continue
            if not reachable:
                say(f"   {leg}: ({tu},{tv},{tz}) d={d:.1f} outside [{d_lo:.1f},{d_hi:.1f}] "
                    f"but was ACCEPTED  FAIL")
                ok = False
                continue
            solved += 1
            ru, rv, rz = leg_fk(leg, *ang)
            err = math.dist((tu, tv, tz), (ru, rv, rz))
            worst = max(worst, err)
            if err > 0.05:
                say(f"   {leg}: target=({tu:7.2f},{tv:6.2f},{tz:7.2f}) "
                    f"got=({ru:7.2f},{rv:6.2f},{rz:7.2f}) err={err:.4f}  FAIL")
                ok = False
    say(f"   {solved} poses solved and round-tripped, {rejected} correctly rejected")
    say(f"   worst round-trip error: {worst:.6f} mm")
    ok &= worst < 0.05 and solved > 0

    # ---- 3. the sign conventions, asserted directly ----
    say("\n3. Joint-direction assertions at the nominal standing pose")
    leg = "RF"
    base = leg_ik(leg, 144.5, 0.0, -155.0)
    say(f"   {leg} standing: {_fmt(base)}")

    # raising the femur target must raise the raw femur angle
    higher = leg_ik(leg, 144.5, 0.0, -120.0)
    d_fem = higher[1] - base[1]
    say(f"   foot raised 35mm -> femur delta {d_fem:+.2f} deg  (expect > 0: 'increase = lift up')")
    ok &= d_fem > 0

    # curling the knee must LOWER the raw tibia angle (TIBIA_SIGN = -1).
    # Pull the foot inboard and up so d shrinks and the knee genuinely bends.
    curled = leg_ik(leg, 125.0, 0.0, -150.0)
    d_tib = curled[2] - base[2]
    say(f"   knee bent further -> tibia delta {d_tib:+.2f} deg  (expect < 0: 'increase = straighten')")
    ok &= d_tib < 0

    # The tibia must track the knee bend monotonically: raw = cal + TIBIA_SIGN*phi,
    # and phi grows as the femur-pivot->foot distance d shrinks.  So the sign of
    # the tibia response to a lift depends on which way the lift moves d -- it is
    # NOT always one direction, and asserting a fixed sign is how the original
    # self-test came to pass with an inverted TIBIA_SIGN.
    def d_of(u, v, z):
        return math.hypot(math.hypot(u, v) - COXA_LEN, z)

    say("   tibia response to a lift, predicted from the change in d:")
    lift_cases = [
        # (u, v, z0, z1, why)
        (144.5, 0.0, -155.0, -130.0, "normal standing reach: lift shrinks d, knee bends more"),
        (207.67, 0.0, 0.0, 20.0, "near full stretch: lift grows d, knee STRAIGHTENS"),
        (160.0, 0.0, -140.0, -120.0, "further outboard: lift shrinks d"),
    ]
    for (cu, cv, z0, z1, why) in lift_cases:
        a0 = leg_ik(leg, cu, cv, z0)
        a1 = leg_ik(leg, cu, cv, z1)
        d0, d1 = d_of(cu, cv, z0), d_of(cu, cv, z1)
        got = a1[2] - a0[2]
        want = TIBIA_SIGN * (-1.0 if d1 > d0 else +1.0)   # d up -> straighter
        good = (got > 0) == (want > 0)
        say(f"     d {d0:7.2f} -> {d1:7.2f}  tibia delta {got:+7.2f}  "
            f"expect sign {want:+.0f}  {'ok' if good else 'FAIL'}   [{why}]")
        ok &= good

    # Regression guard for the exact case the original test got wrong.
    old_a = leg_ik(leg, 207.67, 0.0, 0.0)[2]
    old_b = leg_ik(leg, 207.67, 0.0, 20.0)[2]
    say(f"   regression guard (old lift test, RF at 207.67mm reach): tibia "
        f"{old_a:.2f} -> {old_b:.2f}, delta {old_b-old_a:+.2f}")
    say("     history: the original test asserted '>0' under TIBIA_SIGN=-1 and")
    say("     passed by coincidence; under +1 the truth is -1.36; under the now")
    say("     hardware-settled -1 it is +1.36 again.  The oracle is the d-based")
    say("     prediction above, not a memorised sign.")
    ok &= (old_b - old_a) > 0 and abs((old_b - old_a) - 1.36) < 0.05

    # coxa mirror: on a right leg +v (forward) must increase raw coxa,
    # on a left leg it must decrease it.
    rf_fwd = leg_ik("RF", 144.5, 25.0, -155.0)
    lf_fwd = leg_ik("LF", 144.5, 25.0, -155.0)
    d_rf = rf_fwd[0] - leg_ik("RF", 144.5, 0.0, -155.0)[0]
    d_lf = lf_fwd[0] - leg_ik("LF", 144.5, 0.0, -155.0)[0]
    say(f"   foot swept forward: RF coxa delta {d_rf:+.2f} (expect >0), "
        f"LF coxa delta {d_lf:+.2f} (expect <0)")
    ok &= d_rf > 0 and d_lf < 0

    # ---- 4. body-frame transform, which the old test never touched ----
    say("\n4. body_to_leg_local / leg_local_to_body coverage")
    # Use a realistic gait foot position: 144.5mm outboard of the coxa axis,
    # swept 30mm forward, 155mm below the coxa plane.
    want_u, want_v, want_z = 144.5, 30.0, -155.0
    for leg in LEG_ORDER:
        mx, my = MOUNT_POS[leg]
        outboard_sign = -1.0 if leg.startswith("R") else 1.0
        bx = mx + want_v
        by = my + outboard_sign * want_u
        bz = want_z
        u, v, z = body_to_leg_local(leg, bx, by, bz)
        bad = (abs(u - want_u) > 1e-9 or abs(v - want_v) > 1e-9 or abs(z - want_z) > 1e-9)
        rx, ry, rz = leg_local_to_body(leg, u, v, z)
        rt = math.dist((bx, by, bz), (rx, ry, rz))
        say(f"   {leg}: body({bx:+7.1f},{by:+7.1f},{bz:+7.1f}) -> "
            f"u={u:7.2f} v={v:+6.2f} z={z:7.2f}  roundtrip={rt:.2e}  "
            f"{'FAIL' if bad or rt > 1e-9 else 'ok'}")
        ok &= not bad and rt < 1e-9

    # A left and a right leg at mirrored body positions must solve to the same
    # GEOMETRY.  Compare offsets from each leg's own calibration, not raw angles:
    # the raw values legitimately differ because each servo's horn sits on its
    # own spline tooth (RF cal_tibia 141.0 vs LF 151.5, an 10.5 deg spread).
    # Comparing raw angles here would fail even with perfect code.
    #
    # The coxa offsets must CANCEL (right +forward = raw up, left +forward = raw
    # down), and the femur/tibia offsets must MATCH exactly.  This is the check
    # that catches a COXA_SIGN error, and it needs body_to_leg_local to be right.
    say("\n   left/right mirror symmetry (offsets from each leg's own calibration):")
    for r, l in (("RF", "LF"), ("RM", "LM"), ("RR", "LR")):
        mx, my = MOUNT_POS[r]
        bx, bz = mx + 20.0, -150.0
        ar = leg_ik_body(r, bx, my - 144.5, bz)
        al = leg_ik_body(l, bx, -my + 144.5, bz)
        cr, cl = CALIBRATION[r], CALIBRATION[l]
        coxa_cancel = (ar[0] - cr["coxa"]) + (al[0] - cl["coxa"])
        femur_match = (ar[1] - cr["femur"]) - (al[1] - cl["femur"])
        tibia_match = (ar[2] - cr["tibia"]) - (al[2] - cl["tibia"])
        good = abs(coxa_cancel) < 1e-6 and abs(femur_match) < 1e-6 and abs(tibia_match) < 1e-6
        say(f"   {r}/{l}: coxa offsets sum {coxa_cancel:+.2e} (want 0), "
            f"femur offsets differ {femur_match:+.2e}, tibia offsets differ {tibia_match:+.2e} "
            f"(want 0)  {'ok' if good else 'FAIL'}")
        ok &= good

    # ---- 5. workspace limits and rejection of bad targets ----
    say("\n5. Workspace limits and error handling")
    for leg in LEG_ORDER:
        dmn, dmx = leg_reach_limits(leg)
        fmn, fmx = femur_travel(leg)
        say(f"   {leg}: d in [{dmn:6.2f}, {dmx:6.2f}] mm "
            f"({100*dmn/PLANAR_MAX:3.0f}-100% of stretch), "
            f"femur elevation [{fmn:+6.1f}, {fmx:+6.1f}] deg")
    for bad, why in [((144.5, 0.0, 10.0), "beyond full stretch"),
                     ((30.0, 0.0, -20.0), "inside the tibia-travel floor"),
                     ((-50.0, 0.0, -100.0), "inboard of the coxa axis")]:
        try:
            leg_ik("RF", *bad)
            say(f"   ({bad[0]},{bad[1]},{bad[2]}) [{why}] -> ACCEPTED  FAIL")
            ok = False
        except WorkspaceError:
            say(f"   ({bad[0]},{bad[1]},{bad[2]}) [{why}] -> correctly rejected")

    say("\n" + ("SELF-TEST PASS" if ok else "SELF-TEST FAIL"))
    return ok


if __name__ == "__main__":
    import sys
    raise SystemExit(0 if self_test(verbose="--quiet" not in sys.argv) else 1)
