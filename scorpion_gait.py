#!/usr/bin/env python3
#=================================================================================
# Scorpion hexapod - leg IK (270-degree servos) and walking Gait class
#
# Hardware: Raspberry Pi 5, Hiwonder LSC-32 servo controller (library: ServoControl),
#           18 x RDS3218 digital servos, 270 deg travel (coxa, femur, tibia per leg).
#
# Reference: skills_12oct.md (sections 2, 3, 4, 5.1, 5.3.1, 6, 7, 10).
#
# ANGLE UNITS
#   "raw"   0..180 units. The calibration (leg_calibration.json), the stand table and
#           the pulse mapping all use raw units.
#   "phys"  physical shaft degrees. phys = raw * 270 / 180 = raw * 1.5.
#           All link geometry (IK and FK) uses phys. This is the correction for the
#           270-degree servo. The old IK treated raw as phys, which is off by 1.5x.
#   Pulse:  pulse_us = 500 + raw / 180 * 2000  (= 500 + phys / 270 * 2000).
#           This assumes the RDS3218 is 500..2500 us over 0..270 deg. Verify on the robot.
#
# COORDINATES
#   Body frame (mm): X = forward, Y = left, Z = up. Origin = body centre at coxa height.
#   Leg frame: origin = that leg's coxa axis. lx = forward, ly = outward, z = up.
#   Feet are below the coxa axis (z < 0).
#
# NEUTRAL (STAND) POSE
#   The recorded stand (skills 5.1, 2026-10-12) is converted to foot positions with FK.
#   Each leg's foot stays at its own recorded height. The stand is NOT the
#   65 mm "ground clearance" of scorpion_controller_v2.py (see gait_ik_notes.md).
#
# GAIT
#   Same phase scheme as skills section 6 (tripod, ripple, double ripple; N steps per cycle).
#   Steering uses the ICC (skills section 7). Each leg moves on a circle about the ICC
#   at a radius set by its neutral position. Stance is first in each leg's cycle, then
#   swing. Swing = smooth lift (sin profile) with horizontal travel back-to-front.
#   Stance and swing are symmetric about the neutral point, so every leg stays on the
#   same circle and the gait has no drift. Start: run_hardware() glides from the stand to
#   joints_at(0) over 1 s before walking (tick 0 can be ~20 mm from the stand).
#   Deviations from skills section 6:
#     - swing angle is the full stride angle (skills 6.4 uses half: open point Q)
#     - swing is one smooth sin-lift phase (not three linear lift/swing/lower phases)
#
# Usage
#   python3 scorpion_gait.py                    simulation checks, no servo output
#   python3 scorpion_gait.py --gait ripple_123456 --cycles 2
#   python3 scorpion_gait.py --pose stand_initial             checks with the first stand
#   python3 scorpion_gait.py --hardware --goto sit --legs RF  move one leg to sit (asks YES)
#   python3 scorpion_gait.py --hardware --legs RF --cycles 1   real servos (asks YES)
#
# POSTURES (raw coxa/shoulder, femur, tibia): stand (default, recorded 2026-10-12),
#   stand_initial (the first manual stand, skills 5.1) and sit (skills 5.2, unverified).
#=================================================================================

import argparse
import json
import math
import os
import sys
import time
from fractions import Fraction

# Hiwonder LSC-32 servo library, used for the RDS3218 servos. Importing it opens the
# serial port on the Raspberry Pi. On a PC it is missing: simulation and tests still run.
try:
    import ServoControl
except ImportError:
    ServoControl = None

#---------------------------------------------------------------------------------
# Settings

SERVO_TRAVEL_DEG = 270.0          # RDS3218 physical travel
RAW_RANGE_DEG = 180.0             # raw units used by calibration and pulse mapping
PHYS_PER_RAW = SERVO_TRAVEL_DEG / RAW_RANGE_DEG      # 1.5

PULSE_MIN = 500                   # us  (0 deg)
PULSE_MAX = 2500                  # us  (270 deg)

# Safe band (skills 5.3.1): joints kept inside 2..268 phys deg
SAFE_PHYS_MIN = 2.0
SAFE_PHYS_MAX = 268.0
RAW_MIN = SAFE_PHYS_MIN / PHYS_PER_RAW        # 1.33
RAW_MAX = SAFE_PHYS_MAX / PHYS_PER_RAW        # 178.67

LINK_1 = 64.25                    # coxa link (coxa axis -> femur joint), mm
LINK_2 = 64.25                    # femur link, mm
LINK_3 = 135.6                    # tibia (knee -> foot tip), mm

REACH_EPS = 1e-6

HERE = os.path.dirname(os.path.abspath(__file__))
CALIBRATION_FILE = os.path.join(HERE, "leg_calibration.json")

LEG_ORDER = ["LF", "RF", "LM", "RM", "LR", "RR"]

# Servo IDs on the LSC-32, order [coxa, femur, tibia] (skills 3.1)
SERVO_IDS = {
    "RF": [29, 30, 31], "RM": [3, 4, 5],    "RR": [9, 1, 2],
    "LF": [24, 23, 22], "LM": [13, 14, 15], "LR": [10, 11, 12],
}

# Coxa-axis mount in the body frame (skills 2.2), mm
MOUNT = {
    "LF": (115.0, 60.0),  "LM": (0.0, 60.0),  "LR": (-115.0, 60.0),
    "RF": (115.0, -60.0), "RM": (0.0, -60.0), "RR": (-115.0, -60.0),
}

# Straight-leg zeros, raw [coxa, femur, tibia] (skills 3.4, leg_calibration.json)
CAL_FALLBACK = {
    "RF": (98, 88, 94), "RM": (90, 92, 84), "RR": (103, 87, 90),
    "LF": (115, 93, 101), "LM": (90, 90, 84), "LR": (87, 90, 91),
}

# Postures, raw [coxa (shoulder), femur, tibia] per leg. Pins per SERVO_IDS.
#   stand          skills 5.1, re-recorded on the robot 2026-10-12. Default; walking neutral.
#   stand_initial  skills 5.1 "superseded original manual table" (the first stand).
#   sit            skills 5.2. The 5.4 sit feet were computed with the old scale and are
#                  stale, so the sit is NOT verified in this model. Check it on the robot.
POSES = {
    "stand": {
        "LF": (113, 116, 39), "LM": (90, 112, 22), "LR": (89, 110, 30),
        "RF": (97, 114, 27),  "RM": (88, 117, 19), "RR": (101, 110, 25),
    },
    "stand_initial": {
        "LF": (115, 110, 40), "LM": (90, 110, 20), "LR": (90, 110, 30),
        "RF": (102, 110, 35), "RM": (90, 110, 35), "RR": (103, 105, 35),
    },
    "sit": {
        "LF": (115, 80, 65), "LM": (90, 80, 45), "LR": (90, 80, 55),
        "RF": (102, 80, 60), "RM": (90, 80, 60), "RR": (101, 75, 60),
    },
}
POSE_NAMES = sorted(POSES)
DEFAULT_POSE = "stand"

# Recorded stand, kept under its old name for existing code
STAND_RAW = POSES["stand"]

# Foot-position check values from skills 5.3.1 (body frame relative to coxa axis)
# (u = outward radial distance, v = forward, z = up) in mm
STAND_TABLE_531 = {
    "RF": (178.8, -4.7, -78.7), "RM": (182.8, -9.6, -78.3), "RR": (178.5, -9.4, -84.4),
    "LF": (187.8, 9.8, -79.2),  "LM": (185.9, 0.0, -82.4),  "LR": (184.3, -9.7, -87.0),
}

#---------------------------------------------------------------------------------
# Errors

class KinematicsError(Exception):
    """Foot target cannot be reached (outside the leg reach or below the coxa axis)."""


class JointLimitError(KinematicsError):
    """A solved joint is outside the safe band."""


#---------------------------------------------------------------------------------
# Calibration

def load_calibration(path=CALIBRATION_FILE):
    """Straight-leg zero per servo (raw), from leg_calibration.json. Falls back to CAL_FALLBACK."""
    if os.path.exists(path):
        with open(path, "r") as f:
            data = json.load(f)
        return {leg: (data[leg]["coxa"]["angle"], data[leg]["femur"]["angle"],
                      data[leg]["tibia"]["angle"]) for leg in LEG_ORDER}
    return dict(CAL_FALLBACK)


CALIBRATION = load_calibration()

#---------------------------------------------------------------------------------
# Pulse mapping

def clamp_raw(raw):
    return max(0.0, min(RAW_RANGE_DEG, raw))


def pulse_us(raw):
    """Raw angle (0..180) -> servo pulse in microseconds (500..2500)."""
    raw = clamp_raw(raw)
    return int(min(PULSE_MAX, max(PULSE_MIN, round(PULSE_MIN + raw / RAW_RANGE_DEG * (PULSE_MAX - PULSE_MIN)))))


def raw_to_phys(raw):
    return raw * PHYS_PER_RAW


#---------------------------------------------------------------------------------
# Leg kinematics

class Leg:
    """One leg. Solves IK/FK in the leg frame and converts to body frame."""

    def __init__(self, name, cal=None, mount=None, servo_ids=None, pose=DEFAULT_POSE):
        self.name = name
        self.side = name[0]                       # "L" or "R"
        self.mount_x, self.mount_y = mount or MOUNT[name]
        self.cal = cal or CALIBRATION[name]       # straight-leg zeros (raw)
        self.servo_ids = servo_ids or SERVO_IDS[name]
        self.set_pose(pose)

    def set_pose(self, pose):
        """Select the posture used as this leg's neutral (for walking and for stand())."""
        if pose not in POSES:
            raise ValueError("Unknown pose %r. Choose from %s" % (pose, POSE_NAMES))
        self.pose = pose
        self.pose_raw = tuple(POSES[pose][self.name])
        # Neutral foot position, body frame, from FK of this posture
        self.neutral = self.fk(self.pose_raw)

    # --- forward kinematics ---------------------------------------------------
    def radial_signed(self, raw):
        """Horizontal distance from the coxa axis to the foot, along the leg. Negative = folded inside."""
        raw_f, raw_t = raw[1], raw[2]
        cal_f, cal_t = self.cal[1], self.cal[2]
        elev = math.radians(PHYS_PER_RAW * (raw_f - cal_f))
        knee = 180.0 + PHYS_PER_RAW * (raw_t - cal_t)
        d = math.radians(PHYS_PER_RAW * (raw_f - cal_f) - (180.0 - knee))
        return LINK_1 + LINK_2 * math.cos(elev) + LINK_3 * math.cos(d)

    def fk_leg(self, raw):
        """Foot in the leg frame (lx forward, ly outward, z up) for raw (coxa, femur, tibia)."""
        raw_c, raw_f, raw_t = raw
        cal_c, cal_f, cal_t = self.cal
        if self.side == "L":
            yaw = PHYS_PER_RAW * (cal_c - raw_c)          # +yaw = forward for left
        else:
            yaw = PHYS_PER_RAW * (raw_c - cal_c)          # +yaw = forward for right
        elev = PHYS_PER_RAW * (raw_f - cal_f)             # femur above horizontal
        knee = 180.0 + PHYS_PER_RAW * (raw_t - cal_t)     # knee interior angle (180 = straight)
        tibia_dir = elev - (180.0 - knee)                 # tibia direction from horizontal

        e, d = math.radians(elev), math.radians(tibia_dir)
        fx = LINK_2 * math.cos(e) + LINK_3 * math.cos(d)  # femur joint -> foot, horizontal
        fz = LINK_2 * math.sin(e) + LINK_3 * math.sin(d)  # femur joint -> foot, vertical
        r = LINK_1 + fx                                   # coxa axis -> foot, horizontal
        yaw_r = math.radians(yaw)
        lx = r * math.sin(yaw_r)
        ly = r * math.cos(yaw_r) if self.side == "L" else -r * math.cos(yaw_r)
        return lx, ly, fz

    def fk(self, raw):
        """Foot in the body frame for raw (coxa, femur, tibia)."""
        lx, ly, z = self.fk_leg(raw)
        return self.mount_x + lx, self.mount_y + ly, z

    # --- inverse kinematics ---------------------------------------------------
    def ik_leg(self, lx, ly, z):
        """Foot in the leg frame -> raw (coxa, femur, tibia). Raises KinematicsError."""
        if z >= 0.0:
            raise KinematicsError("%s: foot must be below the coxa axis (z=%.1f)" % (self.name, z))
        cal_c, cal_f, cal_t = self.cal

        # coxa: yaw of the foot about the coxa axis
        if self.side == "L":
            yaw = math.degrees(math.atan2(lx, ly))
            raw_c = cal_c - yaw / PHYS_PER_RAW
        else:
            yaw = math.degrees(math.atan2(lx, -ly))
            raw_c = cal_c + yaw / PHYS_PER_RAW

        # femur / tibia: planar 2-link solve in the vertical plane through the leg
        xy = math.hypot(lx, ly) - LINK_1                  # horizontal, femur joint -> foot
        ext = math.hypot(xy, z)                           # femur joint -> foot distance
        lo, hi = abs(LINK_3 - LINK_2), LINK_2 + LINK_3
        if ext < lo - REACH_EPS or ext > hi + REACH_EPS:
            raise KinematicsError("%s: foot out of reach (distance %.1f, range %.1f..%.1f)"
                                  % (self.name, ext, lo, hi))
        ext = min(max(ext, lo), hi)

        knee = math.degrees(math.acos(_clip((LINK_2**2 + LINK_3**2 - ext**2) / (2 * LINK_2 * LINK_3))))
        phi_p1 = math.degrees(math.atan2(xy, -z))         # femur-foot line from straight down
        phi_p2 = math.degrees(math.acos(_clip((LINK_2**2 + ext**2 - LINK_3**2) / (2 * LINK_2 * ext))))
        elev = phi_p1 + phi_p2 - 90.0                     # femur above horizontal

        raw_f = cal_f + elev / PHYS_PER_RAW
        raw_t = cal_t + (knee - 180.0) / PHYS_PER_RAW
        return raw_c, raw_f, raw_t

    def ik(self, x, y, z):
        """Body-frame foot target -> raw (coxa, femur, tibia). Checks the safe band."""
        raw = self.ik_leg(x - self.mount_x, y - self.mount_y, z)
        for joint, value in zip(("coxa", "femur", "tibia"), raw):
            if not (RAW_MIN <= value <= RAW_MAX):
                raise JointLimitError("%s %s out of safe band: raw %.1f (phys %.1f, band %.0f..%.0f)"
                                      % (self.name, joint, value, raw_to_phys(value),
                                         SAFE_PHYS_MIN, SAFE_PHYS_MAX))
        return raw


def _clip(c):
    return max(-1.0, min(1.0, c))


LEGS = {name: Leg(name) for name in LEG_ORDER}

#---------------------------------------------------------------------------------
# Gait

# Swing fraction of the cycle, per gait (skills 6.2). Support = 1 - swing.
GAIT_SWING_FRACTION = {
    "tripod": Fraction(1, 2),
    "ripple_123456": Fraction(1, 6),
    "ripple_654321": Fraction(1, 6),
    "ripple_135246": Fraction(1, 6),
    "double_ripple": Fraction(1, 3),
}

# Phase offset of each leg (fraction of the cycle), skills 6.3
GAIT_OFFSET = {
    "tripod": {"LF": 0, "RM": 0, "LR": 0,
               "RF": Fraction(1, 2), "LM": Fraction(1, 2), "RR": Fraction(1, 2)},
    "ripple_123456": {"RR": 0, "LR": Fraction(1, 6), "RM": Fraction(2, 6),
                      "LM": Fraction(3, 6), "RF": Fraction(4, 6), "LF": Fraction(5, 6)},
    "ripple_654321": {"LF": 0, "RF": Fraction(1, 6), "LM": Fraction(2, 6),
                      "RM": Fraction(3, 6), "LR": Fraction(4, 6), "RR": Fraction(5, 6)},
    "ripple_135246": {"LF": Fraction(5, 6), "RF": Fraction(2, 6), "LM": Fraction(4, 6),
                      "RM": Fraction(1, 6), "LR": Fraction(3, 6), "RR": 0},
    "double_ripple": {"LF": Fraction(2, 3), "RM": Fraction(2, 3), "LM": Fraction(1, 3),
                      "RR": Fraction(1, 3), "LR": 0, "RF": 0},
}

DEFAULT_STEPS = 30          # N, ticks per cycle (multiple of 6)
DEFAULT_STRIDE = 45.0       # mm, chord of the outermost leg per stance
DEFAULT_LIFT = 30.0         # mm, swing height
FAR_ICC_Y = 10000.0         # mm, ICC distance for straight walking (skills 7.1)


class GaitError(Exception):
    pass


class Gait:
    """Tripod / ripple / double-ripple walking gait with ICC steering and IK.

    Usage:
        g = Gait("tripod", steps=30, stride=45, lift=30)
        g.set_straight(forward=True)
        joints = g.next_joints()          # dict leg -> (coxa, femur, tibia) raw, one tick
    """

    def __init__(self, gait_type="tripod", steps=DEFAULT_STEPS, stride=DEFAULT_STRIDE,
                 lift=DEFAULT_LIFT, legs=None):
        self.legs = legs or LEGS
        self.stride = float(stride)
        self.lift = float(lift)
        self.tick = 0
        self.icc = (0.0, FAR_ICC_Y)
        self.direction = "Stop"
        self.set_gait(gait_type, steps)
        self.set_straight(forward=True)

    # --- configuration -----------------------------------------------------------
    def set_gait(self, gait_type, steps=None):
        if gait_type not in GAIT_SWING_FRACTION:
            raise ValueError("Unknown gait %r. Choose from %s" % (gait_type, sorted(GAIT_SWING_FRACTION)))
        if steps is None:
            steps = getattr(self, "steps", DEFAULT_STEPS)
        if steps <= 0 or steps % 6 != 0:
            raise ValueError("steps must be a positive multiple of 6 (got %d)" % steps)
        self.gait_type = gait_type
        self.steps = int(steps)
        self.tick = 0

    def set_stride(self, stride):
        if stride < 0:
            raise ValueError("stride must be >= 0")
        self.stride = float(stride)

    def set_lift(self, lift):
        if lift < 0:
            raise ValueError("lift must be >= 0")
        self.lift = float(lift)

    def set_icc(self, x, y, direction):
        """ICC in the body frame (mm) and direction 'CW', 'CCW' or 'Stop'."""
        if direction not in ("CW", "CCW", "Stop"):
            raise ValueError("direction must be CW, CCW or Stop")
        self.icc = (float(x), float(y))
        self.direction = direction

    def set_straight(self, forward=True):
        """Straight walking. ICC far out at +Y: CCW walks forward, CW walks backward."""
        self.set_icc(0.0, FAR_ICC_Y, "CCW" if forward else "CW")

    def set_turn_in_place(self, direction="CCW"):
        """Turn about the body centre. CCW turns left (body yaws CCW), CW turns right."""
        self.set_icc(0.0, 0.0, direction)

    def stop(self):
        self.direction = "Stop"
        self.tick = 0

    # --- geometry (per leg, cached per ICC) -----------------------------------
    def _geometry(self):
        cx, cy = self.icc
        geo = {}
        for name, leg in self.legs.items():
            px, py, pz = leg.neutral
            R = math.hypot(px - cx, py - cy)
            gamma = math.atan2(py - cy, px - cx)
            geo[name] = (R, gamma, pz)
        R_max = max(v[0] for v in geo.values())
        # stance / swing angular span: same for all legs, so the body turns as one rigid unit
        if R_max > 0:
            delta = 2.0 * math.asin(min(1.0, self.stride / (2.0 * R_max)))
        else:
            delta = 0.0
        return geo, delta

    # --- targets -----------------------------------------------------------------
    def foot_targets(self, tick=None):
        """Body-frame foot target for every leg at the given tick (default: current tick)."""
        t = self.tick if tick is None else tick
        if self.direction == "Stop":
            return {name: leg.neutral for name, leg in self.legs.items()}
        geo, delta = self._geometry()
        cx, cy = self.icc
        sigma = 1.0 if self.direction == "CCW" else -1.0
        N = self.steps
        swing_frac = GAIT_SWING_FRACTION[self.gait_type]
        n_stance = N * (1 - swing_frac)                     # ticks in stance
        n_swing = N * swing_frac                            # ticks in swing
        targets = {}
        for name in self.legs:
            R, gamma, pz = geo[name]
            # phase of this leg within the cycle, in ticks (exact with Fractions)
            s = (Fraction(t) + GAIT_OFFSET[self.gait_type][name] * N) % N
            if s < n_stance:
                u = float(s) / float(n_stance)
                phi = gamma + sigma * (delta / 2.0 - u * delta)   # stance: foot sweeps backward
                z = pz
            else:
                u = float(s - n_stance) / float(n_swing)
                phi = gamma - sigma * (delta / 2.0 - u * delta)   # swing: foot sweeps forward
                z = pz + self.lift * math.sin(math.pi * u)        # smooth lift and landing
            targets[name] = (cx + R * math.cos(phi), cy + R * math.sin(phi), z)
        return targets

    def joints_at(self, tick=None):
        """Raw joint angles (coxa, femur, tibia) per leg for one tick. Raises on any failure."""
        targets = self.foot_targets(tick)
        out = {}
        for name, (x, y, z) in targets.items():
            try:
                out[name] = self.legs[name].ik(x, y, z)
            except KinematicsError as exc:
                raise GaitError("tick %s: %s" % (self.tick if tick is None else tick, exc))
        return out

    def neutral_joints(self):
        return {name: leg.pose_raw for name, leg in self.legs.items()}

    def next_joints(self):
        """Joint angles for the current tick, then advance the tick by one."""
        joints = self.joints_at(self.tick)
        self.tick = (self.tick + 1) % self.steps
        return joints

    # --- checks ------------------------------------------------------------------
    def check(self, cycles=2):
        """Simulates the gait with no servo output. Returns a list of problem strings (empty = OK)."""
        problems = []
        saved = (self.tick, self.direction, self.icc, self.gait_type, self.steps)
        try:
            if self.direction == "Stop":
                problems.append("direction is Stop: nothing to check")
                return problems
            for _ in range(cycles * self.steps):
                try:
                    joints = self.joints_at(self.tick)
                except GaitError as exc:
                    problems.append(str(exc))
                    break
                for name, raw in joints.items():
                    for value in raw:
                        if not (RAW_MIN - 1e-6 <= value <= RAW_MAX + 1e-6):
                            problems.append("tick %d %s joint raw %.1f outside safe band" % (self.tick, name, value))
                self.tick = (self.tick + 1) % self.steps
            # periodicity: one cycle later the targets must repeat
            a = self.joints_at(0)
            b = self.joints_at(self.steps)
            for name in a:
                if max(abs(p - q) for p, q in zip(a[name], b[name])) > 1e-6:
                    problems.append("%s does not repeat after one cycle" % name)
            # swing count per tick (must equal 6 * swing fraction)
            want = round(6 * GAIT_SWING_FRACTION[self.gait_type])
            for t in range(self.steps):
                n_swing = sum(1 for name in self.legs
                              if self._is_swing(name, t))
                if n_swing != want:
                    problems.append("tick %d has %d legs in swing (expected %d)" % (t, n_swing, want))
                    break
        finally:
            self.tick, self.direction, self.icc, self.gait_type, self.steps = saved
        return problems

    def _is_swing(self, name, t):
        N = self.steps
        s = (Fraction(t) + GAIT_OFFSET[self.gait_type][name] * N) % N
        return s >= N * (1 - GAIT_SWING_FRACTION[self.gait_type])

    def worst_joint_step(self, cycles=1):
        """Largest raw change of one joint between consecutive ticks (degrees)."""
        saved = (self.tick,)
        worst = 0.0
        try:
            prev = None
            for _ in range(cycles * self.steps + 1):
                j = self.joints_at(self.tick)
                if prev is not None:
                    for name in j:
                        worst = max(worst, max(abs(a - b) for a, b in zip(j[name], prev[name])))
                prev = j
                self.tick = (self.tick + 1) % self.steps
        finally:
            self.tick = saved[0]
        return worst


#---------------------------------------------------------------------------------
# Servo output

class DryRunBus:
    """Records the commands instead of sending them. Used for simulation and tests."""

    def __init__(self):
        self.commands = []   # (servo_id, pulse_us, move_ms)

    def send(self, joints, move_ms, legs=None):
        for name, raw in joints.items():
            if legs and name not in legs:
                continue
            for servo_id, angle in zip(SERVO_IDS[name], raw):
                self.commands.append((servo_id, pulse_us(angle), move_ms))


class ServoControlBus:
    """Sends commands with the Hiwonder ServoControl library (Raspberry Pi only)."""

    def __init__(self):
        if ServoControl is None:
            raise ImportError("ServoControl library not found. Run this on the Raspberry Pi.")
        self._sc = ServoControl

    def send(self, joints, move_ms, legs=None):
        for name, raw in joints.items():
            if legs and name not in legs:
                continue
            for servo_id, angle in zip(SERVO_IDS[name], raw):
                self._sc.setPWMServoMove(servo_id, pulse_us(angle), move_ms)


def set_active_pose(pose):
    """Set the posture used as the neutral for all legs (stand(), walking)."""
    for leg in LEGS.values():
        leg.set_pose(pose)


def posture_joints(pose):
    """Raw joint angles of every leg for a named posture (no change to the active pose)."""
    if pose not in POSES:
        raise ValueError("Unknown pose %r. Choose from %s" % (pose, POSE_NAMES))
    return {name: tuple(POSES[pose][name]) for name in LEG_ORDER}


def move_to_pose(bus, pose, move_ms=1500, legs=None):
    """Move the legs to a named posture and wait for the move to finish."""
    bus.send(posture_joints(pose), move_ms, legs)
    time.sleep(move_ms / 1000.0)


def stand(bus, move_ms=1500, legs=None):
    """Move every leg to the active neutral posture (default: the recorded stand)."""
    move_to_pose(bus, LEGS["LF"].pose, move_ms, legs)


def sit(bus, move_ms=2000, legs=None):
    """Move every leg to the sit posture (skills 5.2). Unverified: check on the robot."""
    move_to_pose(bus, "sit", move_ms, legs)


def walk(gait, bus, cycles=1, move_ms=200, legs=None, on_tick=None):
    """Runs the gait for a number of cycles. move_ms is also the tick period."""
    for _ in range(cycles * gait.steps):
        t0 = time.time()
        joints = gait.next_joints()
        bus.send(joints, move_ms, legs)
        if on_tick:
            on_tick(gait.tick, joints)
        if not isinstance(bus, DryRunBus):
            time.sleep(max(0.0, move_ms / 1000.0 - (time.time() - t0)))


#---------------------------------------------------------------------------------
# Self-checks (no hardware)

def check_ik_round_trip(samples=2000, seed=1):
    """IK(FK(raw)) must give raw back (for raw inside the safe band). Returns the worst error in raw."""
    import random
    rnd = random.Random(seed)
    worst = 0.0
    done = 0
    for _ in range(samples):
        name = rnd.choice(LEG_ORDER)
        leg = LEGS[name]
        # knee must be 0..180 deg (tibia raw <= straight-leg zero), the range the IK can produce
        raw = (rnd.uniform(40, 140), rnd.uniform(60, 150), rnd.uniform(20, leg.cal[2]))
        x, y, z = leg.fk(raw)
        if z >= 0 or leg.radial_signed(raw) <= 0:   # foot folded inside its own coxa link
            continue
        try:
            back = leg.ik(x, y, z)
        except KinematicsError:
            continue
        done += 1
        worst = max(worst, max(abs(a - b) for a, b in zip(back, raw)))
    return worst, done


def check_stand_table():
    """FK of the recorded stand must match skills 5.3.1 (u, v, z)."""
    rows = []
    worst = 0.0
    for name in LEG_ORDER:
        leg = LEGS[name]
        lx, ly, z = leg.fk_leg(POSES["stand"][name])
        u = math.hypot(lx, ly)
        v = lx
        ref = STAND_TABLE_531[name]
        err = max(abs(u - ref[0]), abs(v - ref[1]), abs(z - ref[2]))
        worst = max(worst, err)
        rows.append((name, u, v, z, ref, err))
    return worst, rows


def check_stand_round_trip():
    worst = 0.0
    for name in LEG_ORDER:
        leg = LEGS[name]
        back = leg.ik(*leg.neutral)
        worst = max(worst, max(abs(a - b) for a, b in zip(back, leg.pose_raw)))
    return worst


#---------------------------------------------------------------------------------
# Command line

def check_postures():
    """For every named posture: foot position and whether it is inside reach and the safe band.
    Returns a list of (pose, leg, foot, problem-or-None)."""
    rows = []
    for pose in POSE_NAMES:
        for name in LEG_ORDER:
            leg = LEGS[name]
            raw = POSES[pose][name]
            foot = leg.fk(raw)
            problem = None
            try:
                back = leg.ik(*foot)
                worst = max(abs(a - b) for a, b in zip(back, raw))
                if worst > 1e-6:
                    problem = "IK does not reproduce the posture (error %.2f raw)" % worst
            except KinematicsError as exc:
                problem = str(exc)
            rows.append((pose, name, foot, problem))
    return rows


def _print_joint_table(gait, ticks):
    print("tick  " + "  ".join("%-17s" % n for n in LEG_ORDER))
    for t in ticks:
        j = gait.joints_at(t)
        cells = ["%5.1f %5.1f %5.1f" % j[n] for n in LEG_ORDER]
        print("%4d  " % t + "  ".join("%-17s" % c for c in cells))


def run_checks(args):
    ok = True
    set_active_pose(args.pose)
    print("=" * 74)
    print("Scorpion gait - checks (270-deg servos, phys = 1.5 * raw)")
    print("Active posture (walking neutral): %s" % args.pose)
    print("=" * 74)

    err, rows = check_stand_table()
    print("\n[1] FK of recorded stand vs skills 5.3.1 (u, v, z in mm):")
    for name, u, v, z, ref, e in rows:
        print("    %s  u %.1f (ref %.1f)  v %.1f (ref %.1f)  z %.1f (ref %.1f)" %
              (name, u, ref[0], v, ref[1], z, ref[2]))
    passed = err < 0.5
    ok &= passed
    print("    worst difference %.2f mm -> %s" % (err, "PASS" if passed else "FAIL"))

    err = check_stand_round_trip()
    passed = err < 1e-6
    ok &= passed
    print("\n[2] IK of the neutral feet returns the recorded stand: worst %.2e raw -> %s"
          % (err, "PASS" if passed else "FAIL"))

    err, n = check_ik_round_trip()
    passed = err < 1e-6
    ok &= passed
    print("[3] IK(FK(q)) == q on %d random joint sets: worst %.2e raw -> %s"
          % (n, err, "PASS" if passed else "FAIL"))

    print("\n[4] Gait checks (%s, N=%d, stride %.0f mm, lift %.0f mm):" %
          (args.gait, args.steps, args.stride, args.lift))
    for label, setter in [
        ("forward", lambda g: g.set_straight(True)),
        ("backward", lambda g: g.set_straight(False)),
        ("turn left (in place)", lambda g: g.set_turn_in_place("CCW")),
        ("turn right (in place)", lambda g: g.set_turn_in_place("CW")),
    ]:
        g = Gait(args.gait, steps=args.steps, stride=args.stride, lift=args.lift)
        setter(g)
        problems = g.check(cycles=args.cycles)
        step = g.worst_joint_step(cycles=1)
        passed = not problems
        ok &= passed
        print("    %-22s problems: %d   worst joint change per tick %.1f deg -> %s"
              % (label, len(problems), step, "PASS" if passed else "FAIL"))
        for p in problems[:5]:
            print("        " + p)

    if args.table:
        g = Gait(args.gait, steps=args.steps, stride=args.stride, lift=args.lift)
        g.set_straight(True)
        print("\n[5] Joint angles (raw coxa femur tibia) for forward walking:")
        _print_joint_table(g, range(0, args.steps, max(1, args.steps // 10)))

    print("\n[6] Postures (shoulder = coxa). Each leg must be reachable and inside the safe band:")
    posture_rows = check_postures()
    for pose in POSE_NAMES:
        rows = [r for r in posture_rows if r[0] == pose]
        bad = [r for r in rows if r[3]]
        raws = [v for name in LEG_ORDER for v in POSES[pose][name]]
        lo, hi = min(raws), max(raws)
        print("    %-14s legs ok: %d/6   raw range %d..%d (phys %.0f..%.0f)  -> %s" % (
            pose, len(rows) - len(bad), lo, hi, raw_to_phys(lo), raw_to_phys(hi),
            "PASS" if not bad else "FAIL"))
        for r in bad:
            print("        %s: %s" % (r[1], r[3]))
        if pose == "sit":
            print("        NOTE: sit angles are from skills 5.2. Feet are not verified (5.4 is stale).")
        ok &= not bad
    print("    feet of the active posture (%s):" % args.pose)
    for r in posture_rows:
        if r[0] == args.pose:
            print("        %s  x %7.1f  y %7.1f  z %7.1f" % (r[1], r[2][0], r[2][1], r[2][2]))

    print("\nRESULT: %s" % ("ALL CHECKS PASS" if ok else "SOME CHECKS FAILED"))
    return 0 if ok else 1


def _confirm():
    """Asks for YES on the console. Anything else (or no input) cancels."""
    try:
        return input("Type YES to continue: ").strip() == "YES"
    except EOFError:
        return False


def run_hardware(args):
    if args.legs:
        legs = [s.strip().upper() for s in args.legs.split(",") if s.strip()]
        for name in legs:
            if name not in LEGS:
                print("Unknown leg %s" % name)
                return 2
    else:
        legs = None

    set_active_pose(args.pose)

    if args.goto:
        # only move to one posture, then stop (no walking)
        print("About to move the REAL servos to posture '%s'." % args.goto)
        print("  legs: %s" % ("all" if legs is None else ", ".join(legs)))
        print("  Hold the robot or keep the feet free. Keep power current-limited.")
        if not _confirm():
            print("Cancelled.")
            return 0
        try:
            bus = ServoControlBus()
        except ImportError:
            print("ServoControl library not found. Run this on the Raspberry Pi.")
            return 2
        move_to_pose(bus, args.goto, move_ms=args.goto_ms, legs=legs)
        print("At posture '%s'. Nothing else is sent." % args.goto)
        return 0

    gait = Gait(args.gait, steps=args.steps, stride=args.stride, lift=args.lift)
    gait.set_straight(not args.backward)
    problems = gait.check(cycles=args.cycles)
    if problems:
        print("Simulation check failed. Not walking.")
        for p in problems[:10]:
            print("  " + p)
        return 1

    print("About to move the REAL servos.")
    print("  gait %s, steps %d, stride %.0f mm, lift %.0f mm, cycles %d" %
          (args.gait, args.steps, args.stride, args.lift, args.cycles))
    print("  legs: %s" % ("all" if legs is None else ", ".join(legs)))
    print("  Robot must be propped up or the feet free. Keep power current-limited.")
    if not _confirm():
        print("Cancelled.")
        return 0

    try:
        bus = ServoControlBus()
    except ImportError:
        print("ServoControl library not found. Run this on the Raspberry Pi.")
        return 2

    try:
        stand(bus, legs=legs)
        # glide from the stand into the first gait pose (avoids a jump at the start)
        bus.send(gait.joints_at(0), move_ms=1000, legs=legs)
        time.sleep(1.0)
        walk(gait, bus, cycles=args.cycles, move_ms=args.move_ms, legs=legs)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except GaitError as exc:
        print("\nGait stopped: %s" % exc)
    finally:
        stand(bus, legs=legs)
        print("Back to the stand pose.")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Scorpion gait and IK (270-deg servos).")
    parser.add_argument("--gait", default="tripod", choices=sorted(GAIT_SWING_FRACTION))
    parser.add_argument("--steps", type=int, default=DEFAULT_STEPS, help="ticks per cycle (multiple of 6)")
    parser.add_argument("--stride", type=float, default=DEFAULT_STRIDE, help="stride, mm")
    parser.add_argument("--lift", type=float, default=DEFAULT_LIFT, help="lift height, mm")
    parser.add_argument("--cycles", type=int, default=2, help="cycles to simulate or walk")
    parser.add_argument("--table", action="store_true", help="print joint angles over one cycle")
    parser.add_argument("--hardware", action="store_true", help="send commands to the real servos")
    parser.add_argument("--backward", action="store_true", help="walk backward (hardware mode)")
    parser.add_argument("--legs", default="", help="comma list of legs to move, e.g. RF (hardware mode)")
    parser.add_argument("--move-ms", type=int, default=200, help="servo move time = tick period (hardware)")
    parser.add_argument("--pose", default=DEFAULT_POSE, choices=POSE_NAMES,
                        help="posture used as the walking neutral and for stand (default: stand)")
    parser.add_argument("--goto", default="", choices=[""] + POSE_NAMES,
                        help="hardware: move to this posture and stop (no walking)")
    parser.add_argument("--goto-ms", type=int, default=2000, help="move time for --goto (ms)")
    args = parser.parse_args(argv)
    if args.goto and not args.hardware:
        parser.error("--goto needs --hardware")
    if args.hardware:
        return run_hardware(args)
    return run_checks(args)


if __name__ == "__main__":
    sys.exit(main())
