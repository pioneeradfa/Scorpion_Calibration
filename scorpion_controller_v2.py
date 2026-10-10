#!/usr/bin/env python3
#=================================================================================
# Scorpion hexapod controller v2 - 18 leg servos only
# Raspberry Pi 5 + Hiwonder LSC-32 (library: ServoControl)
#
# Coordinates (body frame, mm): X = forward, Y = left, Z = up.
# Origin = body centre at coxa-axis height. Feet are below it (Z < 0).
#
# Stand pose
#   The calibrated stand angles are the starting point (they reproduce the calibrated
#   foot positions exactly). STAND_MODE selects what is done with them:
#     "ground"     every foot is set to GROUND_CLEARANCE below the coxa axis (6.5 cm),
#                  and the IK gives the angles. Default.
#     "calibrated" the calibrated stand angles are sent unchanged.
#
# Servo pulse: joint angle 0..180 deg -> PULSE_MIN..PULSE_MAX us (linear).
#=================================================================================

import math
import time
from fractions import Fraction

import ServoControl

#---------------------------------------------------------------------------------
# Settings

STAND_MODE = "ground"            # "ground" or "calibrated"
GROUND_CLEARANCE = 65.0          # mm, coxa axis above the ground (6.5 cm)

LINK_1 = 64.25                   # coxa link (coxa axis -> femur joint), mm
LINK_2 = 64.25                   # femur link, mm
LINK_3 = 135.6                   # tibia (knee -> foot tip), mm

PULSE_MIN = 500                  # us
PULSE_MAX = 2500                 # us
WALK_MOVE_TIME_MS = 20           # servo move time during walking
STAND_MOVE_TIME_MS = 800         # servo move time for stand / sit

# Servo IDs on the LSC-32, order [coxa, femur, tibia]
SERVO_IDS = {
    "RF": [29, 30, 31],
    "RM": [3, 4, 5],
    "RR": [9, 1, 2],
    "LF": [24, 23, 22],
    "LM": [13, 14, 15],
    "LR": [10, 11, 12],
}
LEG_ORDER = ["LF", "RF", "LM", "RM", "LR", "RR"]

# Leg data
#   side:  "L" or "R"
#   mount: coxa-axis position in the body frame (x, y), mm
#   cal:   calibrated raw servo angles at the straight pose (coxa, femur, tibia)
#   stand: calibrated stand raw angles (coxa, femur, tibia)
#   sit:   calibrated sit raw angles (coxa, femur, tibia)
LEG_DATA = {
    "LF": dict(side="L", mount=(115.0, 60.0),    cal=(115, 93, 101), stand=(115, 110, 40), sit=(115, 80, 65)),
    "LM": dict(side="L", mount=(0.0, 60.0),      cal=(90, 90, 84),   stand=(90, 110, 20),  sit=(90, 80, 45)),
    "LR": dict(side="L", mount=(-115.0, 60.0),   cal=(87, 90, 91),   stand=(90, 110, 30),  sit=(90, 80, 55)),
    "RF": dict(side="R", mount=(115.0, -60.0),   cal=(98, 88, 94),   stand=(102, 110, 35), sit=(102, 80, 60)),
    "RM": dict(side="R", mount=(0.0, -60.0),     cal=(90, 92, 84),   stand=(90, 110, 35),  sit=(90, 80, 60)),
    "RR": dict(side="R", mount=(-115.0, -60.0),  cal=(103, 87, 90),  stand=(103, 105, 35), sit=(101, 75, 60)),
}

#---------------------------------------------------------------------------------
# Kinematics

def fk(side, cal, raw):
    """Forward kinematics. Returns the foot (x, y, z) relative to the coxa axis."""
    cal_c, cal_f, cal_t = cal
    raw_c, raw_f, raw_t = raw
    theta = math.radians(cal_c - raw_c) if side == "L" else math.radians(180.0 + cal_c - raw_c)
    femur_el = math.radians(raw_f - cal_f)
    knee = math.radians(180.0 + raw_t - cal_t)
    tibia_dir = femur_el - (math.pi - knee)
    fx = LINK_2 * math.cos(femur_el) + LINK_3 * math.cos(tibia_dir)
    fz = LINK_2 * math.sin(femur_el) + LINK_3 * math.sin(tibia_dir)
    r = LINK_1 + fx
    return r * math.sin(theta), r * math.cos(theta), fz

def ik(side, cal, x, y, z):
    """Inverse kinematics (7.d.a equations). x, y, z relative to the coxa axis.
    Returns raw servo angles (coxa, femur, tibia), or None if the foot cannot be reached."""
    cal_c, cal_f, cal_t = cal
    if z >= 0:
        return None
    if side == "L":
        raw_c = cal_c - math.degrees(math.atan2(x, y))      # left: +Y is outward
    else:
        raw_c = cal_c + math.degrees(math.atan2(x, -y))     # right: -Y is outward (no angle wrap)

    xy = math.hypot(x, y) - LINK_1
    ext = math.hypot(xy, z)
    if ext > LINK_2 + LINK_3 or ext < abs(LINK_3 - LINK_2):
        return None
    knee = math.degrees(math.acos((LINK_2**2 + LINK_3**2 - ext**2) / (2 * LINK_2 * LINK_3)))
    phi_p1 = math.degrees(math.atan2(xy, -z))
    phi_p2 = math.degrees(math.acos((LINK_2**2 + ext**2 - LINK_3**2) / (2 * LINK_2 * ext)))
    femur_el = 90.0 - (180.0 - phi_p1 - phi_p2)

    raw_f = cal_f + femur_el
    raw_t = cal_t + knee - 180.0
    raw = (raw_c, raw_f, raw_t)
    if not all(0.0 <= a <= 180.0 for a in raw):
        return None
    return raw

def pulse_us(angle):
    """Joint angle (deg) -> servo pulse (us), clamped to the servo range."""
    return int(min(PULSE_MAX, max(PULSE_MIN, round(500 + angle / 180.0 * 2000))))

#---------------------------------------------------------------------------------
# Leg

class Leg:
    def __init__(self, name):
        data = LEG_DATA[name]
        self.name = name
        self.side = data["side"]
        self.mount_x, self.mount_y = data["mount"]
        self.cal = data["cal"]
        self.sit_raw = data["sit"]
        self.servo_ids = SERVO_IDS[name]

        # Stand foot position from the calibrated stand angles (body frame)
        fx, fy, fz = fk(self.side, self.cal, data["stand"])
        self.base_x = self.mount_x + fx
        self.base_y = self.mount_y + fy
        self.base_z = fz

        if STAND_MODE == "ground":
            self.base_z = -GROUND_CLEARANCE
            raw = ik(self.side, self.cal, fx, fy, self.base_z)
            if raw is None:
                raise ValueError("%s: stand foot at ground height is out of reach" % name)
            self.stand_raw = raw
        else:
            self.stand_raw = tuple(data["stand"])

        # Current target and joint state
        self.x, self.y, self.z = self.base_x, self.base_y, self.base_z
        self.raw = self.stand_raw
        self.step = 0

    def solve(self, x, y, z):
        """Set the foot target (body frame). Returns False and keeps the last pose if unreachable."""
        raw = ik(self.side, self.cal, x - self.mount_x, y - self.mount_y, z)
        if raw is None:
            return False
        self.x, self.y, self.z = x, y, z
        self.raw = raw
        return True

    def set_raw(self, raw):
        """Sets the joint angles directly and updates the foot position to match."""
        self.raw = tuple(raw)
        fx, fy, fz = fk(self.side, self.cal, self.raw)
        self.x, self.y, self.z = self.mount_x + fx, self.mount_y + fy, fz

    def send(self, move_time_ms):
        for servo_id, angle in zip(self.servo_ids, self.raw):
            ServoControl.setPWMServoMove(servo_id, pulse_us(angle), move_time_ms)

LEGS = {name: Leg(name) for name in LEG_ORDER}

#---------------------------------------------------------------------------------
# Gait

GAIT_FRACTION = {"tripod": Fraction(1, 2), "ripple_123456": Fraction(1, 6),
                 "ripple_654321": Fraction(1, 6), "ripple_135246": Fraction(1, 6),
                 "double_ripple": Fraction(1, 3)}

# Phase offset of each leg, as a fraction of the gait cycle
GAIT_OFFSET = {
    "tripod":        {"LF": 0,             "RM": 0,             "LR": 0,
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


class Gait:
    def __init__(self):
        self.stride_length = 45.0    # mm, full stride
        self.stride_height = 30.0    # mm, lift height
        self.icc = [0.0, 10000.0, "CCW"]   # ICC x, y (mm, body frame) and direction
        self.set_gait("tripod", 30)

    def set_gait(self, gait_type, steps=30):
        if gait_type not in GAIT_FRACTION:
            raise ValueError("Unknown gait: %s" % gait_type)
        self.gait_type = gait_type
        self.set_cycle(steps)
        self.step = 0
        self.one_cycle_done = False

    def set_cycle(self, steps):
        """Set the number of steps per cycle and the phase lengths for the gait."""
        self.steps = steps
        f = float(GAIT_FRACTION[self.gait_type])
        span = self.stride_length + 2 * self.stride_height
        self.support = steps * (1 - f)
        self.lift = self.lower = steps * f / span * self.stride_height
        self.swing = steps * f / span * self.stride_length

    def set_icc(self, x, y, direction):
        if direction not in ("CW", "CCW", "Stop"):
            raise ValueError("ICC direction must be CW, CCW or Stop")
        self.icc = [float(x), float(y), direction]

    def _advance_steps(self):
        """Give each leg its step number for this tick, then move the shared step on by one."""
        for name, leg in LEGS.items():
            leg.step = (self.step + int(self.steps * float(GAIT_OFFSET[self.gait_type][name]))) % self.steps
        self.step += 1
        if self.step >= self.steps:
            self.step = 0
            self.one_cycle_done = True
        self.stride_length = 45.0 if self.one_cycle_done else 20.0

    def _target(self, leg, R, Rb, max_R, max_Rb):
        cx, cy, direction = self.icc
        s = leg.step
        if s <= self.support:                                    # support: foot moves on the ICC arc
            stride = self.stride_length * R / max_R
            alpha = 2 * math.asin(0.5 * stride / R)
            beta = math.atan2(leg.y - cy, leg.x - cx)
            beta += alpha / self.support if direction == "CW" else -alpha / self.support
            return cx + R * math.cos(beta), cy + R * math.sin(beta), leg.base_z
        if s <= self.support + self.lift:                        # lift
            return leg.x, leg.y, leg.z + self.stride_height / self.lift
        swing_end = self.support + self.lift + self.swing
        if s <= swing_end:                                       # swing: foot moves to the next spot
            stride = self.stride_length * Rb / max_Rb
            alpha = math.asin(0.5 * stride / Rb)
            gamma = math.atan2(leg.base_y - cy, leg.base_x - cx)
            beta = gamma + alpha if direction == "CCW" else gamma - alpha
            fx = cx + Rb * math.cos(beta)
            fy = cy + Rb * math.sin(beta)
            k = s - (self.support + self.lift)
            return leg.x + (fx - leg.x) / self.swing * k, leg.y + (fy - leg.y) / self.swing * k, leg.z
        if s <= swing_end + self.lower:                          # lower
            return leg.x, leg.y, leg.z - self.stride_height / self.lower
        return leg.x, leg.y, leg.z

    def walk_step(self):
        """One gait tick: compute every foot target, solve the IK and move the servos."""
        if self.icc[2] == "Stop":
            return
        self._advance_steps()
        cx, cy, _ = self.icc
        R = {n: math.hypot(l.x - cx, l.y - cy) for n, l in LEGS.items()}
        Rb = {n: math.hypot(l.base_x - cx, l.base_y - cy) for n, l in LEGS.items()}
        max_R, max_Rb = max(R.values()), max(Rb.values())
        for name, leg in LEGS.items():
            x, y, z = self._target(leg, R[name], Rb[name], max_R, max_Rb)
            if not leg.solve(x, y, z):
                print("%s: target out of reach, pose held" % name)
        move_legs(WALK_MOVE_TIME_MS)

    def stop(self):
        """Stops walking and lowers each foot to its base height."""
        self.icc[2] = "Stop"
        for leg in LEGS.values():
            leg.solve(leg.base_x, leg.base_y, leg.base_z)
        move_legs(STAND_MOVE_TIME_MS)

GAIT = Gait()

#---------------------------------------------------------------------------------
# Poses

def move_legs(move_time_ms):
    for name in LEG_ORDER:
        LEGS[name].send(move_time_ms)

def stand(move_time_ms=STAND_MOVE_TIME_MS):
    """Moves all feet to their stand positions (see STAND_MODE)."""
    for leg in LEGS.values():
        leg.solve(leg.base_x, leg.base_y, leg.base_z)
        if STAND_MODE != "ground":
            leg.set_raw(leg.stand_raw)
    move_legs(move_time_ms)
    time.sleep(move_time_ms / 1000.0)

def sit(move_time_ms=STAND_MOVE_TIME_MS):
    """Moves all legs to the calibrated sit angles."""
    for leg in LEGS.values():
        leg.set_raw(leg.sit_raw)
    move_legs(move_time_ms)
    time.sleep(move_time_ms / 1000.0)

if __name__ == "__main__":
    stand()