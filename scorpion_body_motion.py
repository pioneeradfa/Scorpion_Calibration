#!/usr/bin/env python3
#=================================================================================
# Scorpion body motion and walk-to-base helpers
# Works with scorpion_controller_v2.py (18 leg servos, ServoControl library).
#
# Contents
#   Body attitude:   set_attitude() / reset_attitude()  (roll, pitch, yaw)
#   Feet:            put_feet_down(), move_feet_to_base(), leg_to_base(),
#                    legs_to_base(), tripod_to_base(), double_ripple_to_base(),
#                    reposition_to_base()
#
# Coordinates: body frame, mm. X = forward, Y = left, Z = up.
# Attitude angles are in degrees.
# All functions move the real servos. Import this file without calling anything
# to avoid moving the robot.
#=================================================================================

import math
import time

import scorpion_controller_v2 as bot

HELPER_STEP_MS = 50     # servo move time for each interpolation step

# Current body attitude (radians)
_attitude = {"roll": 0.0, "pitch": 0.0, "yaw": 0.0}

#---------------------------------------------------------------------------------
# Attitude

def rotate_point(x, y, z, roll, pitch, yaw):
    """Rotate a body-frame point by roll (about X), pitch (about Y), yaw (about Z),
    applied in that order. Angles in radians. Single-axis results match the old
    controller's calculate_leg_roll / calculate_leg_pitch / calculate_leg_yaw."""
    y, z = y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)
    x, z = x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)
    x, y = x * math.cos(yaw) + y * math.sin(yaw), -x * math.sin(yaw) + y * math.cos(yaw)
    return x, y, z

def _apply(roll, pitch, yaw):
    """Move every foot to its base position rotated by the given attitude (radians)."""
    for name in bot.LEG_ORDER:
        leg = bot.LEGS[name]
        x, y, z = rotate_point(leg.base_x, leg.base_y, leg.base_z, roll, pitch, yaw)
        if not leg.solve(x, y, z):
            raise RuntimeError("%s: attitude out of reach (roll %.1f, pitch %.1f, yaw %.1f deg)"
                               % (name, math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))

def set_attitude(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0, steps=10, step_ms=HELPER_STEP_MS):
    """Tilts the body to the given attitude from the current attitude, in `steps` moves.
    Use only with the feet on the ground at their base positions (stand)."""
    r0, p0, y0 = _attitude["roll"], _attitude["pitch"], _attitude["yaw"]
    r1, p1, y1 = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
    for i in range(1, steps + 1):
        f = i / float(steps)
        roll, pitch, yaw = r0 + (r1 - r0) * f, p0 + (p1 - p0) * f, y0 + (y1 - y0) * f
        _apply(roll, pitch, yaw)
        bot.move_legs(step_ms)
        time.sleep(step_ms / 1000.0)
        _attitude.update(roll=roll, pitch=pitch, yaw=yaw)

def reset_attitude(steps=10, step_ms=HELPER_STEP_MS):
    """Returns the body to level (roll = pitch = yaw = 0)."""
    set_attitude(0.0, 0.0, 0.0, steps, step_ms)

def current_attitude():
    """Returns (roll, pitch, yaw) in degrees."""
    return tuple(math.degrees(_attitude[k]) for k in ("roll", "pitch", "yaw"))

#---------------------------------------------------------------------------------
# Feet helpers

def _send(step_ms):
    bot.move_legs(step_ms)
    time.sleep(step_ms / 1000.0)

def _solve_or_fail(leg, x, y, z):
    if not leg.solve(x, y, z):
        raise RuntimeError("%s: target (%.1f, %.1f, %.1f) out of reach" % (leg.name, x, y, z))

def put_feet_down(steps=5, step_ms=HELPER_STEP_MS):
    """Lowers every foot straight down to its base height."""
    start = {n: bot.LEGS[n] for n in bot.LEG_ORDER}
    z0 = {n: start[n].z for n in bot.LEG_ORDER}
    for i in range(1, steps + 1):
        f = i / float(steps)
        for n in bot.LEG_ORDER:
            leg = start[n]
            _solve_or_fail(leg, leg.x, leg.y, z0[n] + (leg.base_z - z0[n]) * f)
        _send(step_ms)

def move_feet_to_base(steps=20, step_ms=HELPER_STEP_MS):
    """Moves all six feet in a straight line to their base positions, together."""
    start = {n: (bot.LEGS[n].x, bot.LEGS[n].y, bot.LEGS[n].z) for n in bot.LEG_ORDER}
    for i in range(1, steps + 1):
        f = i / float(steps)
        for n in bot.LEG_ORDER:
            leg = bot.LEGS[n]
            x0, y0, z0 = start[n]
            _solve_or_fail(leg, x0 + (leg.base_x - x0) * f, y0 + (leg.base_y - y0) * f,
                           z0 + (leg.base_z - z0) * f)
        _send(step_ms)

def _lift_move_lower(names, steps, step_ms):
    """Lifts the named legs by the gait stride height, moves them to their base X/Y,
    then lowers them to base height. All named legs move together."""
    lift = bot.GAIT.stride_height
    start = {n: (bot.LEGS[n].x, bot.LEGS[n].y, bot.LEGS[n].z) for n in names}
    # 1. lift
    for i in range(1, steps + 1):
        for n in names:
            leg = bot.LEGS[n]
            x0, y0, z0 = start[n]
            _solve_or_fail(leg, x0, y0, z0 + lift * i / float(steps))
        _send(step_ms)
    # 2. move X/Y to base (raised)
    for i in range(1, steps + 1):
        f = i / float(steps)
        for n in names:
            leg = bot.LEGS[n]
            x0, y0, z0 = start[n]
            _solve_or_fail(leg, x0 + (leg.base_x - x0) * f, y0 + (leg.base_y - y0) * f, z0 + lift)
        _send(step_ms)
    # 3. lower to base height
    for i in range(1, steps + 1):
        f = i / float(steps)
        for n in names:
            leg = bot.LEGS[n]
            _solve_or_fail(leg, leg.base_x, leg.base_y, start[n][2] + lift + (leg.base_z - start[n][2] - lift) * f)
        _send(step_ms)

def leg_to_base(name, steps=10, step_ms=HELPER_STEP_MS):
    """Moves one leg to its base position: lift, move, lower."""
    _lift_move_lower([name], steps, step_ms)

def legs_to_base(names, steps=5, step_ms=HELPER_STEP_MS):
    """Moves a group of legs to their base positions together (lift, move, lower)."""
    _lift_move_lower(list(names), steps, step_ms)

def tripod_to_base(speed=5):
    """Tripod groups in turn: [LF, RM, LR] then [RF, LM, RR]."""
    legs_to_base(["LF", "RM", "LR"], speed)
    legs_to_base(["RF", "LM", "RR"], speed)

def double_ripple_to_base(speed=5):
    """Double ripple pairs in turn: [LF, RM], [LM, RR], [LR, RF]."""
    legs_to_base(["LF", "RM"], speed)
    legs_to_base(["LM", "RR"], speed)
    legs_to_base(["LR", "RF"], speed)

def reset_gait():
    """Resets the gait step counters to the start of a cycle."""
    bot.GAIT.step = 0
    bot.GAIT.one_cycle_done = False
    for leg in bot.LEGS.values():
        leg.step = 0

def reposition_to_base(speed=5):
    """Puts every foot down, moves each leg to its base in turn, then resets the gait."""
    put_feet_down(speed)
    for n in bot.LEG_ORDER:
        leg_to_base(n, speed)
    reset_gait()

if __name__ == "__main__":
    print("Import this module and call set_attitude(), reset_attitude(), reposition_to_base(), ...")