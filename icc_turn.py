#!/usr/bin/env python3
#=================================================================================
# Curved walking and turning with an Instantaneous Centre of Curvature (ICC).
#
# Reference: skills section 7 ("Gait with an Instantaneous Centre of Curvature").
#   7.1  One ICC point in the body X-Y plane (+X forward, +Y left, origin = body centre)
#        plus a direction (CW / CCW). All feet sweep circles about that one point.
#        ICC far ahead/behind = straight walking; ICC at the body centre = spin in place.
#   7.2  Each leg's stride is scaled by its distance from the ICC:
#          legStride = stride * legDist / maxDist
#   7.3  Arc angle of one stance / swing:
#          alpha_total = 2 * asin(0.5 * legStride / legDist)
#          alpha_step  = alpha_total / stepsInPhase
#   7.4  Each step the foot's bearing about the ICC moves by alpha_step (CW: +, CCW: -),
#        radius fixed. Z comes from the normal lift profile, not from the ICC.
#   7.5  Guards: ICC on a foot (legDist = 0) and a stride longer than the outer leg's
#        circle (asin domain) are refused with a clear message, not silently clamped.
#
# How this relates to scorpion_gait.Gait: the same circle geometry is used, so both give the
# same foot positions (test_icc_turn checks this). IccTurnGait writes the section 7 steps out
# explicitly, adds the 7.5 guards, and gives the body speed and turn rate from the geometry.
#
# Direction convention (checked in the tests):
#   ICC on the LEFT  (+Y) + CCW -> walks forward, curving LEFT.   (curve left)
#   ICC on the RIGHT (-Y) + CW  -> walks forward, curving RIGHT.  (curve right)
#   Same ICC with CW / CCW -> walks backward instead (--backward).
#   ICC at (0, 0) + CCW -> spin left in place.   CW -> spin right in place.
#
# Usage (dry run, prints the section 7 numbers and one cycle of joints):
#   python3 icc_turn.py --curve left --radius 300
#   python3 icc_turn.py --curve right --radius 200 --backward --gait tripod
#=================================================================================

import argparse
import math
import sys
from fractions import Fraction

import scorpion_gait as sg

MIN_ICC_TO_FOOT_MM = 5.0       # 7.5: ICC closer than this to a foot is refused
TURN_RADIUS_LIMIT_MM = 2000.0  # above this the path counts as straight for the speed display


class IccError(sg.GaitError):
    """The ICC or stride cannot be walked (section 7.5). Subclass of GaitError so the usual
    handlers catch it."""


#---------------------------------------------------------------------------------
# Section 7.2 - 7.3: per-leg stride and arc angle

def leg_distances(feet_xy, icc):
    """7.2 legDistanceFromICC for every leg. feet_xy: name -> (x, y) in body frame."""
    cx, cy = icc
    dist = {}
    for name, (x, y) in feet_xy.items():
        d = math.hypot(x - cx, y - cy)
        if d < MIN_ICC_TO_FOOT_MM:
            raise IccError("ICC (%.0f, %.0f) is on the %s foot (%.1f mm away). Move the ICC."
                           % (cx, cy, name, d))
        dist[name] = d
    return dist


def scaled_strides(feet_xy, icc, stride):
    """7.2: legStride = stride * legDist / maxDist. Returns name -> (legDist, legStride)."""
    if stride < 0:
        raise IccError("stride must be >= 0")
    dist = leg_distances(feet_xy, icc)
    d_max = max(dist.values())
    if stride > 2.0 * d_max + 1e-9:
        raise IccError("stride %.0f mm is longer than the outer leg's circle (%.0f mm across). "
                       "Use a stride up to %.0f mm or a larger radius."
                       % (stride, 2.0 * d_max, 2.0 * d_max))
    return {name: (d, stride * d / d_max) for name, d in dist.items()}


def alpha_total(leg_stride, leg_dist):
    """7.3: arc angle (rad) of one stance or swing for one leg. Clamped only to rounding error."""
    ratio = 0.5 * leg_stride / leg_dist
    if ratio > 1.0 + 1e-9:
        raise IccError("leg stride %.1f mm does not fit a circle of radius %.1f mm"
                       % (leg_stride, leg_dist))
    return 2.0 * math.asin(min(1.0, ratio))


def alpha_step(alpha, steps_in_phase):
    """7.3: angle moved per step within one phase."""
    return alpha / steps_in_phase


#---------------------------------------------------------------------------------
# Section 7.4: the gait with an ICC

class IccTurnGait(sg.Gait):
    """Gait.foot_targets rewritten with the section 7 steps. Phase timing, lift and IK are inherited."""

    def foot_targets(self, tick=None):
        t = self.tick if tick is None else tick
        if self.direction == "Stop":
            return {name: leg.neutral for name, leg in self.legs.items()}

        cx, cy = self.icc
        sigma = 1.0 if self.direction == "CCW" else -1.0    # CCW: bearing decreases
        N = self.steps
        swing = sg.GAIT_SWING_FRACTION[self.gait_type]
        n_stance = float(N * (1 - swing))                   # steps in stance phase
        n_swing = float(N * swing)                          # steps in swing phase

        feet = {name: (leg.neutral[0], leg.neutral[1]) for name, leg in self.legs.items()}
        scaled = scaled_strides(feet, self.icc, self.stride)  # 7.2
        targets = {}
        for name, leg in self.legs.items():
            px, py, pz = leg.neutral
            dist, leg_stride = scaled[name]
            alpha = alpha_total(leg_stride, dist)                                  # 7.3
            a_stance = alpha_step(alpha, n_stance)
            a_swing = alpha_step(alpha, n_swing)
            beta0 = math.atan2(py - cy, px - cx)           # bearing of the foot about the ICC

            s = (Fraction(t) + sg.GAIT_OFFSET[self.gait_type][name] * N) % N
            if s < N * (1 - swing):
                # stance: the foot starts at the front of its arc and steps backward (7.4)
                k = float(s)
                beta = beta0 + sigma * (alpha / 2.0 - k * a_stance)
                z = pz
            else:
                # swing: the foot starts at the back and steps forward again (7.4)
                k = float(s - N * (1 - swing))
                beta = beta0 - sigma * (alpha / 2.0 - k * a_swing)
                z = pz + self.lift * math.sin(math.pi * k / n_swing)   # lift profile, section 6
            targets[name] = (cx + dist * math.cos(beta), cy + dist * math.sin(beta), z)
        return targets

    def check(self, cycles=2):
        try:
            return super().check(cycles)
        except IccError as exc:
            return [str(exc)]


#---------------------------------------------------------------------------------
# Building a curve, and the body motion it gives

def make_curve_gait(gait_type, steps, stride, lift, radius, side, backward=False):
    """Walk a curve. side: 'left' or 'right' (the side the robot turns toward).
    radius: distance from the body centre to the ICC (mm). radius 0 = spin in place (use 'left'/'right'
    turn-in-place in walk.py instead)."""
    if side not in ("left", "right"):
        raise ValueError("curve side must be 'left' or 'right'")
    if radius <= 0:
        raise ValueError("curve radius must be > 0 mm")
    icc = (0.0, float(radius)) if side == "left" else (0.0, -float(radius))
    forward = not backward
    if side == "left":
        direction = "CCW" if forward else "CW"
    else:
        direction = "CW" if forward else "CCW"
    gait = IccTurnGait(gait_type, steps=steps, stride=stride, lift=lift)
    gait.set_icc(icc[0], icc[1], direction)
    return gait


def body_motion(gait, cycle_s):
    """Body speed (mm/s) and turn rate (deg/s) from the geometry.

    In stance the feet are planted, so the body moves while they are down: each stance moves the
    body along an arc of angle delta about the ICC. Stance lasts (1 - swing fraction) of the cycle.
    speed = delta * |ICC| / stance_time; turn rate = delta / stance_time (0 for straight walking)."""
    _, delta = gait._geometry()
    stance_s = cycle_s * (1 - sg.GAIT_SWING_FRACTION[gait.gait_type])
    radius = math.hypot(*gait.icc)
    if radius >= TURN_RADIUS_LIMIT_MM:
        # straight walking: the body moves one stride per stance (the ICC is only a stand-in for infinity)
        return gait.stride / stance_s, 0.0
    speed = delta * radius / stance_s
    rate = math.degrees(delta) / stance_s
    return speed, rate


def describe(gait, cycle_s):
    speed, rate = body_motion(gait, cycle_s)
    if math.hypot(*gait.icc) < TURN_RADIUS_LIMIT_MM:
        return "ICC (%.0f, %.0f) %s: speed %.1f mm/s, turn %.1f deg/s" % (
            gait.icc[0], gait.icc[1], gait.direction, speed, rate)
    return "straight: speed %.1f mm/s" % speed


#---------------------------------------------------------------------------------
# Command line (dry run only; walk.py sends servo commands)

def main(argv=None):
    parser = argparse.ArgumentParser(description="ICC curve / turn planner (dry run).")
    parser.add_argument("--gait", default="tripod", choices=sorted(sg.GAIT_SWING_FRACTION))
    parser.add_argument("--steps", type=int, default=None)
    parser.add_argument("--curve", choices=("left", "right"), required=True)
    parser.add_argument("--radius", type=float, required=True, help="mm, body centre to ICC")
    parser.add_argument("--stride", type=float, default=sg.DEFAULT_STRIDE)
    parser.add_argument("--lift", type=float, default=sg.DEFAULT_LIFT)
    parser.add_argument("--tick-ms", dest="tick_ms", type=int, default=200)
    parser.add_argument("--backward", action="store_true")
    args = parser.parse_args(argv)

    steps = args.steps or sg.GAIT_DEFAULT_STEPS[args.gait]
    sg.set_active_pose(sg.DEFAULT_POSE)
    try:
        gait = make_curve_gait(args.gait, steps, args.stride, args.lift, args.radius,
                               args.curve, args.backward)
    except ValueError as exc:
        parser.error(str(exc))

    feet = {n: (gait.legs[n].neutral[0], gait.legs[n].neutral[1]) for n in sg.LEG_ORDER}
    try:
        scaled = scaled_strides(feet, gait.icc, gait.stride)
    except IccError as exc:
        print("Not walking: %s" % exc)
        return 1
    print("Section 7 numbers (stride %.0f mm, ICC %.0f, %.0f, %s):"
          % (args.stride, gait.icc[0], gait.icc[1], gait.direction))
    print("  %-3s  %9s  %10s  %10s  %14s" % ("leg", "legDist", "legStride", "alpha_tot", "step (stance)"))
    n_phase = steps * (1 - sg.GAIT_SWING_FRACTION[args.gait])
    for n in sg.LEG_ORDER:
        d, ls = scaled[n]
        a = alpha_total(ls, d)
        print("  %-3s  %7.1f mm  %8.1f mm  %8.2f deg  %8.3f deg" % (n, d, ls, math.degrees(a),
                                                                  math.degrees(a / n_phase)))
    problems = gait.check(cycles=1)
    if problems:
        print("Simulation check failed. Not walking.")
        for p in problems[:10]:
            print("  " + p)
        return 1
    cycle_s = steps * args.tick_ms / 1000.0
    print(describe(gait, cycle_s))
    print("Dry run: nothing was sent.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
