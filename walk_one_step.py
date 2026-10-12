#!/usr/bin/env python3
#=================================================================================
# One complete tripod walking step (one full gait cycle) for the scorpion.
#
# A step here is one full tripod cycle: each leg swings once and the body moves
# forward by one stride (45 mm). With 12 ticks at 200 ms that is 2.4 s.
#
# Sequence on the robot (hardware mode):
#   1. initial stand  (stand_initial)       slow move
#   2. walking stand  (stand)               slow move
#   3. glide into the first step pose       1 s
#   4. one full tripod cycle                12 ticks x 200 ms
#   5. back to the walking stand            slow move
#
# Default is a DRY RUN: it prints the joint angles of the step and sends nothing.
#
# Usage
#   python3 walk_one_step.py                    dry run (prints the step)
#   python3 walk_one_step.py --backward         dry run, one step backward
#   python3 walk_one_step.py --hardware         real servos (asks YES)
#   python3 walk_one_step.py --hardware --legs RF   one leg only (first test)
#=================================================================================

import argparse
import sys
import time

import scorpion_gait as sg

GAIT = "tripod"
STRIDE_MM = 45.0
LIFT_MM = 30.0
TICK_MS = 200          # servo move time = tick period
POSE_MS = 2000         # slow moves to and from the postures
GLIDE_MS = 1000        # glide from the stand into the first step pose


def build_gait(backward=False):
    gait = sg.Gait(GAIT, steps=sg.GAIT_DEFAULT_STEPS[GAIT], stride=STRIDE_MM, lift=LIFT_MM)
    gait.set_straight(not backward)
    return gait


def print_step(gait):
    """Dry run: one row per tick, with the swinging legs and the raw joint angles of all six legs."""
    print("One tripod step: %d ticks x %d ms = %.1f s, stride %.0f mm, lift %.0f mm"
          % (gait.steps, TICK_MS, gait.steps * TICK_MS / 1000.0, STRIDE_MM, LIFT_MM))
    print("Walking stand (neutral): %s    Direction: %s" %
          (sg.DEFAULT_POSE, "backward" if gait.icc[1] > 0 and gait.direction == "CW" else "forward"))
    print()
    header = "tick  swing      " + "  ".join("%-17s" % n for n in sg.LEG_ORDER)
    print(header)
    print("-" * len(header))
    for t in range(gait.steps):
        swing = [n for n in sg.LEG_ORDER if gait._is_swing(n, t)]
        joints = gait.joints_at(t)
        cells = ["%5.1f %5.1f %5.1f" % joints[n] for n in sg.LEG_ORDER]
        print("%4d  %-10s " % (t, ",".join(swing)) + "  ".join("%-17s" % c for c in cells))
    print()
    print("Angles are raw (coxa/shoulder, femur, tibia). Feet return to their starting spots.")


def confirm():
    try:
        return input("Type YES to take one step on the REAL robot: ").strip().lower() in ("yes", "y")
    except EOFError:
        return False


def run_hardware(gait, legs):
    print("About to move the REAL servos for ONE step.")
    print("  1. initial stand '%s' (%d ms)" % (sg.START_POSE, POSE_MS))
    print("  2. walking stand '%s' (%d ms)" % (sg.DEFAULT_POSE, POSE_MS))
    print("  3. glide to step start (%d ms), then one tripod cycle (%d ticks x %d ms)"
          % (GLIDE_MS, gait.steps, TICK_MS))
    print("  4. back to '%s' (%d ms)" % (sg.DEFAULT_POSE, POSE_MS))
    print("  legs: %s" % ("all" if legs is None else ", ".join(legs)))
    print("  Robot must be propped up or the feet free. Keep power current-limited.")
    if not confirm():
        print("Cancelled. Nothing was sent.")
        return 0

    try:
        bus = sg.ServoControlBus()
    except ImportError:
        print("ServoControl library not found. Run this on the Raspberry Pi.")
        return 2

    try:
        sg.move_to_pose(bus, sg.START_POSE, move_ms=POSE_MS, legs=legs)
        print("At initial stand.")
        sg.move_to_pose(bus, sg.DEFAULT_POSE, move_ms=POSE_MS, legs=legs)
        print("At walking stand.")
        bus.send(gait.joints_at(0), move_ms=GLIDE_MS, legs=legs)
        time.sleep(GLIDE_MS / 1000.0)
        sg.walk(gait, bus, cycles=1, move_ms=TICK_MS, legs=legs)
        print("Step done.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except sg.GaitError as exc:
        print("\nGait stopped: %s" % exc)
    finally:
        try:
            sg.move_to_pose(bus, sg.DEFAULT_POSE, move_ms=POSE_MS, legs=legs)
            print("Back at walking stand.")
        except Exception as exc:  # noqa: BLE001  (never hide the first error)
            print("Could not return to the walking stand: %s" % exc)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="One complete tripod walking step.")
    parser.add_argument("--hardware", action="store_true", help="send commands to the real servos")
    parser.add_argument("--backward", action="store_true", help="step backward")
    parser.add_argument("--legs", default="", help="comma list of legs to move, e.g. RF (hardware)")
    args = parser.parse_args(argv)

    legs = None
    if args.legs:
        legs = [s.strip().upper() for s in args.legs.split(",") if s.strip()]
        bad = [n for n in legs if n not in sg.LEGS]
        if bad:
            parser.error("unknown leg(s): %s" % ", ".join(bad))

    sg.set_active_pose(sg.DEFAULT_POSE)
    gait = build_gait(backward=args.backward)

    problems = gait.check(cycles=1)
    if problems:
        print("Simulation check failed. Not walking.")
        for p in problems[:10]:
            print("  " + p)
        return 1

    if not args.hardware:
        print_step(gait)
        return 0
    return run_hardware(gait, legs)


if __name__ == "__main__":
    sys.exit(main())
