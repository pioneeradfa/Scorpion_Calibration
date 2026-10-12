#!/usr/bin/env python3
#=================================================================================
# Keyboard walking control for scorpion_controller_v2 (gaits + ICC).
#
# SAFETY GATES (walking on hardware is refused unless both pass)
#   1. leg_test_passed.json exists. It is written by leg_step_test.py --hardware
#      after all six legs complete one step.
#   2. The selected gait passes the simulated cycle check: no unreachable targets,
#      the foot pattern repeats each cycle, and the body makes progress.
#      Gaits that fail this check cannot be walked on hardware.
#
# Default is a DRY RUN: no servo commands are sent.
#
# Commands (type and press Enter):
#   gait NAME        tripod | ripple_123456 | ripple_654321 | ripple_135246 | double_ripple
#   icc X Y DIR      set the centre of turn (mm, body frame) and CW / CCW / Stop
#   w N              walk N gait ticks with the current ICC (default N = 30)
#   stop             lower feet to base and stop
#   check            run the cycle check for the current gait
#   status           show gait, ICC and foot positions
#   q                quit (feet are lowered to base first)
#
# Usage
#   python3 walk_keyboard.py                  dry run
#   python3 walk_keyboard.py --hardware       real walking (gates apply, asks "yes")
#=================================================================================

import argparse
import contextlib
import io
import json
import os
import sys
import time
import types

parser = argparse.ArgumentParser(description="Keyboard walking control for scorpion_controller_v2.")
parser.add_argument("--hardware", action="store_true", help="send commands to the real servos")
args = parser.parse_args()

SENT = []
if args.hardware:
    import ServoControl  # noqa: F401
else:
    stub = types.ModuleType("ServoControl")
    stub.setPWMServoMove = lambda pin, pulse, ms: SENT.append((pin, pulse, ms))
    sys.modules["ServoControl"] = stub

import scorpion_controller_v2 as bot  # noqa: E402

GAITS = ["tripod", "ripple_123456", "ripple_654321", "ripple_135246", "double_ripple"]
CYCLE_STEPS = 30
CHECK_CYCLES = 5
PERIOD_TOL_MM = 1.0       # foot positions must repeat within this after one cycle
MIN_PROGRESS_MM = 5.0     # the body must move at least this much per cycle

def leg_test_passed():
    if not os.path.exists("leg_test_passed.json"):
        return False
    with open("leg_test_passed.json") as f:
        return bool(json.load(f).get("passed"))

def _reset_pose():
    for n in bot.LEG_ORDER:
        L = bot.LEGS[n]
        L.solve(L.base_x, L.base_y, L.base_z)
    bot.GAIT.step = 0
    bot.GAIT.one_cycle_done = False
    for L in bot.LEGS.values():
        L.step = 0

def cycle_check(gait):
    """Simulates the gait with no servo output. Returns (ok, list of problems)."""
    saved_move = bot.move_legs
    saved = (bot.GAIT.gait_type, bot.GAIT.steps, list(bot.GAIT.icc), bot.GAIT.stride_length)
    bot.move_legs = lambda ms: None          # no servo output during the check
    problems = []
    try:
        icc = list(bot.GAIT.icc)
        if icc[2] == "Stop":                 # check with a real direction only
            icc[2] = "CCW"
        bot.GAIT.set_gait(gait, CYCLE_STEPS)
        bot.GAIT.set_icc(icc[0], icc[1], icc[2])
        _reset_pose()
        starts = []
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            for c in range(CHECK_CYCLES + 1):
                starts.append({n: (bot.LEGS[n].x, bot.LEGS[n].y) for n in bot.LEG_ORDER})
                for _ in range(CYCLE_STEPS):
                    bot.GAIT.walk_step()
        unreachable = buf.getvalue().count("out of reach")
        if unreachable:
            problems.append("%d unreachable target(s) over %d cycles" % (unreachable, CHECK_CYCLES))
        # Periodicity: the foot pattern after cycle k must match cycle k+1.
        drift = []
        for c in range(1, CHECK_CYCLES):
            d = max(abs(starts[c][n][0] - starts[c + 1][n][0]) + abs(starts[c][n][1] - starts[c + 1][n][1])
                    for n in bot.LEG_ORDER)
            drift.append(round(d, 1))
        if drift and max(drift) > PERIOD_TOL_MM:
            problems.append("foot pattern does not repeat each cycle (change per cycle, mm: %s)" % drift)
        # Progress: mean foot shift per cycle relative to the start of the cycle.
        shift = sum(bot.LEGS[n].x - starts[-1][n][0] for n in bot.LEG_ORDER) / 6.0
        if abs(shift) < MIN_PROGRESS_MM:
            problems.append("no progress per cycle (mean foot shift %.1f mm)" % shift)
    finally:
        bot.move_legs = saved_move
        _reset_pose()
        bot.GAIT.gait_type, bot.GAIT.steps = saved[0], saved[1]
        bot.GAIT.icc = saved[2]
        bot.GAIT.stride_length = saved[3]
    return (not problems), problems

def check_gait(gait):
    ok, problems = cycle_check(gait)
    if ok:
        print("CHECK PASSED for %s." % gait)
    else:
        print("CHECK FAILED for %s:" % gait)
        for p in problems:
            print("  - " + p)
    return ok

def show_status():
    print("gait %s | ICC (%.0f, %.0f, %s) | stride %.0f mm, lift %.0f mm"
          % (bot.GAIT.gait_type, bot.GAIT.icc[0], bot.GAIT.icc[1], bot.GAIT.icc[2],
             bot.GAIT.stride_length, bot.GAIT.stride_height))
    for n in bot.LEG_ORDER:
        L = bot.LEGS[n]
        print("  %s (%.1f, %.1f, %.1f)" % (n, L.x, L.y, L.z))

def walk(ticks):
    """Walks `ticks` gait ticks. Refuses on hardware unless both gates pass."""
    if args.hardware:
        if not leg_test_passed():
            print("refused: run leg_step_test.py --hardware (all six legs) first.")
            return
        if not check_gait(bot.GAIT.gait_type):
            print("refused: this gait failed the cycle check. Nothing sent.")
            return
    if bot.GAIT.icc[2] == "Stop":
        print("set a turn centre and direction first: icc X Y CW  (or CCW)")
        return
    for _ in range(ticks):
        bot.GAIT.walk_step()
        if args.hardware:
            time.sleep(0.02)
    print("walked %d ticks (%s)" % (ticks, "hardware" if args.hardware else "dry run"))

def stop():
    bot.GAIT.stop()
    print("stopped; feet at base")

def main():
    print("Mode:", "HARDWARE" if args.hardware else "DRY RUN (no servo commands)")
    if args.hardware:
        if input("Type 'yes' to enable walking on the robot: ").strip().lower() != "yes":
            sys.exit("cancelled")
        bot.stand()
    _reset_pose()
    bot.GAIT.set_gait("tripod", CYCLE_STEPS)
    print("Type 'help' for commands. Gait: tripod.")
    while True:
        try:
            line = input("walk> ").strip()
        except EOFError:
            break
        if not line:
            continue
        p = line.split()
        c = p[0].lower()
        try:
            if c in ("q", "quit", "exit"):
                break
            elif c == "help":
                print(__doc__.split("# Commands")[1].split("# Usage")[0])
            elif c == "gait" and len(p) == 2:
                g = p[1].lower()
                if g not in GAITS:
                    print("unknown gait. Choose:", ", ".join(GAITS))
                    continue
                if args.hardware and not check_gait(g):
                    print("gait not changed.")
                    continue
                if not args.hardware:
                    check_gait(g)
                bot.GAIT.set_gait(g, CYCLE_STEPS)
                print("gait:", g)
            elif c == "icc" and len(p) == 4 and p[3] in ("CW", "CCW", "Stop"):
                bot.GAIT.set_icc(float(p[1]), float(p[2]), p[3])
                print("ICC set")
            elif c == "w":
                walk(int(p[1]) if len(p) > 1 else 30)
            elif c == "stop":
                stop()
            elif c == "check":
                check_gait(bot.GAIT.gait_type)
            elif c == "status":
                show_status()
            else:
                print("unknown command. Type 'help'.")
        except ValueError as e:
            print("input error:", e)
    stop()

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\ninterrupted. Check the robot before continuing.")