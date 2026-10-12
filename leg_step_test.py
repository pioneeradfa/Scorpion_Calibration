#!/usr/bin/env python3
#=================================================================================
# Leg step test for scorpion_controller_v2.
#
# One leg at a time. The chosen leg does a single step:
#     up -> forward -> down -> up -> back -> down (to base)
# and the other five legs stay at their stand positions the whole time.
#
# SAFETY
#   Default is a DRY RUN: nothing is sent to the servos. Each waypoint is checked for
#   IK reach and the commands are counted and printed.
#   Real motion needs --hardware AND typing "yes" at the prompt.
#   Default leg is RF only. All six legs need --legs all.
#   Stop at any time with Ctrl-C (the robot holds its last pose; it does not
#   return the legs to stand).
#
# Values to confirm before hardware (defaults are not verified on the robot):
#   --lift   lift height above base, mm        (default 30 = controller stride_height)
#   --stride forward step length, mm           (default 20 = controller first-cycle stride)
#   --move-ms total time per segment, ms       (default 1500, slow on purpose)
#
# Usage
#   python3 leg_step_test.py                      dry run, RF only
#   python3 leg_step_test.py --legs all           dry run, all six legs in turn
#   python3 leg_step_test.py --hardware           real move, RF only (asks "yes")
#=================================================================================

import argparse
import sys
import time
import types

ORDER_ALL = ["LF", "RF", "LM", "RM", "LR", "RR"]

parser = argparse.ArgumentParser(description="One-leg-at-a-time step test for scorpion_controller_v2.")
parser.add_argument("--hardware", action="store_true", help="send commands to the real servos")
parser.add_argument("--legs", default="RF", help="RF (default), all, or a list such as RF,LF")
parser.add_argument("--lift", type=float, default=30.0, help="lift height, mm")
parser.add_argument("--stride", type=float, default=20.0, help="forward step, mm")
parser.add_argument("--steps", type=int, default=6, help="interpolation steps per segment")
parser.add_argument("--move-ms", type=int, default=1500, help="time per segment, ms")
args = parser.parse_args()

# Servo library: real one on hardware, a logging stub for a dry run.
SENT = []
if args.hardware:
    import ServoControl  # noqa: F401  (must exist on the Pi)
else:
    stub = types.ModuleType("ServoControl")
    stub.setPWMServoMove = lambda pin, pulse, ms: SENT.append((pin, pulse, ms))
    sys.modules["ServoControl"] = stub

import scorpion_controller_v2 as bot  # noqa: E402

if args.legs.lower() == "all":
    LEGS_TO_TEST = ORDER_ALL
else:
    LEGS_TO_TEST = [n.strip().upper() for n in args.legs.split(",") if n.strip()]
    bad = [n for n in LEGS_TO_TEST if n not in bot.LEGS]
    if bad:
        sys.exit("unknown leg(s): %s" % ", ".join(bad))

def waypoints(leg):
    """Foot targets (body frame) for one step, starting and ending at the base position."""
    bx, by, bz = leg.base_x, leg.base_y, leg.base_z
    up = bz + args.lift
    fwd = bx + args.stride
    return [
        (bx, by, up),     # 1. lift
        (fwd, by, up),    # 2. swing forward
        (fwd, by, bz),    # 3. lower
        (fwd, by, up),    # 4. lift
        (bx, by, up),     # 5. swing back
        (bx, by, bz),     # 6. lower to base
    ]

def reachable(leg, x, y, z):
    return bot.ik(leg.side, leg.cal, x - leg.mount_x, y - leg.mount_y, z) is not None

def check_plan(leg):
    bad = [i + 1 for i, (x, y, z) in enumerate(waypoints(leg)) if not reachable(leg, x, y, z)]
    return bad

def run_leg(leg):
    """Moves one leg through its step. Others are not touched."""
    tick_ms = max(1, args.move_ms // args.steps)
    cur = (leg.x, leg.y, leg.z)
    for wp in waypoints(leg):
        for i in range(1, args.steps + 1):
            f = i / float(args.steps)
            x = cur[0] + (wp[0] - cur[0]) * f
            y = cur[1] + (wp[1] - cur[1]) * f
            z = cur[2] + (wp[2] - cur[2]) * f
            if not leg.solve(x, y, z):
                raise RuntimeError("%s: tick target out of reach (%.1f, %.1f, %.1f)" % (leg.name, x, y, z))
            leg.send(tick_ms)
            if args.hardware:
                time.sleep(tick_ms / 1000.0)
        cur = wp

def main():
    print("Mode:", "HARDWARE" if args.hardware else "DRY RUN (no servo commands)")
    print("Legs:", ", ".join(LEGS_TO_TEST), "| lift %.0f mm, stride %.0f mm, %d ms per segment"
          % (args.lift, args.stride, args.move_ms))

    # Plan check before any motion.
    for name in LEGS_TO_TEST:
        bad = check_plan(bot.LEGS[name])
        if bad:
            sys.exit("%s: waypoint(s) %s out of reach. Reduce --lift or --stride. Nothing sent."
                     % (name, ", ".join(map(str, bad))))
    print("Plan check: all waypoints reachable for", ", ".join(LEGS_TO_TEST))

    if args.hardware:
        if input("Type 'yes' to move the robot (stand first, then one leg at a time): ").strip().lower() != "yes":
            sys.exit("cancelled, nothing sent")
        bot.stand()
        print("Standing. Check the robot is stable, then continue.")
    else:
        SENT.clear()
        bot.stand()
        print("Dry run: stand would send %d servo commands." % len(SENT))
        SENT.clear()

    for name in LEGS_TO_TEST:
        leg = bot.LEGS[name]
        if args.hardware:
            if input("Next: %s. Enter to step, q to stop: " % name).strip().lower() == "q":
                print("stopped before", name)
                break
        print("Stepping %s ..." % name)
        run_leg(leg)
        if not args.hardware:
            print("  %s: %d servo commands (dry run)" % (name, len(SENT)))
            SENT.clear()
        # Confirm the leg is back at base and the other legs were not moved.
        ok = abs(leg.x - leg.base_x) + abs(leg.y - leg.base_y) + abs(leg.z - leg.base_z) < 0.5
        print("  %s back at base: %s" % (name, "yes" if ok else "NO"))
        if not ok:
            sys.exit("%s did not return to base. Stopping." % name)
    print("Done.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted. The robot holds its last pose. Check it before continuing.")