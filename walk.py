#!/usr/bin/env python3
#=================================================================================
# Walk the scorpion: any gait, any number of steps and cycles, any tick timing.
#
# Gaits (see scorpion_gait.GAIT_DEFAULT_STEPS for the default steps per cycle):
#   tripod          3 legs in the air at once, fastest (default 12 steps)
#   ripple_123456   1 leg in the air at a time, order LF RF LM RM LR RR (default 30)
#   ripple_654321   1 leg at a time, order RR LR RM LM RF LF            (default 30)
#   ripple_135246   1 leg at a time, order LF LM LR RF RM RR (legs 1,3,5,2,4,6) (default 30)
#   double_ripple   2 legs at a time, in pairs                          (default 24)
#   caterpillar     1 leg at a time, a wave along the body:
#                   LR, LM, LF (left, rear to front), then RF, RM, RR (right, front to rear)
#                                                                       (default 30)
#
# "Step" here = one gait cycle. --cycles repeats it. --steps is the number of ticks
# per cycle. --tick-ms is the time per tick (the servo move time, default 200 ms).
#   cycle time = steps x tick-ms.   Smaller steps = faster gait but bigger joint moves.
#
# Sequence on the robot (hardware):
#   1. initial stand (stand_initial)           slow move   (--pose-ms)
#   2. walking stand (--pose, default stand)   slow move
#   3. glide into the first gait pose          1 s
#   4. walk --cycles cycles, one tick per tick-ms
#   5. back to the walking stand               slow move
#
# Default is a DRY RUN: it prints the plan and one cycle of joint angles, and sends nothing.
#
# Usage
#   python3 walk.py                                    tripod, dry run
#   python3 walk.py --gait caterpillar --steps 36      dry run with other settings
#   python3 walk.py --hardware --legs RF --cycles 1    one leg, asks YES
#   python3 walk.py --hardware --gait tripod --cycles 3 --tick-ms 250
#=================================================================================

import argparse
import sys
import time

import icc_turn as it
import scorpion_gait as sg

GLIDE_MS = 1000         # glide from the stand into the first gait pose
DEFAULT_TICK_MS = 200   # servo move time = tick period
DEFAULT_POSE_MS = 2000  # slow moves to and from the postures


def build_gait(args):
    steps = args.steps if args.steps is not None else sg.GAIT_DEFAULT_STEPS[args.gait]
    if args.curve:
        return it.make_curve_gait(args.gait, steps, args.stride, args.lift,
                                  args.curve_radius, args.curve, args.backward)
    gait = sg.Gait(args.gait, steps=steps, stride=args.stride, lift=args.lift)
    if args.turn_left:
        gait.set_turn_in_place("CCW")
    elif args.turn_right:
        gait.set_turn_in_place("CW")
    else:
        gait.set_straight(not args.backward)
    return gait


def direction_text(args):
    if args.curve:
        return "curve %s, radius %.0f mm, %s" % (args.curve, args.curve_radius,
                                                  "backward" if args.backward else "forward")
    if args.turn_left:
        return "turn left in place"
    if args.turn_right:
        return "turn right in place"
    return "backward" if args.backward else "forward"


def print_plan(args, gait):
    cycle_s = gait.steps * args.tick_ms / 1000.0
    total_s = cycle_s * args.cycles
    print("Gait %s, %s" % (args.gait, direction_text(args)))
    print("  steps per cycle %d, tick %d ms -> cycle %.1f s, %d cycle(s) = %.1f s"
          % (gait.steps, args.tick_ms, cycle_s, args.cycles, total_s))
    speed, rate = it.body_motion(gait, cycle_s)
    if args.turn_left or args.turn_right:
        print("  turn rate about %.1f deg/s (spin in place, no body travel)" % rate)
    elif args.curve:
        print("  " + it.describe(gait, cycle_s))
    else:
        print("  stride %.0f mm per stance -> body about %.1f mm/s, %.0f mm in total"
              % (args.stride, speed, speed * args.cycles * cycle_s))
    print("  lift %.0f mm, walking stand '%s', start from '%s'"
          % (args.lift, args.pose, args.start_pose))


def print_cycle(gait):
    """One cycle of raw joint angles (coxa/shoulder, femur, tibia) for all six legs."""
    header = "tick  in air     " + "  ".join("%-17s" % n for n in sg.LEG_ORDER)
    print()
    print(header)
    print("-" * len(header))
    for t in range(gait.steps):
        air = [n for n in sg.LEG_ORDER if gait._is_swing(n, t)]
        joints = gait.joints_at(t)
        cells = ["%5.1f %5.1f %5.1f" % joints[n] for n in sg.LEG_ORDER]
        print("%4d  %-11s " % (t, ",".join(air) or "-") + "  ".join("%-17s" % c for c in cells))


def confirm(args):
    print("About to move the REAL servos (%s)." % ("all legs" if args.legs is None else ", ".join(args.legs)))
    try:
        return input("Type YES to walk: ").strip().lower() in ("yes", "y")
    except EOFError:
        return False


def run_hardware(args, gait):
    try:
        bus = sg.ServoControlBus()
    except ImportError:
        print("ServoControl library not found. Run this on the Raspberry Pi.")
        return 2

    legs = args.legs
    try:
        sg.move_to_pose(bus, args.start_pose, move_ms=args.pose_ms, legs=legs)
        print("At initial stand '%s'." % args.start_pose)
        if args.pose != args.start_pose:
            sg.move_to_pose(bus, args.pose, move_ms=args.pose_ms, legs=legs)
            print("At walking stand '%s'." % args.pose)
        bus.send(gait.joints_at(0), move_ms=GLIDE_MS, legs=legs)
        time.sleep(GLIDE_MS / 1000.0)
        sg.walk(gait, bus, cycles=args.cycles, move_ms=args.tick_ms, legs=legs)
        print("Walk done: %d cycle(s)." % args.cycles)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    except sg.GaitError as exc:
        print("\nGait stopped: %s" % exc)
    finally:
        try:
            sg.move_to_pose(bus, args.pose, move_ms=args.pose_ms, legs=legs)
            print("Back at walking stand '%s'." % args.pose)
        except Exception as exc:  # noqa: BLE001  (never hide the first error)
            print("Could not return to the walking stand: %s" % exc)
    return 0


def parse_legs(parser, text):
    if not text:
        return None
    legs = [s.strip().upper() for s in text.split(",") if s.strip()]
    bad = [n for n in legs if n not in sg.LEGS]
    if bad:
        parser.error("unknown leg(s): %s" % ", ".join(bad))
    return legs


def main(argv=None):
    parser = argparse.ArgumentParser(description="Walk the scorpion with any gait.")
    parser.add_argument("--gait", default="tripod", choices=sorted(sg.GAIT_SWING_FRACTION))
    parser.add_argument("--steps", type=int, default=None,
                        help="ticks per cycle, multiple of 6 (default depends on the gait)")
    parser.add_argument("--tick-ms", dest="tick_ms", type=int, default=DEFAULT_TICK_MS,
                        help="time per tick in ms (default %d)" % DEFAULT_TICK_MS)
    parser.add_argument("--cycles", type=int, default=1, help="gait cycles to walk (default 1 = one step)")
    parser.add_argument("--stride", type=float, default=sg.DEFAULT_STRIDE, help="stride per cycle, mm")
    parser.add_argument("--lift", type=float, default=sg.DEFAULT_LIFT, help="lift height, mm")
    direction = parser.add_mutually_exclusive_group()
    direction.add_argument("--backward", action="store_true", help="walk backward")
    direction.add_argument("--turn-left", action="store_true", help="turn left in place")
    direction.add_argument("--turn-right", action="store_true", help="turn right in place")
    parser.add_argument("--curve", choices=("left", "right"),
                        help="walk a curve turning left or right (ICC on that side; see icc_turn.py)")
    parser.add_argument("--curve-radius", dest="curve_radius", type=float,
                        help="curve radius in mm: body centre to the ICC (needed with --curve)")
    parser.add_argument("--pose", default=sg.DEFAULT_POSE, choices=sg.POSE_NAMES,
                        help="walking stand (default: stand)")
    parser.add_argument("--start-pose", default=sg.START_POSE, choices=sg.POSE_NAMES,
                        help="initial stand moved to first on the robot (default: stand_initial)")
    parser.add_argument("--pose-ms", type=int, default=DEFAULT_POSE_MS, help="slow move time, ms")
    parser.add_argument("--hardware", action="store_true", help="send commands to the real servos")
    parser.add_argument("--legs", default="", help="comma list of legs to move, e.g. RF (hardware)")
    args = parser.parse_args(argv)

    if args.cycles < 1 or args.tick_ms < 20:
        parser.error("--cycles must be >= 1 and --tick-ms must be >= 20")
    if args.curve and (args.turn_left or args.turn_right):
        parser.error("--curve cannot be combined with --turn-left / --turn-right")
    if bool(args.curve) != (args.curve_radius is not None):
        parser.error("--curve and --curve-radius must be used together")
    args.legs = parse_legs(parser, args.legs)

    sg.set_active_pose(args.pose)
    try:
        gait = build_gait(args)
    except (ValueError, sg.GaitError) as exc:
        parser.error(str(exc))

    problems = gait.check(cycles=1)
    if problems:
        print("Simulation check failed. Not walking.")
        for p in problems[:10]:
            print("  " + p)
        return 1

    print_plan(args, gait)
    if not args.hardware:
        print_cycle(gait)
        print()
        print("Dry run: nothing was sent. Add --hardware to walk.")
        return 0

    if not confirm(args):
        print("Cancelled. Nothing was sent.")
        return 0
    return run_hardware(args, gait)


if __name__ == "__main__":
    sys.exit(main())
