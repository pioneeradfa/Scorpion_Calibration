#!/usr/bin/env python3
"""
Scorpion hexapod - six-leg walking bring-up CLI.  Legs only; the tail and claw
channels are allocated in servo_lsc24.py but nothing here drives them.

Typical order of use
--------------------
  1. python3 walk.py --self-test          pure math, no hardware, run anywhere
  2. python3 walk.py --check              margins + torque for the standing pose
  3. python3 walk.py --trajectory         eyeball the foot path
  4. python3 walk.py --dry-run --cycles 1 compute a whole walk, send nothing
  5. python3 walk.py --identify           map channels to servos on the bench
  6. python3 walk.py --belly              robot propped up: fold to belly-down
  7. python3 walk.py --stand              rise to the standing pose
  8. python3 walk.py --walk 2             two gait cycles, then sit

Steps 1-4 need no hardware and no ServoControl.py.  Steps 5-8 refuse to move
without an explicit --yes (or an interactive confirmation), because the first
thing that happens is 18 servos moving at once.

--force-dry makes steps 5-8 compute and pace the motion but send nothing.  They
still sleep in real time, so --demo --force-dry takes about 10 s end to end --
that is deliberate, it lets you feel the actual gait speed before committing to
it on hardware.

Tune the geometry with --reach / --height / --stride / --lift / --mass and
re-run --check until the margins look right for your actual build.
"""

from __future__ import annotations

import argparse
from dataclasses import replace

import ik_module
import tripod_gait
from servo_lsc24 import (
    DEGREE_SAFE_MAX,
    DEGREE_SAFE_MIN,
    LEG_CHANNELS,
    LEG_ORDER,
    ServoBus,
    all_leg_channel_ids,
    tx_ms,
)
from tripod_gait import GaitConfig


def build_config(a: argparse.Namespace) -> GaitConfig:
    cfg = GaitConfig()
    for field, value in (("reach_out", a.reach), ("body_height", a.height),
                         ("stride", a.stride), ("lift", a.lift),
                         ("cycle_ms", a.cycle), ("sub_phases", a.phases),
                         ("mass_kg", a.mass)):
        if value is not None:
            cfg = replace(cfg, **{field: value})
    return cfg


def confirm(a: argparse.Namespace, what: str) -> bool:
    if a.yes:
        return True
    print(f"\nAbout to {what}.")
    print("Make sure the robot is clear, propped or on the floor, and the")
    print("servo supply is connected with the Pi's ground common to it.\n")
    try:
        return input("Type 'go' to proceed: ").strip().lower() == "go"
    except (EOFError, KeyboardInterrupt):
        print("\naborted")
        return False


# ============================================================
# ACTIONS
# ============================================================
def cmd_self_test(_: argparse.Namespace) -> int:
    return 0 if ik_module.self_test() else 1


def cmd_check(a: argparse.Namespace) -> int:
    cfg = build_config(a)
    try:
        rep = tripod_gait.print_report(cfg)
    except (ValueError, ik_module.WorkspaceError) as e:
        print(f"\nNOT FEASIBLE: {e}")
        return 1
    return 0 if rep.feasible else 2


def cmd_trajectory(a: argparse.Namespace) -> int:
    cfg = build_config(a)
    tripod_gait.describe_trajectory(cfg)
    return 0


def cmd_dry_run(a: argparse.Namespace) -> int:
    cfg = build_config(a)
    bus = ServoBus(dry_run=True)
    tripod_gait.print_report(cfg)
    print(f"\nComputing {a.cycles} gait cycle(s) with no hardware attached...")
    segs = tripod_gait.walk(bus, cfg, cycles=a.cycles, dry_report=True)
    print(f"  {len(segs)} segments computed, {len(segs[0])} channels each, "
          f"{cfg.step_ms:.0f} ms per segment")
    print(f"  total body travel = {a.cycles * cfg.stride:.0f} mm forward")
    print(f"  serial cost = {len(segs) * tx_ms(18):.0f} ms of wire time "
          f"inside {len(segs) * cfg.step_ms:.0f} ms of motion")

    first, last = segs[0], segs[-1]
    print("\n  first segment (channel: degrees)")
    for leg in LEG_ORDER:
        ch = LEG_CHANNELS[leg]
        print(f"    {leg}: " + "  ".join(f"{c}={last[c]:7.2f}" for c in ch))
    print("\n  worst-case angle seen per channel over the whole walk:")
    for leg in LEG_ORDER:
        for name, c in zip(("coxa", "femur", "tibia"), LEG_CHANNELS[leg]):
            vals = [s[c] for s in segs]
            print(f"    {leg} {name:6s} ch{c:>2}: {min(vals):7.2f} .. {max(vals):7.2f}  "
                  f"(safe band {DEGREE_SAFE_MIN:.0f}-{DEGREE_SAFE_MAX:.0f})")
    return 0


def cmd_identify(a: argparse.Namespace) -> int:
    """Step each leg channel through two angles so the wiring can be mapped.

    Disconnect every servo except the one you are identifying, or prop the robot
    up so nothing can walk off the bench.  Note that channel 0 is never used:
    Hiwonder's driver remaps id<1 to 254, which broadcasts to all 24 servos.
    """
    if not confirm(a, "sweep each leg channel 1-18 one at a time"):
        return 1
    bus = ServoBus(dry_run=a.force_dry)
    mid = 135.0
    print(f"\nSweeping {len(all_leg_channel_ids())} channels. "
          f"Watch which servo moves and note it against this table.\n")
    print("  Expected wiring (edit LEG_CHANNELS in servo_lsc24.py if yours differs):")
    for leg in LEG_ORDER:
        c = LEG_CHANNELS[leg]
        print(f"    {leg}: coxa={c[0]:>2}  femur={c[1]:>2}  tibia={c[2]:>2}")
    print()
    for leg in LEG_ORDER:
        for name, ch in zip(("coxa", "femur", "tibia"), LEG_CHANNELS[leg]):
            for deg in (mid - 20.0, mid + 20.0, mid):
                bus.send_one(ch, deg, duration_ms=600)
            print(f"  channel {ch:>2} = {leg} {name}  -- did that servo just move?")
            if not a.yes:
                input("     press Enter for the next channel...")
    return 0


def cmd_belly(a: argparse.Namespace) -> int:
    if not confirm(a, "fold all six legs out to the belly-down calibration pose"):
        return 1
    bus = ServoBus(dry_run=a.force_dry)
    tripod_gait.belly_down(bus, a.duration)
    print("belly down")
    return 0


def cmd_stand(a: argparse.Namespace) -> int:
    cfg = build_config(a)
    if not confirm(a, f"rise to the standing pose (body {cfg.body_height:.0f}mm, "
                      f"stance width {cfg.stance_width:.0f}mm)"):
        return 1
    bus = ServoBus(dry_run=a.force_dry)
    tripod_gait.rise(bus, cfg, a.duration)
    print("standing")
    return 0


def cmd_walk(a: argparse.Namespace) -> int:
    cfg = build_config(a)
    rep = tripod_gait.check_cycle(cfg)
    tripod_gait.print_report(cfg, rep)
    if not rep.feasible:
        print("\nrefusing to walk: fix the geometry first (see --check)")
        return 2
    print(f"\n{a.cycles} cycle(s) = {a.cycles * cfg.stride:.0f} mm of forward travel, "
          f"{a.cycles * cfg.cycle_ms / 1000.0:.1f} s")
    if not confirm(a, "walk"):
        return 1
    bus = ServoBus(dry_run=a.force_dry)
    tripod_gait.walk(bus, cfg, cycles=a.cycles)
    tripod_gait.sit(bus, 2500)
    print("done - sat back down")
    return 0


def cmd_demo(a: argparse.Namespace) -> int:
    """Full safe sequence: belly down, rise, walk, sit."""
    cfg = build_config(a)
    tripod_gait.print_report(cfg)
    if not confirm(a, "run the full sequence: belly down -> rise -> walk -> sit"):
        return 1
    bus = ServoBus(dry_run=a.force_dry)
    print("\n[1/4] belly down"); tripod_gait.belly_down(bus, 2500)
    print("[2/4] rising");     tripod_gait.rise(bus, cfg, 3000)
    print(f"[3/4] walking {a.cycles} cycles"); tripod_gait.walk(bus, cfg, cycles=a.cycles)
    print("[4/4] sitting");    tripod_gait.sit(bus, 2500)
    print("\nsequence complete")
    return 0


# ============================================================
# CLI
# ============================================================
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Scorpion hexapod six-leg walking bring-up (LSC-24, 270-degree servos)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    mode = ap.add_mutually_exclusive_group(required=True)
    for flag, help_text in (
        ("--self-test", "IK math check, no hardware"),
        ("--check", "margins and torque for a pose"),
        ("--trajectory", "print the foot path"),
        ("--dry-run", "compute a walk, send nothing"),
        ("--identify", "sweep channels to map wiring"),
        ("--belly", "fold to belly-down"),
        ("--stand", "rise to standing"),
        ("--walk", "run gait cycles then sit"),
        ("--demo", "belly -> rise -> walk -> sit"),
    ):
        # store_const into a single `mode` attribute: mutually exclusive by
        # construction, and the dispatcher cannot be fooled by a flag that
        # happens to share a name with a mode.
        mode.add_argument(flag, action="store_const", dest="mode",
                          const=flag.lstrip("-").replace("-", "_"), help=help_text)

    ap.add_argument("--cycles", type=int, default=2, help="gait cycles for --walk/--dry-run/--demo")
    ap.add_argument("--duration", type=int, default=2500, help="ms for --belly/--stand")
    ap.add_argument("--yes", "-y", action="store_true", help="skip the go confirmation")
    ap.add_argument("--force-dry", action="store_true", dest="force_dry",
                    help="never touch hardware even if ServoControl imports")

    g = ap.add_argument_group("geometry overrides (mm / ms / kg)")
    g.add_argument("--reach", type=float, help="femur pivot -> planted foot (default 80)")
    g.add_argument("--height", type=float, help="body height, coxa axis above ground (default 155)")
    g.add_argument("--stride", type=float, help="body advance per cycle (default 50)")
    g.add_argument("--lift", type=float, help="peak foot clearance (default 25)")
    g.add_argument("--cycle", type=int, help="full gait cycle ms (default 2000)")
    g.add_argument("--phases", type=int, help="array commands per cycle (default 16)")
    g.add_argument("--mass", type=float, help="robot mass kg, torque report only (default 3.0)")

    a = ap.parse_args(argv)
    handlers = {
        "self_test": cmd_self_test, "check": cmd_check, "trajectory": cmd_trajectory,
        "dry_run": cmd_dry_run, "identify": cmd_identify, "belly": cmd_belly,
        "stand": cmd_stand, "walk": cmd_walk, "demo": cmd_demo,
    }
    return handlers[a.mode](a)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\ninterrupted - servos left wherever they were; run --belly to fold down")
        raise SystemExit(130)
