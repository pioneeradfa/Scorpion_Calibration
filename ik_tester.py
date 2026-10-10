#!/usr/bin/env python3
"""
Scorpion Hexapod - IK TESTER for all six legs (RF, RM, RR, LR, LM, LF)
======================================================================
Tests the inverse kinematics one leg at a time (right and left), and lets you
type raw servo angles, see where the foot is, and save the poses so the IK
parameters can be tuned later.

Pure math. Nothing is sent to the servos.

MODES
-----
1) Test every leg (round trip + reachable-target sweep):
       python3 ik_tester.py --test-all
   Or one leg:
       python3 ik_tester.py --test LF

2) Interactive: enter angles, check the foot, save the pose:
       python3 ik_tester.py --interactive

   Commands inside interactive mode:
       leg LF              select a leg (RF RM RR LR LM LF)
       angles 115 90 40    set raw [coxa femur tibia] for the selected leg; shows foot position
       foot 130 -280 -50   set a target foot position (body frame, mm); shows IK angles
       stand               load the stand pose for this leg
       sit                 load the sit pose for this leg
       save [note]         save the current angles + foot position to ik_tuning/saved_poses.json
       list                list saved poses
       show N              show saved pose N and its foot position
       del N               delete saved pose N
       help                show commands
       q                   quit

Raw angle convention: [coxa, femur, tibia], same as stand_sit_walk_tail_claw_belly.py.
Body frame: +X forward, +Y left, +Z up, origin at body centre, z measured from the coxa axis.
"""

import datetime
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ik_module as ikm

C, F, T = ikm.COXA_LEN, ikm.FEMUR_LEN, ikm.TIBIA_LEN
LEGS = ikm.LEG_ORDER if set(ikm.LEG_ORDER) == set(ikm.MOUNT_POS) else list(ikm.MOUNT_POS)
ALL_LEGS = ["RF", "RM", "RR", "LR", "LM", "LF"]
SAVE_FILE = os.path.join(HERE, "ik_tuning", "saved_poses.json")
ANGLE_MIN, ANGLE_MAX = 0.0, 180.0

# User-calibrated poses (from stand_sit_walk_tail_claw_belly.py)
POSES = {
    "stand": {
        "RF": (102, 110, 35), "RM": (90, 110, 35), "RR": (103, 105, 35),
        "LF": (115, 110, 40), "LM": (90, 110, 20), "LR": (90, 110, 30),
    },
    "sit": {
        "RF": (102, 80, 60), "RM": (90, 80, 60), "RR": (101, 75, 60),
        "LF": (115, 80, 65), "LM": (90, 80, 45), "LR": (90, 80, 55),
    },
}


def cal(leg):
    return ikm.CALIBRATION[leg]


def side_sign(leg):
    return 1 if leg[0] == "R" else -1


def mount(leg):
    return ikm.MOUNT_POS[leg]


def in_range(raw):
    return all(ANGLE_MIN <= a <= ANGLE_MAX for a in raw)


# ------------------------------------------------------------
# FORWARD KINEMATICS (works for left and right legs)
# ------------------------------------------------------------
def fk(leg, coxa_raw, femur_raw, tibia_raw):
    k = cal(leg)
    side = side_sign(leg)
    mx, my = mount(leg)
    yaw = math.radians(ikm.COXA_SIGN[leg[0]] * (coxa_raw - k["coxa"]))   # inverse of coxa = cal + S*yaw
    femur_el = math.radians(ikm.FEMUR_SIGN * (femur_raw - k["femur"]))
    knee_deg = 180.0 - (tibia_raw - k["tibia"]) / ikm.TIBIA_SIGN          # inverse of tibia = cal + S*(180-knee)
    gamma = math.radians(knee_deg)
    kx, kz = F * math.cos(femur_el), F * math.sin(femur_el)
    tibia_dir = femur_el - (math.pi - gamma)
    fx, fz = kx + T * math.cos(tibia_dir), kz + T * math.sin(tibia_dir)
    r = C + fx
    v = r * math.sin(yaw)
    u = r * math.cos(yaw)
    body = (mx + v, my - side * u, fz)
    return {
        "body": body,
        "yaw_deg": math.degrees(yaw),
        "femur_elev_deg": math.degrees(femur_el),
        "knee_deg": knee_deg,
        "tibia_dir_deg": math.degrees(tibia_dir),
    }


# ------------------------------------------------------------
# INVERSE KINEMATICS, step by step (works for left and right legs)
# ------------------------------------------------------------
def ik_steps(leg, x, y, z):
    k = cal(leg)
    side = side_sign(leg)
    mx, my = mount(leg)
    s = {"leg": leg, "target": (x, y, z)}
    s["v"] = x - mx                          # A: forward offset from coxa axis
    s["u"] = -side * (y - my)                # A: outward distance (right: outward = -y, left: +y)
    s["z"] = z
    s["yaw"] = math.atan2(s["v"], s["u"])    # B: coxa yaw
    s["r"] = math.hypot(s["u"], s["v"])      # C: reach
    s["L"] = s["r"] - C
    s["d"] = math.hypot(s["L"], s["z"])
    s["d_min"] = abs(F - T)
    s["d_max"] = F + T
    s["reachable"] = s["d_min"] <= s["d"] <= s["d_max"]   # D
    d = min(max(s["d"], s["d_min"] + 1e-6), s["d_max"] - 1e-6)
    s["alpha"] = math.atan2(s["z"], s["L"])  # E
    s["beta"] = math.acos(max(-1.0, min(1.0, (F * F + d * d - T * T) / (2 * F * d))))
    s["femur_el"] = s["alpha"] + s["beta"]
    s["gamma"] = math.acos(max(-1.0, min(1.0, (F * F + T * T - d * d) / (2 * F * T))))  # F
    s["knee_deg"] = math.degrees(s["gamma"])
    s["coxa_raw"] = k["coxa"] + ikm.COXA_SIGN[leg[0]] * math.degrees(s["yaw"])          # G
    s["femur_raw"] = k["femur"] + ikm.FEMUR_SIGN * math.degrees(s["femur_el"])
    s["tibia_raw"] = k["tibia"] + ikm.TIBIA_SIGN * (180.0 - s["knee_deg"])
    s["raw"] = (s["coxa_raw"], s["femur_raw"], s["tibia_raw"])
    s["in_range"] = in_range(s["raw"])
    return s


def ik(leg, x, y, z):
    return ik_steps(leg, x, y, z)["raw"]


# ------------------------------------------------------------
# TESTS
# ------------------------------------------------------------
def test_leg(leg, verbose=True):
    """Round trip for stand/sit + sweep of reachable targets around stand."""
    problems = []
    lines = []
    for name in ("stand", "sit"):
        raw = POSES[name][leg]
        x, y, z = fk(leg, *raw)["body"]
        back = ik(leg, x, y, z)
        err = max(abs(a - b) for a, b in zip(back, raw))
        mod = ikm.leg_ik_body(leg, x, y, z)
        diff = max(abs(a - b) for a, b in zip(mod, back))
        ok = err < 0.05 and diff < 1e-6
        if not ok:
            problems.append(f"{name} round trip err {err:.4f}, module diff {diff:.2e}")
        lines.append(f"  {name:5s} raw {raw} -> foot ({x:8.2f}, {y:8.2f}, {z:8.2f}) -> IK {tuple(round(a, 2) for a in back)}"
                     f"  err {err:.4f}  vs ik_module {diff:.1e}  {'OK' if ok else 'FAIL'}")

    # Sweep: foot targets around the stand foot. Count how many are reachable and in joint range.
    sx, sy, sz = fk(leg, *POSES["stand"][leg])["body"]
    total = reach_ok = range_ok = 0
    bad = []
    for dx in (-30, -15, 0, 15, 30):
        for dz in (-20, -10, 0, 10, 20):
            total += 1
            s = ik_steps(leg, sx + dx, sy, sz + dz)
            if s["reachable"]:
                reach_ok += 1
            if s["in_range"]:
                range_ok += 1
            if not (s["reachable"] and s["in_range"]):
                bad.append((dx, dz, s["reachable"], s["in_range"]))
    lines.append(f"  sweep: {total} targets (+/-30 mm x, +/-20 mm z around stand foot): "
                 f"{reach_ok} reachable, {range_ok} inside 0-180 deg")
    for dx, dz, r_ok, a_ok in bad:
        lines.append(f"    not ok: dx={dx:+d} dz={dz:+d} reachable={r_ok} in_range={a_ok}")
    if bad:
        problems.append(f"{len(bad)} sweep targets unreachable or out of range")

    status = "PASS" if not problems else "FAIL"
    if verbose:
        print(f"\n=== {leg} ({'right' if leg[0] == 'R' else 'left'}) mount {mount(leg)} "
              f"cal {cal(leg)} -> {status}")
        print("\n".join(lines))
    return status == "PASS", problems


def test_all():
    print("=" * 74)
    print("IK TEST - ALL SIX LEGS, one by one (right, then left)")
    print("=" * 74)
    results = {}
    for leg in ALL_LEGS:
        ok, probs = test_leg(leg)
        results[leg] = (ok, probs)
    print("\n" + "-" * 74)
    print("SUMMARY")
    for leg in ALL_LEGS:
        ok, probs = results[leg]
        print(f"  {leg}: {'PASS' if ok else 'FAIL'}" + ("" if ok else "  " + "; ".join(probs)))
    all_ok = all(r[0] for r in results.values())
    print("\nALL LEGS PASS" if all_ok else "\nSOME LEGS FAILED")
    return all_ok


# ------------------------------------------------------------
# SAVED POSES
# ------------------------------------------------------------
def load_saved():
    if os.path.exists(SAVE_FILE):
        with open(SAVE_FILE) as fh:
            return json.load(fh)
    return []


def save_saved(items):
    os.makedirs(os.path.dirname(SAVE_FILE), exist_ok=True)
    with open(SAVE_FILE, "w") as fh:
        json.dump(items, fh, indent=2)


def add_saved(leg, raw, note=""):
    items = load_saved()
    body = fk(leg, *raw)["body"]
    items.append({
        "time": datetime.datetime.now().isoformat(timespec="seconds"),
        "leg": leg,
        "raw": [round(a, 2) for a in raw],
        "foot_body_mm": [round(a, 2) for a in body],
        "note": note,
    })
    save_saved(items)
    return len(items)


def print_saved_list():
    items = load_saved()
    if not items:
        print("  (no saved poses yet)")
        return
    print(f"  {'#':>3}  {'leg':3s}  {'raw [coxa, femur, tibia]':26s}  {'foot (x, y, z) mm':28s}  note")
    for i, it in enumerate(items, 1):
        b = it["foot_body_mm"]
        print(f"  {i:3d}  {it['leg']:3s}  {str(it['raw']):26s}  ({b[0]:7.2f}, {b[1]:8.2f}, {b[2]:7.2f})  {it['note']}")


# ------------------------------------------------------------
# INTERACTIVE
# ------------------------------------------------------------
HELP = """Commands:
  leg XX              select leg (RF RM RR LR LM LF)
  angles c f t        set raw [coxa femur tibia] for this leg; show foot position
  foot x y z          set target foot (body frame mm); show IK angles
  stand | sit         load stand or sit pose for this leg
  save [note]         save current angles + foot to ik_tuning/saved_poses.json
  list                list saved poses
  show N              show saved pose N
  del N               delete saved pose N
  test                run the round-trip and sweep test for this leg
  help                this text
  q                   quit"""


def show_angles(leg, raw):
    f = fk(leg, *raw)
    x, y, z = f["body"]
    back = ik(leg, x, y, z)
    print(f"  raw {leg} = coxa {raw[0]:.2f}, femur {raw[1]:.2f}, tibia {raw[2]:.2f}"
          f"{'' if in_range(raw) else '   <-- OUT OF 0-180 RANGE'}")
    print(f"  knee {f['knee_deg']:.2f} deg, femur elevation {f['femur_elev_deg']:+.2f} deg, "
          f"tibia direction {f['tibia_dir_deg']:+.2f} deg, yaw {f['yaw_deg']:+.2f} deg")
    print(f"  foot body frame: x={x:.2f}  y={y:.2f}  z={z:.2f} mm")
    print(f"  IK of that foot: {tuple(round(a, 2) for a in back)}")


def show_foot(leg, x, y, z):
    s = ik_steps(leg, x, y, z)
    print(f"  target foot: x={x:.2f} y={y:.2f} z={z:.2f}")
    print(f"  A v={s['v']:.2f} u={s['u']:.2f} z={s['z']:.2f} | B yaw={math.degrees(s['yaw']):+.2f} "
          f"| C r={s['r']:.2f} L={s['L']:.2f} d={s['d']:.2f}")
    print(f"  D reachable: {s['reachable']} (need {s['d_min']:.2f} <= d <= {s['d_max']:.2f})")
    print(f"  E femur elevation={math.degrees(s['femur_el']):+.2f} | F knee={s['knee_deg']:.2f}")
    print(f"  G raw: coxa {s['coxa_raw']:.2f}, femur {s['femur_raw']:.2f}, tibia {s['tibia_raw']:.2f}"
          f"{'' if s['in_range'] else '   <-- OUT OF 0-180 RANGE'}")
    return s["raw"]


def interactive():
    print("IK TESTER - interactive. Type 'help' for commands. Nothing is sent to the servos.")
    leg = "LF"
    raw = POSES["stand"][leg]
    print(f"Selected leg {leg}, starting at stand pose {raw}.")
    while True:
        try:
            line = input(f"[{leg}] > ").strip()
        except EOFError:
            print()
            break
        if not line:
            continue
        parts = line.split()
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "help":
                print(HELP)
            elif cmd == "leg":
                leg = args[0].upper()
                if leg not in ALL_LEGS:
                    raise ValueError(f"unknown leg {leg}")
                raw = POSES["stand"][leg]
                print(f"Selected {leg}; angles reset to stand {raw}.")
            elif cmd == "angles":
                raw = tuple(float(a) for a in args[:3])
                if len(raw) != 3:
                    raise ValueError("give three angles: coxa femur tibia")
                show_angles(leg, raw)
            elif cmd == "foot":
                x, y, z = (float(a) for a in args[:3])
                raw = show_foot(leg, x, y, z)
            elif cmd == "stand":
                raw = POSES["stand"][leg]
                show_angles(leg, raw)
            elif cmd == "sit":
                raw = POSES["sit"][leg]
                show_angles(leg, raw)
            elif cmd == "save":
                note = " ".join(args)
                n = add_saved(leg, raw, note)
                print(f"  saved as #{n} -> {os.path.relpath(SAVE_FILE, HERE)}")
            elif cmd == "list":
                print_saved_list()
            elif cmd == "show":
                items = load_saved()
                it = items[int(args[0]) - 1]
                print(f"  #{args[0]} leg {it['leg']} raw {it['raw']} note '{it['note']}'")
                show_angles(it["leg"], tuple(it["raw"]))
            elif cmd == "del":
                items = load_saved()
                removed = items.pop(int(args[0]) - 1)
                save_saved(items)
                print(f"  deleted #{args[0]} ({removed['leg']} {removed['raw']})")
            elif cmd == "test":
                test_leg(leg)
            else:
                print("  unknown command (type 'help')")
        except (ValueError, IndexError) as e:
            print(f"  error: {e}")
    print("Bye.")


# ------------------------------------------------------------
def main(argv):
    if "--test-all" in argv:
        return 0 if test_all() else 1
    if "--test" in argv:
        leg = argv[argv.index("--test") + 1].upper()
        ok, _ = test_leg(leg)
        return 0 if ok else 1
    if "--interactive" in argv or len(argv) == 0:
        interactive()
        return 0
    print(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))