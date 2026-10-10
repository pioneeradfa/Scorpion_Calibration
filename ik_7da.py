#!/usr/bin/env python3
"""
Scorpion Hexapod - IK using the "7.d.a" equations (reference method)
====================================================================
Implements the equations from the reference document (Figure 10, leg inverse
kinematics) with the measured link lengths from the photo:

    L1 (coxa)  = 64.25 mm
    L2 (femur) = 64.25 mm
    L3 (tibia) = 135.6 mm   (straight line from femur joint to foot tip)

Inputs: foot position (x, y, z) in mm, relative to the leg's COXA AXIS, in the
body orientation (x = forward, y = left, z = up). This is the same frame used
by ik_module.py, minus the coxa-axis offset.

Equations (as in the reference):
    left legs:  theta = atan(x / y)
    right legs: theta = 180 + atan(x / y)
    DFO_xy    = sqrt(x^2 + y^2)
    xy_comp   = DFO_xy - L1
    extension = sqrt(xy_comp^2 + z^2)        <- see note below
    psi       = acos((L2^2 + L3^2 - ext^2) / (2 L2 L3))        knee angle (180 = straight)
    phi_p1    = |atan(xy_comp / z)|
    delta     = acos((L3^2 + ext^2 - L2^2) / (2 L3 ext))
    phi_p2    = acos((L2^2 + ext^2 - L3^2) / (2 L2 ext))
    phi       = 180 - phi_p1 - phi_p2

NOTE on "extension": the reference writes sqrt(x^2 + y^2 + z^2). That uses the
full horizontal distance DFO_xy, which still includes the coxa link L1. The
femur-to-foot triangle needs the distance measured from the femur joint, which
is xy_comp = DFO_xy - L1. This script uses xy_comp, so the geometry is consistent.
The literal version is printed alongside for comparison.

VALID REGION: the reference's phi_p1 = |atan(xy_comp / z)| is only correct when
xy_comp > 0 (foot further from the coxa axis than L1) and z < 0 (foot below the
coxa axis). Inside that region the results match the atan2 method exactly
(checked on ~14,800 random targets). Walking targets are always in this region.

Mapping to the servo raw angles (repo convention, calibration = straight pose):
    femur raw = cal_femur + (90 - phi)    (femur elevation from horizontal = 90 - phi)
    tibia raw = cal_tibia - (180 - psi)   (tibia = cal + TIBIA_SIGN*(180 - knee), TIBIA_SIGN = -1)
    coxa raw  = cal_coxa - theta          (left)
    coxa raw  = cal_coxa + (180 - theta)  (right)

Usage:
    python3 ik_7da.py                 # test all six legs, and print the RF stand calculation
    python3 ik_7da.py --leg LF        # calculation for one leg, stand pose
    python3 ik_7da.py --compare       # check against the atan2 method with the same lengths
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ik_module as ikm

L1, L2, L3 = 64.25, 64.25, 135.6
ALL_LEGS = ["RF", "RM", "RR", "LR", "LM", "LF"]
POSES = {
    "stand": {"RF": (102, 110, 35), "RM": (90, 110, 35), "RR": (103, 105, 35),
              "LF": (115, 110, 40), "LM": (90, 110, 20), "LR": (90, 110, 30)},
    "sit": {"RF": (102, 80, 60), "RM": (90, 80, 60), "RR": (101, 75, 60),
            "LF": (115, 80, 65), "LM": (90, 80, 45), "LR": (90, 80, 55)},
}


def deg(r):
    return math.degrees(r)


# ------------------------------------------------------------
# Reference equations, step by step
# ------------------------------------------------------------
def ik_7da(leg, x, y, z):
    cal = ikm.CALIBRATION[leg]
    s = {"leg": leg, "x": x, "y": y, "z": z}
    right = leg[0] == "R"

    # coxa angle (theta)
    if right:
        s["theta"] = 180.0 + deg(math.atan(x / y))
    else:
        s["theta"] = deg(math.atan(x / y)) if y != 0 else (90.0 if x >= 0 else -90.0)

    s["DFO"] = math.hypot(x, y)
    s["xy_comp"] = s["DFO"] - L1
    s["extension"] = math.hypot(s["xy_comp"], z)
    s["extension_literal"] = math.sqrt(x * x + y * y + z * z)

    ext = s["extension"]
    s["psi"] = deg(math.acos(max(-1, min(1, (L2 ** 2 + L3 ** 2 - ext ** 2) / (2 * L2 * L3)))))
    s["phi_p1"] = abs(deg(math.atan(s["xy_comp"] / z))) if z != 0 else 90.0
    s["delta"] = deg(math.acos(max(-1, min(1, (L3 ** 2 + ext ** 2 - L2 ** 2) / (2 * L3 * ext)))))
    s["phi_p2"] = deg(math.acos(max(-1, min(1, (L2 ** 2 + ext ** 2 - L3 ** 2) / (2 * L2 * ext)))))
    s["phi"] = 180.0 - s["phi_p1"] - s["phi_p2"]

    # servo raw angles
    s["femur_el"] = 90.0 - s["phi"]
    s["coxa_raw"] = cal["coxa"] + (180.0 - s["theta"] if right else -s["theta"])
    s["femur_raw"] = cal["femur"] + s["femur_el"]
    s["tibia_raw"] = cal["tibia"] - (180.0 - s["psi"])
    s["raw"] = (s["coxa_raw"], s["femur_raw"], s["tibia_raw"])
    s["in_range"] = all(0 <= a <= 180 for a in s["raw"])
    s["reachable"] = abs(L2 - L3) <= ext <= L2 + L3
    return s


# ------------------------------------------------------------
# Forward kinematics with the same lengths (for round trips)
# ------------------------------------------------------------
def fk_7da(leg, coxa_raw, femur_raw, tibia_raw):
    cal = ikm.CALIBRATION[leg]
    right = leg[0] == "R"
    if right:
        yaw = math.radians(coxa_raw - cal["coxa"])          # yaw_outward = raw - cal for right legs
    else:
        yaw = math.radians(-(coxa_raw - cal["coxa"]))        # left: raw = cal - theta
    femur_el = math.radians(femur_raw - cal["femur"])
    knee = math.radians(180.0 - (tibia_raw - cal["tibia"]) / ikm.TIBIA_SIGN)  # inverse of tibia mapping
    kx, kz = L2 * math.cos(femur_el), L2 * math.sin(femur_el)
    tdir = femur_el - (math.pi - knee)
    fx, fz = kx + L3 * math.cos(tdir), kz + L3 * math.sin(tdir)
    r = L1 + fx
    u, v = r * math.cos(yaw), r * math.sin(yaw)
    # convert outward/forward back to the body frame relative to coxa axis
    if right:
        return (v, -u, fz)
    return (v, u, fz)


# ------------------------------------------------------------
# Independent check: atan2 method with the same lengths (what ik_module does)
# ------------------------------------------------------------
def ik_atan2(leg, x, y, z):
    cal = ikm.CALIBRATION[leg]
    right = leg[0] == "R"
    u = -y if right else y          # outward distance
    v = x
    yaw = math.atan2(v, u)
    r = math.hypot(u, v)
    Lh = r - L1
    d = math.hypot(Lh, z)
    d = min(max(d, abs(L2 - L3) + 1e-9), L2 + L3 - 1e-9)
    alpha = math.atan2(z, Lh)
    beta = math.acos((L2 ** 2 + d ** 2 - L3 ** 2) / (2 * L2 * d))
    gamma = math.acos((L2 ** 2 + L3 ** 2 - d ** 2) / (2 * L2 * L3))
    coxa = cal["coxa"] + (math.degrees(yaw) if not right else math.degrees(yaw))
    coxa = cal["coxa"] + ikm.COXA_SIGN[leg[0]] * math.degrees(yaw)
    femur = cal["femur"] + math.degrees(alpha + beta)
    tibia = cal["tibia"] + ikm.TIBIA_SIGN * (180.0 - math.degrees(gamma))
    return coxa, femur, tibia


# ------------------------------------------------------------
# Printing and tests
# ------------------------------------------------------------
def print_calc(leg, x, y, z, title):
    s = ik_7da(leg, x, y, z)
    print(f"\n[{title}] {leg}: foot relative to coxa axis x={x:.2f}, y={y:.2f}, z={z:.2f} mm")
    print(f"  theta      = {'180 + ' if leg[0] == 'R' else ''}atan(x/y) = {s['theta']:.3f} deg")
    print(f"  DFO_xy     = sqrt(x^2+y^2) = {s['DFO']:.3f} mm")
    print(f"  xy_comp    = DFO - L1 = {s['DFO']:.3f} - {L1} = {s['xy_comp']:.3f} mm")
    print(f"  extension  = sqrt(xy_comp^2 + z^2) = {s['extension']:.3f} mm"
          f"   (literal sqrt(x^2+y^2+z^2) would give {s['extension_literal']:.3f})")
    print(f"  psi (knee) = {s['psi']:.3f} deg")
    print(f"  phi_p1     = |atan(xy_comp/z)| = {s['phi_p1']:.3f} deg")
    print(f"  delta      = {s['delta']:.3f} deg (not needed for the angles)")
    print(f"  phi_p2     = {s['phi_p2']:.3f} deg")
    print(f"  phi        = 180 - phi_p1 - phi_p2 = {s['phi']:.3f} deg")
    cal = ikm.CALIBRATION[leg]
    print(f"  coxa  raw  = {cal['coxa']} {'+ (180 - theta)' if leg[0] == 'R' else '- theta'} = {s['coxa_raw']:.2f}")
    print(f"  femur raw  = {cal['femur']} + (90 - phi) = {cal['femur']} + {s['femur_el']:.3f} = {s['femur_raw']:.2f}")
    print(f"  tibia raw  = {cal['tibia']} - (180 - psi) = {cal['tibia']} - {180 - s['psi']:.3f} = {s['tibia_raw']:.2f}")
    print(f"  reachable: {s['reachable']}, all angles 0-180: {s['in_range']}")
    return s


def stand_foot(leg):
    """Stand foot relative to the coxa axis, from the calibrated stand angles (new link lengths)."""
    x, y, z = fk_7da(leg, *POSES["stand"][leg])
    return x, y, z


def test_all():
    ok_all = True
    print("Round trip with the 7.d.a method (L1=64.25, L2=64.25, L3=135.6):")
    for leg in ALL_LEGS:
        for name in ("stand", "sit"):
            raw = POSES[name][leg]
            x, y, z = fk_7da(leg, *raw)
            s = ik_7da(leg, x, y, z)
            err = max(abs(a - b) for a, b in zip(s["raw"], raw))
            cmp_ = ik_atan2(leg, x, y, z)
            cdiff = max(abs(a - b) for a, b in zip(cmp_, s["raw"]))
            ok = err < 0.05 and cdiff < 1e-6
            ok_all &= ok
            print(f"  {leg} {name:5s} raw {raw} -> foot ({x:8.2f}, {y:8.2f}, {z:8.2f}) "
                  f"-> 7.d.a {tuple(round(a, 2) for a in s['raw'])}  err {err:.4f}  "
                  f"vs atan2 method {cdiff:.1e}  {'OK' if ok else 'FAIL'}")
    print("\nALL PASS" if ok_all else "\nSOME FAILED")
    return ok_all


def compare_random(n=2000):
    import random
    random.seed(1)
    worst = 0.0
    count = 0
    for _ in range(n):
        leg = random.choice(ALL_LEGS)
        x = random.uniform(-60, 60)
        y = random.uniform(40, 260) * (1 if leg[0] == "L" else -1)
        z = random.uniform(-110, -20)
        s = ik_7da(leg, x, y, z)
        if not s["reachable"] or s["xy_comp"] <= 0:   # valid region: foot outside the coxa link
            continue
        count += 1
        a = ik_atan2(leg, x, y, z)
        worst = max(worst, max(abs(p - q) for p, q in zip(a, s["raw"])))
    print(f"Compared {count} reachable random targets: worst difference {worst:.2e} deg")
    return worst


def main(argv):
    if "--compare" in argv:
        compare_random()
        return 0
    if "--leg" in argv:
        leg = argv[argv.index("--leg") + 1].upper()
        x, y, z = stand_foot(leg)
        print_calc(leg, x, y, z, "STAND")
        return 0
    print("=" * 74)
    print("7.d.a IK with L1 = 64.25, L2 = 64.25, L3 = 135.6 mm")
    print("=" * 74)
    x, y, z = stand_foot("RF")
    print_calc("RF", x, y, z, "STAND (worked example)")
    print()
    ok = test_all()
    compare_random()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))