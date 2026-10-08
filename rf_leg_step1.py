#!/usr/bin/env python3
"""
Scorpion Hexapod - Right Front (RF) leg, STEP 1
===============================================
Only the RF leg is handled here. No other leg is commanded, and nothing is
sent to the servos.

What this script does (pure math, safe to run on any PC):
  1. Forward kinematics (FK) of the user's manually calibrated STAND and SIT
     poses -> foot position in body frame (mm).
  2. Inverse kinematics (IK) for RF with every intermediate value printed, so
     the math can be followed by hand. Cross-checked against ik_module.py.
  3. Round-trip test: IK(FK(pose)) must give back the same raw angles.
  4. Dry-run swing preview: lift -> swing forward -> lower, one RF step, with
     reach and joint-range checks. Prints the angle table.
  5. --plot writes docs/rf_leg_diagram.png.

Run:
    python3 rf_leg_step1.py            # numbers only
    python3 rf_leg_step1.py --plot     # numbers + diagram PNG (needs matplotlib)

Raw servo angle convention (same as stand_sit_walk_tail_claw_belly.py):
    RF = [coxa, femur, tibia]
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ik_module as ik  # reuses measured geometry, calibration and sign conventions

LEG = "RF"
C, F, T = ik.COXA_LEN, ik.FEMUR_LEN, ik.TIBIA_LEN
MX, MY = ik.MOUNT_POS[LEG]          # RF coxa axis in body frame: (115, -60)
CAL = ik.CALIBRATION[LEG]           # {'coxa': 98, 'femur': 88, 'tibia': 94}

# User-calibrated poses (copied from stand_sit_walk_tail_claw_belly.py)
STAND_RF = (102, 110, 35)
SIT_RF = (102, 80, 60)

# Dry-run swing settings (defaults to review, not yet hardware-tested)
SWING_STRIDE = 30.0   # mm, forward travel of the foot during the swing
SWING_LIFT = 20.0     # mm, peak lift above stand height
SWING_STEPS = 10      # intervals (SWING_STEPS + 1 waypoints)
ANGLE_MIN, ANGLE_MAX = 0.0, 180.0


# ------------------------------------------------------------
# FORWARD KINEMATICS: raw servo angles -> body-frame foot
# ------------------------------------------------------------
def fk_rf(coxa_raw, femur_raw, tibia_raw):
    """Return a dict with every intermediate FK value and the body-frame foot."""
    yaw = math.radians(coxa_raw - CAL["coxa"])            # COXA_SIGN['R'] = +1
    femur_el = math.radians(femur_raw - CAL["femur"])     # FEMUR_SIGN = +1 (elevation, +ve = up)
    # invert tibia_deg = cal + TIBIA_SIGN*(180 - knee), TIBIA_SIGN = -1
    knee_deg = 180.0 + (tibia_raw - CAL["tibia"])
    gamma = math.radians(knee_deg)

    kx, kz = F * math.cos(femur_el), F * math.sin(femur_el)      # knee, relative to femur axis
    tibia_dir = femur_el - (math.pi - gamma)                       # knee bent -> tibia points down
    fx, fz = kx + T * math.cos(tibia_dir), kz + T * math.sin(tibia_dir)

    r = C + fx                        # radial reach from the coxa axis
    v = r * math.sin(yaw)             # forward offset in body frame
    u = r * math.cos(yaw)             # outward distance in body frame
    body = (MX + v, MY - u, fz)       # RF is on the right: outward = -y
    return {
        "yaw_deg": math.degrees(yaw),
        "femur_elev_deg": math.degrees(femur_el),
        "knee_deg": knee_deg,
        "tibia_dir_deg": math.degrees(tibia_dir),
        "knee_xz": (kx, kz),
        "foot_xz": (fx, fz),
        "body": body,
    }


# ------------------------------------------------------------
# INVERSE KINEMATICS, written out step by step
# ------------------------------------------------------------
def ik_rf_steps(body_x, body_y, body_z):
    """IK for RF; every step is kept in a dict. Same math as ik_module.leg_ik."""
    s = {"body_target": (body_x, body_y, body_z)}
    # A: body frame -> leg frame (origin at RF coxa axis)
    s["v"] = body_x - MX                 # forward offset
    s["u"] = -(body_y - MY)              # outward distance (RF: outward = -y)
    s["z"] = body_z                      # height relative to coxa axis
    # B: coxa yaw
    s["yaw"] = math.atan2(s["v"], s["u"])
    # C: horizontal reach after the coxa link, and femur-axis -> foot distance
    s["r"] = math.hypot(s["u"], s["v"])
    s["L"] = s["r"] - C
    s["d"] = math.hypot(s["L"], s["z"])
    # D: reachability (law of cosines is only valid between |F-T| and F+T)
    s["d_min"] = abs(F - T)
    s["d_max"] = F + T
    s["reachable"] = s["d_min"] <= s["d"] <= s["d_max"]
    d = min(max(s["d"], s["d_min"] + 1e-6), s["d_max"] - 1e-6)
    # E: femur elevation = alpha + beta
    s["alpha"] = math.atan2(s["z"], s["L"])
    s["beta"] = math.acos(max(-1.0, min(1.0, (F * F + d * d - T * T) / (2 * F * d))))
    s["femur_el"] = s["alpha"] + s["beta"]
    # F: knee interior angle (180 = straight leg)
    s["gamma"] = math.acos(max(-1.0, min(1.0, (F * F + T * T - d * d) / (2 * F * T))))
    s["knee_deg"] = math.degrees(s["gamma"])
    # G: raw servo angles = calibration + signed offset
    s["coxa_raw"] = CAL["coxa"] + ik.COXA_SIGN["R"] * math.degrees(s["yaw"])
    s["femur_raw"] = CAL["femur"] + ik.FEMUR_SIGN * math.degrees(s["femur_el"])
    s["tibia_raw"] = CAL["tibia"] + ik.TIBIA_SIGN * (180.0 - s["knee_deg"])
    return s


def ik_rf(body_x, body_y, body_z):
    s = ik_rf_steps(body_x, body_y, body_z)
    return s["coxa_raw"], s["femur_raw"], s["tibia_raw"]


# ------------------------------------------------------------
# SWING PREVIEW (dry run, no hardware)
# ------------------------------------------------------------
def smooth(s):
    """0 -> 1 with zero velocity at both ends (cosine ease)."""
    return 0.5 - 0.5 * math.cos(math.pi * s)


def swing_path(start_body, stride=SWING_STRIDE, lift=SWING_LIFT, steps=SWING_STEPS):
    """Foot path: starts exactly at the stand foot, lifts, moves forward by
    `stride`, and lowers back to stand height."""
    x0, y0, z0 = start_body
    pts = []
    for i in range(steps + 1):
        s = i / steps
        x = x0 + stride * smooth(s)                 # forward travel
        z = z0 + lift * math.sin(math.pi * s)       # lift arc, 0 at both ends
        pts.append((x, y0, z))
    return pts


def angles_in_range(angles):
    return all(ANGLE_MIN <= a <= ANGLE_MAX for a in angles)


# ------------------------------------------------------------
# PRINTING
# ------------------------------------------------------------
def print_fk(name, raw):
    fk = fk_rf(*raw)
    bx, by, bz = fk["body"]
    print(f"\n[FK {name}] RF raw = coxa {raw[0]}, femur {raw[1]}, tibia {raw[2]}")
    print(f"  yaw from centre       : {fk['yaw_deg']:+7.2f} deg  (coxa - {CAL['coxa']})")
    print(f"  femur elevation       : {fk['femur_elev_deg']:+7.2f} deg  (femur - {CAL['femur']}; +ve = up)")
    print(f"  knee (interior) angle : {fk['knee_deg']:7.2f} deg  (180 = straight)")
    print(f"  tibia direction       : {fk['tibia_dir_deg']:+7.2f} deg  (0 = horizontal)")
    print(f"  body-frame foot       : x={bx:7.2f}  y={by:7.2f}  z={bz:7.2f} mm  (z relative to coxa axis)")
    return fk


def print_ik(name, body):
    s = ik_rf_steps(*body)
    print(f"\n[IK {name}] target body = x {body[0]:.2f}, y {body[1]:.2f}, z {body[2]:.2f}")
    print(f"  A  v = x - {MX:.0f} = {s['v']:.3f}      u = -(y - ({MY:.0f})) = {s['u']:.3f}      z = {s['z']:.3f}")
    print(f"  B  yaw = atan2(v, u) = {math.degrees(s['yaw']):.3f} deg")
    print(f"  C  r = hypot(u, v) = {s['r']:.3f}   L = r - {C} = {s['L']:.3f}   d = hypot(L, z) = {s['d']:.3f}")
    print(f"  D  reachable: {s['d_min']:.2f} <= d <= {s['d_max']:.2f} ?  {s['reachable']}")
    print(f"  E  alpha = atan2(z, L) = {math.degrees(s['alpha']):.3f} deg")
    print(f"     beta  = acos((F^2 + d^2 - T^2)/(2 F d)) = {math.degrees(s['beta']):.3f} deg")
    print(f"     femur elevation = alpha + beta = {math.degrees(s['femur_el']):.3f} deg")
    print(f"  F  knee = acos((F^2 + T^2 - d^2)/(2 F T)) = {s['knee_deg']:.3f} deg")
    print(f"  G  coxa  = {CAL['coxa']} + ({math.degrees(s['yaw']):+.3f})  = {s['coxa_raw']:.2f}")
    print(f"     femur = {CAL['femur']} + ({math.degrees(s['femur_el']):+.3f})  = {s['femur_raw']:.2f}")
    print(f"     tibia = {CAL['tibia']} + (-1)*(180 - {s['knee_deg']:.3f}) = {s['tibia_raw']:.2f}")
    return s


def run_round_trip():
    print("\n=== Round-trip test: IK(FK(pose)) must return the same raw angles ===")
    ok = True
    for name, raw in (("stand", STAND_RF), ("sit", SIT_RF)):
        bx, by, bz = fk_rf(*raw)["body"]
        back = ik_rf(bx, by, bz)
        err = max(abs(a - b) for a, b in zip(back, raw))
        ok &= err < 0.05
        print(f"  {name:5s}: in {raw} -> body ({bx:.2f}, {by:.2f}, {bz:.2f}) -> out "
              f"({back[0]:.2f}, {back[1]:.2f}, {back[2]:.2f})  max err {err:.4f} deg")
    bx, by, bz = fk_rf(*STAND_RF)["body"]
    mod = ik.leg_ik_body(LEG, bx, by, bz)
    diff = max(abs(a - b) for a, b in zip(mod, ik_rf(bx, by, bz)))
    ok &= diff < 1e-9
    print(f"  cross-check vs ik_module.leg_ik_body: max diff {diff:.2e} deg")
    print("  PASS" if ok else "  FAIL")
    return ok


def run_swing_preview():
    print("\n=== Dry-run RF swing (lift -> forward -> lower). NO servo commands sent. ===")
    print(f"  stride={SWING_STRIDE:.0f} mm, lift={SWING_LIFT:.0f} mm, waypoints={SWING_STEPS + 1}")
    path = swing_path(fk_rf(*STAND_RF)["body"])
    print("   i   body x     body y     body z   |  coxa   femur   tibia  | reachable  in-range")
    prev, max_step, all_ok = None, 0.0, True
    for i, p in enumerate(path):
        s = ik_rf_steps(*p)
        raw = (s["coxa_raw"], s["femur_raw"], s["tibia_raw"])
        all_ok &= s["reachable"] and angles_in_range(raw)
        if prev is not None:
            max_step = max(max_step, max(abs(a - b) for a, b in zip(raw, prev)))
        prev = raw
        print(f"  {i:2d}  {p[0]:8.2f}  {p[1]:8.2f}  {p[2]:8.2f}  | {raw[0]:6.2f} {raw[1]:7.2f} {raw[2]:7.2f} |"
              f"   {str(s['reachable']):5s}     {str(angles_in_range(raw)):5s}")
    print(f"  Largest joint change between waypoints: {max_step:.2f} deg")
    print(f"  Start = stand foot x {path[0][0]:.2f} mm; end x {path[-1][0]:.2f} mm "
          f"(+{SWING_STRIDE:.0f} mm), back to stand height z {path[-1][2]:.2f}")
    print("  All waypoints inside reach and joint range:", "YES" if all_ok else "NO")
    return all_ok, path


def make_plot(out_path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 3, figsize=(18, 6.5))

    # ---------- Panel 1: top view of the whole body ----------
    ax = axes[0]
    ax.add_patch(plt.Rectangle((-115, -60), 230, 120, fill=False, lw=1.5, color="gray"))
    for name, (mx, my) in ik.MOUNT_POS.items():
        col = "red" if name == LEG else "k"
        ax.plot(mx, my, "o", color=col, ms=7)
        ax.annotate(name, (mx, my), textcoords="offset points", xytext=(8, 6), color=col)
    fx_, fy_, _ = fk_rf(*STAND_RF)["body"]
    ax.plot([MX, fx_], [MY, fy_], "-", color="red", lw=2)
    ax.plot(fx_, fy_, "x", color="red", ms=10)
    ax.annotate("RF foot (stand)", (fx_, fy_), textcoords="offset points", xytext=(-60, -18), color="red")
    ax.annotate("", xy=(190, -20), xytext=(150, -20), arrowprops=dict(arrowstyle="->"))
    ax.text(120, -12, "forward (+X)", fontsize=9)
    ax.annotate("", xy=(-150, 40), xytext=(-150, 0), arrowprops=dict(arrowstyle="->"))
    ax.text(-145, 25, "left (+Y)", fontsize=9)
    ax.set_title("1. Top view of body (RF in red)")
    ax.set_xlabel("X forward (mm)")
    ax.set_ylabel("Y left (mm)")
    ax.set_aspect("equal")
    ax.set_xlim(-230, 260)
    ax.set_ylim(-340, 200)
    ax.grid(alpha=0.3)

    # ---------- Panel 2: side view of the RF leg in STAND pose ----------
    ax = axes[1]
    fk = fk_rf(*STAND_RF)
    kx, kz = fk["knee_xz"]
    fxx, fzz = fk["foot_xz"]
    ax.plot(0, 0, "ko", ms=9)
    ax.text(-4, 8, "coxa axis", ha="right", fontsize=9)
    ax.plot([0, C], [0, 0], "-", color="tab:blue", lw=6, solid_capstyle="butt", label=f"coxa {C} mm")
    ax.plot([C, C + kx], [0, kz], "-", color="tab:green", lw=6, solid_capstyle="butt", label=f"femur {F} mm")
    ax.plot([C + kx, C + fxx], [kz, fzz], "-", color="tab:orange", lw=6, solid_capstyle="butt",
            label=f"tibia {T} mm")
    ax.plot(C + kx, kz, "o", color="k", ms=6)
    ax.text(C + kx + 6, kz + 6, "knee", fontsize=9)
    ax.plot(C + fxx, fzz, "o", color="red", ms=7)
    ax.text(C + fxx - 40, fzz - 16, f"foot ({C + fxx:.0f}, {fzz:.1f})", color="red", fontsize=9)
    ax.axhline(fzz, color="saddlebrown", ls="--", lw=1)
    ax.text(60, fzz - 9, "ground = stand foot height", color="saddlebrown", fontsize=8)
    ax.plot([0, 260], [0, 0], ":", color="gray")
    ax.text(190, 4, "horizontal (0\u00b0)", color="gray", fontsize=8)
    # femur elevation arc (from horizontal to femur direction)
    th = fk["femur_elev_deg"]
    arc = [(C + 0.35 * F * math.cos(math.radians(a)), 0.35 * F * math.sin(math.radians(a)))
           for a in [i * th / 20 for i in range(21)]]
    ax.plot([p[0] for p in arc], [p[1] for p in arc], color="tab:green")
    ax.text(C + 0.5 * F, -16,
            f"femur {th:+.1f}\u00b0", color="tab:green", fontsize=9)
    ax.text(C + kx - 8, kz + 12, f"knee {fk['knee_deg']:.1f}\u00b0", fontsize=9)
    ax.text(150, -90,
            f"tibia direction {fk['tibia_dir_deg']:+.1f}\u00b0\n(from horizontal)", fontsize=9)
    ax.set_title("2. RF side view, STAND pose (leg plane)")
    ax.set_xlabel("u: outward from coxa axis (mm)")
    ax.set_ylabel("z: up, relative to coxa axis (mm)")
    ax.set_aspect("equal")
    ax.set_xlim(-30, 260)
    ax.set_ylim(-95, 60)
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)

    # ---------- Panel 3: dry-run swing path ----------
    ax = axes[2]
    path = swing_path(fk_rf(*STAND_RF)["body"])
    ax.plot([p[0] for p in path], [p[2] for p in path], "-o", color="red", ms=4, label="foot path")
    for i in (0, len(path) // 2, len(path) - 1):
        p = path[i]
        s = ik_rf_steps(*p)
        ax.annotate(f"{i}: [{s['coxa_raw']:.0f}, {s['femur_raw']:.0f}, {s['tibia_raw']:.0f}]",
                    (p[0], p[2]), textcoords="offset points", xytext=(6, 8), fontsize=8)
    ax.axhline(path[0][2], color="saddlebrown", ls="--", lw=1, label="stand height (ground)")
    ax.set_title("3. Dry-run RF swing: lift \u2192 forward \u2192 lower")
    ax.set_xlabel("body X forward (mm)")
    ax.set_ylabel("body Z up (mm)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    fig.suptitle("Scorpion hexapod - Right Front leg, step 1: IK from calibrated stand pose "
                 "[raw servo angles: coxa, femur, tibia]", fontsize=12)
    fig.tight_layout()
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    fig.savefig(out_path, dpi=130)
    print(f"\nDiagram written to {out_path}")


def main():
    print("=" * 72)
    print("SCORPION - RIGHT FRONT LEG, STEP 1: geometry, FK of stand/sit, IK, dry-run swing")
    print("=" * 72)
    print(f"Geometry (mm): coxa {C}, femur {F}, tibia {T}; RF coxa axis at x={MX:.0f}, y={MY:.0f}")
    print(f"RF calibration (raw servo angle at straight, belly-down): {CAL}")
    print_fk("STAND (initial pose)", STAND_RF)
    print_fk("SIT", SIT_RF)
    print_ik("STAND foot", fk_rf(*STAND_RF)["body"])
    run_round_trip()
    run_swing_preview()
    if "--plot" in sys.argv:
        make_plot(os.path.join(HERE, "docs", "rf_leg_diagram.png"))


if __name__ == "__main__":
    main()
