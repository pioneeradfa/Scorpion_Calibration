#!/usr/bin/env python3
"""
Scorpion Hexapod - ALL RIGHT LEGS (RF, RM, RR), step 1
=======================================================
Same method as rf_leg_step1.py, applied to every right-side leg:
  * forward kinematics (FK) of the calibrated stand and sit poses,
  * step-by-step inverse kinematics (IK) per leg, cross-checked with ik_module.py,
  * round-trip test, and a dry-run single-step swing (lift -> forward -> lower).

Nothing is sent to the servos. Pure math.

Usage:
    python3 right_legs_step1.py                 # print everything
    python3 right_legs_step1.py --plot          # also write docs/right_legs_*.png
    python3 right_legs_step1.py --doc           # also write docs/Right_legs_step1.md

Raw angle convention: [coxa, femur, tibia] per leg, same as
stand_sit_walk_tail_claw_belly.py.
"""

import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import ik_module as ikm

C, F, T = ikm.COXA_LEN, ikm.FEMUR_LEN, ikm.TIBIA_LEN
RIGHT_LEGS = ["RF", "RM", "RR"]

# User-calibrated poses (from stand_sit_walk_tail_claw_belly.py)
POSES = {
    "stand": {"RF": (102, 110, 35), "RM": (90, 110, 35), "RR": (103, 105, 35)},
    "sit":   {"RF": (102, 80, 60),  "RM": (90, 80, 60),  "RR": (101, 75, 60)},
}

SWING_STRIDE = 30.0   # mm forward
SWING_LIFT = 20.0     # mm peak lift
SWING_STEPS = 10      # intervals
ANGLE_MIN, ANGLE_MAX = 0.0, 180.0


def cal(leg):
    return ikm.CALIBRATION[leg]


def mount(leg):
    return ikm.MOUNT_POS[leg]


def side_sign(leg):
    return 1 if leg[0] == "R" else -1


# ------------------------------------------------------------
# FORWARD KINEMATICS
# ------------------------------------------------------------
def fk(leg, coxa_raw, femur_raw, tibia_raw):
    k = cal(leg)
    mx, my = mount(leg)
    yaw = math.radians(ikm.COXA_SIGN["R"] * (coxa_raw - k["coxa"]))
    femur_el = math.radians(ikm.FEMUR_SIGN * (femur_raw - k["femur"]))
    knee_deg = 180.0 - (tibia_raw - k["tibia"]) / ikm.TIBIA_SIGN   # inverse of tibia = cal + S*(180-knee)
    gamma = math.radians(knee_deg)
    kx, kz = F * math.cos(femur_el), F * math.sin(femur_el)
    tibia_dir = femur_el - (math.pi - gamma)
    fx, fz = kx + T * math.cos(tibia_dir), kz + T * math.sin(tibia_dir)
    r = C + fx
    v = r * math.sin(yaw)
    u = r * math.cos(yaw)
    body = (mx + v, my - side_sign(leg) * u, fz)
    return {
        "yaw_deg": math.degrees(yaw),
        "femur_elev_deg": math.degrees(femur_el),
        "knee_deg": knee_deg,
        "tibia_dir_deg": math.degrees(tibia_dir),
        "body": body,
        "knee_xz": (kx, kz),
        "foot_xz": (fx, fz),
    }


# ------------------------------------------------------------
# INVERSE KINEMATICS, step by step
# ------------------------------------------------------------
def ik_steps(leg, x, y, z):
    mx, my = mount(leg)
    k = cal(leg)
    s = {"leg": leg, "target": (x, y, z), "mount": (mx, my), "cal": k}
    s["v"] = x - mx                              # A: forward offset
    s["u"] = -side_sign(leg) * (y - my)          # A: outward distance
    s["z"] = z
    s["yaw"] = math.atan2(s["v"], s["u"])        # B: coxa yaw
    s["r"] = math.hypot(s["u"], s["v"])          # C: reach
    s["L"] = s["r"] - C
    s["d"] = math.hypot(s["L"], s["z"])
    s["d_min"] = abs(F - T)
    s["d_max"] = F + T
    s["reachable"] = s["d_min"] <= s["d"] <= s["d_max"]   # D
    d = min(max(s["d"], s["d_min"] + 1e-6), s["d_max"] - 1e-6)
    s["alpha"] = math.atan2(s["z"], s["L"])      # E
    s["beta"] = math.acos(max(-1.0, min(1.0, (F * F + d * d - T * T) / (2 * F * d))))
    s["femur_el"] = s["alpha"] + s["beta"]
    s["gamma"] = math.acos(max(-1.0, min(1.0, (F * F + T * T - d * d) / (2 * F * T))))  # F
    s["knee_deg"] = math.degrees(s["gamma"])
    s["coxa_raw"] = k["coxa"] + ikm.COXA_SIGN["R"] * math.degrees(s["yaw"])            # G
    s["femur_raw"] = k["femur"] + ikm.FEMUR_SIGN * math.degrees(s["femur_el"])
    s["tibia_raw"] = k["tibia"] + ikm.TIBIA_SIGN * (180.0 - s["knee_deg"])
    return s


def ik(leg, x, y, z):
    s = ik_steps(leg, x, y, z)
    return s["coxa_raw"], s["femur_raw"], s["tibia_raw"]


# ------------------------------------------------------------
# SWING (dry run)
# ------------------------------------------------------------
def smooth(s):
    return 0.5 - 0.5 * math.cos(math.pi * s)


def swing_path(start, stride=SWING_STRIDE, lift=SWING_LIFT, steps=SWING_STEPS):
    x0, y0, z0 = start
    out = []
    for i in range(steps + 1):
        s = i / steps
        out.append((x0 + stride * smooth(s), y0, z0 + lift * math.sin(math.pi * s)))
    return out


def in_range(raw):
    return all(ANGLE_MIN <= a <= ANGLE_MAX for a in raw)


# ------------------------------------------------------------
# REPORTING
# ------------------------------------------------------------
def swing_table(leg):
    start = fk(leg, *POSES["stand"][leg])["body"]
    rows = []
    for i, p in enumerate(swing_path(start)):
        s = ik_steps(leg, *p)
        raw = (s["coxa_raw"], s["femur_raw"], s["tibia_raw"])
        rows.append({"i": i, "body": p, "raw": raw, "ok": s["reachable"] and in_range(raw)})
    return rows


def round_trip(leg):
    results = {}
    for name in ("stand", "sit"):
        raw = POSES[name][leg]
        x, y, z = fk(leg, *raw)["body"]
        back = ik(leg, x, y, z)
        err = max(abs(a - b) for a, b in zip(back, raw))
        mod = ikm.leg_ik_body(leg, x, y, z)
        diff = max(abs(a - b) for a, b in zip(mod, back))
        results[name] = (err, diff)
    return results


def print_leg(leg):
    k = cal(leg)
    mx, my = mount(leg)
    print("=" * 74)
    print(f"LEG {leg}: coxa axis x={mx:.0f}, y={my:.0f}; calibration coxa {k['coxa']}, "
          f"femur {k['femur']}, tibia {k['tibia']}")
    print("=" * 74)
    for name in ("stand", "sit"):
        raw = POSES[name][leg]
        f = fk(leg, *raw)
        x, y, z = f["body"]
        print(f"[FK {name}] raw {raw}: yaw {f['yaw_deg']:+.2f}, femur elev {f['femur_elev_deg']:+.2f}, "
              f"knee {f['knee_deg']:.2f}, tibia dir {f['tibia_dir_deg']:+.2f}")
        print(f"            body foot x={x:.2f} y={y:.2f} z={z:.2f}")
    s = ik_steps(leg, *fk(leg, *POSES["stand"][leg])["body"])
    print(f"[IK stand] A v={s['v']:.3f} u={s['u']:.3f} z={s['z']:.3f} | B yaw={math.degrees(s['yaw']):.3f} "
          f"| C r={s['r']:.3f} L={s['L']:.3f} d={s['d']:.3f} | D reachable={s['reachable']} "
          f"| E alpha={math.degrees(s['alpha']):.3f} beta={math.degrees(s['beta']):.3f} "
          f"elev={math.degrees(s['femur_el']):.3f} | F knee={s['knee_deg']:.3f} "
          f"| G coxa={s['coxa_raw']:.2f} femur={s['femur_raw']:.2f} tibia={s['tibia_raw']:.2f}")
    for name, (err, diff) in round_trip(leg).items():
        print(f"  round trip {name:5s}: max err {err:.4f} deg | vs ik_module diff {diff:.2e}")
    print(f"  swing (stride {SWING_STRIDE:.0f}, lift {SWING_LIFT:.0f}):")
    print("   i   body x     body z   |  coxa   femur   tibia | ok")
    for r in swing_table(leg):
        print(f"  {r['i']:2d}  {r['body'][0]:8.2f}  {r['body'][2]:8.2f}  | "
              f"{r['raw'][0]:6.2f} {r['raw'][1]:7.2f} {r['raw'][2]:7.2f} | {r['ok']}")
    print()


def make_plots():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # Figure 1: top view + side view per right leg (stand pose)
    fig, axes = plt.subplots(1, 4, figsize=(20, 6))
    ax = axes[0]
    ax.add_patch(plt.Rectangle((-115, -60), 230, 120, fill=False, color="gray", lw=1.5))
    for name, (mx, my) in ikm.MOUNT_POS.items():
        col = "red" if name in RIGHT_LEGS else "k"
        ax.plot(mx, my, "o", color=col, ms=7)
        ax.annotate(name, (mx, my), textcoords="offset points", xytext=(8, 6), color=col)
    for leg in RIGHT_LEGS:
        x, y, _ = fk(leg, *POSES["stand"][leg])["body"]
        ax.plot([mount(leg)[0], x], [mount(leg)[1], y], "-", color="red", lw=2, alpha=0.6)
        ax.plot(x, y, "x", color="red", ms=9)
    ax.set_title("Top view: right legs (red) and stand feet")
    ax.set_xlabel("X forward (mm)")
    ax.set_ylabel("Y left (mm)")
    ax.set_aspect("equal")
    ax.set_xlim(-230, 260)
    ax.set_ylim(-340, 200)
    ax.grid(alpha=0.3)

    for ax, leg in zip(axes[1:], RIGHT_LEGS):
        f = fk(leg, *POSES["stand"][leg])
        kx, kz = f["knee_xz"]
        fx_, fz_ = f["foot_xz"]
        ax.plot(0, 0, "ko", ms=8)
        ax.plot([0, C], [0, 0], color="tab:blue", lw=6, solid_capstyle="butt", label=f"coxa {C}")
        ax.plot([C, C + kx], [0, kz], color="tab:green", lw=6, solid_capstyle="butt", label=f"femur {F}")
        ax.plot([C + kx, C + fx_], [kz, fz_], color="tab:orange", lw=6, solid_capstyle="butt",
                label=f"tibia {T}")
        ax.plot(C + fx_, fz_, "o", color="red", ms=7)
        ax.axhline(fz_, color="saddlebrown", ls="--", lw=1)
        ax.text(60, fz_ - 9, "ground (stand foot)", color="saddlebrown", fontsize=8)
        ax.text(C + kx + 8, kz + 8, f"knee {f['knee_deg']:.1f}\u00b0", fontsize=9)
        ax.text(C + 0.5 * F, -18, f"femur {f['femur_elev_deg']:+.1f}\u00b0", color="tab:green", fontsize=9)
        ax.text(150, -90, f"tibia dir {f['tibia_dir_deg']:+.1f}\u00b0", fontsize=9)
        ax.set_title(f"{leg} side view, STAND  (raw {POSES['stand'][leg]})")
        ax.set_xlabel("u outward from coxa axis (mm)")
        ax.set_ylabel("z up from coxa axis (mm)")
        ax.set_aspect("equal")
        ax.set_xlim(-30, 260)
        ax.set_ylim(-95, 60)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
    fig.suptitle("Right legs - step 1: geometry and stand pose")
    fig.tight_layout()
    p1 = os.path.join(HERE, "docs", "right_legs_diagram.png")
    os.makedirs(os.path.dirname(p1), exist_ok=True)
    fig.savefig(p1, dpi=120)

    # Figure 2: swing paths for each right leg (body frame, side view)
    fig, axes = plt.subplots(1, 3, figsize=(18, 5.5))
    for ax, leg in zip(axes, RIGHT_LEGS):
        rows = swing_table(leg)
        ax.plot([r["body"][0] for r in rows], [r["body"][2] for r in rows], "-o", color="red", ms=4)
        for r in (rows[0], rows[len(rows) // 2], rows[-1]):
            ax.annotate(f"{r['i']}: [{r['raw'][0]:.0f},{r['raw'][1]:.0f},{r['raw'][2]:.0f}]",
                        (r["body"][0], r["body"][2]), textcoords="offset points", xytext=(6, 8), fontsize=8)
        ax.axhline(rows[0]["body"][2], color="saddlebrown", ls="--", lw=1)
        ax.set_title(f"{leg} swing: lift \u2192 forward \u2192 lower")
        ax.set_xlabel("body X forward (mm)")
        ax.set_ylabel("body Z up (mm)")
        ax.grid(alpha=0.3)
    fig.suptitle("Dry-run single-step swing, right legs (labels: waypoint [coxa, femur, tibia])")
    fig.tight_layout()
    p2 = os.path.join(HERE, "docs", "right_legs_swing.png")
    fig.savefig(p2, dpi=120)
    print(f"Diagrams written: {p1}, {p2}")


def write_doc(path):
    L = []
    L.append("# Right legs (RF, RM, RR) - Step 1\n")
    L.append("Same method as `docs/RF_leg_step1.md`, applied to each right-side leg. "
             "Pure math: nothing is sent to the servos.\n")
    L.append("Files: `right_legs_step1.py` (calculations, `--plot`, `--doc`), "
             "`docs/right_legs_diagram.png`, `docs/right_legs_swing.png`.\n")
    L.append("## Geometry (mm)\n")
    L.append(f"Coxa {C}, femur {F}, tibia {T}. Body frame: +X forward, +Y left, +Z up. "
             "z is measured from the coxa axis.\n")
    L.append("| Leg | Coxa axis (x, y) | Calibration coxa / femur / tibia |")
    L.append("|---|---|---|")
    for leg in RIGHT_LEGS:
        k = cal(leg)
        L.append(f"| {leg} | ({mount(leg)[0]:.0f}, {mount(leg)[1]:.0f}) | "
                 f"{k['coxa']} / {k['femur']} / {k['tibia']} |")
    L.append("\n## Equations (same for every right leg)\n")
    L.append("```")
    L.append("A  v = x - mx        u = -(y - my)        z = z")
    L.append("B  yaw = atan2(v, u)                     coxa  = cal_coxa  + yaw[deg]")
    L.append("C  r = hypot(u, v)   L = r - 64.5   d = hypot(L, z)")
    L.append(f"D  |{F} - {T}| <= d <= {F + T:.2f}        (reachability)")
    L.append("E  alpha = atan2(z, L)   beta = acos((F^2 + d^2 - T^2) / (2 F d))")
    L.append("   femur = cal_femur + (alpha + beta)[deg]")
    L.append("F  knee = acos((F^2 + T^2 - d^2) / (2 F T))        (180 = straight)")
    L.append("G  tibia = cal_tibia + (-1) * (180 - knee[deg])")
    L.append("```\n")
    L.append("Here mx, my is the coxa axis: RF (115, -60), RM (0, -60), RR (-115, -60).\n")
    for leg in RIGHT_LEGS:
        k = cal(leg)
        L.append(f"## {leg}\n")
        L.append("| Pose | Raw [coxa, femur, tibia] | Foot x | Foot y | Foot z | Femur elev | Knee | Tibia dir |")
        L.append("|---|---|---|---|---|---|---|---|")
        for name in ("stand", "sit"):
            raw = POSES[name][leg]
            f = fk(leg, *raw)
            x, y, z = f["body"]
            L.append(f"| {name} | {list(raw)} | {x:.2f} | {y:.2f} | {z:.2f} | "
                     f"{f['femur_elev_deg']:+.2f}\u00b0 | {f['knee_deg']:.2f}\u00b0 | {f['tibia_dir_deg']:+.2f}\u00b0 |")
        s = ik_steps(leg, *fk(leg, *POSES["stand"][leg])["body"])
        L.append("\nIK worked example, stand foot:\n")
        L.append("| Step | Result |")
        L.append("|---|---|")
        L.append(f"| A | v = {s['v']:.3f}, u = {s['u']:.3f}, z = {s['z']:.3f} |")
        L.append(f"| B | yaw = {math.degrees(s['yaw']):.3f}\u00b0 -> coxa = {s['coxa_raw']:.2f} |")
        L.append(f"| C | r = {s['r']:.3f}, L = {s['L']:.3f}, d = {s['d']:.3f} |")
        L.append(f"| D | {s['d_min']:.2f} <= d <= {s['d_max']:.2f}: reachable = {s['reachable']} |")
        L.append(f"| E | alpha = {math.degrees(s['alpha']):.3f}\u00b0, beta = {math.degrees(s['beta']):.3f}\u00b0, "
                 f"elevation = {math.degrees(s['femur_el']):.3f}\u00b0 -> femur = {s['femur_raw']:.2f} |")
        L.append(f"| F | knee = {s['knee_deg']:.3f}\u00b0 |")
        L.append(f"| G | tibia = {s['tibia_raw']:.2f} |")
        rt = round_trip(leg)
        L.append(f"\nRound trip: stand max error {rt['stand'][0]:.4f}\u00b0, sit max error "
                 f"{rt['sit'][0]:.4f}\u00b0. Matches `ik_module.py` (diff {max(rt['stand'][1], rt['sit'][1]):.1e}).\n")
        L.append("Dry-run swing (30 mm forward, 20 mm lift, 11 waypoints):\n")
        L.append("| i | Foot x | Foot z | Coxa | Femur | Tibia | In range |")
        L.append("|---|---|---|---|---|---|---|")
        for r in swing_table(leg):
            L.append(f"| {r['i']} | {r['body'][0]:.2f} | {r['body'][2]:.2f} | {r['raw'][0]:.2f} | "
                     f"{r['raw'][1]:.2f} | {r['raw'][2]:.2f} | {r['ok']} |")
        L.append("")
    L.append("## Open items\n")
    L.append("- Tibia sign: the comment in `ik_module.py` disagrees with its formula (see `docs/RF_leg_step1.md`). Confirm on hardware.")
    L.append("- Stand foot height is assumed to be ground level for every leg.")
    L.append("- Servo safe ranges are not yet confirmed. Nothing here has been run on the robot.")
    with open(path, "w") as fh:
        fh.write("\n".join(L) + "\n")
    print(f"Doc written: {path}")


def main():
    print("SCORPION - ALL RIGHT LEGS, STEP 1\n")
    for leg in RIGHT_LEGS:
        print_leg(leg)
    if "--plot" in sys.argv:
        make_plots()
    if "--doc" in sys.argv:
        write_doc(os.path.join(HERE, "docs", "Right_legs_step1.md"))


if __name__ == "__main__":
    main()