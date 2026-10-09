#!/usr/bin/env python3
"""Regenerate rf_leg_diagram.png from the current ik_module constants.

Three panels: the pitch plane with the hand-calibrated stand/sit poses on the
true linkage; a top view of where the stand pose puts the RF foot; and one
smooth RF-only swing/stance cycle (the --rf-cycle motion) in joint space.
Run:  python3 rf_leg_diagram.py
"""
import math
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

from ik_module import COXA_LEN as COXA, FEMUR_LEN as FEM, TIBIA_LEN as TIB, CALIBRATION
from tripod_gait import USER_STAND_PHYS, rf_cycle_segments, rf_stand_foot

CAL = CALIBRATION["RF"]
SF, ST = +1, -1


def fk(raw):
    c, f, t = raw
    yaw = math.radians(c - CAL["coxa"])
    elev = math.radians(SF * (f - CAL["femur"]))
    phi = ST * (t - CAL["tibia"])
    ta = elev - math.radians(phi)
    hip = (COXA, 0.0)
    knee = (hip[0] + FEM * math.cos(elev), hip[1] + FEM * math.sin(elev))
    foot = (knee[0] + TIB * math.cos(ta), knee[1] + TIB * math.sin(ta))
    u = foot[0] * math.cos(yaw); v = foot[0] * math.sin(yaw)
    return hip, knee, foot, math.degrees(elev), phi, u, v, foot[1]


STAND = USER_STAND_PHYS["RF"]
SIT = (153.0, 120.0, 90.0)
hipS, kneeS, footS, elS, phS, uS, vS, zS = fk(STAND)
hipT, kneeT, footT, elT, phT, uT, vT, zT = fk(SIT)
hipC, kneeC, footC, *_ = fk((CAL["coxa"], CAL["femur"], CAL["tibia"]))

fig = plt.figure(figsize=(16.5, 10.5))
gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1.0], hspace=0.30, wspace=0.22)

ax = fig.add_subplot(gs[0, 0])
ax.set_aspect("equal")
ax.axhline(0, color="0.7", lw=1)
ax.text(2, 4, "body / coxa plane  (z = 0)", color="0.45", fontsize=9)
ax.axhline(zS, color="0.7", lw=1)
ax.text(2, zS - 9, "ground at stand  (z = %.1f mm)" % zS, color="0.45", fontsize=9)
ax.plot([0, hipC[0], kneeC[0], footC[0]], [0, 0, 0, 0], color="0.75", lw=1.6, ls="--")
ax.text(283, 62, "belly-down calibration zero\n(raw %.0f/%.0f/%.0f, leg straight out)"
        % (CAL["coxa"], CAL["femur"], CAL["tibia"]), color="0.55", fontsize=8.5, ha="right", va="top")
ax.plot([0, hipT[0], kneeT[0], footT[0]], [0, hipT[1], kneeT[1], footT[1]], color="tab:orange", lw=1.8, ls=":")
ax.plot([0, hipS[0], kneeS[0], footS[0]], [0, hipS[1], kneeS[1], footS[1]], color="tab:blue", lw=3.2, solid_capstyle="round")
for p in ((0, 0), hipS, kneeS):
    ax.plot(*p, "o", ms=9, mfc="w", mec="k", mew=1.6, zorder=5)
ax.plot(*footS, "s", ms=9, color="tab:blue", zorder=5)
ax.annotate("coxa %.2f" % COXA, (32, 0), (30, 16), fontsize=9, arrowprops=dict(arrowstyle="-", color="0.4", lw=0.8))
ax.annotate("femur %.2f" % FEM, ((hipS[0] + kneeS[0]) / 2, (hipS[1] + kneeS[1]) / 2), (60, 52), fontsize=9,
            arrowprops=dict(arrowstyle="-", color="0.4", lw=0.8))
ax.annotate("tibia + foot %.2f" % TIB, ((kneeS[0] + footS[0]) / 2, (kneeS[1] + footS[1]) / 2), (150, -30), fontsize=9,
            arrowprops=dict(arrowstyle="-", color="0.4", lw=0.8))
ax.annotate("", xy=(kneeS[0], kneeS[1]), xytext=(kneeS[0], 0), arrowprops=dict(arrowstyle="<->", color="tab:green", lw=1.2))
ax.text(96, 12, "knee +%.1f\nabove plane" % kneeS[1], color="tab:green", fontsize=8.5, ha="right")
ax.annotate("", xy=(footS[0] + 14, footS[1]), xytext=(footS[0] + 14, 0), arrowprops=dict(arrowstyle="<->", color="tab:red", lw=1.2))
ax.text(footS[0] + 17, footS[1] / 2, "body height\n%.1f mm" % abs(zS), color="tab:red", fontsize=8.5)
ax.text(6, 52, "femur %+.0f deg above horizontal" % elS, color="tab:blue", fontsize=9.5, weight="bold")
ax.text(kneeS[0] + 7, kneeS[1] - 24, "knee bend %.1f$^\\circ$\n(interior %.1f$^\\circ$)" % (phS, 180 - phS), color="tab:blue", fontsize=9)
ax.text(8, -118, "STAND  raw %.1f / %.1f / %.1f   (your table 102/110/35 x1.5)" % STAND, color="tab:blue", fontsize=10, weight="bold")
ax.text(8, -132, "SIT       raw %.1f / %.1f / %.1f    (your table 102/80/60 x1.5)   note: computes HIGHER than stand - see RF_LEG_IK.md" % SIT, color="tab:orange", fontsize=9)
ax.set_xlim(-20, 285); ax.set_ylim(-150, 75)
ax.set_xlabel("mm outboard from coxa axis"); ax.set_ylabel("mm up")
ax.set_title("RF leg, pitch plane - your calibrated poses on the true linkage", fontsize=11, weight="bold")
ax.grid(alpha=0.25)

ax = fig.add_subplot(gs[0, 1])
ax.set_aspect("equal")
ax.add_patch(Rectangle((-140, -75), 280, 150, fc="0.92", ec="0.4", lw=1.2))
ax.text(0, 0, "body\n230 long", ha="center", va="center", color="0.45", fontsize=9)
mounts = {"RF": (115, -60), "RM": (0, -60), "RR": (-115, -60),
          "LF": (115, 60), "LM": (0, 60), "LR": (-115, 60)}
for leg, (mx, my) in mounts.items():
    ax.plot(mx, my, "o", ms=6, mfc="0.55", mec="k", mew=0.8)
    ax.text(mx, my + (9 if my > 0 else -13), leg, ha="center", fontsize=8.5, color="0.4")
mx, my = mounts["RF"]
bx, by = mx + vS, my - uS
ax.plot([mx, mx + 18 * math.sin(math.radians(6))], [my, my - 18 * math.cos(math.radians(6))], color="tab:blue", lw=2)
ax.plot([mx, bx], [my, by], color="tab:blue", lw=2, ls="--")
ax.plot(bx, by, "s", ms=9, color="tab:blue")
ax.text(bx + 8, by - 6, "RF foot at stand\nbody (%.0f, %.0f)" % (bx, by), color="tab:blue", fontsize=9)
ax.annotate("yaw %+.0f$^\\circ$ fwd" % (STAND[0] - CAL["coxa"]), (mx + 6, my - 24), (mx + 34, my - 40), fontsize=9,
            color="tab:blue", arrowprops=dict(arrowstyle="-", color="tab:blue", lw=0.8))
ax.annotate("", xy=(bx, by), xytext=(bx, -by), arrowprops=dict(arrowstyle="<->", color="tab:red", lw=1.1))
ax.text(bx + 10, 0, "stance width\n%.0f mm" % (2 * abs(by)), color="tab:red", fontsize=9, va="center")
ax.set_xlim(-300, 320); ax.set_ylim(-300, 300)
ax.set_xlabel("body x, mm forward"); ax.set_ylabel("body y, mm")
ax.set_title("top view - where the stand pose puts the RF foot", fontsize=11, weight="bold")
ax.grid(alpha=0.25)

ax = fig.add_subplot(gs[1, :])
segs = rf_cycle_segments()
u0, v0, z0 = rf_stand_foot()
ph, cc, ff, tt, zz = [], [], [], [], []
stride, lift = 50.0, 25.0
N = 96
for i in range(N + 1):
    s = i / N
    if s <= 0.5:
        t = s / 0.5
        v = v0 + stride / 2 * (1 - 2 * t); z = z0
    else:
        t = (s - 0.5) / 0.5
        e = t * t * (3 - 2 * t)
        v = v0 - stride / 2 + stride * e
        z = z0 + lift * math.sin(math.pi * t)
    from ik_module import leg_ik
    c, f, tg = leg_ik("RF", u0, v, z)
    ph.append(s); cc.append(c); ff.append(f); tt.append(tg); zz.append(z)
ax.axvspan(0, 0.5, color="tab:blue", alpha=0.07)
ax.axvspan(0.5, 1.0, color="tab:orange", alpha=0.09)
ax.text(0.25, max(ff) + 4.5, "STANCE  (foot planted, sweeps backward -> body moves forward)", ha="center", color="tab:blue", fontsize=9.5)
ax.text(0.75, max(ff) + 4.5, "SWING  (foot lifted %.0f mm, smoothstep forward)" % lift, ha="center", color="tab:orange", fontsize=9.5)
ax.plot(ph, cc, lw=2.4, label="coxa raw  (ch 1)   %.1f .. %.1f" % (min(cc), max(cc)))
ax.plot(ph, ff, lw=2.4, label="femur raw (ch 2)   %.1f .. %.1f" % (min(ff), max(ff)))
ax.plot(ph, tt, lw=2.4, label="tibia raw (ch 3)   %.1f .. %.1f" % (min(tt), max(tt)))
ax.plot(ph, zz, lw=1.4, ls="--", color="0.4", label="foot height z  (%.1f .. %.1f)" % (min(zz), max(zz)))
ax.axhline(STAND[2], color="0.6", lw=0.8, ls=":")
ax.text(0.01, STAND[2] + 1.5, "stand tibia %.1f" % STAND[2], color="0.5", fontsize=8)
ax.set_xlabel("cycle phase  (0 -> 1)"); ax.set_ylabel("true physical servo degrees")
ax.set_title("RF only, one 2000 ms cycle around YOUR stand pose: stride %.0f mm, lift %.0f mm - every joint continuous, zero velocity at touch-down and lift-off" % (stride, lift),
             fontsize=10.5, weight="bold")
ax.legend(loc="lower left", fontsize=9, ncol=2)
ax.grid(alpha=0.25)
ax.set_xlim(0, 1)

fig.suptitle("Scorpion hexapod - RIGHT FRONT leg: geometry, calibrated stand pose, and single-leg gait",
             fontsize=13.5, weight="bold")
fig.savefig("rf_leg_diagram.png", dpi=150, bbox_inches="tight", facecolor="white")
print("saved rf_leg_diagram.png")
