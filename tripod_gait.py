#!/usr/bin/env python3
"""
Tripod gait generator for the Scorpion hexapod - six legs, no tail or claw.

THE TWO BUGS THIS REPLACES
--------------------------
The caterpillar_walk() and tripod_walk() in stand_sit_walk_tail_claw_belly.py
do not translate the robot.  They lift a leg, swing its coxa +forward, plant
it, and then on the next step lift it back to the ORIGINAL base angle.  `base`
is never updated, so every cycle returns the feet to the same absolute
position and the body walks in place.  Both functions also apply
`coxa += forward` to left and right legs alike, but increasing raw coxa is
FORWARD on a right leg and BACKWARD on a left leg, so the two sides push
against each other and the body twists instead of advancing.

Here the foot targets are expressed in the BODY frame and solved through the
IK, so the left/right mirroring is handled by the solver rather than by hand
signed deltas.  Body translation comes from the stance foot sweeping backward
through the body frame: each leg's foot travels from +stride/2 to -stride/2
while planted, so the body advances by `stride` per gait cycle.

SMOOTHNESS
----------
The LSC-24 interpolates onboard, so each gait sub-phase is ONE 18-servo array
frame with a duration, never a Python stepping loop.  Sub-phases are short
enough that the piecewise-linear foot path reads as smooth, and they are
pipelined: ServoBus.send_pose() sleeps for exactly the commanded duration, so
the next frame finishes arriving at the instant the previous segment ends and
the board never idles between segments.

The swing uses a smoothstep in the fore/aft axis and sin(pi*t) for the lift,
both of which have zero velocity at touchdown and lift-off, so the foot does
not jerk at the transitions.

WHY A TRIPOD
------------
RF+LM+RR against LF+RM+LR keeps three legs planted at all times, which is
statically stable for a wide-sprawling hexapod and needs no balance control.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from ik_module import (
    CALIBRATION,
    COXA_LEN,
    LEG_ORDER,
    MOUNT_POS,
    PLANAR_MAX,
    WorkspaceError,
    all_legs_pose,
    femur_travel,
    leg_fk,
    leg_ik,
    leg_ik_body,
    leg_reach_limits,
)
from servo_lsc24 import (
    DEGREE_SAFE_MAX,
    DEGREE_SAFE_MIN,
    TRIPOD_A,
    ServoBus,
    tx_ms,
)

N_LEG_SERVOS = 18


# ============================================================
# CONFIG
# ============================================================
@dataclass
class GaitConfig:
    """Standing geometry and gait parameters, all in mm / ms / kg.

    The defaults below came from a constrained scan over the whole workspace:
    they maximise the tightest servo margin across all six legs and the entire
    stride, subject to peak femur torque staying under 60% of the HPS-2027's
    20 kg.cm stall figure at an assumed 3.0 kg robot mass.

    reach_out  : horizontal distance from the FEMUR PIVOT to the planted foot.
                 This alone sets femur torque (torque = leg_load x reach_out),
                 independent of body height.  Smaller is gentler on the servo
                 but forces more knee bend, which costs tibia margin.
    body_height: coxa axis above ground.  Taller straightens the leg and buys
                 tibia margin, but raises the centre of gravity and pushes
                 toward the full-stretch singularity.
    """

    reach_out: float = 80.0        # mm past the femur pivot
    body_height: float = 155.0     # mm, coxa axis above ground
    stride: float = 50.0           # mm of body advance per gait cycle
    lift: float = 25.0             # mm of foot clearance at peak swing
    cycle_ms: int = 2000           # full cycle (stance + swing)
    sub_phases: int = 16           # array commands per cycle
    mass_kg: float = 3.0           # whole robot, for the torque report only
    dynamic_factor: float = 1.4    # stance peak vs static, for the torque report
    stall_torque_kgcm: float = 20.0

    # ---------- derived ----------
    @property
    def foot_outboard(self) -> float:
        """Coxa axis -> planted foot, horizontal."""
        return COXA_LEN + self.reach_out

    @property
    def stance_width(self) -> float:
        return 2.0 * (abs(MOUNT_POS["RF"][1]) + self.foot_outboard)

    @property
    def step_ms(self) -> float:
        return self.cycle_ms / self.sub_phases

    def validate_timing(self) -> None:
        """A sub-phase shorter than its own frame cannot be pipelined: frames
        would queue faster than the 9600-baud wire drains them and latency
        would grow without bound."""
        floor = tx_ms(N_LEG_SERVOS)
        if self.step_ms < floor:
            raise ValueError(
                f"sub_phases={self.sub_phases} gives {self.step_ms:.1f} ms per segment, "
                f"but an {N_LEG_SERVOS}-servo array frame takes {floor:.1f} ms to transmit "
                f"at 9600 baud.  Use at most {int(self.cycle_ms // floor)} sub-phases "
                f"for a {self.cycle_ms} ms cycle."
            )


# ============================================================
# FOOT TRAJECTORY
# ============================================================
def _smoothstep(t: float) -> float:
    return t * t * (3.0 - 2.0 * t)


def leg_phase(global_phase: float, leg: str) -> float:
    """That leg's local phase in [0,1).  Tripod A leads tripod B by half a cycle."""
    offset = 0.0 if leg in TRIPOD_A else 0.5
    return (global_phase + offset) % 1.0


def leg_foot_offset(p: float, cfg: GaitConfig) -> Tuple[float, float]:
    """(fore/aft offset v, lift) for one leg at local phase p.

    p in [0, 0.5) : STANCE - foot planted, sweeping backward through the body
                    frame at a constant rate, so the body advances smoothly.
    p in [0.5, 1) : SWING  - foot lifted, returning forward, eased at both ends.
    """
    half = cfg.stride / 2.0
    if p < 0.5:
        s = p / 0.5
        return half - cfg.stride * s, 0.0
    s = (p - 0.5) / 0.5
    e = _smoothstep(s)
    return -half + cfg.stride * e, cfg.lift * math.sin(math.pi * e)


def foot_target_body(leg: str, p: float, cfg: GaitConfig) -> Tuple[float, float, float]:
    """Body-frame foot target (x forward, y left, z up) for one leg at phase p."""
    mx, my = MOUNT_POS[leg]
    outboard = -1.0 if leg.startswith("R") else 1.0
    v, lift = leg_foot_offset(p, cfg)
    return mx + v, my + outboard * cfg.foot_outboard, -cfg.body_height + lift


def solve_phase(global_phase: float, cfg: GaitConfig
                ) -> Dict[str, Tuple[float, float, float]]:
    """IK for all six legs at one point in the gait cycle."""
    out: Dict[str, Tuple[float, float, float]] = {}
    for leg in LEG_ORDER:
        p = leg_phase(global_phase, leg)
        x, y, z = foot_target_body(leg, p, cfg)
        out[leg] = leg_ik_body(leg, x, y, z)
    return out


# ============================================================
# MARGIN / TORQUE REPORT
# ============================================================
@dataclass
class MarginReport:
    worst_servo_margin: float
    worst_leg: str
    worst_phase: float
    peak_torque_kgcm: float
    peak_torque_pct: float
    d_min_seen: float
    d_max_seen: float
    per_leg: Dict[str, float]
    feasible: bool


def torque_kgcm(cfg: GaitConfig, reach_out: float) -> float:
    """Femur joint torque in kg.cm for one stance leg in a tripod.

    Three legs carry the whole robot.  The coxa is a vertical yaw axis so
    gravity barely loads it; the femur is the joint that matters, and its
    moment arm is the horizontal reach -- independent of body height.
    """
    newton_per_leg = cfg.mass_kg * 9.81 / 3.0
    return newton_per_leg * (reach_out / 1000.0) * 10.197 * cfg.dynamic_factor


def check_cycle(cfg: GaitConfig, samples: int = 24) -> MarginReport:
    """Sample the whole gait cycle and report the tightest margins.

    Run this BEFORE walking.  It solves every leg at every sample point and
    raises WorkspaceError through solve_phase() if any pose is unreachable, so
    the robot never discovers that mid-stride.
    """
    cfg.validate_timing()
    worst_margin = 1e9
    worst_leg, worst_phase = "", 0.0
    per_leg: Dict[str, float] = {leg: 1e9 for leg in LEG_ORDER}
    peak_torque = 0.0
    d_lo, d_hi = 1e9, 0.0

    for i in range(samples):
        gp = i / samples
        angles = solve_phase(gp, cfg)
        for leg, (c, f, t) in angles.items():
            m = min(c - DEGREE_SAFE_MIN, DEGREE_SAFE_MAX - c,
                    f - DEGREE_SAFE_MIN, DEGREE_SAFE_MAX - f,
                    t - DEGREE_SAFE_MIN, DEGREE_SAFE_MAX - t)
            per_leg[leg] = min(per_leg[leg], m)
            if m < worst_margin:
                worst_margin, worst_leg, worst_phase = m, leg, gp
        p = leg_phase(gp, "RF")
        v, lift = leg_foot_offset(p, cfg)
        r = math.hypot(cfg.foot_outboard, v)
        L = r - COXA_LEN
        # Torque only counts while the foot is planted; a swinging leg is
        # unloaded, so use the stance reach (lift == 0) for the peak.
        if lift == 0.0:
            peak_torque = max(peak_torque, torque_kgcm(cfg, L))
        # But the knee extension range must cover the swing lift too, or the
        # reported band misses the tightest point of the whole cycle.
        d = math.hypot(L, cfg.body_height - lift)
        d_lo, d_hi = min(d_lo, d), max(d_hi, d)

    return MarginReport(
        worst_servo_margin=worst_margin,
        worst_leg=worst_leg,
        worst_phase=worst_phase,
        peak_torque_kgcm=peak_torque,
        peak_torque_pct=100.0 * peak_torque / cfg.stall_torque_kgcm,
        d_min_seen=d_lo,
        d_max_seen=d_hi,
        per_leg=per_leg,
        feasible=worst_margin >= 15.0 and peak_torque <= 0.6 * cfg.stall_torque_kgcm,
    )


def print_report(cfg: GaitConfig, rep: Optional[MarginReport] = None) -> MarginReport:
    rep = rep or check_cycle(cfg)
    print(f"Gait config: reach_out={cfg.reach_out:.0f}mm  body_height={cfg.body_height:.0f}mm  "
          f"stride={cfg.stride:.0f}mm  lift={cfg.lift:.0f}mm")
    print(f"             cycle={cfg.cycle_ms}ms in {cfg.sub_phases} segments "
          f"({cfg.step_ms:.0f}ms each, {tx_ms(N_LEG_SERVOS):.0f}ms to transmit)")
    print(f"             foot {cfg.foot_outboard:.1f}mm outboard of each coxa axis, "
          f"stance width {cfg.stance_width:.0f}mm")
    print(f"\n  Servo margins (degrees from the safe-band stop), worst over the whole cycle:")
    for leg in LEG_ORDER:
        cal = CALIBRATION[leg]
        dmn, dmx = leg_reach_limits(leg)
        fmn, fmx = femur_travel(leg)
        print(f"    {leg}: margin {rep.per_leg[leg]:6.1f}   "
              f"cal coxa={cal['coxa']:6.1f} femur={cal['femur']:6.1f} tibia={cal['tibia']:6.1f}   "
              f"d [{dmn:.1f},{dmx:.1f}]  femur elev [{fmn:+.0f},{fmx:+.0f}]")
    print(f"\n  Tightest margin anywhere : {rep.worst_servo_margin:.1f} deg "
          f"on {rep.worst_leg} at cycle phase {rep.worst_phase:.2f}")
    print(f"  Knee extension seen      : {100*rep.d_min_seen/PLANAR_MAX:.0f}-"
          f"{100*rep.d_max_seen/PLANAR_MAX:.0f}% of full stretch "
          f"(d {rep.d_min_seen:.1f}-{rep.d_max_seen:.1f} of {PLANAR_MAX:.1f} mm)")
    print(f"  Peak femur torque        : {rep.peak_torque_kgcm:.1f} kg.cm = "
          f"{rep.peak_torque_pct:.0f}% of the {cfg.stall_torque_kgcm:.0f} kg.cm stall "
          f"figure (at {cfg.mass_kg:.1f} kg, x{cfg.dynamic_factor} dynamic)")
    verdict = "FEASIBLE" if rep.feasible else "MARGINAL - tighten reach_out/stride or raise body_height"
    print(f"\n  Verdict: {verdict}")
    return rep


# ============================================================
# POSTURES
# ============================================================
def calibration_pose() -> Dict[int, float]:
    """Belly down, legs stretched straight out.  This is the recorded zero pose
    and the safe starting/ending position: nothing is loaded, nothing is near a
    limit."""
    return all_legs_pose({leg: (CALIBRATION[leg]["coxa"],
                                CALIBRATION[leg]["femur"],
                                CALIBRATION[leg]["tibia"]) for leg in LEG_ORDER})


def stand_pose(cfg: GaitConfig) -> Dict[int, float]:
    """Mid-stance standing pose: every foot planted at v = 0."""
    return all_legs_pose(solve_phase(0.25, cfg))   # 0.25 = middle of tripod A's stance


# The hand-calibrated standing pose, in TRUE physical degrees.  This is
# POSITIONS["stand"] from stand_sit_walk_tail_claw_belly.py (legacy /180 command
# units) multiplied by LEGACY_CALIBRATION_SCALE, i.e. the pose the robot is
# known to stand in on the bench -- see RF_LEG_IK.md.  It is the initial
# posture before walking; the gait itself runs from the (taller, narrower)
# stance in GaitConfig, reached by a short transition from here.
USER_STAND_PHYS: Dict[str, Tuple[float, float, float]] = {
    "RF": (153.0, 165.0, 52.5),   # legacy 102/110/35
    "RM": (135.0, 165.0, 52.5),   # legacy  90/110/35
    "RR": (154.5, 157.5, 52.5),   # legacy 103/105/35
    "LF": (172.5, 165.0, 60.0),   # legacy 115/110/40
    "LM": (135.0, 165.0, 30.0),   # legacy  90/110/20
    "LR": (135.0, 165.0, 45.0),   # legacy  90/110/30
}


def user_stand_pose() -> Dict[int, float]:
    """Channel map of USER_STAND_PHYS, ready for ServoBus.send_pose()."""
    return all_legs_pose(USER_STAND_PHYS)


def user_stand(bus: ServoBus, duration_ms: int = 2500) -> None:
    """Rise into the hand-calibrated stand pose in one interpolated move."""
    bus.send_pose(user_stand_pose(), duration_ms)


# ============================================================
# RF-ONLY CYCLE  (single-leg bring-up, see RF_LEG_IK.md section 4)
# ============================================================
def rf_stand_foot() -> Tuple[float, float, float]:
    """Leg-local foot position (u, v, z) of the hand-calibrated RF stand pose."""
    return leg_fk("RF", *USER_STAND_PHYS["RF"])


def rf_cycle_segments(stride: float = 50.0, lift: float = 25.0,
                      cycle_ms: int = 2000, phases: int = 16
                      ) -> List[Tuple[Tuple[float, float, float], int]]:
    """One smooth RF swing/stance cycle centred on the hand-calibrated foot.

    Returns [((coxa, femur, tibia) physical degrees, segment ms), ...] for the
    segment END points; the board interpolates each segment onboard, and
    send_pose() pipelines them, so the motion is continuous.

    Stance (s 0..0.5): foot planted at stand height, sweeps v +stride/2 ->
    -stride/2, which is what advances the body.  Swing (s 0.5..1): smoothstep
    forward with a sin(pi t) lift, both zero-velocity at touch-down and
    lift-off.  s = 1 lands exactly on s = 0, so cycles chain.

    Only the three RF channels change; the other five legs hold whatever pose
    they are in.  Raises WorkspaceError if any segment leaves the safe band.
    """
    u, v0, z0 = rf_stand_foot()
    seg_ms = cycle_ms // phases
    need = tx_ms(3)
    if seg_ms < need:
        raise WorkspaceError(
            f"rf cycle: {seg_ms} ms segments are shorter than the {need:.0f} ms "
            f"a 3-servo frame needs at 9600 baud; use fewer --phases or a longer --cycle")

    def foot_at(s: float) -> Tuple[float, float, float]:
        if s <= 0.5:
            t = s / 0.5
            return u, v0 + stride / 2 * (1 - 2 * t), z0
        t = (s - 0.5) / 0.5
        e = t * t * (3 - 2 * t)
        return u, v0 - stride / 2 + stride * e, z0 + lift * math.sin(math.pi * t)

    out = []
    for i in range(phases):
        su, sv, sz = foot_at((i + 1) / phases)
        angles = leg_ik("RF", su, sv, sz)          # raises if out of workspace
        out.append((angles, seg_ms))
    return out


def rf_cycle_report(segments) -> str:
    """Joint ranges over a segment list, for the bring-up printout."""
    lo = [min(s[0][j] for s in segments) for j in range(3)]
    hi = [max(s[0][j] for s in segments) for j in range(3)]
    names = ("coxa", "femur", "tibia")
    return ", ".join(f"{n} {l:.1f}..{h:.1f}" for n, l, h in zip(names, lo, hi))


def belly_down(bus: ServoBus, duration_ms: int = 2500) -> None:
    """Fold out to the calibrated belly-down pose.  Safe from anywhere."""
    bus.send_pose(calibration_pose(), duration_ms)


def rise(bus: ServoBus, cfg: GaitConfig, duration_ms: int = 3000) -> None:
    """Rise from belly-down to standing in one onboard-interpolated move."""
    check_cycle(cfg)                      # refuses to rise into an unreachable pose
    bus.send_pose(stand_pose(cfg), duration_ms)


def sit(bus: ServoBus, duration_ms: int = 2500) -> None:
    belly_down(bus, duration_ms)


# ============================================================
# WALK
# ============================================================
def walk(bus: ServoBus, cfg: GaitConfig, cycles: int = 4,
         dry_report: bool = False) -> List[Dict[int, float]]:
    """Run `cycles` tripod gait cycles.  The body advances cfg.stride mm per cycle.

    With dry_report=True nothing is sent; the computed segment poses are
    returned so the trajectory can be inspected or plotted offline.
    """
    cfg.validate_timing()
    rep = check_cycle(cfg)
    if not rep.feasible:
        raise WorkspaceError(
            f"gait is not feasible: tightest servo margin {rep.worst_servo_margin:.1f} deg "
            f"on {rep.worst_leg}, peak femur torque {rep.peak_torque_pct:.0f}% of stall. "
            f"Reduce reach_out/stride or raise body_height."
        )

    n = cfg.sub_phases
    step = int(round(cfg.step_ms))
    segments: List[Dict[int, float]] = []

    total = cycles * n
    for k in range(total):
        gp = (k % n) / n
        pose = all_legs_pose(solve_phase(gp, cfg))
        segments.append(pose)
        if dry_report:
            continue
        last = (k == total - 1)
        bus.send_pose(pose, step, sleep_after=not last)
    return segments


def describe_trajectory(cfg: GaitConfig, cycles: int = 1) -> None:
    """Print the foot path for one cycle so the trajectory can be eyeballed."""
    print(f"Foot trajectory, one cycle, {cfg.sub_phases} segments "
          f"(body advances {cfg.stride:.0f}mm per cycle)\n")
    for leg in ("RF", "LF"):
        print(f"  {leg}  (tripod {'A' if leg in TRIPOD_A else 'B'})")
        print(f"    {'seg':>3} {'phase':>6} {'mode':>6} {'v(fwd)':>8} {'lift':>7} "
              f"{'coxa':>8} {'femur':>8} {'tibia':>8}")
        for k in range(cfg.sub_phases + 1):
            gp = (k % cfg.sub_phases) / cfg.sub_phases
            p = leg_phase(gp, leg)
            v, lift = leg_foot_offset(p, cfg)
            ang = solve_phase(gp, cfg)[leg]
            mode = "stance" if p < 0.5 else "swing"
            print(f"    {k:>3} {p:>6.3f} {mode:>6} {v:>+8.2f} {lift:>7.2f} "
                  f"{ang[0]:>8.2f} {ang[1]:>8.2f} {ang[2]:>8.2f}")
        print()
