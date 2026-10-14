#!/usr/bin/env python3
#=================================================================================
# Simulated check of ICC walking: does the body really follow the curve we asked for?
#
# No servos are touched. The gait's foot targets are taken as the truth and the body pose is
# worked out from them. While a foot is planted it does not move in the world, so each tick the
# body pose is fitted from the stance feet (a 2-D rigid fit, the same Kabsch method used in
# robot tracking). If the ICC model is right:
#   - the stance feet fit a rigid motion almost exactly (small residual),
#   - the body centre runs on a circle of radius |ICC| (curve), or straight (ICC far away),
#   - the turn rate matches body_motion() in icc_turn.py.
#
# Usage:
#   python3 icc_sim.py                          # table for several radii, both sides, plus straight and spin
#   python3 icc_sim.py --curve left --radius 300 --svg path_left_300.svg
#=================================================================================

import argparse
import math
import sys

import icc_turn as it
import scorpion_gait as sg


#---------------------------------------------------------------------------------
# 2-D rigid transforms as (theta, tx, ty): p -> R(theta) p + t

def compose(a, b):
    """a after b: p -> a(b(p))."""
    ta, ax, ay = a
    tb, bx, by = b
    c, s = math.cos(ta), math.sin(ta)
    return (ta + tb, ax + c * bx - s * by, ay + s * bx + c * by)


def inverse(a):
    ta, ax, ay = a
    c, s = math.cos(-ta), math.sin(-ta)
    return (-ta, -(c * ax - s * ay), -(s * ax + c * ay))


def apply(a, p):
    ta, ax, ay = a
    c, s = math.cos(ta), math.sin(ta)
    return (c * p[0] - s * p[1] + ax, s * p[0] + c * p[1] + ay)


def fit_rigid(src, dst):
    """Best rigid 2-D transform mapping src points onto dst points (least squares). Returns (transform, rms)."""
    n = len(src)
    sx = sum(p[0] for p in src) / n
    sy = sum(p[1] for p in src) / n
    dx = sum(p[0] for p in dst) / n
    dy = sum(p[1] for p in dst) / n
    sxx = sxy = syx = syy = 0.0
    for (ax, ay), (bx, by) in zip(src, dst):
        ax, ay, bx, by = ax - sx, ay - sy, bx - dx, by - dy
        sxx += ax * bx
        sxy += ax * by
        syx += ay * bx
        syy += ay * by
    theta = math.atan2(sxy - syx, sxx + syy)
    c, s = math.cos(theta), math.sin(theta)
    tx = dx - (c * sx - s * sy)
    ty = dy - (s * sx + c * sy)
    t = (theta, tx, ty)
    err = [math.hypot(apply(t, p)[0] - q[0], apply(t, p)[1] - q[1]) for p, q in zip(src, dst)]
    rms = math.sqrt(sum(e * e for e in err) / len(err)) if err else 0.0
    return t, rms


#---------------------------------------------------------------------------------
# Simulation

def simulate(gait, cycles=2):
    """Body poses (world <- body) at every tick of the run, plus the worst fit residual (mm).
    Starts with the body at the world origin, heading +X."""
    N = gait.steps
    pose = (0.0, 0.0, 0.0)
    poses = [pose]
    worst = 0.0
    feet = {t: gait.foot_targets(t) for t in range(N * cycles + 1)}
    for t in range(N * cycles):
        a, b = feet[t], feet[t + 1]
        # legs planted during this step: planted at the start of it. A leg that lifts at t+1 still
        # moves on the ground from t to t+1 (its foot position is continuous at the lift).
        stance = [n for n in sg.LEG_ORDER if not gait._is_swing(n, t % N)]
        if len(stance) < 2:
            raise ValueError("need at least 2 planted feet to fit the body")
        src = [(a[n][0], a[n][1]) for n in stance]
        dst = [(b[n][0], b[n][1]) for n in stance]
        # planted feet are fixed in the world: pose_t(src) == pose_{t+1}(dst)  ->  pose_{t+1} = pose_t o H^-1
        H, rms = fit_rigid(src, dst)
        worst = max(worst, rms)
        pose = compose(pose, inverse(H))
        poses.append(pose)
    return poses, worst


def path_stats(gait, cycles=2):
    """Measured curvature and heading change of the body centre, compared with the model."""
    poses, worst = simulate(gait, cycles)
    centre = [apply(p, (0.0, 0.0)) for p in poses]
    length = sum(math.hypot(centre[i + 1][0] - centre[i][0], centre[i + 1][1] - centre[i][1])
                 for i in range(len(centre) - 1))
    heading = poses[-1][0]
    travel = math.hypot(centre[-1][0] - centre[0][0], centre[-1][1] - centre[0][1])
    end = centre[-1]
    # radius of the constant-curvature path: arc length / heading change
    radius = length / abs(heading) if abs(heading) > 1e-9 else float("inf")
    return {
        "poses": poses, "centre": centre, "fit_rms_mm": worst,
        "end_x": end[0], "end_y": end[1], "heading_deg": math.degrees(heading),
        "path_len_mm": length, "travel_mm": travel,
        "measured_radius_mm": radius,
    }


def predicted(gait, cycles, cycle_s):
    return it.body_motion(gait, cycle_s)   # (speed mm/s, turn rate deg/s)


#---------------------------------------------------------------------------------
# Output

def write_svg(path, poses, centre, icc_world, title):
    xs = [c[0] for c in centre] + [icc_world[0]]
    ys = [c[1] for c in centre] + [icc_world[1]]
    pad = 60
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    w, h = 720, 480
    scale = min(w / (x1 - x0), h / (y1 - y0))

    def sx(x):
        return (x - x0) * scale

    def sy(y):
        return h - (y - y0) * scale

    pts = " ".join("%.1f,%.1f" % (sx(x), sy(y)) for x, y in centre)
    # body outline at a few ticks, to show heading
    marks = []
    for i in range(0, len(poses), max(1, len(poses) // 8)):
        p = poses[i]
        nose = apply(p, (80.0, 0.0))
        marks.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#e8793a" stroke-width="2"/>'
                     % (sx(apply(p, (0, 0))[0]), sy(apply(p, (0, 0))[1]), sx(nose[0]), sy(nose[1])))
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d">'
        '<rect width="100%%" height="100%%" fill="#fbfcfd"/>'
        '<text x="12" y="22" font-family="sans-serif" font-size="14">%s</text>'
        '<text x="12" y="40" font-family="sans-serif" font-size="11" fill="#6b7385">'
        'body centre path (blue); orange = body heading at some ticks; ICC (red)</text>'
        '<polyline points="%s" fill="none" stroke="#2f6fdb" stroke-width="2"/>'
        '%s'
        '<circle cx="%.1f" cy="%.1f" r="5" fill="#c62828"/>'
        '<circle cx="%.1f" cy="%.1f" r="4" fill="#1d2330"/>'
        '</svg>'
    ) % (w, h, w, h, title, pts, "".join(marks), sx(icc_world[0]), sy(icc_world[1]),
         sx(centre[0][0]), sy(centre[0][1]))
    with open(path, "w") as fh:
        fh.write(svg)


def run_case(gait_type, side, radius, cycles, stride, lift, backward, straight=False, spin=False):
    sg.set_active_pose(sg.DEFAULT_POSE)
    if spin:
        gait = sg.Gait(gait_type, steps=sg.GAIT_DEFAULT_STEPS[gait_type], stride=stride, lift=lift)
        gait.set_turn_in_place("CCW")
    elif straight:
        gait = sg.Gait(gait_type, steps=sg.GAIT_DEFAULT_STEPS[gait_type], stride=stride, lift=lift)
        gait.set_straight(not backward)
    else:
        gait = it.make_curve_gait(gait_type, sg.GAIT_DEFAULT_STEPS[gait_type], stride, lift, radius, side, backward)
    stats = path_stats(gait, cycles)
    cycle_s = gait.steps * 0.2
    speed, rate = predicted(gait, cycles, cycle_s)
    return gait, stats, speed, rate


def main(argv=None):
    parser = argparse.ArgumentParser(description="Simulated ICC walking check (no servos).")
    parser.add_argument("--gait", default="tripod", choices=sorted(sg.GAIT_SWING_FRACTION))
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--stride", type=float, default=sg.DEFAULT_STRIDE)
    parser.add_argument("--lift", type=float, default=sg.DEFAULT_LIFT)
    parser.add_argument("--curve", choices=("left", "right"))
    parser.add_argument("--radius", type=float)
    parser.add_argument("--backward", action="store_true")
    parser.add_argument("--svg", help="write the body path to this SVG file (one case only)")
    args = parser.parse_args(argv)

    if args.curve:
        if args.radius is None:
            parser.error("--curve needs --radius")
        cases = [("curve", args.curve, args.radius)]
    else:
        cases = ([("curve", "left", r) for r in (300, 400, 600)]
                 + [("curve", "right", r) for r in (300, 400, 600)]
                 + [("straight", "", 0), ("spin", "left", 0), ("spin", "right", 0)])

    print("Gait %s, %d cycles, stride %.0f mm, %s" % (args.gait, args.cycles, args.stride,
                                                       "backward" if args.backward else "forward"))
    print("%-22s %9s %9s %10s %10s %10s %10s %9s" % (
        "case", "fit rms", "heading", "radius", "model R", "turn/s", "model/s", "travel"))
    code = 0
    for kind, side, radius in cases:
        gait, st, speed, rate = run_case(args.gait, side, radius, args.cycles, args.stride, args.lift,
                                         args.backward, straight=(kind == "straight"),
                                         spin=(kind == "spin"))
        model_r = math.hypot(*gait.icc)
        total_s = args.cycles * gait.steps * 0.2
        turn_per_s = st["heading_deg"] / total_s           # measured turn rate, deg/s
        if kind == "curve":
            name = "curve %s %4.0f mm" % (side, radius)
        elif kind == "spin":
            name = "spin %s" % side
        else:
            name = "straight"
        print("%-22s %7.3f mm %7.1f d %8s %8s %8.1f d/s %6.1f d/s %6.0f mm"
              % (name, st["fit_rms_mm"], st["heading_deg"],
                 "%.0f mm" % st["measured_radius_mm"] if st["measured_radius_mm"] < 1e6 else "straight",
                 "%.0f mm" % model_r if model_r < 1e6 else "straight",
                 turn_per_s, rate, st["travel_mm"]))
        if st["fit_rms_mm"] > 0.5:
            code = 1
        if args.svg:
            write_svg(args.svg, st["poses"], st["centre"], gait.icc, "%s %s" % (args.gait, name))
            print("wrote %s" % args.svg)
    return code


if __name__ == "__main__":
    sys.exit(main())
