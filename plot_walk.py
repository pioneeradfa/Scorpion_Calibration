#!/usr/bin/env python3
#=================================================================================
# Plots a walk log (CSV written by walk_log.py) without any extra packages.
#
#   python3 plot_walk.py walk_logs/walk_20261020_103000.csv          # SVG files next to the CSV
#   python3 plot_walk.py walk_logs/walk_....csv --out plots/         # into another folder
#
# Writes two SVG files (open them in any browser):
#   <name>_angles.svg  every leg: coxa, femur, tibia angle in degrees against time. Shaded = leg in the air.
#   <name>_gait.svg    walking pattern: one row per leg, one column per gait tick, black = leg in the air.
# and prints the angle range of every joint over the walking ticks.
#=================================================================================

import argparse
import csv
import math
import os
import sys
from xml.sax.saxutils import escape

import scorpion_gait as sg

JOINTS = ("coxa", "femur", "tibia")
COLORS = {"coxa": "#2f6fdb", "femur": "#c62828", "tibia": "#2e7d32"}


def read_log(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit("%s has no rows" % path)
    need = ["t_s"] + ["%s_%s_deg" % (leg, j) for leg in sg.LEG_ORDER for j in JOINTS]
    missing = [c for c in need if c not in rows[0]]
    if missing:
        raise SystemExit("%s is not a walk log (missing %s)" % (path, ", ".join(missing[:3])))
    return rows


def num(v):
    return None if v in ("", None) else float(v)


def _svg(width, height, body):
    return ('<svg xmlns="http://www.w3.org/2000/svg" width="%d" height="%d" viewBox="0 0 %d %d" '
            'font-family="sans-serif">\n<rect width="100%%" height="100%%" fill="white"/>\n%s</svg>\n'
            % (width, height, width, height, body))


def _text(x, y, s, size=12, anchor="start", fill="#333"):
    return '<text x="%.1f" y="%.1f" font-size="%d" text-anchor="%s" fill="%s">%s</text>\n' % (
        x, y, size, anchor, fill, escape(str(s)))


def _nice_ticks(lo, hi, n=5):
    if hi <= lo:
        hi = lo + 1.0
    step = 10 ** math.floor(math.log10((hi - lo) / n))
    for m in (1, 2, 5, 10):
        if (hi - lo) / (step * m) <= n:
            step *= m
            break
    start = math.floor(lo / step) * step
    ticks = []
    v = start
    while v <= hi + 1e-9:
        ticks.append(round(v, 6))
        v += step
    return ticks


def angles_svg(rows):
    """Six panels (one per leg), 3 columns x 2 rows: joint angles in degrees against time."""
    pw, ph = 480, 300
    ml, mr, mt, mb = 60, 15, 40, 40
    cols, rws = 3, 2
    width, height = cols * pw + 20, rws * ph + 60
    t = [num(r["t_s"]) for r in rows]
    t0, t1 = min(t), max(t)
    if t1 <= t0:
        t1 = t0 + 1.0
    body = _text(width / 2, 24, "Joint angles per leg (degrees, physical 0..270)", 16, "middle")
    for i, leg in enumerate(sg.LEG_ORDER):
        ox = 10 + (i % cols) * pw
        oy = 40 + (i // cols) * ph
        x0, x1 = ox + ml, ox + pw - mr
        y0, y1 = oy + mt, oy + ph - mb
        vals = [num(r["%s_%s_deg" % (leg, j)]) for r in rows for j in JOINTS]
        vals = [v for v in vals if v is not None]
        vlo, vhi = (min(vals), max(vals)) if vals else (0.0, 270.0)
        pad = max(5.0, 0.05 * (vhi - vlo))
        vlo, vhi = vlo - pad, vhi + pad

        def X(tv):
            return x0 + (tv - t0) / (t1 - t0) * (x1 - x0)

        def Y(v):
            return y1 - (v - vlo) / (vhi - vlo) * (y1 - y0)

        body += '<rect x="%d" y="%d" width="%d" height="%d" fill="none" stroke="#bbb"/>\n' % (x0, y0, x1 - x0, y1 - y0)
        # shade the swing periods of this leg
        sw = [num(r.get("%s_swing" % leg, "")) for r in rows]
        for k in range(len(rows) - 1):
            if sw[k] == 1:
                xa, xb = X(t[k]), X(t[k + 1])
                body += ('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" fill="#f2a900" opacity="0.18"/>\n'
                         % (xa, y0, max(0.5, xb - xa), y1 - y0))
        for v in _nice_ticks(vlo, vhi):
            if vlo <= v <= vhi:
                body += ('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="#eee"/>\n'
                         % (x0, Y(v), x1, Y(v)))
                body += _text(x0 - 6, Y(v) + 4, "%g" % v, 11, "end")
        for tv in _nice_ticks(t0, t1):
            if t0 <= tv <= t1:
                body += _text(X(tv), y1 + 16, "%g" % tv, 11, "middle")
        body += _text(x0, y0 - 10, "Leg %s (servos %s)" % (leg, "/".join(str(s) for s in sg.SERVO_IDS[leg])), 13, "start", "#111")
        body += _text((x0 + x1) / 2, y1 + 34, "time (s)", 11, "middle")
        for j in JOINTS:
            pts = []
            for k, r in enumerate(rows):
                v = num(r["%s_%s_deg" % (leg, j)])
                if v is not None:
                    pts.append("%.1f,%.1f" % (X(t[k]), Y(v)))
            if len(pts) >= 2:
                body += '<polyline points="%s" fill="none" stroke="%s" stroke-width="1.4"/>\n' % (" ".join(pts), COLORS[j])
        lx = x0 + 6
        for j in JOINTS:
            body += '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" stroke-width="3"/>\n' % (lx, y0 + 10, lx + 16, y0 + 10, COLORS[j])
            body += _text(lx + 20, y0 + 14, j, 11)
            lx += 80
    body += _text(width / 2, height - 8, "Shaded = leg in the air (swing). Blank parts = leg not commanded.", 11, "middle", "#666")
    return _svg(width, height, body)


def gait_svg(rows):
    """Walking pattern: one row per leg, one column per walking tick (blank = not a gait tick)."""
    ticks = [r for r in rows if r.get("tick", "") != ""]
    if not ticks:
        return None
    cell, label_w, top = 14, 80, 50
    width = label_w + cell * len(ticks) + 30
    height = top + cell * len(sg.LEG_ORDER) + 60
    body = _text(width / 2, 24, "Walking pattern: black = leg in the air (swing), white = on the ground (stance)", 14, "middle")
    for i, leg in enumerate(sg.LEG_ORDER):
        y = top + i * cell
        body += _text(label_w - 8, y + cell - 3, leg, 12, "end")
        for k, r in enumerate(ticks):
            x = label_w + k * cell
            s = r.get("%s_swing" % leg, "")
            fill = "#222" if s == "1" else "#fff"
            body += '<rect x="%d" y="%d" width="%d" height="%d" fill="%s" stroke="#ddd"/>\n' % (x, y, cell, cell, fill)
    # mark the start of each cycle (tick index 0)
    for k, r in enumerate(ticks):
        if r.get("tick") == "0":
            x = label_w + k * cell
            body += ('<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="#c62828" stroke-width="2"/>\n'
                     % (x, top - 6, x, top + cell * len(sg.LEG_ORDER)))
    body += _text(label_w, height - 26, "Columns = gait ticks in order; red line = start of a cycle (tick 0).", 11, "start", "#666")
    return _svg(width, height, body)


def summary(rows):
    walking = [r for r in rows if r.get("tick", "") != ""]
    use = walking or rows
    out = ["angle summary over %s (degrees, physical)" % ("walking ticks" if walking else "all commands"),
           "%-5s %-6s %8s %8s %8s %14s" % ("leg", "joint", "min", "max", "range", "max step/tick")]
    for leg in sg.LEG_ORDER:
        for j in JOINTS:
            key = "%s_%s_deg" % (leg, j)
            vals = [num(r[key]) for r in use if num(r[key]) is not None]
            if not vals:
                out.append("%-5s %-6s %s" % (leg, j, "not commanded"))
                continue
            step = 0.0
            prev = None
            for r in use:
                v = num(r[key])
                if v is None:
                    prev = None
                    continue
                if prev is not None:
                    step = max(step, abs(v - prev))
                prev = v
            out.append("%-5s %-6s %8.1f %8.1f %8.1f %14.1f" % (leg, j, min(vals), max(vals), max(vals) - min(vals), step))
    return "\n".join(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Plot a walk log (joint angles and walking pattern) as SVG.")
    ap.add_argument("csv", help="walk log CSV written by walk_log.py")
    ap.add_argument("--out", help="folder for the SVG files (default: next to the CSV)")
    args = ap.parse_args(argv)
    rows = read_log(args.csv)
    out_dir = args.out or os.path.dirname(os.path.abspath(args.csv))
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.csv))[0]
    angles = os.path.join(out_dir, stem + "_angles.svg")
    with open(angles, "w") as f:
        f.write(angles_svg(rows))
    print("wrote", angles)
    gait = gait_svg(rows)
    if gait:
        gp = os.path.join(out_dir, stem + "_gait.svg")
        with open(gp, "w") as f:
            f.write(gait)
        print("wrote", gp)
    else:
        print("no walking ticks in this log: no gait diagram")
    print()
    print(summary(rows))
    return 0


if __name__ == "__main__":
    sys.exit(main())
