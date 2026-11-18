#=================================================================================
# Records the servo angles of every walk, so you can plot them afterwards.
#
# Every command sent to the servos is recorded (initial stand, walking stand, glide, every
# gait tick, turns, return to the stand), one row per command, with:
#   t_s                 seconds since the walk started
#   stage, cycle, tick  what the walk was doing (tick is blank outside the gait)
#   move_ms             move time sent with the command
#   <LEG>_<joint>_raw   servo angle in the calibrated raw units (0..180, as sent to the servo)
#   <LEG>_<joint>_deg   the same angle in physical degrees (raw x 1.5 = 0..270 deg, RDS3218)
#   <LEG>_swing         1 = leg in the air (swing), 0 = on the ground (stance), blank outside the gait
#   lidar_nearest_mm    nearest obstacle in the way at that moment (blank if none / not walking)
# Legs that are not commanded (for example a single-leg test) are left blank.
#
# One CSV and one JSON sidecar (parameters, servo IDs, units) are written per walk to walk_logs/.
# Plot them with:  python3 plot_walk.py walk_logs/walk_<time>.csv
#=================================================================================

import csv
import datetime
import json
import os
import re
import time

import scorpion_gait as sg

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "walk_logs")
JOINTS = ("coxa", "femur", "tibia")
FILE_RE = re.compile(r"^walk_\d{8}_\d{6}(_\d+)?\.csv$")


def columns():
    cols = ["t_s", "stage", "cycle", "tick", "move_ms", "lidar_nearest_mm"]
    for leg in sg.LEG_ORDER:
        for j in JOINTS:
            cols += ["%s_%s_raw" % (leg, j), "%s_%s_deg" % (leg, j)]
        cols.append("%s_swing" % leg)
    cols += ["claw_left_raw", "claw_left_deg", "claw_right_raw", "claw_right_deg"]
    return cols


class WalkLog:
    """Keeps the rows of one walk. `source` is the walker (read for stage, cycle and the LiDAR reading)."""

    def __init__(self, params, source=None):
        self.params = dict(params)
        self.source = source
        self.started = datetime.datetime.now()
        self.t0 = time.monotonic()
        self.rows = []
        self.tick = None
        self.swing = None
        self.path = None

    def set_tick(self, tick, swing):
        """Called by the walker just before a gait tick is sent (tick, swing legs) and cleared after."""
        self.tick = tick
        self.swing = swing

    def record(self, joints, move_ms, legs=None):
        src = self.source
        row = {
            "t_s": round(time.monotonic() - self.t0, 3),
            "stage": getattr(src, "stage", "") or "",
            "cycle": getattr(src, "cycle", "") or "",
            "tick": "" if self.tick is None else self.tick,
            "move_ms": move_ms,
            "lidar_nearest_mm": "",
        }
        nearest = getattr(src, "lidar_nearest_mm", None)
        if self.tick is not None and nearest is not None:
            row["lidar_nearest_mm"] = nearest
        for leg in sg.LEG_ORDER:
            commanded = joints is not None and leg in joints and (not legs or leg in legs)
            for k, j in enumerate(JOINTS):
                if commanded:
                    raw = float(joints[leg][k])
                    row["%s_%s_raw" % (leg, j)] = round(raw, 2)
                    row["%s_%s_deg" % (leg, j)] = round(sg.raw_to_phys(raw), 2)
                else:
                    row["%s_%s_raw" % (leg, j)] = ""
                    row["%s_%s_deg" % (leg, j)] = ""
            if self.swing is not None and commanded:
                row["%s_swing" % leg] = 1 if leg in self.swing else 0
            else:
                row["%s_swing" % leg] = ""
        self.rows.append(row)

    def record_claws(self, angles, move_ms):
        """A claw command: one row, legs blank. angles: {side: raw}."""
        src = self.source
        row = {
            "t_s": round(time.monotonic() - self.t0, 3),
            "stage": getattr(src, "stage", "") or "",
            "cycle": getattr(src, "cycle", "") or "",
            "tick": "",
            "move_ms": move_ms,
            "lidar_nearest_mm": "",
        }
        for side in ("left", "right"):
            if side in angles:
                row["claw_%s_raw" % side] = round(float(angles[side]), 2)
                row["claw_%s_deg" % side] = round(sg.raw_to_phys(float(angles[side])), 2)
        self.rows.append(row)

    def sidecar(self):
        return {
            "started": self.started.isoformat(timespec="seconds"),
            "params": self.params,
            "units": {"raw": "servo raw angle 0..180 (as sent to the servo; calibrated zeros)",
                      "deg": "physical degrees = raw x %.2f (RDS3218 270 deg travel)" % sg.PHYS_PER_RAW,
                      "t_s": "seconds since the walk started", "swing": "1 = leg in the air, 0 = on the ground"},
            "servo_ids": {leg: dict(zip(JOINTS, sg.SERVO_IDS[leg])) for leg in sg.LEG_ORDER},
            "rows": len(self.rows),
        }

    def save(self, directory=None):
        """Writes walk_<time>.csv and .json into `directory` (default walk_logs/). Returns the CSV path."""
        directory = directory or LOG_DIR
        os.makedirs(directory, exist_ok=True)
        stem = "walk_" + self.started.strftime("%Y%m%d_%H%M%S")
        name, n = stem, 1
        while os.path.exists(os.path.join(directory, name + ".csv")):
            n += 1
            name = "%s_%d" % (stem, n)
        csv_path = os.path.join(directory, name + ".csv")
        cols = columns()
        with open(csv_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            for row in self.rows:
                w.writerow({c: row.get(c, "") for c in cols})
        with open(os.path.join(directory, name + ".json"), "w") as f:
            json.dump(self.sidecar(), f, indent=2)
        self.path = csv_path
        return csv_path


class LoggedBus:
    """Wraps a servo bus: every send is recorded, then passed on unchanged."""

    def __init__(self, inner, log):
        self.inner = inner
        self.log = log

    def send(self, joints, move_ms, legs=None):
        self.log.record(joints, move_ms, legs)
        return self.inner.send(joints, move_ms, legs)

    def send_claws(self, angles, move_ms):
        self.log.record_claws(angles, move_ms)
        return self.inner.send_claws(angles, move_ms)


def list_logs(directory=None):
    """Newest first: [{name, bytes, time}] for the walk CSVs in the log folder."""
    directory = directory or LOG_DIR
    if not os.path.isdir(directory):
        return []
    names = [n for n in os.listdir(directory) if FILE_RE.match(n)]
    out = []
    for n in sorted(names, reverse=True):
        p = os.path.join(directory, n)
        out.append({"name": n, "bytes": os.path.getsize(p),
                    "time": datetime.datetime.fromtimestamp(os.path.getmtime(p)).isoformat(timespec="seconds")})
    return out


def safe_path(name, directory=None):
    """Full path of a log file, only if the name is a walk CSV inside the log folder."""
    directory = directory or LOG_DIR
    if not FILE_RE.match(name or ""):
        return None
    p = os.path.join(directory, name)
    return p if os.path.isfile(p) else None
