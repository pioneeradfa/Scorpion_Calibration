"""Tests for walk_log.py (joint-angle log of a walk) and plot_walk.py. No servos: DryRunBus only."""

import csv
import json
import os
import tempfile
import unittest

import plot_walk
import scorpion_gait as sg
import walk_gui as wg
import walk_lidar as wl
import walk_log


class FakeSource:
    stage = "walking"
    cycle = 1
    lidar_nearest_mm = None


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


class RecordTests(unittest.TestCase):

    def test_all_legs_recorded_in_raw_and_degrees(self):
        log = walk_log.WalkLog({"gait": "tripod"}, source=FakeSource())
        joints = {leg: (100.0, 90.0, 80.0) for leg in sg.LEG_ORDER}
        log.record(joints, 200)
        row = log.rows[0]
        self.assertEqual(row["LF_coxa_raw"], 100.0)
        self.assertEqual(row["LF_coxa_deg"], round(100.0 * sg.PHYS_PER_RAW, 2))
        self.assertEqual(row["RR_tibia_deg"], round(80.0 * 1.5, 2))
        self.assertEqual(row["move_ms"], 200)
        self.assertEqual(row["tick"], "")                    # not a gait tick
        self.assertEqual(row["LF_swing"], "")               # swing only in the gait

    def test_swing_and_tick_come_from_set_tick(self):
        log = walk_log.WalkLog({}, source=FakeSource())
        log.set_tick(3, ["LF", "RM"])
        log.record({leg: (90.0, 90.0, 90.0) for leg in sg.LEG_ORDER}, 180)
        log.set_tick(None, None)
        row = log.rows[0]
        self.assertEqual(row["tick"], 3)
        self.assertEqual(row["LF_swing"], 1)
        self.assertEqual(row["RM_swing"], 1)
        self.assertEqual(row["LM_swing"], 0)

    def test_only_commanded_legs_are_recorded(self):
        log = walk_log.WalkLog({}, source=FakeSource())
        joints = {leg: (90.0, 90.0, 90.0) for leg in sg.LEG_ORDER}
        log.set_tick(0, ["RF"])
        log.record(joints, 200, legs=["RF"])
        row = log.rows[0]
        self.assertEqual(row["RF_femur_raw"], 90.0)
        self.assertEqual(row["LF_femur_raw"], "")
        self.assertEqual(row["LF_swing"], "")

    def test_lidar_reading_only_during_gait_ticks(self):
        src = FakeSource()
        src.lidar_nearest_mm = 640
        log = walk_log.WalkLog({}, source=src)
        log.record({}, 1000)
        log.set_tick(1, [])
        log.record({}, 200)
        log.set_tick(None, None)
        self.assertEqual(log.rows[0]["lidar_nearest_mm"], "")
        self.assertEqual(log.rows[1]["lidar_nearest_mm"], 640)

    def test_loggedbus_forwards_and_records(self):
        inner = sg.DryRunBus()
        log = walk_log.WalkLog({}, source=FakeSource())
        bus = walk_log.LoggedBus(inner, log)
        bus.send({leg: (90.0, 90.0, 90.0) for leg in sg.LEG_ORDER}, 200)
        self.assertEqual(len(log.rows), 1)
        self.assertEqual(len(inner.commands), 18)           # all 18 servos still driven


class SaveTests(unittest.TestCase):

    def test_save_writes_csv_and_sidecar_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as d:
            log = walk_log.WalkLog({"gait": "tripod", "steps": 12}, source=FakeSource())
            log.record({leg: (90.0, 90.0, 90.0) for leg in sg.LEG_ORDER}, 200)
            p1 = log.save(d)
            p2 = log.save(d)
            self.assertNotEqual(p1, p2)
            rows = read_csv(p1)
            self.assertEqual(len(rows), 1)
            self.assertEqual(list(rows[0].keys()), walk_log.columns())
            with open(p1[:-4] + ".json") as f:
                side = json.load(f)
            self.assertEqual(side["params"]["gait"], "tripod")
            self.assertEqual(side["servo_ids"]["LF"], {"coxa": 24, "femur": 23, "tibia": 22})
            self.assertEqual(len(walk_log.list_logs(d)), 2)

    def test_safe_path_rejects_other_names(self):
        with tempfile.TemporaryDirectory() as d:
            log = walk_log.WalkLog({}, source=FakeSource())
            log.record({}, 200)
            path = log.save(d)
            name = os.path.basename(path)
            self.assertEqual(walk_log.safe_path(name, d), path)
            self.assertIsNone(walk_log.safe_path("../walk_lidar.py", d))
            self.assertIsNone(walk_log.safe_path("walk_20990101_000000.csv", d))
            self.assertIsNone(walk_log.safe_path("notes.csv", d))


class PlotTests(unittest.TestCase):

    def test_plot_writes_both_svgs_and_summary(self):
        with tempfile.TemporaryDirectory() as d:
            rows = []
            for k in range(12):
                row = {"t_s": 0.2 * k, "stage": "walking", "cycle": 1, "tick": k, "move_ms": 200,
                       "lidar_nearest_mm": ""}
                for leg in sg.LEG_ORDER:
                    for j in plot_walk.JOINTS:
                        row["%s_%s_raw" % (leg, j)] = 90.0 + k
                        row["%s_%s_deg" % (leg, j)] = round((90.0 + k) * 1.5, 2)
                    row["%s_swing" % leg] = 1 if (k + sg.LEG_ORDER.index(leg)) % 2 else 0
                rows.append(row)
            path = os.path.join(d, "sample.csv")
            with open(path, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=walk_log.columns())
                w.writeheader()
                for row in rows:
                    w.writerow({c: row.get(c, "") for c in walk_log.columns()})
            self.assertEqual(plot_walk.main([path]), 0)
            with open(os.path.join(d, "sample_angles.svg")) as f:
                angles = f.read()
            with open(os.path.join(d, "sample_gait.svg")) as f:
                gait = f.read()
            self.assertTrue(angles.startswith("<?xml") or angles.startswith("<svg"))
            self.assertEqual(angles.count("<polyline"), 18)  # 6 legs x 3 joints
            self.assertIn("</svg>", gait)
            self.assertEqual(gait.count('fill="#222"'), 6 * 12 // 2)   # half the cells are swing

    def test_plot_rejects_other_csv(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "x.csv")
            with open(path, "w") as f:
                f.write("a,b\n1,2\n")
            with self.assertRaises(SystemExit):
                plot_walk.main([path])


class WalkerLogTests(unittest.TestCase):
    """A real walk through the walker (dry run): the log is written with the gait's values."""

    def test_walk_writes_a_log_with_tripod_swing_pattern(self):
        src = wl.SimSource(wl.Mount(), box_x=5000.0)
        p = wg.parse_params({"gait": "tripod", "steps": 12, "tick_ms": 200, "cycles": 2,
                             "direction": "forward", "pose_ms": 500})
        with tempfile.TemporaryDirectory() as d:
            w = wl.LidarWalker(src, wl.Settings(), bus_factory=sg.DryRunBus)
            w.log_dir = d
            w._stop.clear()
            w._set(state="running", stage="", cycle=0, tick=0, message="")
            w._run(p, wg.build_gait(p), sg.DryRunBus(), None)
            self.assertEqual(w.state, "done", w.message)
            self.assertIsNotNone(w.last_log)
            rows = read_csv(w.last_log)
            ticks = [r for r in rows if r["tick"] != ""]
            self.assertEqual(len(ticks), 2 * 12)
            for r in ticks:
                self.assertEqual(sum(int(r["%s_swing" % leg]) for leg in sg.LEG_ORDER), 3)
            stages = {r["stage"] for r in rows}
            self.assertIn("initial stand", stages)
            self.assertIn("returning to walking stand", stages)
            r = ticks[0]
            self.assertAlmostEqual(float(r["LF_coxa_deg"]), float(r["LF_coxa_raw"]) * 1.5, delta=0.02)
            t = [float(r["t_s"]) for r in rows]
            self.assertEqual(t, sorted(t))


if __name__ == "__main__":
    unittest.main()
