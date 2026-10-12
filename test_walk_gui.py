#!/usr/bin/env python3
# Tests for walk_gui.py: request checks, the dry-run plan, the robot walk (with a fake bus),
# stop, and the web endpoints. No servo is touched.
#   python3 -m unittest test_walk_gui

import json
import threading
import time
import unittest
import urllib.error
import urllib.request

import scorpion_gait as sg
import walk_gui as gui


class SlowRecorder(sg.DryRunBus):
    """Records commands like DryRunBus, but takes real time per send so stop can land mid-walk."""

    def __init__(self, delay=0.0):
        super().__init__()
        self.delay = delay
        self.sends = 0

    def send(self, joints, move_ms, legs=None):
        super().send(joints, move_ms, legs)
        self.sends += 1
        if self.delay:
            time.sleep(self.delay)


def base_params(**kw):
    data = {"gait": "tripod", "tick_ms": 200, "cycles": 1, "stride": 45, "lift": 30,
            "direction": "forward", "pose": "stand", "pose_ms": 500}
    data.update(kw)
    return gui.parse_params(data)


def wait_idle(walker, timeout=20.0):
    end = time.time() + timeout
    while walker.busy and time.time() < end:
        time.sleep(0.02)
    return walker.status()


class TestParams(unittest.TestCase):
    def test_defaults_follow_gait(self):
        p = gui.parse_params({"gait": "ripple_123456"})
        self.assertEqual(p["steps"], sg.GAIT_DEFAULT_STEPS["ripple_123456"])
        self.assertEqual(p["legs"], sg.LEG_ORDER)

    def test_steps_must_be_multiple_of_six(self):
        with self.assertRaises(gui.ParamError):
            gui.parse_params({"gait": "tripod", "steps": 7})

    def test_steps_and_tick_ranges(self):
        with self.assertRaises(gui.ParamError):
            gui.parse_params({"steps": 126})
        with self.assertRaises(gui.ParamError):
            gui.parse_params({"tick_ms": 50})
        with self.assertRaises(gui.ParamError):
            gui.parse_params({"cycles": 0})

    def test_unknown_values_rejected(self):
        for data in ({"gait": "moonwalk"}, {"direction": "up"}, {"pose": "dance"},
                     {"legs": ["LF", "XX"]}, {"legs": "LF"}, {"tick_ms": "fast"}):
            with self.assertRaises(gui.ParamError, msg=str(data)):
                gui.parse_params(data)

    def test_start_pose_is_not_a_choice(self):
        # the page cannot skip stand_initial: start pose is fixed on the server
        self.assertNotIn("start_pose", gui.parse_params({}))
        self.assertEqual(gui.sg.START_POSE, "stand_initial")


class TestPlan(unittest.TestCase):
    def test_every_gait_passes_the_check(self):
        for g in sg.GAIT_SWING_FRACTION:
            with self.subTest(gait=g):
                with gui.POSE_LOCK:
                    res = gui.plan(gui.parse_params({"gait": g}))
                self.assertTrue(res["ok"], res.get("check"))
                self.assertEqual(len(res["frames"]), sg.GAIT_DEFAULT_STEPS[g])

    def test_tripod_summary(self):
        with gui.POSE_LOCK:
            res = gui.plan(base_params(gait="tripod"))
        s = res["summary"]
        self.assertAlmostEqual(s["cycle_s"], 12 * 0.2)
        self.assertAlmostEqual(s["total_s"], 12 * 0.2)
        self.assertAlmostEqual(s["body_speed_mm_s"], 45 / 1.2)   # stance = half the cycle
        self.assertFalse(s["tick_warning"])
        # tripod: three legs in the air on every tick
        self.assertTrue(all(len(f["air"]) == 3 for f in res["frames"]))

    def test_caterpillar_one_leg_in_air(self):
        with gui.POSE_LOCK:
            res = gui.plan(base_params(gait="caterpillar"))
        self.assertTrue(all(len(f["air"]) == 1 for f in res["frames"]))
        # each leg is in the air for steps/6 ticks; the order is the order of first swing
        order = []
        for f in res["frames"]:
            for n in f["air"]:
                if n not in order:
                    order.append(n)
        self.assertEqual(order, ["LR", "LM", "LF", "RF", "RM", "RR"])

    def test_curve_reports_speed_and_turn_rate(self):
        with gui.POSE_LOCK:
            res = gui.plan(base_params(direction="curve_left", radius=300))
        self.assertTrue(res["ok"])
        self.assertIsNotNone(res["summary"]["turn_rate_deg_s"])
        self.assertGreater(res["summary"]["body_speed_mm_s"], 0)

    def test_straight_has_no_turn_rate(self):
        with gui.POSE_LOCK:
            res = gui.plan(base_params())
        self.assertIsNone(res["summary"]["turn_rate_deg_s"])

    def test_turn_has_no_body_speed(self):
        with gui.POSE_LOCK:
            res = gui.plan(base_params(direction="left"))
        self.assertIsNone(res["summary"]["body_speed_mm_s"])
        self.assertEqual(res["summary"]["direction"], "turn left in place")

    def test_slow_tick_warns(self):
        with gui.POSE_LOCK:
            res = gui.plan(base_params(tick_ms=150))
        self.assertTrue(res["summary"]["tick_warning"])

    def test_frames_have_all_legs(self):
        with gui.POSE_LOCK:
            res = gui.plan(base_params(gait="double_ripple"))
        for f in res["frames"]:
            self.assertEqual(sorted(f["feet"]), sorted(sg.LEG_ORDER))
            self.assertEqual(sorted(f["joints"]), sorted(sg.LEG_ORDER))


class TestWalker(unittest.TestCase):
    def test_refuses_without_yes(self):
        w = gui.Walker(bus_factory=SlowRecorder)
        p = base_params()
        for word in (None, "", "no", "YESS", "1"):
            with self.subTest(word=word):
                with self.assertRaises(gui.ParamError):
                    w.start(p, word)
        self.assertFalse(w.busy)

    def test_accepts_yes_and_y_any_case(self):
        for word in ("YES", "yes", "Yes", "y", "Y"):
            with self.subTest(word=word):
                w = gui.Walker(bus_factory=lambda: SlowRecorder())
                w.start(base_params(), word)
                st = wait_idle(w)
                self.assertEqual(st["state"], "done", st)

    def test_refuses_tick_below_servo_limit(self):
        w = gui.Walker(bus_factory=SlowRecorder)
        with self.assertRaises(gui.ParamError):
            w.start(base_params(tick_ms=150), "YES")

    def test_no_robot_library_is_reported_not_crashed(self):
        def no_bus():
            raise ImportError("ServoControl library not found. Run this on the Raspberry Pi.")
        w = gui.Walker(bus_factory=no_bus)
        with self.assertRaises(gui.ParamError) as cm:
            w.start(base_params(), "YES")
        self.assertIn("Raspberry Pi", str(cm.exception))
        self.assertFalse(w.busy)

    def test_full_sequence_order(self):
        bus = SlowRecorder()
        w = gui.Walker(bus_factory=lambda: bus)
        w.start(base_params(gait="tripod", cycles=1, pose="stand", pose_ms=500), "yes")
        st = wait_idle(w)
        self.assertEqual(st["state"], "done", st)
        # first servo commands: initial stand (stand_initial), then the walking stand
        first = bus.commands[0]
        self.assertEqual(first[0], sg.SERVO_IDS["LF"][0])
        init_pulses = {sid: p for sid, p, _ in bus.commands[:18]}
        expected = {}
        for name in sg.LEG_ORDER:
            for sid, ang in zip(sg.SERVO_IDS[name], sg.POSES["stand_initial"][name]):
                expected[sid] = sg.pulse_us(ang)
        self.assertEqual(init_pulses, expected)
        # 18 servos per posture move, glide, then 12 ticks x 18 servos, then walking stand again
        self.assertEqual(len(bus.commands), 18 * (1 + 1 + 1 + 12 + 1))
        self.assertIn("walk finished", st["message"])

    def test_stop_ends_walk_and_returns_to_stand(self):
        bus = SlowRecorder(delay=0.02)
        w = gui.Walker(bus_factory=lambda: bus)
        w.start(base_params(gait="tripod", cycles=50, tick_ms=180, pose_ms=500), "yes")
        # wait until walking has started
        end = time.time() + 15
        while w.status()["stage"] != "walking" and time.time() < end:
            time.sleep(0.02)
        time.sleep(0.2)
        w.stop()
        st = wait_idle(w)
        self.assertEqual(st["state"], "stopped", st)
        self.assertIn("stopped by user", st["message"])
        # far fewer ticks than 50 cycles x 12 steps were sent
        self.assertLess(bus.sends, 50 * 12)
        # the last commands are the walking-stand move (18 servos, pose_ms)
        last = bus.commands[-18:]
        self.assertEqual({m for _, _, m in last}, {500})
        self.assertEqual({p for _, p, _ in last},
                         {sg.pulse_us(a) for n in sg.LEG_ORDER for a in sg.POSES["stand"][n]})

    def test_one_walk_at_a_time(self):
        w = gui.Walker(bus_factory=lambda: SlowRecorder(delay=0.02))
        w.start(base_params(cycles=50, tick_ms=180, pose_ms=500), "yes")
        with self.assertRaises(gui.ParamError):
            w.start(base_params(), "yes")
        w.stop()
        wait_idle(w)

    def test_leg_selection_limits_commands(self):
        bus = SlowRecorder()
        w = gui.Walker(bus_factory=lambda: bus)
        w.start(base_params(legs=["RF"]), "yes")
        wait_idle(w)
        servos = {sid for sid, _, _ in bus.commands}
        self.assertEqual(servos, set(sg.SERVO_IDS["RF"]))


class TestHttp(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = gui.make_server("127.0.0.1", 0)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def call(self, method, path, body=None):
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request("http://127.0.0.1:%d%s" % (self.port, path), data=data, method=method,
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read()

    def test_page_and_config(self):
        code, body = self.call("GET", "/")
        self.assertEqual(code, 200)
        self.assertIn(b"Scorpion walk", body)
        code, body = self.call("GET", "/api/config")
        cfg = json.loads(body)
        self.assertEqual({g["id"] for g in cfg["gaits"]}, set(sg.GAIT_SWING_FRACTION))
        self.assertEqual(cfg["start_pose"], "stand_initial")

    def test_plan_ok_and_errors(self):
        code, body = self.call("POST", "/api/plan", {"gait": "caterpillar"})
        self.assertEqual(code, 200)
        res = json.loads(body)
        self.assertTrue(res["ok"])
        self.assertEqual(len(res["frames"]), sg.GAIT_DEFAULT_STEPS["caterpillar"])
        code, body = self.call("POST", "/api/plan", {"steps": 7})
        self.assertEqual(code, 400)
        self.assertIn("multiple of 6", json.loads(body)["error"])
        code, _ = self.call("POST", "/api/plan", ["not", "an", "object"])
        self.assertEqual(code, 400)

    def test_walk_needs_yes_over_http(self):
        code, body = self.call("POST", "/api/walk", {"gait": "tripod"})
        self.assertEqual(code, 400)
        self.assertIn("YES", json.loads(body)["error"])

    def test_stop_and_status_when_idle(self):
        code, body = self.call("POST", "/api/stop", {})
        self.assertEqual(code, 200)
        self.assertFalse(json.loads(body)["busy"])
        code, body = self.call("GET", "/api/status")
        self.assertEqual(json.loads(body)["state"], "idle")
        code, _ = self.call("GET", "/nope")
        self.assertEqual(code, 404)


if __name__ == "__main__":
    unittest.main()
