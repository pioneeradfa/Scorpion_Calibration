"""Tests for walk_lidar.py: LiDAR packets, obstacle zones, the walk gate and turn-away, and the page routes.
No servos are used (DryRunBus); no serial port is opened (LidarSource fed by hand or SimSource)."""

import json
import math
import threading
import time
import unittest
import urllib.error
import urllib.request

import icc_turn as it
import scorpion_gait as sg
import walk_gui as wg
import walk_lidar as wl


def clear_readings(dist=3000, inten=200):
    return [(dist, inten)] * wl.POINTS_PER_PACKET


def fresh_source(readings=None, start=0.0, end=3.3):
    """A real (non-simulated) LidarSource that has just received one packet."""
    src = wl.LidarSource(wl.Mount())
    src.feed(wl.encode_packet(start, end, readings or clear_readings()), now=time.monotonic())
    return src


class ScriptedSource(wl.SimSource):
    """Scan points come from a script: blocked (one point at `block_point`) for body_points calls in
    [block_from, block_until). health() fails from call `stale_from` on. Counts calls, one per tick gate."""

    def __init__(self, block_from=None, block_until=None, block_point=(400.0, 60.0), stale_from=None):
        super().__init__(wl.Mount(), box_x=5000.0)
        self.calls = 0
        self.block_from = block_from
        self.block_until = block_until
        self.block_point = block_point
        self.stale_from = stale_from

    def health(self, now=None):
        if self.stale_from is not None and self.calls >= self.stale_from:
            return {"ok": False, "simulated": False, "packets": 1, "crc_errors": 0, "age_ms": 900,
                    "rate_hz": 0, "error": None}
        return {"ok": True, "simulated": False, "packets": 1, "crc_errors": 0, "age_ms": 0,
                "rate_hz": 10, "error": None}

    def body_points(self, now=None):
        self.calls += 1
        n = self.calls
        if self.block_from is not None and n >= self.block_from and (self.block_until is None or n < self.block_until):
            return [self.block_point]
        return []


def params(**kw):
    base = {"gait": "tripod", "steps": 12, "tick_ms": 200, "cycles": 1, "stride": 45.0, "lift": 30.0,
            "direction": "forward", "pose": sg.DEFAULT_POSE, "pose_ms": 500, "radius": 300.0,
            "reverse": False, "legs": list(sg.LEG_ORDER)}
    base.update(kw)
    return wg.parse_params(base)


class PacketTests(unittest.TestCase):

    def test_crc_matches_bitwise_definition(self):
        def bitwise(data):
            c = 0
            for b in data:
                c ^= b
                for _ in range(8):
                    c = ((c << 1) ^ 0x4D) & 0xFF if c & 0x80 else (c << 1) & 0xFF
            return c
        for seed in range(50):
            data = bytes((seed * 37 + i * 11) % 256 for i in range(45))
            self.assertEqual(wl.crc8(data), bitwise(data))

    def test_round_trip_angles_and_distances(self):
        readings = [(100 + 50 * i, 10 + i) for i in range(12)]
        pkt = wl.encode_packet(90.0, 95.5, readings)
        self.assertEqual(len(pkt), wl.PACKET_LEN)
        out = wl.decode_packet(pkt)
        self.assertEqual([d for _, d, _ in out], [r[0] for r in readings])
        self.assertEqual([i for _, _, i in out], [r[1] for r in readings])
        self.assertAlmostEqual(out[0][0], 90.0)
        self.assertAlmostEqual(out[-1][0], 95.5)

    def test_angles_wrap_past_360(self):
        pkt = wl.encode_packet(350.0, 10.0, clear_readings())
        angles = [a for a, _, _ in wl.decode_packet(pkt)]
        self.assertAlmostEqual(angles[0], 350.0)
        self.assertAlmostEqual(angles[-1], 10.0)
        self.assertTrue(all(0 <= a < 360 for a in angles))

    def test_parser_handles_split_chunks_and_garbage(self):
        p = wl.Ld19Parser()
        pkt = wl.encode_packet(0.0, 3.3, clear_readings(1234))
        stream = b"\x00\x12junk" + pkt + pkt
        points = []
        for i in range(0, len(stream), 7):          # arrives in small chunks
            points += p.feed(stream[i:i + 7])
        self.assertEqual(p.packets, 2)
        self.assertEqual(p.crc_errors, 0)
        self.assertEqual(len(points), 24)

    def test_parser_counts_bad_crc_and_recovers(self):
        p = wl.Ld19Parser()
        bad = bytearray(wl.encode_packet(0.0, 3.3, clear_readings()))
        bad[10] ^= 0xFF                              # corrupt a distance byte
        good = wl.encode_packet(0.0, 3.3, clear_readings(777))
        points = p.feed(bytes(bad) + good)
        self.assertEqual(p.crc_errors, 1)
        self.assertEqual(p.packets, 1)
        self.assertTrue(all(d == 777 for _, d, _ in points))

    def test_mount_transform(self):
        m = wl.Mount(x=100, y=0, yaw_deg=0, direction=1)
        x, y = wl.to_body(m, 0.0, 500)
        self.assertAlmostEqual(x, 600)
        self.assertAlmostEqual(y, 0, places=6)
        x, y = wl.to_body(wl.Mount(direction=1), 90.0, 300)     # 90 deg = left
        self.assertAlmostEqual(x, 0, places=6)
        self.assertAlmostEqual(y, 300)
        x, y = wl.to_body(wl.Mount(direction=-1), 90.0, 300)    # reversed mount: 90 deg = right
        self.assertAlmostEqual(y, -300)

    def test_self_points_are_dropped(self):
        src = wl.LidarSource(wl.Mount())
        readings = [(100, 0)] + [(3000, 200)] * 11                # first point is the robot's own mast
        src.feed(wl.encode_packet(0.0, 3.3, readings), now=time.monotonic())
        pts = src.body_points()
        self.assertEqual(len(pts), 11)
        self.assertTrue(all(math.hypot(x, y) >= wl.MIN_RANGE_MM for x, y in pts))

    def test_stale_and_scan_window(self):
        src = wl.LidarSource(wl.Mount())
        t0 = 100.0
        src.feed(wl.encode_packet(0.0, 3.3, clear_readings()), now=t0)
        self.assertTrue(src.health(now=t0 + 0.1)["ok"])
        self.assertFalse(src.health(now=t0 + wl.STALE_S + 0.01)["ok"])
        self.assertEqual(src.body_points(now=t0 + wl.SCAN_WINDOW_S + 0.01), [])


class ZoneTests(unittest.TestCase):

    def test_travel_for_each_gait(self):
        g = sg.Gait("tripod", steps=12)
        g.set_straight(True)
        self.assertEqual(wl.travel_of(g), ("straight", True))
        g.set_straight(False)
        self.assertEqual(wl.travel_of(g), ("straight", False))
        g.set_turn_in_place("CCW")
        self.assertEqual(wl.travel_of(g)[0], "spin")
        left_fwd = it.make_curve_gait("tripod", 12, 45.0, 30.0, 300.0, "left")
        self.assertEqual(wl.travel_of(left_fwd)[1][2], True)
        left_back = it.make_curve_gait("tripod", 12, 45.0, 30.0, 300.0, "left", backward=True)
        self.assertEqual(wl.travel_of(left_back)[1][2], False)
        right_fwd = it.make_curve_gait("tripod", 12, 45.0, 30.0, 300.0, "right")
        self.assertEqual(wl.travel_of(right_fwd)[1][2], True)

    def test_straight_zone(self):
        fwd = ("straight", True)
        self.assertTrue(wl.in_zone(300, 0, fwd))
        self.assertTrue(wl.in_zone(300, wl.HALF_WIDTH_MM - 1, fwd))
        self.assertFalse(wl.in_zone(300, wl.HALF_WIDTH_MM + 1, fwd))
        self.assertFalse(wl.in_zone(-300, 0, fwd))                  # behind the robot
        back = ("straight", False)
        self.assertTrue(wl.in_zone(-300, 0, back))
        self.assertFalse(wl.in_zone(300, 0, back))

    def test_curve_zone_follows_the_arc(self):
        travel = wl.travel_of(it.make_curve_gait("tripod", 12, 45.0, 30.0, 300.0, "left"))
        self.assertTrue(wl.in_zone(300, 300, travel))               # on the arc, ahead
        self.assertTrue(wl.in_zone(300, 0, travel))                 # 124 mm from the arc: inside the half-width
        self.assertFalse(wl.in_zone(0, -400, travel))               # 700 mm from the ICC: well off the arc
        self.assertFalse(wl.in_zone(-300, 300, travel))             # on the arc but behind the robot

    def test_spin_uses_footprint_clearance(self):
        self.assertAlmostEqual(wl.stop_for(("spin", None), 1000), wl.FOOTPRINT_MM + wl.SPIN_MARGIN_MM)
        self.assertEqual(wl.stop_for(("spin", None), 200), 200)
        self.assertEqual(wl.stop_for(("straight", True), 427), 427)

    def test_assess_states(self):
        fwd = ("straight", True)
        stop, warn = 427, 677
        self.assertEqual(wl.assess([(300, 0)], fwd, stop, warn)["status"], "blocked")
        self.assertEqual(wl.assess([(600, 0)], fwd, stop, warn)["status"], "warn")
        self.assertEqual(wl.assess([(900, 0)], fwd, stop, warn)["status"], "clear")
        self.assertEqual(wl.assess([(-300, 0)], fwd, stop, warn)["status"], "clear")   # behind: not in the way
        a = wl.assess([(600, 0), (300, 10)], fwd, stop, warn)
        self.assertEqual(a["status"], "blocked")
        self.assertEqual(a["nearest_mm"], round(math.hypot(300, 10)))

    def test_clearer_side(self):
        self.assertEqual(wl.clearer_side([(300, 150)], 427), "right")     # obstacle on the left
        self.assertEqual(wl.clearer_side([(300, -150)], 427), "left")     # obstacle on the right
        self.assertIsNone(wl.clearer_side([(300, 150), (300, -150)], 427))
        self.assertEqual(wl.clearer_side([(300, 150), (300, -600)], 427), "right")

    def test_default_distances(self):
        self.assertGreater(wl.DEFAULT_STOP_MM, wl.FOOTPRINT_MM)
        self.assertGreater(wl.DEFAULT_WARN_MM, wl.DEFAULT_STOP_MM)
        self.assertAlmostEqual(wl.HALF_WIDTH_MM, 248 + wl.MARGIN_MM, delta=1.0)


class SimTests(unittest.TestCase):

    def test_box_seen_at_its_near_face(self):
        src = wl.SimSource(wl.Mount(), box_x=900.0, box_y=0.0)
        pts = src.body_points()
        self.assertTrue(pts)
        nearest = min(math.hypot(x, y) for x, y in pts)
        self.assertAlmostEqual(nearest, 900 - 150, delta=2.0)

    def test_moved_box_changes_side(self):
        src = wl.SimSource(wl.Mount(), box_x=600.0, box_y=300.0)
        left = [math.hypot(x, y) for x, y in src.body_points() if y > 0]
        self.assertTrue(left)
        src.set_box(600.0, -300.0)
        self.assertFalse([1 for x, y in src.body_points() if y > 150])


class WalkStartTests(unittest.TestCase):

    def setUp(self):
        self.bus_calls = []

    def _walker(self, source, avoid=False):
        return wl.LidarWalker(source, wl.Settings(avoid=avoid), bus_factory=sg.DryRunBus)

    def test_refuses_without_yes(self):
        w = self._walker(fresh_source())
        with self.assertRaises(wg.ParamError):
            w.start(params(), None)
        with self.assertRaises(wg.ParamError):
            w.start(params(), "no")

    def test_accepts_yes_in_any_case_then_checks_the_lidar(self):
        w = self._walker(wl.SimSource(wl.Mount()))
        with self.assertRaises(wg.ParamError) as ctx:
            w.start(params(), "Yes")
        self.assertIn("simulated", str(ctx.exception))             # got past the confirmation

    def test_refuses_simulated_lidar(self):
        w = self._walker(wl.SimSource(wl.Mount()))
        with self.assertRaises(wg.ParamError) as ctx:
            w.start(params(), "YES")
        self.assertIn("simulated", str(ctx.exception))

    def test_refuses_without_fresh_data(self):
        w = self._walker(wl.LidarSource(wl.Mount()))
        with self.assertRaises(wg.ParamError) as ctx:
            w.start(params(), "YES")
        self.assertIn("not sending data", str(ctx.exception))

    def test_refuses_blocked_start(self):
        readings = [(300, 200)] * 12                                # obstacle 300 mm straight ahead
        w = self._walker(fresh_source(readings, start=0.0, end=3.3))
        with self.assertRaises(wg.ParamError) as ctx:
            w.start(params(), "YES")
        self.assertIn("clear the way", str(ctx.exception))

    def test_tick_check_still_applies(self):
        w = self._walker(fresh_source())
        with self.assertRaises(wg.ParamError) as ctx:
            w.start(params(tick_ms=150), "YES")
        self.assertIn("tick_ms", str(ctx.exception))


class WalkRunTests(unittest.TestCase):
    """Runs the walk synchronously (no thread) with a dry-run bus and a scripted scan."""

    def _run(self, source, walker_avoid=False, **pkw):
        p = params(**pkw)
        w = wl.LidarWalker(source, wl.Settings(avoid=walker_avoid), bus_factory=sg.DryRunBus)
        bus = sg.DryRunBus()
        gait = wg.build_gait(p)
        legs = None
        w._stop.clear()
        w._set(state="running", stage="", cycle=0, tick=0, message="")
        w._run(p, gait, bus, legs)
        return w, bus, gait

    def test_clear_walk_completes(self):
        w, bus, gait = self._run(wl.SimSource(wl.Mount(), box_x=5000.0), cycles=1)
        self.assertEqual(w.state, "done", w.message)
        self.assertEqual(gait.tick, 0)                             # one full cycle
        self.assertTrue(bus.commands)

    def test_blocked_mid_walk_stops_and_returns_to_stand(self):
        src = ScriptedSource(block_from=5)                         # blocked from the 5th tick check on
        w, bus, _ = self._run(src, cycles=2, pose="stand")
        self.assertEqual(w.state, "obstacle", w.message)
        self.assertIn("back at walking stand", "\n".join(w.log))
        self.assertNotIn("turning", "\n".join(w.log))

    def test_turn_away_then_continue(self):
        src = ScriptedSource(block_from=5, block_until=6)          # one blocked check, then clear
        w, bus, _ = self._run(src, walker_avoid=True, cycles=2, direction="forward")
        log = "\n".join(w.log)
        self.assertEqual(w.state, "done", w.message + "\n" + log)
        self.assertIn("turning right", log)                        # obstacle on the left, so turn right
        self.assertIn("walk done: 2 cycle(s)", log)

    def test_turn_away_not_used_for_turns_or_stale_data(self):
        src = ScriptedSource(block_from=5, block_until=6, block_point=(250.0, 100.0))   # inside the spin clearance
        w, _, _ = self._run(src, walker_avoid=True, cycles=1, direction="left")
        self.assertEqual(w.state, "obstacle", w.message)           # spins are not avoided, only stopped
        src = ScriptedSource(stale_from=5)
        w, _, _ = self._run(src, walker_avoid=True, cycles=2, direction="forward")
        self.assertEqual(w.state, "obstacle")
        self.assertIn("stale", w.message)
        self.assertNotIn("turning", "\n".join(w.log))

    def test_repeated_blocking_gives_up_after_max_turns(self):
        src = ScriptedSource(block_from=5)                         # blocked forever
        w, _, _ = self._run(src, walker_avoid=True, cycles=3, direction="forward")
        self.assertEqual(w.state, "obstacle")
        self.assertEqual("\n".join(w.log).count("turning right"), wl.MAX_AVOID_TURNS)


class PageAndRouteTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.source = wl.SimSource(wl.Mount(), box_x=900.0)
        cls.walker = wl.setup(cls.source)
        cls.server = wl.make_server("127.0.0.1", 0)
        cls.port = cls.server.server_address[1]
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()

    def _req(self, path, body=None):
        url = "http://127.0.0.1:%d%s" % (self.port, path)
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(url, data=data, method="POST" if body is not None else "GET",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, r.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()

    def test_page_has_lidar_card_and_walk_gui_content(self):
        code, body = self._req("/")
        self.assertEqual(code, 200)
        text = body.decode()
        self.assertIn('id="lidarCard"', text)
        self.assertIn('id="lCanvas"', text)
        self.assertIn('id="state"', text)                           # walk_gui's status pill is still there

    def test_lidar_view(self):
        code, body = self._req("/api/lidar")
        view = json.loads(body)
        self.assertEqual(code, 200)
        self.assertTrue(view["simulated"])
        self.assertTrue(view["health"]["ok"])
        self.assertIn("zone", view)
        self.assertTrue(view["points"])

    def test_settings_validation_and_update(self):
        code, _ = self._req("/api/lidar/settings", {"stop_mm": 500, "warn_mm": 400})
        self.assertEqual(code, 400)
        code, body = self._req("/api/lidar/settings", {"stop_mm": 450, "warn_mm": 700, "avoid": True})
        self.assertEqual(code, 200)
        self.assertEqual(json.loads(body)["stop_mm"], 450)
        self.assertTrue(self.walker.settings.avoid)
        self._req("/api/lidar/settings", {"stop_mm": wl.DEFAULT_STOP_MM, "warn_mm": wl.DEFAULT_WARN_MM, "avoid": False})

    def test_move_sim_box(self):
        code, _ = self._req("/api/lidar/sim", {"x": 100, "y": 0})
        self.assertEqual(code, 400)                                 # too close: outside the allowed range
        code, body = self._req("/api/lidar/sim", {"x": 1000, "y": 200})
        self.assertEqual(code, 200)
        self.assertEqual(self.source.box, [1000.0, 200.0])

    def test_walk_refused_with_simulated_lidar(self):
        code, body = self._req("/api/walk", {"gait": "tripod", "direction": "forward", "confirm": "yes", "tick_ms": 200})
        self.assertEqual(code, 400)
        self.assertIn("simulated", json.loads(body)["error"])

    def test_plan_and_status_still_work(self):
        code, body = self._req("/api/plan", {"gait": "tripod", "direction": "curve_left", "radius": 400})
        self.assertEqual(code, 200)
        self.assertTrue(json.loads(body)["ok"])
        code, body = self._req("/api/status")
        st = json.loads(body)
        self.assertIn("lidar_status", st)
        code, body = self._req("/api/config")
        cfg = json.loads(body)
        self.assertIn("lidar", cfg)
        self.assertIn("tripod", [g["id"] for g in cfg["gaits"]])


if __name__ == "__main__":
    unittest.main()
