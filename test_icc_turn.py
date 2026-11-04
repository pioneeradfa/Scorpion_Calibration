#!/usr/bin/env python3
# Tests for icc_turn.py: section 7 of the reference (ICC curves and turns).
#   python3 -m unittest test_icc_turn

import io
import math
import contextlib
import unittest

import icc_turn as it
import scorpion_gait as sg
import walk


class TestSection7Equivalence(unittest.TestCase):
    """The section 7 steps must give the same feet as the existing gait for straight walking and
    for spinning in place (the two cases the existing code already handles)."""

    def setUp(self):
        sg.set_active_pose("stand")

    def _max_diff(self, a, b):
        return max(abs(x - y) for n in a for x, y in zip(a[n], b[n]))

    def test_matches_straight_and_spin(self):
        for gt in sg.GAIT_SWING_FRACTION:
            N = sg.GAIT_DEFAULT_STEPS[gt]
            with self.subTest(gait=gt, mode="straight"):
                old = sg.Gait(gt, steps=N, stride=45, lift=30)
                old.set_straight(True)
                new = it.IccTurnGait(gt, steps=N, stride=45, lift=30)
                new.set_straight(True)
                for t in range(N):
                    self.assertLess(self._max_diff(old.joints_at(t), new.joints_at(t)), 1e-6)
            with self.subTest(gait=gt, mode="spin"):
                old = sg.Gait(gt, steps=N, stride=45, lift=30)
                old.set_turn_in_place("CCW")
                new = it.IccTurnGait(gt, steps=N, stride=45, lift=30)
                new.set_turn_in_place("CCW")
                for t in range(N):
                    self.assertLess(self._max_diff(old.joints_at(t), new.joints_at(t)), 1e-6)


class TestDirections(unittest.TestCase):
    """Walking direction from the ICC side and CW/CCW (see icc_turn.py header)."""

    def setUp(self):
        sg.set_active_pose("stand")

    def _body_motion(self, gait):
        """Body motion in one stance of the LF leg: the planted foot moves opposite to the body."""
        N = gait.steps
        feet = [gait.foot_targets(t)["LF"] for t in range(N)]
        stance = [t for t in range(N) if not gait._is_swing("LF", t)]
        dx = sum(feet[(t + 1) % N][0] - feet[t][0] for t in stance[:-1])
        dy = sum(feet[(t + 1) % N][1] - feet[t][1] for t in stance[:-1])
        return -dx, -dy     # body displacement = minus foot displacement

    def test_curve_left_forward_goes_forward_and_left(self):
        g = it.make_curve_gait("tripod", 12, 45, 30, 300, "left")
        bx, by = self._body_motion(g)
        self.assertGreater(bx, 0)     # forward
        self.assertGreater(by, 0)     # towards the ICC (left)

    def test_curve_right_forward_goes_forward_and_right(self):
        g = it.make_curve_gait("tripod", 12, 45, 30, 300, "right")
        bx, by = self._body_motion(g)
        self.assertGreater(bx, 0)
        self.assertLess(by, 0)

    def test_backward_reverses_travel(self):
        for side in ("left", "right"):
            fwd = self._body_motion(it.make_curve_gait("tripod", 12, 45, 30, 300, side))
            bwd = self._body_motion(it.make_curve_gait("tripod", 12, 45, 30, 300, side, backward=True))
            self.assertLess(fwd[0] * bwd[0], 0, side)

    def test_radius_is_constant_for_every_leg(self):
        # 7.4: the foot moves on a fixed-radius circle about the ICC
        g = it.make_curve_gait("tripod", 12, 45, 30, 300, "left")
        cx, cy = g.icc
        for name in sg.LEG_ORDER:
            radii = [math.hypot(g.foot_targets(t)[name][0] - cx, g.foot_targets(t)[name][1] - cy)
                     for t in range(g.steps)]
            self.assertLess(max(radii) - min(radii), 1e-6, name)

    def test_every_gait_curves_cleanly(self):
        for gt in sg.GAIT_SWING_FRACTION:
            for radius in (150, 300, 600):
                for side in ("left", "right"):
                    with self.subTest(gait=gt, radius=radius, side=side):
                        g = it.make_curve_gait(gt, sg.GAIT_DEFAULT_STEPS[gt], 45, 30, radius, side)
                        self.assertEqual(g.check(cycles=2), [])


class TestGuards(unittest.TestCase):
    """Section 7.5: refuse ICC on a foot and strides longer than the outer circle."""

    def setUp(self):
        sg.set_active_pose("stand")

    def test_icc_on_a_foot_is_refused(self):
        g = it.IccTurnGait("tripod", steps=12, stride=45, lift=30)
        px, py, _ = g.legs["LF"].neutral
        g.set_icc(px, py, "CCW")
        problems = g.check(cycles=1)
        self.assertTrue(problems)
        self.assertIn("on the LF foot", problems[0])

    def test_stride_longer_than_outer_circle_is_refused(self):
        g = it.make_curve_gait("tripod", 12, 1200, 30, 300, "left")
        problems = g.check(cycles=1)
        self.assertTrue(problems)
        self.assertIn("longer than the outer leg's circle", problems[0])

    def test_alpha_domain_error_is_raised_not_clamped(self):
        with self.assertRaises(it.IccError):
            it.alpha_total(leg_stride=300.0, leg_dist=100.0)

    def test_bad_arguments(self):
        with self.assertRaises(ValueError):
            it.make_curve_gait("tripod", 12, 45, 30, 0, "left")
        with self.assertRaises(ValueError):
            it.make_curve_gait("tripod", 12, 45, 30, 300, "up")


class TestSpeed(unittest.TestCase):
    def setUp(self):
        sg.set_active_pose("stand")

    def test_straight_speed_is_stride_per_stance(self):
        g = it.IccTurnGait("tripod", steps=12, stride=45, lift=30)
        g.set_straight(True)
        speed, rate = it.body_motion(g, 2.4)
        self.assertAlmostEqual(speed, 45 / 1.2)   # stance is half of the 2.4 s cycle
        self.assertEqual(rate, 0.0)

    def test_spin_has_no_travel_but_turns(self):
        g = it.IccTurnGait("tripod", steps=12, stride=45, lift=30)
        g.set_turn_in_place("CCW")
        speed, rate = it.body_motion(g, 2.4)
        self.assertAlmostEqual(speed, 0.0)
        self.assertGreater(rate, 0.0)

    def test_curve_speed_uses_radius(self):
        g = it.make_curve_gait("tripod", 12, 45, 30, 300, "left")
        speed, rate = it.body_motion(g, 2.4)
        _, delta = g._geometry()
        self.assertAlmostEqual(speed, delta * 300 / 1.2)
        self.assertAlmostEqual(rate, math.degrees(delta) / 1.2)


class TestCommandLine(unittest.TestCase):
    def test_planner_prints_section7_table(self):
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            code = it.main(["--curve", "left", "--radius", "300"])
        text = out.getvalue()
        self.assertEqual(code, 0)
        self.assertIn("legDist", text)
        self.assertIn("Dry run: nothing was sent.", text)

    def test_walk_curve_needs_radius(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), self.assertRaises(SystemExit):
            walk.main(["--curve", "left"])
        self.assertIn("must be used together", err.getvalue())

    def test_walk_curve_cannot_mix_with_spin(self):
        err = io.StringIO()
        with contextlib.redirect_stderr(err), self.assertRaises(SystemExit):
            walk.main(["--curve", "left", "--curve-radius", "300", "--turn-left"])
        self.assertIn("cannot be combined", err.getvalue())


class TestSimulatedBodyPath(unittest.TestCase):
    """Body pose worked out from the planted feet (icc_sim.py). The gait's own model is the reference,
    so this checks that the body really follows the requested ICC. Not a test on the robot."""

    def setUp(self):
        sg.set_active_pose("stand")

    def _run(self, gait, cycles=2):
        import icc_sim
        return icc_sim.path_stats(gait, cycles)

    def test_stance_feet_fit_a_rigid_motion(self):
        # if the ICC model is right, the planted feet are exactly a rigid motion each step
        for side in ("left", "right"):
            for backward in (False, True):
                with self.subTest(side=side, backward=backward):
                    g = it.make_curve_gait("tripod", 12, 45, 30, 300, side, backward)
                    self.assertLess(self._run(g)["fit_rms_mm"], 1e-6)
        g = it.IccTurnGait("tripod", steps=12, stride=45, lift=30)
        g.set_turn_in_place("CCW")
        self.assertLess(self._run(g)["fit_rms_mm"], 1e-6)

    def test_curve_radius_matches_request(self):
        for side in ("left", "right"):
            for radius in (300, 600):
                with self.subTest(side=side, radius=radius):
                    g = it.make_curve_gait("tripod", 12, 45, 30, radius, side)
                    st = self._run(g)
                    self.assertAlmostEqual(st["measured_radius_mm"] / radius, 1.0, delta=0.005)

    def test_curve_turn_rate_matches_model(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                g = it.make_curve_gait("tripod", 12, 45, 30, 300, side)
                st = self._run(g)
                _, rate = it.body_motion(g, 12 * 0.2)
                measured = st["heading_deg"] / (2 * 12 * 0.2)
                self.assertAlmostEqual(abs(measured) / rate, 1.0, delta=0.01)
                self.assertEqual(measured > 0, side == "left")      # left turns CCW (positive)

    def test_spin_stays_in_place(self):
        g = it.IccTurnGait("tripod", steps=12, stride=45, lift=30)
        g.set_turn_in_place("CCW")
        st = self._run(g)
        self.assertLess(math.hypot(st["end_x"], st["end_y"]), 1.0)
        self.assertGreater(st["heading_deg"], 0)

    def test_straight_walk_distance_per_cycle(self):
        # tripod: two planted stances per cycle, each one stride -> 2 x 45 mm per cycle (ICC at 10 m is ~2% off)
        g = sg.Gait("tripod", steps=12, stride=45, lift=30)
        g.set_straight(True)
        st = self._run(g, cycles=2)
        self.assertAlmostEqual(st["end_x"] / (2 * 2 * 45), 1.0, delta=0.03)
        self.assertLess(abs(st["end_y"]), 2.0)

    def test_curve_direction_in_the_path(self):
        cases = {
            ("left", False): (+1, +1),    # forward, bends left
            ("right", False): (+1, -1),   # forward, bends right
            ("left", True): (-1, +1),     # backward, still bends left
            ("right", True): (-1, -1),
        }
        for (side, backward), (sx, sy) in cases.items():
            with self.subTest(side=side, backward=backward):
                g = it.make_curve_gait("tripod", 12, 45, 30, 300, side, backward)
                st = self._run(g)
                self.assertEqual(st["end_x"] > 0, sx > 0)
                self.assertEqual(st["end_y"] > 0, sy > 0)


if __name__ == "__main__":
    unittest.main()
