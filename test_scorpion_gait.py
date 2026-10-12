#!/usr/bin/env python3
"""Unit tests for scorpion_gait.py (no hardware needed).

Run:  python3 -m unittest -v test_scorpion_gait
"""

import math
import sys
import types
import unittest

import scorpion_gait as sg


def dist(a, b):
    return math.dist(a[:2], b[:2])


class ResetPose(unittest.TestCase):
    """Every test starts from the default posture (the active pose is global state)."""

    def setUp(self):
        sg.set_active_pose(sg.DEFAULT_POSE)


class TestUnits(ResetPose):
    def test_pulse_mapping_270_deg(self):
        # raw 0..180 spans the full 270-deg travel over 500..2500 us
        self.assertEqual(sg.pulse_us(0), 500)
        self.assertEqual(sg.pulse_us(90), 1500)
        self.assertEqual(sg.pulse_us(180), 2500)
        self.assertEqual(sg.pulse_us(-5), 500)     # clamped
        self.assertEqual(sg.pulse_us(200), 2500)   # clamped
        self.assertAlmostEqual(sg.raw_to_phys(180), 270.0)

    def test_safe_band_in_raw(self):
        self.assertAlmostEqual(sg.raw_to_phys(sg.RAW_MIN), 2.0)
        self.assertAlmostEqual(sg.raw_to_phys(sg.RAW_MAX), 268.0)

    def test_calibration_loaded(self):
        self.assertEqual(sg.CALIBRATION["RF"], (98, 88, 94))
        self.assertEqual(sg.CALIBRATION["LF"], (115, 93, 101))


class TestKinematics(ResetPose):
    def test_stand_table_527_1(self):
        worst, rows = sg.check_stand_table()
        self.assertLess(worst, 0.5, rows)

    def test_neutral_is_stand(self):
        self.assertLess(sg.check_stand_round_trip(), 1e-9)

    def test_random_round_trip(self):
        worst, n = sg.check_ik_round_trip(samples=800, seed=7)
        self.assertGreater(n, 300)
        self.assertLess(worst, 1e-6)

    def test_stand_height_levels(self):
        zs = [sg.LEGS[n].neutral[2] for n in sg.LEG_ORDER]
        self.assertAlmostEqual(min(zs), -87.0, delta=0.2)
        self.assertAlmostEqual(max(zs), -78.3, delta=0.2)

    def test_foot_above_coxa_rejected(self):
        leg = sg.LEGS["RF"]
        with self.assertRaises(sg.KinematicsError):
            leg.ik(leg.mount_x + 150, leg.mount_y - 100, 5.0)

    def test_out_of_reach_rejected(self):
        leg = sg.LEGS["LM"]
        with self.assertRaises(sg.KinematicsError):
            leg.ik(0.0, 60.0 + 400.0, -80.0)

    def test_joint_limit_rejected(self):
        # Foot much closer to the coxa than the leg can fold: tibia leaves the safe band
        leg = sg.LEGS["LM"]
        with self.assertRaises(sg.JointLimitError):
            leg.ik(0.0, 60.0 + 80.0, -100.0)

    def test_outward_foot_gives_neutral_coxa(self):
        # Foot straight out from a right leg (no forward/back component): coxa = calibrated zero
        leg = sg.LEGS["RM"]
        raw = leg.ik(leg.mount_x, leg.mount_y - 230.0, -80.0)
        self.assertAlmostEqual(raw[0], leg.cal[0], places=6)

    def test_forward_is_positive_yaw_for_right_legs(self):
        # Right leg: moving the foot forward (+x) increases the raw coxa angle
        leg = sg.LEGS["RF"]
        back = leg.ik(leg.neutral[0] - 20, leg.neutral[1], leg.neutral[2])
        front = leg.ik(leg.neutral[0] + 20, leg.neutral[1], leg.neutral[2])
        self.assertGreater(front[0], back[0])

    def test_forward_is_negative_yaw_for_left_legs(self):
        leg = sg.LEGS["LF"]
        back = leg.ik(leg.neutral[0] - 20, leg.neutral[1], leg.neutral[2])
        front = leg.ik(leg.neutral[0] + 20, leg.neutral[1], leg.neutral[2])
        self.assertLess(front[0], back[0])


class TestGaitStructure(ResetPose):
    def test_all_gaits_pass_check_in_all_directions(self):
        for gait_type in sg.GAIT_SWING_FRACTION:
            for setter in (lambda g: g.set_straight(True),
                           lambda g: g.set_straight(False),
                           lambda g: g.set_turn_in_place("CCW"),
                           lambda g: g.set_turn_in_place("CW")):
                g = sg.Gait(gait_type, steps=30, stride=45, lift=30)
                setter(g)
                self.assertEqual(g.check(cycles=2), [], (gait_type, g.direction))

    def test_swing_count_per_tick(self):
        expect = {"tripod": 3, "ripple_123456": 1, "ripple_654321": 1,
                  "ripple_135246": 1, "double_ripple": 2}
        for gait_type, n in expect.items():
            g = sg.Gait(gait_type)
            for t in range(g.steps):
                swing = sum(1 for leg in sg.LEG_ORDER if g._is_swing(leg, t))
                self.assertEqual(swing, n, (gait_type, t))

    def test_tripod_groups(self):
        g = sg.Gait("tripod")
        group_a = {"LF", "RM", "LR"}
        for t in range(g.steps):
            swing_a = {leg for leg in group_a if g._is_swing(leg, t)}
            swing_b = {leg for leg in sg.LEG_ORDER if leg not in group_a and g._is_swing(leg, t)}
            self.assertTrue(not swing_a or not swing_b)      # the two tripods alternate
            self.assertEqual(len(swing_a) + len(swing_b), 3)

    def test_steps_must_be_multiple_of_6(self):
        with self.assertRaises(ValueError):
            sg.Gait("tripod", steps=25)

    def test_unknown_gait(self):
        with self.assertRaises(ValueError):
            sg.Gait("moonwalk")

    def test_stop_returns_neutral(self):
        g = sg.Gait("tripod")
        g.set_straight(True)
        g.stop()
        t = g.foot_targets(7)
        for name in sg.LEG_ORDER:
            self.assertEqual(t[name], sg.LEGS[name].neutral)


class TestGaitMotion(unittest.TestCase):
    def _stance_ticks(self, g, leg):
        return [t for t in range(g.steps) if not g._is_swing(leg, t)]

    def test_forward_stance_moves_feet_backward(self):
        # Body moves forward when the stance feet move backward (-X in body frame)
        g = sg.Gait("tripod", stride=45)
        g.set_straight(True)
        for name in sg.LEG_ORDER:
            ticks = self._stance_ticks(g, name)
            xs = [g.foot_targets(t)[name][0] for t in ticks]
            self.assertLess(xs[-1], xs[0], name)

    def test_backward_reverses_stance(self):
        fwd = sg.Gait("tripod", stride=45)
        bwd = sg.Gait("tripod", stride=45)
        fwd.set_straight(True)
        bwd.set_straight(False)
        for name in sg.LEG_ORDER:
            ticks = self._stance_ticks(fwd, name)
            xf = [fwd.foot_targets(t)[name][0] for t in ticks]
            xb = [bwd.foot_targets(t)[name][0] for t in ticks]
            self.assertGreater(xb[-1], xb[0], name)
            self.assertLess(xf[-1], xf[0], name)

    def test_stance_travel_is_stride_scaled_by_radius(self):
        # Stride is set for the outermost leg (R_max). Other legs scale by R_leg / R_max
        # (skills 6.4). The stance travel is chord * (n_stance - 1) / n_stance (tick sampling).
        g = sg.Gait("tripod", stride=45)
        g.set_straight(True)
        cx, cy = g.icc
        radii = {n: math.hypot(l.neutral[0] - cx, l.neutral[1] - cy) for n, l in sg.LEGS.items()}
        r_max = max(radii.values())
        n_st = 15
        for name in sg.LEG_ORDER:
            ticks = self._stance_ticks(g, name)
            xs = [g.foot_targets(t)[name][0] for t in ticks]
            expect = 45.0 * radii[name] / r_max * (n_st - 1) / n_st
            self.assertAlmostEqual(xs[0] - xs[-1], expect, delta=0.3, msg=name)

    def test_straight_walk_has_no_lateral_drift(self):
        g = sg.Gait("tripod", stride=45)
        g.set_straight(True)
        for name in sg.LEG_ORDER:
            ys = [g.foot_targets(t)[name][1] for t in range(g.steps)]
            self.assertLess(max(ys) - min(ys), 1.0, name)

    def test_turn_keeps_radius_about_icc(self):
        g = sg.Gait("tripod", stride=45)
        g.set_turn_in_place("CCW")
        cx, cy = g.icc
        for name in sg.LEG_ORDER:
            radii = [dist(g.foot_targets(t)[name], (cx, cy)) for t in range(g.steps)]
            self.assertLess(max(radii) - min(radii), 1e-6, name)

    def test_turn_direction(self):
        # CCW turn: the body yaws CCW, so the stance feet move clockwise about the centre
        g = sg.Gait("tripod", stride=45)
        g.set_turn_in_place("CCW")
        leg = "LF"
        a0 = math.atan2(g.foot_targets(0)[leg][1], g.foot_targets(0)[leg][0])
        a1 = math.atan2(g.foot_targets(1)[leg][1], g.foot_targets(1)[leg][0])
        self.assertLess(a1, a0)     # angle decreases in stance (CW about the centre)

    def test_swing_lift_height(self):
        g = sg.Gait("tripod", stride=45, lift=30)
        g.set_straight(True)
        for name in sg.LEG_ORDER:
            pz = sg.LEGS[name].neutral[2]
            swing = [t for t in range(g.steps) if g._is_swing(name, t)]
            stance = [t for t in range(g.steps) if not g._is_swing(name, t)]
            heights = [g.foot_targets(t)[name][2] - pz for t in swing]
            self.assertLess(max(heights), 30.0 + 1e-9, name)
            self.assertGreater(max(heights), 30.0 - 3.0, name)   # the sampled peak is near the top
            for t in stance:                                    # stance: on the ground
                self.assertAlmostEqual(g.foot_targets(t)[name][2], pz, places=9, msg=name)

    def test_joint_step_is_small(self):
        # Largest joint change per tick. Keeps the servo moves smooth at a 200 ms tick.
        g = sg.Gait("tripod", stride=45, lift=30)
        g.set_straight(True)
        self.assertLess(g.worst_joint_step(cycles=1), 6.0)

    def test_joints_within_safe_band_over_cycles(self):
        for gait_type in sg.GAIT_SWING_FRACTION:
            g = sg.Gait(gait_type)
            g.set_straight(True)
            for t in range(3 * g.steps):
                joints = g.joints_at(t % g.steps)
                for name, raw in joints.items():
                    for v in raw:
                        self.assertTrue(sg.RAW_MIN - 1e-6 <= v <= sg.RAW_MAX + 1e-6, (gait_type, name, v))

    def test_next_joints_advances_tick(self):
        g = sg.Gait("tripod")
        g.set_straight(True)
        g.next_joints()
        g.next_joints()
        self.assertEqual(g.tick, 2)
        for _ in range(g.steps - 2):
            g.next_joints()
        self.assertEqual(g.tick, 0)

    def test_stride_zero_steps_in_place(self):
        g = sg.Gait("tripod", stride=0.0, lift=30)
        g.set_straight(True)
        for name in sg.LEG_ORDER:
            xs = [g.foot_targets(t)[name][0] for t in range(g.steps)]
            self.assertAlmostEqual(max(xs) - min(xs), 0.0, places=6, msg=name)


class TestServoOutput(ResetPose):
    def test_dry_run_bus_sends_three_servos_per_leg(self):
        bus = sg.DryRunBus()
        g = sg.Gait("tripod")
        g.set_straight(True)
        bus.send(g.next_joints(), move_ms=200)
        self.assertEqual(len(bus.commands), 18)
        ids = sorted(c[0] for c in bus.commands)
        self.assertEqual(ids, sorted(sum(sg.SERVO_IDS.values(), [])))
        for _, pulse, ms in bus.commands:
            self.assertTrue(500 <= pulse <= 2500)
            self.assertEqual(ms, 200)

    def test_servo_control_bus_with_stub(self):
        calls = []
        stub = types.ModuleType("ServoControl")
        stub.setPWMServoMove = lambda sid, pulse, ms: calls.append((sid, pulse, ms))
        saved = sg.ServoControl
        sg.ServoControl = stub
        try:
            bus = sg.ServoControlBus()
            bus.send({"RF": sg.STAND_RAW["RF"]}, move_ms=300)
        finally:
            sg.ServoControl = saved
        expect = [(29, sg.pulse_us(97), 300), (30, sg.pulse_us(114), 300), (31, sg.pulse_us(27), 300)]
        self.assertEqual(calls, expect)

    def test_bus_without_library_raises(self):
        saved = sg.ServoControl
        sg.ServoControl = None
        try:
            with self.assertRaises(ImportError):
                sg.ServoControlBus()
        finally:
            sg.ServoControl = saved

    def test_module_has_servocontrol_name(self):
        # the name exists at module level (None on a PC, the library on the Pi)
        self.assertTrue(hasattr(sg, "ServoControl"))


class TestPostures(ResetPose):
    def test_postures_have_all_legs(self):
        for pose in sg.POSE_NAMES:
            self.assertEqual(sorted(sg.POSES[pose]), sorted(sg.LEG_ORDER), pose)
            for name in sg.LEG_ORDER:
                self.assertEqual(len(sg.POSES[pose][name]), 3, (pose, name))

    def test_stand_initial_is_the_first_manual_table(self):
        self.assertEqual(sg.POSES["stand_initial"]["LF"], (115, 110, 40))
        self.assertEqual(sg.POSES["stand_initial"]["RR"], (103, 105, 35))

    def test_sit_matches_skills_52(self):
        self.assertEqual(sg.POSES["sit"]["LF"], (115, 80, 65))
        self.assertEqual(sg.POSES["sit"]["RR"], (101, 75, 60))

    def test_every_posture_reachable_and_in_band(self):
        bad = [r for r in sg.check_postures() if r[3]]
        self.assertEqual(bad, [])

    def test_set_active_pose_moves_neutral(self):
        stand_neutral = sg.LEGS["LF"].neutral
        sg.set_active_pose("stand_initial")
        self.assertNotEqual(sg.LEGS["LF"].neutral, stand_neutral)
        self.assertEqual(sg.LEGS["LF"].pose_raw, sg.POSES["stand_initial"]["LF"])
        sg.set_active_pose("stand")
        self.assertEqual(sg.LEGS["LF"].neutral, stand_neutral)

    def test_unknown_pose_rejected(self):
        with self.assertRaises(ValueError):
            sg.set_active_pose("lie_down")
        with self.assertRaises(ValueError):
            sg.posture_joints("lie_down")

    def test_posture_joints_is_a_copy(self):
        j = sg.posture_joints("sit")
        self.assertEqual(j["RF"], sg.POSES["sit"]["RF"])
        j["RF"] = (0, 0, 0)
        self.assertEqual(sg.POSES["sit"]["RF"], (102, 80, 60))

    def test_sit_sends_sit_angles(self):
        bus = sg.DryRunBus()
        sg.sit(bus, move_ms=0)
        self.assertEqual(len(bus.commands), 18)
        self.assertIn((29, sg.pulse_us(102), 0), bus.commands)   # RF coxa on servo 29
        self.assertIn((31, sg.pulse_us(60), 0), bus.commands)    # RF tibia on servo 31

    def test_stand_follows_active_pose(self):
        bus = sg.DryRunBus()
        sg.set_active_pose("stand_initial")
        sg.stand(bus, move_ms=0)
        self.assertIn((29, sg.pulse_us(102), 0), bus.commands)   # stand_initial RF coxa 102

    def test_gait_check_from_first_stand(self):
        sg.set_active_pose("stand_initial")
        g = sg.Gait("tripod", stride=45, lift=30)
        g.set_straight(True)
        self.assertEqual(g.check(cycles=1), [])


if __name__ == "__main__":
    unittest.main()
