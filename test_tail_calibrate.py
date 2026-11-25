import io
import json
import os
import tempfile
import unittest
from contextlib import redirect_stdout

import scorpion_gait as sg
import tail_calibrate as tc


class Fixture(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.out = os.path.join(self._tmp.name, "tail_poses.json")
        self.bus = sg.DryRunBus()
        self.cal = tc.TailCalibrator(self.bus, move_ms=1500, out_path=self.out)

    def tearDown(self):
        self._tmp.cleanup()

    def last_tail(self):
        """Pins and raw angles of the last four commands (one send_tail call)."""
        cmds = self.bus.commands[-4:]
        return [c[0] for c in cmds], [round((c[1] - tc.sg.PULSE_MIN) / (sg.PULSE_MAX - sg.PULSE_MIN) * 180) for c in cmds]

    def run_lines(self, *lines):
        texts = []
        for line in lines:
            text, keep = self.cal.handle(line)
            texts.append(text)
        return texts


class ParseTests(Fixture):
    def test_angle_range_and_type(self):
        self.assertEqual(tc.parse_angle("90"), 90)
        self.assertEqual(tc.parse_angle("0"), 0)
        self.assertEqual(tc.parse_angle("180"), 180)
        for bad in ("181", "-1", "abc", ""):
            with self.assertRaises(tc.CalibrationError):
                tc.parse_angle(bad)

    def test_motor_names(self):
        self.assertEqual(tc.parse_motor("21"), 0)
        self.assertEqual(tc.parse_motor("20"), 1)
        self.assertEqual(tc.parse_motor("7"), 2)
        self.assertEqual(tc.parse_motor("8"), 3)
        self.assertEqual(tc.parse_motor("claw"), "claw")
        self.assertEqual(tc.parse_motor("6"), "claw")
        with self.assertRaises(tc.CalibrationError):
            tc.parse_motor("9")


class CommandTests(Fixture):
    def test_set_motor_sends_all_four_and_holds_the_others(self):
        self.cal.handle("set 20 110")
        pins, _ = self.last_tail()
        self.assertEqual(pins, [21, 20, 7, 8])
        self.assertEqual(self.cal.current, [90, 110, 60, 60])   # only pin 20 changed

    def test_bad_angle_sends_nothing(self):
        before = len(self.bus.commands)
        text, keep = self.cal.handle("set 20 200")
        self.assertTrue(text.startswith("error"))
        self.assertTrue(keep)
        self.assertEqual(len(self.bus.commands), before)
        self.assertEqual(self.cal.current, [90, 90, 60, 60])

    def test_nudge_from_current_angle(self):
        self.cal.handle("nudge 8 -5")
        self.assertEqual(self.cal.current[3], 55)
        self.cal.handle("nudge 8 +10")
        self.assertEqual(self.cal.current[3], 65)

    def test_nudge_past_range_is_refused(self):
        self.cal.handle("set 21 178")
        text, _ = self.cal.handle("nudge 21 +5")
        self.assertTrue(text.startswith("error"))
        self.assertEqual(self.cal.current[0], 178)

    def test_claw_nudge_needs_a_known_angle(self):
        text, _ = self.cal.handle("nudge claw 5")
        self.assertTrue(text.startswith("error"))
        self.cal.handle("claw open")
        self.cal.handle("nudge claw -10")
        self.assertEqual(self.cal.claw, 80)

    def test_save_and_pose_round_trip(self):
        self.cal.handle("set 20 70")
        self.cal.handle("save yaw_left")
        self.cal.handle("set 20 110")
        self.cal.handle("pose yaw_left")
        self.assertEqual(self.cal.current, [90, 70, 60, 60])
        pins, _ = self.last_tail()
        self.assertEqual(pins, [21, 20, 7, 8])

    def test_pose_by_builtin_name(self):
        self.cal.handle("pose neutral")
        self.assertEqual(self.cal.current, list(sg.TAIL_POSES["neutral"]))

    def test_unknown_pose_is_an_error(self):
        text, keep = self.cal.handle("pose nope")
        self.assertTrue(text.startswith("error"))
        self.assertTrue(keep)

    def test_bad_pose_name_is_refused(self):
        text, _ = self.cal.handle("save Down")
        self.assertTrue(text.startswith("error"))
        self.assertEqual(self.cal.poses, {})

    def test_wrong_argument_count_shows_usage(self):
        text, _ = self.cal.handle("save Down Hard")
        self.assertIn("usage: save <name>", text)

    def test_claw_named_and_numeric(self):
        self.cal.handle("claw grip")
        self.assertEqual(self.cal.claw, sg.TAIL_CLAW_POSES["grip"])
        self.assertEqual(self.bus.commands[-1][0], sg.TAIL_CLAW_PIN)
        self.cal.handle("claw 45")
        self.assertEqual(self.cal.claw, 45)

    def test_claw_save(self):
        self.cal.handle("claw 40")
        self.cal.handle("claw save half")
        self.assertEqual(self.cal.claw_poses, {"half": 40})
        text, _ = self.cal.handle("claw save half")
        self.assertIn("saved claw", text)

    def test_claw_save_before_move_is_an_error(self):
        text, _ = self.cal.handle("claw save early")
        self.assertTrue(text.startswith("error"))

    def test_time_limits(self):
        self.cal.handle("time 900")
        self.assertEqual(self.cal.move_ms, 900)
        text, _ = self.cal.handle("time 50")
        self.assertTrue(text.startswith("error"))
        self.assertEqual(self.cal.move_ms, 900)

    def test_quit_and_unknown_and_blank(self):
        self.assertEqual(self.cal.handle("quit"), ("", False))
        self.assertEqual(self.cal.handle("   "), ("", True))
        text, keep = self.cal.handle("dance")
        self.assertIn("unknown command", text)
        self.assertTrue(keep)

    def test_show_and_help(self):
        self.assertIn("pin 20", self.cal.show())
        self.assertIn("set <motor>", self.cal.handle("help")[0])


class OutputTests(Fixture):
    def test_list_prints_tail_poses_form(self):
        self.assertEqual(self.cal.list_text(), "nothing saved yet")
        self.cal.handle("set 7 40")
        self.cal.handle("save low")
        self.cal.handle("claw 30")
        self.cal.handle("claw save shut")
        text = self.cal.list_text()
        self.assertIn('"low": [90, 90, 40, 60],', text)
        self.assertIn('TAIL_CLAW_POSES = {"shut": 30}', text)

    def test_write_creates_json_and_loop_writes_on_quit(self):
        self.cal.handle("set 8 100")
        self.cal.handle("save down_test")
        out = io.StringIO()
        with redirect_stdout(out):
            tc.run_loop(self.cal, read=lambda prompt="": "quit", out=print)
        with open(self.out) as fh:
            data = json.load(fh)
        self.assertEqual(data["pins"], [21, 20, 7, 8])
        self.assertEqual(data["poses"], {"down_test": [90, 90, 60, 100]})
        self.assertFalse(self.cal.dirty)

    def test_loop_survives_bad_input_and_eof(self):
        lines = iter(["set 20 999", "show"])

        def read(prompt):
            try:
                return next(lines)
            except StopIteration:
                raise EOFError
        out = io.StringIO()
        with redirect_stdout(out):
            code = tc.run_loop(self.cal, read=read, out=print)
        self.assertEqual(code, 0)
        self.assertIn("error", out.getvalue())
        self.assertIn("motors (raw)", out.getvalue())

    def test_main_dry_run_sends_nothing_real(self):
        import builtins
        from unittest import mock
        lines = iter(["set 20 100", "quit"])
        with mock.patch.object(builtins, "input", lambda prompt="": next(lines)):
            with redirect_stdout(io.StringIO()) as out:
                code = tc.main(["--out", self.out])
        self.assertEqual(code, 0)
        self.assertIn("DRY RUN", out.getvalue())

    def test_main_rejects_bad_move_time(self):
        with redirect_stdout(io.StringIO()):
            self.assertEqual(tc.main(["--move-ms", "10"]), 2)


class LegsFirstTests(unittest.TestCase):
    def test_hardware_mode_moves_legs_to_stand_first(self):
        """The hardware path must start with the legs at stand_initial. Checked by reading the source order."""
        import inspect
        src = inspect.getsource(tc.main)
        self.assertLess(src.index("move_to_pose(bus, sg.START_POSE"), src.index("send_tail(list(cal.current)"))
        self.assertLess(src.index("_confirm()"), src.index("ServoControlBus()"))


if __name__ == "__main__":
    unittest.main()
