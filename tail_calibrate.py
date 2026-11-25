#!/usr/bin/env python3
"""Tail calibrator: find the raw angles of each tail motor and of the tail claw, one at a time.

This is a separate tool from the walk. Use it to find the angles for each tail pose
(TAIL_POSES and TAIL_CLAW_POSES in scorpion_gait.py): move one motor, look at the tail,
then save the pose under a name. `list` prints the saved poses in the same form as
TAIL_POSES. `write` saves them to tail_poses.json (ignored by Git), so the values can be
given to the code.

Motors (raw angles, 0 to 180, in pin order 21, 20, 7, 8):
  pin 21  base, between the body      pin 20  yaw (left and right)
  pin 7   fold forward and backward   pin 8   fold forward and backward
Tail claw: pin 6, at the end of the tail.

  python3 tail_calibrate.py               dry run: prints what would be sent, sends nothing
  python3 tail_calibrate.py --hardware    real servos. Moves the legs to the initial stand
                                          (stand_initial) first, then the tail to neutral,
                                          then asks for YES.

Commands at the prompt:
  set <motor> <angle>          move one motor, e.g. "set 20 110". motor = 21, 20, 7, 8 or claw
  nudge <motor> <+/-degrees>   move from the current angle, e.g. "nudge 8 -5"
  show                         show the current angles and saved poses
  save <name>                  save the four motor angles as a pose, e.g. "save down"
  pose <name>                  move the four motors to a saved pose (or a TAIL_POSES name)
  claw <angle|open|closed|grip>  move the tail claw
  claw save <name>             save the current claw angle, e.g. "claw save grip"
  time <ms>                    move time for each command (200 to 5000, default 1500)
  list                         print the saved poses in TAIL_POSES form
  write                        save everything to the JSON file
  help, quit
"""

import argparse
import json
import os
import re
import sys
import time

import scorpion_gait as sg

DEFAULT_OUT = "tail_poses.json"
DEFAULT_MOVE_MS = 1500
MIN_MOVE_MS, MAX_MOVE_MS = 200, 5000
MOTOR_INDEX = {"21": 0, "20": 1, "7": 2, "8": 3}     # position in TAIL_PINS
MOTOR_LABEL = {0: "pin 21 (base)", 1: "pin 20 (yaw)", 2: "pin 7 (fold)", 3: "pin 8 (fold)"}
USAGE = {
    "set": "set <motor> <angle>", "nudge": "nudge <motor> <+/-degrees>", "save": "save <name>",
    "pose": "pose <name>", "claw": "claw <angle|open|closed|grip>  or  claw save <name>", "time": "time <ms>",
}
NAME_RE = re.compile(r"[a-z][a-z0-9_]*")
HELP = __doc__.split("Commands at the prompt:", 1)[1].strip("\n")


class CalibrationError(Exception):
    """A bad command or value. The calibrator reports it and carries on."""


def parse_angle(text):
    try:
        value = float(text)
    except (TypeError, ValueError):
        raise CalibrationError("angle must be a number, got %r" % (text,))
    if not 0 <= value <= 180:
        raise CalibrationError("angle must be 0 to 180 (raw), got %s" % text)
    return int(round(value))


def parse_motor(text):
    """'20' -> 1 (index in TAIL_PINS); 'claw' or '6' -> 'claw'."""
    key = str(text).strip().lower()
    if key in ("claw", "6"):
        return "claw"
    if key in MOTOR_INDEX:
        return MOTOR_INDEX[key]
    raise CalibrationError("motor must be 21, 20, 7, 8 or claw, got %r" % (text,))


class TailCalibrator:
    """Holds the last angles sent, so one motor can change while the others hold still."""

    def __init__(self, bus, move_ms=DEFAULT_MOVE_MS, out_path=DEFAULT_OUT, start=None):
        self.bus = bus
        self.move_ms = move_ms
        self.out_path = out_path
        self.current = list(start if start is not None else sg.TAIL_POSES["neutral"])
        self.claw = None                  # unknown until the claw is first moved
        self.poses = {}                   # saved motor poses: name -> [21, 20, 7, 8]
        self.claw_poses = {}              # saved claw angles: name -> raw
        self.dirty = False

    # ----- sending -----
    def _send_motors(self):
        self.bus.send_tail(list(self.current), self.move_ms)   # the servos move over move_ms by themselves

    def _send_claw(self, raw):
        self.bus.send_tail_claw(raw, self.move_ms)
        self.claw = raw

    def _check_time(self, ms):
        if not MIN_MOVE_MS <= ms <= MAX_MOVE_MS:
            raise CalibrationError("time must be %d to %d ms" % (MIN_MOVE_MS, MAX_MOVE_MS))

    # ----- commands -----
    def set_motor(self, motor, angle):
        if motor == "claw":
            self._send_claw(angle)
            return "claw -> %d" % angle
        self.current[motor] = angle
        self._send_motors()
        return "%s -> %d" % (MOTOR_LABEL[motor], angle)

    def nudge(self, motor, delta):
        try:
            step = float(delta)
        except (TypeError, ValueError):
            raise CalibrationError("nudge step must be a number, got %r" % (delta,))
        if motor == "claw":
            if self.claw is None:
                raise CalibrationError("the claw angle is unknown: set it first, e.g. \"claw open\"")
            return self.set_motor("claw", self._in_range(self.claw + step))
        return self.set_motor(motor, self._in_range(self.current[motor] + step))

    @staticmethod
    def _in_range(value):
        value = int(round(value))
        if not 0 <= value <= 180:
            raise CalibrationError("would move past the 0 to 180 range (%d)" % value)
        return value

    def save(self, name):
        if not NAME_RE.fullmatch(name or ""):
            raise CalibrationError("pose name: lower case letters, digits and _, starting with a letter")
        self.poses[name] = list(self.current)
        self.dirty = True
        return "saved pose '%s' = %s" % (name, self.current)

    def pose(self, name):
        if name in self.poses:
            values = self.poses[name]
        elif name in sg.TAIL_POSES:
            values = sg.TAIL_POSES[name]
        else:
            raise CalibrationError("no pose named %r (saved: %s)" % (name, ", ".join(sorted(self.poses)) or "none"))
        self.current = list(values)
        self._send_motors()
        return "pose '%s' = %s" % (name, self.current)

    def claw_command(self, text):
        if text in sg.TAIL_CLAW_POSES:
            self._send_claw(sg.TAIL_CLAW_POSES[text])
            return "claw '%s' = %d" % (text, self.claw)
        self._send_claw(parse_angle(text))
        return "claw -> %d" % self.claw

    def claw_save(self, name):
        if self.claw is None:
            raise CalibrationError("the claw angle is unknown: move the claw first")
        if not NAME_RE.fullmatch(name or ""):
            raise CalibrationError("claw pose name: lower case letters, digits and _")
        self.claw_poses[name] = self.claw
        self.dirty = True
        return "saved claw '%s' = %d" % (name, self.claw)

    def set_time(self, text):
        try:
            ms = int(text)
        except (TypeError, ValueError):
            raise CalibrationError("time must be a whole number of ms")
        self._check_time(ms)
        self.move_ms = ms
        return "move time = %d ms" % ms

    def show(self):
        lines = ["motors (raw): " + "  ".join("%s %d" % (MOTOR_LABEL[i], a) for i, a in enumerate(self.current))]
        lines.append("claw (raw): %s" % ("unknown" if self.claw is None else self.claw))
        lines.append("move time: %d ms" % self.move_ms)
        lines.append("saved poses: %s" % (", ".join(sorted(self.poses)) or "none"))
        lines.append("saved claw: %s" % (", ".join("%s=%d" % kv for kv in sorted(self.claw_poses.items())) or "none"))
        return "\n".join(lines)

    def list_text(self):
        if not self.poses and not self.claw_poses:
            return "nothing saved yet"
        lines = []
        if self.poses:
            lines.append("TAIL_POSES = {")
            for name, values in self.poses.items():
                lines.append('    "%s": %s,' % (name, values))
            lines.append("}")
        if self.claw_poses:
            lines.append("TAIL_CLAW_POSES = {%s}" % ", ".join('"%s": %d' % kv for kv in self.claw_poses.items()))
        return "\n".join(lines)

    def write(self):
        data = {"pins": list(sg.TAIL_PINS), "poses": self.poses, "claw_poses": self.claw_poses}
        folder = os.path.dirname(os.path.abspath(self.out_path))
        os.makedirs(folder, exist_ok=True)
        with open(self.out_path, "w") as fh:
            json.dump(data, fh, indent=2)
            fh.write("\n")
        self.dirty = False
        return "wrote %s" % self.out_path

    # ----- prompt -----
    def handle(self, line):
        """Runs one command. Returns (text, keep_going). Bad input gives an error text, never an exception."""
        parts = line.strip().split()
        if not parts:
            return "", True
        cmd, args = parts[0].lower(), parts[1:]
        try:
            if cmd in ("quit", "exit", "q"):
                return "", False
            if cmd == "help":
                return HELP, True
            if cmd == "show":
                return self.show(), True
            if cmd == "list":
                return self.list_text(), True
            if cmd == "write":
                return self.write(), True
            if cmd == "time" and len(args) == 1:
                return self.set_time(args[0]), True
            if cmd == "set" and len(args) == 2:
                return self.set_motor(parse_motor(args[0]), parse_angle(args[1])), True
            if cmd == "nudge" and len(args) == 2:
                return self.nudge(parse_motor(args[0]), args[1]), True
            if cmd == "save" and len(args) == 1:
                return self.save(args[0]), True
            if cmd == "pose" and len(args) == 1:
                return self.pose(args[0]), True
            if cmd == "claw" and len(args) == 2 and args[0].lower() == "save":
                return self.claw_save(args[1]), True
            if cmd == "claw" and len(args) == 1:
                return self.claw_command(args[0].lower()), True
            if cmd in USAGE:
                return "error: usage: %s" % USAGE[cmd], True
            return "unknown command: %s (type help)" % line.strip(), True
        except CalibrationError as exc:
            return "error: %s" % exc, True


def run_loop(cal, read=input, out=print):
    out("Type help for the commands. Angles are raw, 0 to 180.")
    while True:
        try:
            line = read("tail> ")
        except EOFError:
            line = "quit"
        except KeyboardInterrupt:
            out("")
            line = "quit"
        text, keep_going = cal.handle(line)
        if text:
            out(text)
        if not keep_going:
            break
    if cal.dirty:
        out(cal.write())
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description="Tail motor and tail claw calibrator (separate from the walk).")
    parser.add_argument("--hardware", action="store_true", help="send commands to the real servos")
    parser.add_argument("--out", default=DEFAULT_OUT, help="JSON file for the saved poses (default %s)" % DEFAULT_OUT)
    parser.add_argument("--move-ms", type=int, default=DEFAULT_MOVE_MS, help="move time per command, ms")
    parser.add_argument("--pose-ms", type=int, default=2000, help="move time for the legs' initial stand, ms")
    args = parser.parse_args(argv)
    if not MIN_MOVE_MS <= args.move_ms <= MAX_MOVE_MS:
        print("--move-ms must be %d to %d" % (MIN_MOVE_MS, MAX_MOVE_MS))
        return 2

    if not args.hardware:
        print("DRY RUN: nothing is sent to the servos. Use --hardware on the Raspberry Pi.")
        return run_loop(TailCalibrator(sg.DryRunBus(), args.move_ms, args.out))

    print("About to move the REAL servos.")
    print("  1. legs to the initial stand '%s' (%d ms)" % (sg.START_POSE, args.pose_ms))
    print("  2. tail to neutral, then the tail motors as you type them")
    print("  Robot must be propped up or the feet free. Keep power current-limited.")
    if not sg._confirm():
        print("Cancelled.")
        return 0
    try:
        bus = sg.ServoControlBus()
    except ImportError:
        print("ServoControl library not found. Run this on the Raspberry Pi.")
        return 2
    cal = TailCalibrator(bus, args.move_ms, args.out)
    try:
        sg.move_to_pose(bus, sg.START_POSE, move_ms=args.pose_ms)
        print("At initial stand '%s'." % sg.START_POSE)
        bus.send_tail(list(cal.current), args.pose_ms)
        time.sleep(args.pose_ms / 1000.0)
        print("Tail at neutral %s." % cal.current)
        return run_loop(cal)
    except KeyboardInterrupt:
        print("\nInterrupted. The servos hold their last pose.")
        return 1
    except sg.GaitError as exc:
        print("\nStopped: %s" % exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
