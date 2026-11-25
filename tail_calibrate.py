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
  add --web to either form for a browser page with a slider for each motor
  (port 8010 by default, change it with --port).

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
import http.server
import json
import os
import re
import sys
import threading
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


#---------------------------------------------------------------------------------
# Browser page: one slider per tail motor, and one for the tail claw. Each slider moves its
# servo when you let go. Same commands as the prompt, through the same TailCalibrator.

PAGE = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Tail calibrator</title>
<style>
body{font-family:system-ui,sans-serif;margin:16px;max-width:760px;color:#1d2330}
h1{font-size:20px} h2{font-size:16px;margin:22px 0 6px}
.motor{display:grid;grid-template-columns:170px 1fr 80px;align-items:center;gap:10px;margin:8px 0}
.motor input[type=range]{width:100%}
.val{font-variant-numeric:tabular-nums;text-align:right}
button{padding:6px 10px;margin:3px;border:1px solid #888;border-radius:6px;background:#f4f6fa;cursor:pointer}
input[type=text],input[type=number]{padding:5px;border:1px solid #888;border-radius:6px}
.hint{color:#555;font-size:13px}
#status{margin-top:16px;min-height:1.4em;color:#0a5;font-weight:600}
#status.err{color:#b00}
pre{background:#f4f6fa;padding:8px;border-radius:6px;white-space:pre-wrap}
</style></head><body>
<h1>Tail calibrator</h1>
<p class="hint">Drag a slider and let go: that servo moves to the raw angle (0 to 180). The other motors stay where they are.</p>
<h2>Motors</h2>
<div id="motors"></div>
<h2>Tail claw (pin 6)</h2>
<div id="claw"></div>
<div id="clawposes"></div>
<p><input type="text" id="clawName" placeholder="claw pose name"> <button id="clawSave">Save claw angle</button></p>
<h2>Move time</h2>
<p><label>ms <input type="number" id="time" min="200" max="5000" step="100" value="1500"></label>
<button id="setTime">Set</button></p>
<h2>Poses</h2>
<p class="hint">Save stores the four motor angles now. Names: lower case letters, digits and _.</p>
<p><input type="text" id="saveName" placeholder="e.g. down"> <button id="save">Save pose</button></p>
<div id="poses"></div>
<h2>Files</h2>
<p><button id="write">Write tail_poses.json</button> <button id="list">Show TAIL_POSES text</button></p>
<pre id="listing"></pre>
<div id="status"></div>
<script>
var MOTORS = [["21", "Pin 21 (base)"], ["20", "Pin 20 (yaw)"], ["7", "Pin 7 (fold)"], ["8", "Pin 8 (fold)"]];
var state = null;
function $(id) { return document.getElementById(id); }
function say(text, bad) { var el = $("status"); el.textContent = text; el.className = bad ? "err" : ""; }
function call(path, body) {
  var opts = body === undefined ? {} : {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)};
  return fetch(path, opts).then(function (r) {
    return r.json().catch(function () { return {}; }).then(function (data) {
      if (!r.ok) { throw new Error(data.error || ("HTTP " + r.status)); }
      return data;
    });
  });
}
function done(data) {
  if (data.state) { state = data.state; render(); }
  if (data.message) { say(data.message, false); }
}
function fail(err) { say(err.message, true); if (state) { render(); } }
function addSlider(parent, key, label) {
  var row = document.createElement("div"); row.className = "motor";
  var lab = document.createElement("label"); lab.htmlFor = "s" + key; lab.textContent = label;
  var sl = document.createElement("input"); sl.type = "range"; sl.min = 0; sl.max = 180; sl.step = 1; sl.id = "s" + key;
  var val = document.createElement("span"); val.className = "val"; val.id = "v" + key;
  sl.addEventListener("input", function () { val.textContent = sl.value; });
  sl.addEventListener("change", function () {
    call("/api/motor", {motor: key, angle: Number(sl.value)}).then(done).catch(fail);
  });
  row.appendChild(lab); row.appendChild(sl); row.appendChild(val); parent.appendChild(row);
}
function setSlider(key, value) {
  var sl = $("s" + key), val = $("v" + key);
  if (value === null || value === undefined) { val.textContent = "unknown"; return; }
  sl.value = value; val.textContent = value;
}
function fillButtons(box, items, onClick) {       // items: [label, argument]
  box.innerHTML = "";
  items.forEach(function (it) {
    var b = document.createElement("button"); b.textContent = it[0];
    b.addEventListener("click", function () { onClick(it[1]).then(done).catch(fail); });
    box.appendChild(b);
  });
}
function render() {
  if (!state) { return; }
  MOTORS.forEach(function (m, i) { setSlider(m[0], state.motors[i]); });
  setSlider("claw", state.claw);
  $("time").value = state.move_ms;
  var poseItems = state.builtin_poses.map(function (n) { return [n + " (built-in)", n]; })
    .concat(Object.keys(state.poses).map(function (n) { return [n, n]; }));
  fillButtons($("poses"), poseItems, function (name) { return call("/api/pose", {name: name}); });
  var clawItems = state.builtin_claw.map(function (n) { return [n, n]; })
    .concat(Object.keys(state.claw_poses).map(function (n) { return [n + " = " + state.claw_poses[n], String(state.claw_poses[n])]; }));
  fillButtons($("clawposes"), clawItems, function (name) { return call("/api/claw", {name: name}); });
}
MOTORS.forEach(function (m) { addSlider($("motors"), m[0], m[1]); });
addSlider($("claw"), "claw", "Tail claw");
$("setTime").addEventListener("click", function () { call("/api/time", {ms: Number($("time").value)}).then(done).catch(fail); });
$("save").addEventListener("click", function () { call("/api/save", {name: $("saveName").value.trim()}).then(done).catch(fail); });
$("clawSave").addEventListener("click", function () { call("/api/claw_save", {name: $("clawName").value.trim()}).then(done).catch(fail); });
$("write").addEventListener("click", function () { call("/api/write", {}).then(done).catch(fail); });
$("list").addEventListener("click", function () {
  call("/api/list").then(function (d) { $("listing").textContent = d.text; }).catch(fail);
});
call("/api/state").then(function (s) { state = s; render(); }).catch(fail);
</script>
</body></html>
"""


class CalibratorWeb(http.server.BaseHTTPRequestHandler):
    cal = None          # set by make_server
    lock = None         # one request at a time touches the calibrator

    def log_message(self, fmt, *args):   # keep the console quiet
        pass

    def _send(self, code, data, ctype):
        body = data.encode("utf-8") if isinstance(data, str) else data
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj), "application/json")

    def _state(self):
        cal = self.cal
        return {
            "motors": list(cal.current),
            "claw": cal.claw,
            "move_ms": cal.move_ms,
            "poses": cal.poses,
            "claw_poses": cal.claw_poses,
            "builtin_poses": sorted(sg.TAIL_POSES),
            "builtin_claw": sorted(sg.TAIL_CLAW_POSES),
            "dirty": cal.dirty,
        }

    def do_GET(self):
        if self.path == "/":
            return self._send(200, PAGE, "text/html; charset=utf-8")
        if self.path == "/api/state":
            with self.lock:
                return self._json(200, self._state())
        if self.path == "/api/list":
            with self.lock:
                return self._json(200, {"text": self.cal.list_text()})
        return self._json(404, {"error": "not found"})

    def _action(self, body):
        cal, path = self.cal, self.path
        if path == "/api/motor":
            return cal.set_motor(parse_motor(body.get("motor")), parse_angle(body.get("angle")))
        if path == "/api/time":
            return cal.set_time(str(body.get("ms")))
        if path == "/api/save":
            return cal.save(str(body.get("name", "")))
        if path == "/api/pose":
            return cal.pose(str(body.get("name", "")))
        if path == "/api/claw":
            return cal.claw_command(str(body.get("name", "")))
        if path == "/api/claw_save":
            return cal.claw_save(str(body.get("name", "")))
        if path == "/api/write":
            return cal.write()
        return None

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b""
        try:
            body = json.loads(raw or b"{}")
        except ValueError:
            return self._json(400, {"error": "request was not JSON"})
        if not isinstance(body, dict):
            return self._json(400, {"error": "request must be a JSON object"})
        try:
            with self.lock:
                message = self._action(body)
                if message is None:
                    return self._json(404, {"error": "no such action"})
                return self._json(200, {"state": self._state(), "message": message})
        except CalibrationError as exc:
            return self._json(400, {"error": str(exc)})


def make_server(cal, host="0.0.0.0", port=8010):
    handler = type("TailCalibratorHandler", (CalibratorWeb,), {"cal": cal, "lock": threading.Lock()})
    server = http.server.ThreadingHTTPServer((host, port), handler)
    server.daemon_threads = True
    return server


def serve_web(cal, host, port):
    server = make_server(cal, host, port)
    print("Tail calibrator page: http://localhost:%d" % port)
    print("From another computer on the same network, use the Pi's address: http://<pi-address>:%d" % port)
    print("Press Ctrl+C here to stop. The servos hold their last pose.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping the page.")
    finally:
        server.server_close()
        if cal.dirty:
            print(cal.write())
    return 0


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
    parser.add_argument("--web", action="store_true", help="browser page with a slider for each motor")
    parser.add_argument("--host", default="0.0.0.0", help="address for the --web page (default all interfaces)")
    parser.add_argument("--port", type=int, default=8010, help="port for the --web page (default 8010)")
    args = parser.parse_args(argv)
    if not MIN_MOVE_MS <= args.move_ms <= MAX_MOVE_MS:
        print("--move-ms must be %d to %d" % (MIN_MOVE_MS, MAX_MOVE_MS))
        return 2

    if not args.hardware:
        print("DRY RUN: nothing is sent to the servos. Use --hardware on the Raspberry Pi.")
        cal = TailCalibrator(sg.DryRunBus(), args.move_ms, args.out)
        return serve_web(cal, args.host, args.port) if args.web else run_loop(cal)

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
        if args.web:
            return serve_web(cal, args.host, args.port)
        return run_loop(cal)
    except KeyboardInterrupt:
        print("\nInterrupted. The servos hold their last pose.")
        return 1
    except sg.GaitError as exc:
        print("\nStopped: %s" % exc)
        return 1


if __name__ == "__main__":
    sys.exit(main())
