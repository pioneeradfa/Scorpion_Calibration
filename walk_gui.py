#!/usr/bin/env python3
#=================================================================================
# Visual control panel for the scorpion walk: pick the gait, speed, steps, cycles,
# stride, lift and direction, see the plan and the gait pattern, then walk.
#
# It runs a small web server (Python standard library only, nothing to install) and
# you open the page in a browser. On the Raspberry Pi:
#     python3 walk_gui.py                  # page at http://localhost:8000 (this machine only)
#     python3 walk_gui.py --host 0.0.0.0   # also reachable from another computer on the network
#
# Everything except "Walk on robot" is a dry run: the page only shows the plan.
# "Walk on robot" needs the box ticked AND the word YES typed (yes/y in any case).
# Anyone who can reach the page can move the robot, so keep --host at 127.0.0.1
# unless you need the network access.
#
# Sequence on the robot (same as walk.py): initial stand (stand_initial), walking stand,
# glide into the first gait pose, the cycles, back to the walking stand.
# "Stop" ends the walk after the tick in progress, then returns to the walking stand.
#=================================================================================

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import scorpion_gait as sg
from walk import GLIDE_MS

DEFAULT_PORT = 8000
DEFAULT_TICK_MS = 200
DEFAULT_POSE_MS = 2000
MIN_TICK_PLAN_MS = 100        # plan (dry run) accepts this; the UI warns below SAFE_TICK_MS
SAFE_TICK_MS = 180            # serial link: ~180 ms per 18-servo update (see gait_ik_notes.md)
MAX_TICK_MS = 1000
MAX_CYCLES = 50
MAX_STEPS = 120
MAX_STRIDE = 120.0            # mm
MAX_LIFT = 80.0               # mm
DIRECTIONS = ("forward", "backward", "left", "right")
DIRECTION_LABEL = {"forward": "forward", "backward": "backward",
                   "left": "turn left in place", "right": "turn right in place"}

GAIT_INFO = {
    "tripod": "3 legs in the air at once. Fastest, least stable.",
    "ripple_123456": "1 leg at a time, order LF RF LM RM LR RR. Slow, very stable.",
    "ripple_654321": "1 leg at a time, order RR LR RM LM RF LF. Slow, very stable.",
    "ripple_135246": "1 leg at a time, order LF LM LR RF RM RR. Slow, very stable.",
    "double_ripple": "2 legs at a time, in pairs. Middle ground.",
    "caterpillar": "1 leg at a time, a wave along the body (LR LM LF, then RF RM RR).",
}


class ParamError(ValueError):
    pass


def _number(data, key, default, lo, hi, cast=float):
    raw = data.get(key, default)
    try:
        value = cast(raw)
    except (TypeError, ValueError):
        raise ParamError("%s must be a number (got %r)" % (key, raw))
    if not (lo <= value <= hi):
        raise ParamError("%s must be between %s and %s (got %s)" % (key, lo, hi, value))
    return value


def parse_params(data):
    """Validates the request. Returns a clean dict. Raises ParamError with a readable message."""
    gait = data.get("gait", "tripod")
    if gait not in sg.GAIT_SWING_FRACTION:
        raise ParamError("unknown gait %r" % gait)
    steps = _number(data, "steps", sg.GAIT_DEFAULT_STEPS[gait], 6, MAX_STEPS, int)
    if steps % 6 != 0:
        raise ParamError("steps must be a multiple of 6 (got %d)" % steps)
    direction = data.get("direction", "forward")
    if direction not in DIRECTIONS:
        raise ParamError("direction must be one of %s" % ", ".join(DIRECTIONS))
    pose = data.get("pose", sg.DEFAULT_POSE)
    if pose not in sg.POSE_NAMES:
        raise ParamError("unknown walking stand %r" % pose)
    legs = data.get("legs")
    if legs is None:
        legs = list(sg.LEG_ORDER)
    if not isinstance(legs, list):
        raise ParamError("legs must be a list")
    bad = [n for n in legs if n not in sg.LEG_ORDER]
    if bad:
        raise ParamError("unknown leg(s): %s" % ", ".join(map(str, bad)))
    return {
        "gait": gait,
        "steps": steps,
        "tick_ms": _number(data, "tick_ms", DEFAULT_TICK_MS, MIN_TICK_PLAN_MS, MAX_TICK_MS, int),
        "cycles": _number(data, "cycles", 1, 1, MAX_CYCLES, int),
        "stride": _number(data, "stride", sg.DEFAULT_STRIDE, 0.0, MAX_STRIDE),
        "lift": _number(data, "lift", sg.DEFAULT_LIFT, 0.0, MAX_LIFT),
        "direction": direction,
        "pose": pose,
        "pose_ms": _number(data, "pose_ms", DEFAULT_POSE_MS, 500, 6000, int),
        "legs": legs,
    }


def build_gait(p):
    gait = sg.Gait(p["gait"], steps=p["steps"], stride=p["stride"], lift=p["lift"])
    if p["direction"] == "left":
        gait.set_turn_in_place("CCW")
    elif p["direction"] == "right":
        gait.set_turn_in_place("CW")
    else:
        gait.set_straight(p["direction"] == "forward")
    return gait


def plan(p):
    """Dry run: summary, safety check and one cycle of frames for the picture. Sends nothing.
    Call with the lock held (it sets the active walking stand)."""
    sg.set_active_pose(p["pose"])
    try:
        gait = build_gait(p)
    except ValueError as exc:
        raise ParamError(str(exc))
    problems = gait.check(cycles=1)
    result = {
        "params": p,
        "gait_info": GAIT_INFO[p["gait"]],
        "legs": list(sg.LEG_ORDER),
        "mounts": {n: list(sg.MOUNT[n]) for n in sg.LEG_ORDER},
        "check": problems,
        "ok": not problems,
    }
    if problems:
        result["error"] = "Simulation check failed. Not walking."
        return result

    cycle_s = p["steps"] * p["tick_ms"] / 1000.0
    turning = p["direction"] in ("left", "right")
    result["summary"] = {
        "steps": p["steps"],
        "tick_ms": p["tick_ms"],
        "cycle_s": cycle_s,
        "total_s": cycle_s * p["cycles"],
        "direction": DIRECTION_LABEL[p["direction"]],
        "body_speed_mm_s": None if turning else p["stride"] / cycle_s,
        "worst_joint_step_deg": gait.worst_joint_step(cycles=1),
        "tick_warning": p["tick_ms"] < SAFE_TICK_MS,
        "legs_moved": len(p["legs"]),
    }
    frames = []
    for t in range(gait.steps):
        targets = gait.foot_targets(t)
        joints = gait.joints_at(t)
        frames.append({
            "tick": t,
            "air": [n for n in sg.LEG_ORDER if gait._is_swing(n, t)],
            "feet": {n: [round(v, 1) for v in targets[n]] for n in sg.LEG_ORDER},
            "joints": {n: [round(v, 1) for v in joints[n]] for n in sg.LEG_ORDER},
        })
    result["frames"] = frames
    return result


#---------------------------------------------------------------------------------
# Walking on the robot (one walk at a time, stoppable)

# Guards the global walking-stand pose. A plan may only change it while no walk is running.
POSE_LOCK = threading.Lock()


class Walker:
    """Runs one hardware walk in a background thread and keeps its status for the page."""

    def __init__(self, bus_factory=sg.ServoControlBus):
        self._bus_factory = bus_factory
        self._lock = threading.Lock()           # guards the status fields
        self._stop = threading.Event()
        self._thread = None
        self.state = "idle"                     # idle | running | stopping | done | error
        self.stage = ""
        self.cycle = 0
        self.tick = 0
        self.message = ""
        self.log = []

    @property
    def busy(self):
        return self._thread is not None and self._thread.is_alive()

    def _note(self, text):
        self.log.append("%s  %s" % (time.strftime("%H:%M:%S"), text))
        del self.log[:-40]

    def status(self):
        with self._lock:
            return {"state": self.state, "stage": self.stage, "cycle": self.cycle,
                    "tick": self.tick, "message": self.message, "log": list(self.log),
                    "busy": self.busy}

    def start(self, p, confirm):
        if confirm is None or str(confirm).strip().lower() not in ("yes", "y"):
            raise ParamError("Type YES to walk on the robot. Nothing was sent.")
        if p["tick_ms"] < SAFE_TICK_MS:
            raise ParamError("tick_ms must be at least %d for the robot (got %d)" % (SAFE_TICK_MS, p["tick_ms"]))
        if not p["legs"]:
            raise ParamError("select at least one leg")
        legs = None if len(p["legs"]) == len(sg.LEG_ORDER) else list(p["legs"])
        with POSE_LOCK:
            if self.busy:
                raise ParamError("a walk is already running")
            sg.set_active_pose(p["pose"])
            gait = build_gait(p)
            problems = gait.check(cycles=1)
            if problems:
                raise ParamError("simulation check failed: " + problems[0])
            try:
                bus = self._bus_factory()
            except ImportError as exc:
                raise ParamError(str(exc))
            self._stop.clear()
            with self._lock:
                self.state, self.stage, self.cycle, self.tick, self.message = "running", "starting", 0, 0, ""
            self._note("walk %s, %d cycle(s), tick %d ms, legs %s, walking stand %s"
                       % (p["gait"], p["cycles"], p["tick_ms"], legs or "all", p["pose"]))
            self._thread = threading.Thread(target=self._run, args=(p, gait, bus, legs), daemon=True)
            self._thread.start()
        return self.status()

    def stop(self):
        with self._lock:
            if self.busy:
                self.state = "stopping"
                self._note("stop requested")
        self._stop.set()
        return self.status()

    def _set(self, **kw):
        with self._lock:
            for k, v in kw.items():
                setattr(self, k, v)

    def _check_stop(self):
        if self._stop.is_set():
            raise sg.GaitError("stopped by user")

    def _run(self, p, gait, bus, legs):
        error = ""
        try:
            self._set(stage="initial stand")
            self._check_stop()
            sg.move_to_pose(bus, sg.START_POSE, move_ms=p["pose_ms"], legs=legs)
            self._note("at initial stand '%s'" % sg.START_POSE)

            if p["pose"] != sg.START_POSE:
                self._set(stage="walking stand")
                self._check_stop()
                sg.move_to_pose(bus, p["pose"], move_ms=p["pose_ms"], legs=legs)
                self._note("at walking stand '%s'" % p["pose"])

            self._set(stage="glide to first pose")
            self._check_stop()
            bus.send(gait.joints_at(0), move_ms=GLIDE_MS, legs=legs)
            time.sleep(GLIDE_MS / 1000.0)

            self._set(stage="walking")
            count = [0]   # ticks sent so far; gait.tick wraps every cycle, so count separately

            def on_tick(tick, _joints):
                count[0] += 1
                self._set(cycle=(count[0] - 1) // gait.steps + 1, tick=tick)
                self._check_stop()

            sg.walk(gait, bus, cycles=p["cycles"], move_ms=p["tick_ms"], legs=legs, on_tick=on_tick)
            self._note("walk done: %d cycle(s)" % p["cycles"])
        except sg.GaitError as exc:
            # a user Stop is not an error; any other gait failure is
            error = "" if self._stop.is_set() else str(exc)
            self._note(str(exc))
        except Exception as exc:  # noqa: BLE001  (report, never hide)
            error = "unexpected error: %s" % exc
            self._note(error)
        finally:
            self._set(stage="returning to walking stand")
            try:
                sg.move_to_pose(bus, p["pose"], move_ms=p["pose_ms"], legs=legs)
                self._note("back at walking stand '%s'" % p["pose"])
            except Exception as exc:  # noqa: BLE001
                error = (error + "; " if error else "") + "could not return to walking stand: %s" % exc
                self._note(error)
            with self._lock:
                self.stage = ""
                if error:
                    self.state, self.message = "error", error
                elif self._stop.is_set():
                    self.state, self.message = "stopped", "stopped by user; back at walking stand"
                else:
                    self.state, self.message = "done", "walk finished"


#---------------------------------------------------------------------------------
# Web server

WALKER = Walker()


def config_payload():
    return {
        "gaits": [{"id": g, "default_steps": sg.GAIT_DEFAULT_STEPS[g], "info": GAIT_INFO[g]}
                  for g in sorted(sg.GAIT_SWING_FRACTION)],
        "poses": sg.POSE_NAMES,
        "default_pose": sg.DEFAULT_POSE,
        "start_pose": sg.START_POSE,
        "legs": list(sg.LEG_ORDER),
        "defaults": {"tick_ms": DEFAULT_TICK_MS, "stride": sg.DEFAULT_STRIDE, "lift": sg.DEFAULT_LIFT,
                     "pose_ms": DEFAULT_POSE_MS, "cycles": 1},
        "limits": {"tick_min": MIN_TICK_PLAN_MS, "safe_tick_min": SAFE_TICK_MS, "tick_max": MAX_TICK_MS,
                   "cycles_max": MAX_CYCLES, "steps_max": MAX_STEPS, "stride_max": MAX_STRIDE,
                   "lift_max": MAX_LIFT},
        "directions": list(DIRECTIONS),
    }


class Handler(BaseHTTPRequestHandler):
    server_version = "ScorpionWalkGUI/1.0"

    def log_message(self, fmt, *args):  # keep the console quiet
        pass

    def _send(self, code, body, ctype="application/json"):
        data = body.encode("utf-8") if isinstance(body, str) else body
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _json(self, code, obj):
        self._send(code, json.dumps(obj))

    def _body(self):
        n = int(self.headers.get("Content-Length") or 0)
        if n <= 0:
            return {}
        try:
            data = json.loads(self.rfile.read(n).decode("utf-8") or "{}")
        except ValueError:
            raise ParamError("request is not valid JSON")
        if not isinstance(data, dict):
            raise ParamError("request must be a JSON object")
        return data

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/":
            self._send(200, PAGE, "text/html; charset=utf-8")
        elif path == "/api/config":
            self._json(200, config_payload())
        elif path == "/api/status":
            self._json(200, WALKER.status())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        try:
            data = self._body()
            if path == "/api/plan":
                with POSE_LOCK:
                    if WALKER.busy:
                        self._json(409, {"error": "A walk is running. Stop it first."})
                        return
                    result = plan(parse_params(data))
                self._json(200, result)
            elif path == "/api/walk":
                p = parse_params(data)
                self._json(200, WALKER.start(p, data.get("confirm")))
            elif path == "/api/stop":
                self._json(200, WALKER.stop())
            else:
                self._json(404, {"error": "not found"})
        except ParamError as exc:
            self._json(400, {"error": str(exc)})


def make_server(host, port):
    return ThreadingHTTPServer((host, port), Handler)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Browser control panel for the scorpion walk.")
    parser.add_argument("--host", default="127.0.0.1",
                        help="address to listen on (default 127.0.0.1 = this machine only; "
                             "0.0.0.0 = any computer on the network)")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT, help="port (default %d)" % DEFAULT_PORT)
    args = parser.parse_args(argv)
    server = make_server(args.host, args.port)
    print("Scorpion walk panel: http://%s:%d  (Ctrl+C to quit)" % (args.host, args.port))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping the server. A walk in progress is not interrupted; use Stop first.")
    finally:
        server.server_close()
    return 0


PAGE = r"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Scorpion walk</title>
<style>
  :root { --bg:#f5f6f8; --card:#fff; --ink:#1d2330; --muted:#6b7385; --line:#d9dde6;
          --stance:#2e9e6a; --swing:#e8793a; --accent:#2f6fdb; --bad:#c62828; --warn:#b26a00; }
  * { box-sizing: border-box; }
  body { margin:0; font:14px/1.4 system-ui, -apple-system, Segoe UI, Roboto, sans-serif; background:var(--bg); color:var(--ink); }
  header { padding:14px 20px; background:#1d2330; color:#fff; }
  header h1 { margin:0; font-size:18px; font-weight:600; }
  header small { color:#aab2c3; }
  main { display:grid; grid-template-columns: 330px 1fr; gap:16px; padding:16px; align-items:start; }
  .card { background:var(--card); border:1px solid var(--line); border-radius:10px; padding:14px; margin-bottom:14px; }
  .card h2 { font-size:13px; text-transform:uppercase; letter-spacing:.06em; color:var(--muted); margin:0 0 10px; }
  label { display:block; margin:10px 0 4px; font-weight:600; }
  select, input[type=number] { width:100%; padding:7px 8px; border:1px solid var(--line); border-radius:6px; font:inherit; background:#fff; }
  .hint { color:var(--muted); font-size:12px; margin-top:4px; min-height:1.2em; }
  .row { display:flex; gap:8px; align-items:center; }
  .row > * { flex:1; }
  .legs { display:grid; grid-template-columns:repeat(3,1fr); gap:6px; }
  .legs label { font-weight:500; margin:0; display:flex; gap:6px; align-items:center; }
  .seg { display:flex; flex-wrap:wrap; gap:6px; }
  .seg button { flex:1 1 auto; padding:6px 8px; border:1px solid var(--line); border-radius:6px; background:#fff; cursor:pointer; font:inherit; }
  .seg button.on { background:var(--accent); border-color:var(--accent); color:#fff; }
  .cards { display:grid; grid-template-columns:repeat(4,1fr); gap:10px; }
  .stat { border:1px solid var(--line); border-radius:8px; padding:10px; }
  .stat b { display:block; font-size:18px; }
  .stat span { color:var(--muted); font-size:12px; }
  .banner { padding:10px 12px; border-radius:8px; margin-bottom:12px; display:none; }
  .banner { white-space:pre-line; }
  .banner.err { display:block; background:#fdecea; color:var(--bad); }
  .banner.warn { display:block; background:#fff4e0; color:var(--warn); }
  .banner.ok { display:block; background:#e8f6ef; color:#1b6b45; }
  button.primary { width:100%; padding:10px; border:0; border-radius:8px; background:var(--accent); color:#fff; font:inherit; font-weight:600; cursor:pointer; }
  button.danger { width:100%; padding:10px; border:0; border-radius:8px; background:var(--bad); color:#fff; font:inherit; font-weight:600; cursor:pointer; }
  button:disabled { opacity:.45; cursor:not-allowed; }
  button.ghost { padding:6px 12px; border:1px solid var(--line); border-radius:6px; background:#fff; cursor:pointer; font:inherit; }
  .muted { color:var(--muted); }
  canvas { width:100%; display:block; background:#fbfcfd; border:1px solid var(--line); border-radius:8px; }
  .viz { display:grid; grid-template-columns: 1fr 1fr; gap:14px; }
  .scrub { display:flex; gap:10px; align-items:center; margin-top:8px; }
  .scrub input { flex:1; }
  table { width:100%; border-collapse:collapse; font-variant-numeric: tabular-nums; }
  th, td { padding:5px 8px; border-bottom:1px solid var(--line); text-align:right; font-size:13px; }
  th:first-child, td:first-child { text-align:left; }
  .air { color:var(--swing); font-weight:600; }
  .stance { color:var(--stance); }
  .log { font:12px/1.5 ui-monospace, Menlo, monospace; max-height:150px; overflow:auto; background:#fbfcfd; border:1px solid var(--line); border-radius:8px; padding:8px; white-space:pre-wrap; }
  .pill { display:inline-block; padding:2px 8px; border-radius:99px; font-size:12px; font-weight:600; }
  .pill.idle { background:#eef0f4; color:var(--muted); }
  .pill.running { background:#e3edff; color:var(--accent); }
  .pill.stopping { background:#fff4e0; color:var(--warn); }
  .pill.done { background:#e8f6ef; color:#1b6b45; }
  .pill.stopped { background:#fff4e0; color:var(--warn); }
  .pill.error { background:#fdecea; color:var(--bad); }
  @media (max-width: 960px) { main { grid-template-columns:1fr; } .viz { grid-template-columns:1fr; } .cards { grid-template-columns:repeat(2,1fr); } }
</style>
</head>
<body>
<header>
  <h1>Scorpion walk</h1>
  <small>Pick a gait, speed and steps. The plan updates as you change them. Only "Walk on robot" moves the servos.</small>
</header>
<main>
  <div>
    <div class="card">
      <h2>Gait</h2>
      <select id="gait"></select>
      <div class="hint" id="gaitInfo"></div>

      <label>Direction</label>
      <div class="seg" id="direction"></div>

      <label>Speed: <span id="speedLabel"></span></label>
      <input type="range" id="tick" min="100" max="500" step="10">
      <div class="hint" id="speedHint">Lower ms = faster. Each step takes this long.</div>

      <label for="steps">Steps per cycle</label>
      <select id="steps"></select>
      <div class="hint">Multiple of 6. More steps = slower gait with smaller joint moves.</div>

      <label for="cycles">Cycles (repeats of the whole gait)</label>
      <input type="number" id="cycles" min="1" max="50" step="1">
    </div>

    <div class="card">
      <h2>Body</h2>
      <div class="row">
        <div><label for="stride">Stride (mm)</label><input type="number" id="stride" min="0" max="120" step="1"></div>
        <div><label for="lift">Lift (mm)</label><input type="number" id="lift" min="0" max="80" step="1"></div>
      </div>
      <label for="pose">Walking stand</label>
      <select id="pose"></select>
      <div class="hint">The robot first moves to <b>stand_initial</b>, then to this stand.</div>
    </div>

    <div class="card">
      <h2>Legs (robot only)</h2>
      <div class="legs" id="legs"></div>
      <div class="hint">The dry run always shows all six legs.</div>
      <button class="ghost" id="resetLegs" style="margin-top:8px">All legs</button>
    </div>

    <div class="card">
      <h2>On the robot</h2>
      <label style="font-weight:500"><input type="checkbox" id="hw"> I want to move the real servos</label>
      <div class="hint">Robot propped up with the feet clear of the floor for the first tries.</div>
      <div style="margin-top:10px">
        <button class="primary" id="walk" disabled>Walk on robot…</button>
      </div>
      <div style="margin-top:8px">
        <button class="danger" id="stop" disabled>Stop (finish this tick, then stand)</button>
      </div>
      <div style="margin-top:12px"><span class="pill idle" id="state">idle</span>
        <span class="muted" id="stageText"></span></div>
      <div class="hint" id="walkMsg"></div>
      <div class="log" id="log" style="margin-top:8px"></div>
    </div>
  </div>

  <div>
    <div class="banner" id="banner"></div>
    <div class="card">
      <h2>Plan</h2>
      <div class="cards">
        <div class="stat"><b id="sCycle">-</b><span>seconds per cycle</span></div>
        <div class="stat"><b id="sTotal">-</b><span>seconds total</span></div>
        <div class="stat"><b id="sSpeed">-</b><span>body speed (mm/s)</span></div>
        <div class="stat"><b id="sJoint">-</b><span>fastest joint change (deg/tick)</span></div>
      </div>
      <div class="hint" id="planNote" style="margin-top:8px"></div>
    </div>

    <div class="card">
      <h2>Gait pattern (one cycle)</h2>
      <canvas id="timeline" height="200"></canvas>
      <div class="hint">Orange = leg in the air (swing). Green = foot on the ground (stance).</div>
    </div>

    <div class="viz">
      <div class="card">
        <h2>Top view</h2>
        <canvas id="top" height="360"></canvas>
        <div class="scrub">
          <button class="ghost" id="play">Play</button>
          <input type="range" id="scrub" min="0" max="0" value="0">
          <span class="muted" id="tickLabel">tick 0</span>
        </div>
        <div class="hint">Front of the body is at the top. Play runs at the selected speed.</div>
      </div>
      <div class="card">
        <h2>Joints at this tick (raw)</h2>
        <table>
          <thead><tr><th>Leg</th><th>State</th><th>Coxa</th><th>Femur</th><th>Tibia</th></tr></thead>
          <tbody id="jointRows"></tbody>
        </table>
        <div class="hint">Raw joint values as used for the servo commands (calibration units, 0-180 range).</div>
      </div>
    </div>
  </div>
</main>

<script>
const $ = (id) => document.getElementById(id);
let config = null;
let plan = null;          // last successful plan response
let planTimer = null;
let frame = 0;
let playing = false;
let playTimer = null;
let pollTimer = null;
let tickMs = 200;
let direction = "forward";
let legsOn = new Set();

const LEG_ORDER = ["LF", "RF", "LM", "RM", "LR", "RR"];
const LEG_NAME = {LF:"Left front", RF:"Right front", LM:"Left middle", RM:"Right middle", LR:"Left rear", RR:"Right rear"};
const DIRS = [["forward","Forward"],["backward","Backward"],["left","Turn left"],["right","Turn right"]];

function toast(text, kind) {
  const b = $("banner");
  if (!text) { b.className = "banner"; b.textContent = ""; return; }
  b.className = "banner " + kind;
  b.textContent = text;
}

async function api(path, body) {
  const opt = body === undefined ? {} : {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)};
  const r = await fetch(path, opt);
  let data = {};
  try { data = await r.json(); } catch (e) {}
  if (!r.ok) throw new Error(data.error || ("HTTP " + r.status));
  return data;
}

function params() {
  const legs = [...legsOn];
  return {
    gait: $("gait").value,
    steps: parseInt($("steps").value, 10),
    tick_ms: parseInt($("tick").value, 10),
    cycles: parseInt($("cycles").value, 10) || 1,
    stride: parseFloat($("stride").value) || 0,
    lift: parseFloat($("lift").value) || 0,
    direction: direction,
    pose: $("pose").value,
    pose_ms: config.defaults.pose_ms,
    legs: legs.length === 6 ? null : legs,
  };
}

function buildControls() {
  const gsel = $("gait");
  config.gaits.forEach(g => {
    const o = document.createElement("option"); o.value = g.id; o.textContent = g.id; gsel.appendChild(o);
  });
  gsel.onchange = () => {
    const g = config.gaits.find(x => x.id === gsel.value);
    $("steps").value = g.default_steps;
    showGaitInfo();
    schedulePlan();
  };

  const psel = $("pose");
  config.poses.forEach(p => {
    const o = document.createElement("option"); o.value = p; o.textContent = p; psel.appendChild(o);
  });
  psel.value = config.default_pose;
  psel.onchange = schedulePlan;

  const ssel = $("steps");
  for (let s = 6; s <= config.limits.steps_max; s += 6) {
    const o = document.createElement("option"); o.value = s; o.textContent = s; ssel.appendChild(o);
  }
  ssel.onchange = schedulePlan;

  const dwrap = $("direction");
  DIRS.forEach(([id, label]) => {
    const b = document.createElement("button");
    b.textContent = label; b.dataset.id = id;
    b.onclick = () => { direction = id; markDirection(); schedulePlan(); };
    dwrap.appendChild(b);
  });
  markDirection();

  const t = $("tick");
  t.min = config.limits.tick_min; t.max = config.limits.tick_max; t.step = 10;
  t.value = config.defaults.tick_ms;
  t.oninput = () => { showSpeed(); };
  t.onchange = schedulePlan;

  $("cycles").value = config.defaults.cycles;
  $("stride").value = config.defaults.stride;
  $("lift").value = config.defaults.lift;
  ["cycles", "stride", "lift"].forEach(id => { $(id).onchange = schedulePlan; });

  const lwrap = $("legs");
  LEG_ORDER.forEach(n => {
    const l = document.createElement("label");
    const c = document.createElement("input"); c.type = "checkbox"; c.checked = true; c.value = n;
    c.onchange = () => { c.checked ? legsOn.add(n) : legsOn.delete(n); updateWalkButton(); };
    l.appendChild(c); l.appendChild(document.createTextNode(n + " - " + LEG_NAME[n]));
    lwrap.appendChild(l);
    legsOn.add(n);
  });
  $("resetLegs").onclick = () => {
    lwrap.querySelectorAll("input").forEach(c => { c.checked = true; legsOn.add(c.value); });
    updateWalkButton();
  };

  $("gait").value = "tripod";
  $("steps").value = config.gaits.find(g => g.id === "tripod").default_steps;
  showGaitInfo();
  showSpeed();
}

function markDirection() {
  document.querySelectorAll("#direction button").forEach(b => b.classList.toggle("on", b.dataset.id === direction));
}

function showGaitInfo() {
  const g = config.gaits.find(x => x.id === $("gait").value);
  $("gaitInfo").textContent = g ? g.info : "";
}

function showSpeed() {
  const ms = parseInt($("tick").value, 10);
  tickMs = ms;
  $("speedLabel").textContent = ms + " ms per step";
  $("speedHint").textContent = ms < config.limits.safe_tick_min
    ? "Below " + config.limits.safe_tick_min + " ms the servo link cannot keep up. Robot walks are blocked."
    : "Lower ms = faster. Each step takes this long.";
  $("speedHint").style.color = ms < config.limits.safe_tick_min ? "var(--warn)" : "";
}

function schedulePlan() {
  clearTimeout(planTimer);
  planTimer = setTimeout(runPlan, 150);
}

async function runPlan() {
  const p = params();
  try {
    plan = await api("/api/plan", p);
    if (plan.ok === false || plan.error) {
      toast((plan.error || "Plan failed") + (plan.check && plan.check.length ? "\n" + plan.check.slice(0, 3).join("\n") : ""), "err");
      plan = null;
      drawEmpty();
    } else {
      toast("", "");
      renderPlan();
    }
  } catch (e) {
    toast(e.message, "err");
    plan = null;
    drawEmpty();
  }
  updateWalkButton();
}

function fmt(v, d) { return v === null || v === undefined ? "turn" : Number(v).toFixed(d); }

function renderPlan() {
  const s = plan.summary;
  $("sCycle").textContent = fmt(s.cycle_s, 1);
  $("sTotal").textContent = fmt(s.total_s, 1);
  $("sSpeed").textContent = s.body_speed_mm_s === null ? "turn" : fmt(s.body_speed_mm_s, 1);
  $("sJoint").textContent = fmt(s.worst_joint_step_deg, 1);
  $("planNote").textContent = plan.params.gait + ", " + s.steps + " steps, " + s.direction
    + ", " + s.legs_moved + " leg(s) on the robot, " + plan.params.cycles + " cycle(s).";
  if (s.tick_warning) toast("Speed is faster than the servo link can keep up with (" + s.tick_ms + " ms). Robot walks are blocked below " + config.limits.safe_tick_min + " ms.", "warn");
  frame = Math.min(frame, s.steps - 1);
  $("scrub").max = s.steps - 1;
  $("scrub").value = frame;
  drawAll();
}

function drawEmpty() {
  ["sCycle","sTotal","sSpeed","sJoint"].forEach(id => $(id).textContent = "-");
  $("planNote").textContent = "";
  const c = $("timeline"), t = $("top");
  [c, t].forEach(cv => { const g = cv.getContext("2d"); g.clearRect(0, 0, cv.width, cv.height); });
  $("jointRows").innerHTML = "";
}

function sizeCanvas(cv) {
  const dpr = window.devicePixelRatio || 1;
  const w = cv.clientWidth, h = cv.clientHeight || parseInt(cv.getAttribute("height"), 10);
  if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) {
    cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr);
  }
  const g = cv.getContext("2d");
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  return {g, w, h};
}

function drawAll() {
  if (!plan) return;
  drawTimeline();
  drawTop();
  drawJoints();
  $("tickLabel").textContent = "tick " + frame + " / " + (plan.summary.steps - 1);
}

function drawTimeline() {
  const {g, w, h} = sizeCanvas($("timeline"));
  g.clearRect(0, 0, w, h);
  const steps = plan.summary.steps;
  const left = 70, top = 10, rowH = (h - 24) / 6;
  const cellW = (w - left - 10) / steps;
  g.font = "12px system-ui"; g.textBaseline = "middle";
  LEG_ORDER.forEach((n, r) => {
    const y = top + r * rowH;
    g.fillStyle = "#1d2330"; g.textAlign = "left";
    g.fillText(n, 8, y + rowH / 2);
    for (let t = 0; t < steps; t++) {
      const air = plan.frames[t].air.includes(n);
      g.fillStyle = air ? "#e8793a" : "#2e9e6a";
      g.fillRect(left + t * cellW + 1, y + 3, cellW - 2, rowH - 6);
    }
  });
  const x = left + frame * cellW;
  g.strokeStyle = "#1d2330"; g.lineWidth = 2;
  g.beginPath(); g.moveTo(x + cellW / 2, top); g.lineTo(x + cellW / 2, top + 6 * rowH); g.stroke();
  g.fillStyle = "#6b7385"; g.textAlign = "center";
  for (let t = 0; t < steps; t += Math.max(1, Math.round(steps / 12))) {
    g.fillText(String(t), left + t * cellW + cellW / 2, h - 8);
  }
}

function drawTop() {
  const {g, w, h} = sizeCanvas($("top"));
  g.clearRect(0, 0, w, h);
  // scale so the widest reach fits
  let maxR = 0;
  plan.frames.forEach(f => LEG_ORDER.forEach(n => {
    const [x, y] = f.feet[n]; maxR = Math.max(maxR, Math.abs(x), Math.abs(y));
  }));
  const cx = w / 2, cy = h / 2;
  const s = Math.min(w, h) / 2 / (maxR * 1.25 || 1);
  const X = (y) => cx - y * s;     // body Y (left) -> screen left
  const Y = (x) => cy - x * s;     // body X (forward) -> screen up
  // body outline
  g.strokeStyle = "#c7cdd9"; g.lineWidth = 1.5; g.fillStyle = "#eef1f6";
  g.beginPath(); g.rect(X(60), Y(115), X(-60) - X(60), Y(-115) - Y(115)); g.fill(); g.stroke();
  g.fillStyle = "#6b7385"; g.textAlign = "center"; g.font = "12px system-ui"; g.textBaseline = "alphabetic";
  g.fillText("FRONT", cx, Y(115) - 6);
  // foot path over the whole cycle
  g.strokeStyle = "rgba(47,111,219,0.25)"; g.lineWidth = 1.2;
  LEG_ORDER.forEach(n => {
    g.beginPath();
    plan.frames.forEach((f, i) => {
      const [x, y] = f.feet[n];
      i === 0 ? g.moveTo(X(y), Y(x)) : g.lineTo(X(y), Y(x));
    });
    g.closePath(); g.stroke();
  });
  // mounts and current feet
  const f = plan.frames[frame];
  LEG_ORDER.forEach(n => {
    const [mx, my] = plan.mounts[n];
    g.fillStyle = "#1d2330";
    g.beginPath(); g.arc(X(my), Y(mx), 3, 0, 6.283); g.fill();
    const [x, y] = f.feet[n];
    const air = f.air.includes(n);
    g.strokeStyle = "#999"; g.lineWidth = 1;
    g.beginPath(); g.moveTo(X(my), Y(mx)); g.lineTo(X(y), Y(x)); g.stroke();
    g.fillStyle = air ? "#e8793a" : "#2e9e6a";
    g.beginPath(); g.arc(X(y), Y(x), 8, 0, 6.283); g.fill();
    g.fillStyle = "#1d2330"; g.font = "bold 11px system-ui"; g.textAlign = "center"; g.textBaseline = "middle";
    g.fillText(n, X(y), Y(x) - 14);
  });
  g.fillStyle = "#6b7385"; g.font = "12px system-ui"; g.textBaseline = "alphabetic"; g.textAlign = "left";
  g.fillText("orange = in the air, green = on the ground", 10, h - 10);
}

function drawJoints() {
  const f = plan.frames[frame];
  const rows = LEG_ORDER.map(n => {
    const air = f.air.includes(n);
    const j = f.joints[n];
    return '<tr><td>' + n + '</td><td class="' + (air ? "air" : "stance") + '">' + (air ? "in air" : "ground")
      + '</td><td>' + j[0].toFixed(1) + '</td><td>' + j[1].toFixed(1) + '</td><td>' + j[2].toFixed(1) + '</td></tr>';
  });
  $("jointRows").innerHTML = rows.join("");
}

function setFrame(v) {
  frame = v;
  $("scrub").value = v;
  if (plan) drawAll();
}

function togglePlay() {
  if (!plan) return;
  playing = !playing;
  $("play").textContent = playing ? "Pause" : "Play";
  clearInterval(playTimer);
  if (playing) {
    playTimer = setInterval(() => setFrame((frame + 1) % plan.summary.steps), tickMs);
  }
}

function updateWalkButton() {
  const hw = $("hw").checked;
  const busy = walking;
  const ok = plan && plan.ok && hw && !busy && legsOn.size > 0 && tickMs >= config.limits.safe_tick_min;
  $("walk").disabled = !ok;
  $("stop").disabled = !busy;
}

let walking = false;

async function walkOnRobot() {
  const p = params();
  const ok = confirm("Walk the REAL robot?\n\nGait: " + p.gait + "\nCycles: " + p.cycles
    + "\nSpeed: " + p.tick_ms + " ms per step\nLegs: " + (p.legs ? p.legs.join(", ") : "all")
    + "\n\nThe robot first moves to stand_initial, then to " + p.pose + ".");
  if (!ok) return;
  const word = prompt("Type YES to move the servos (yes or y in any case also works):");
  if (word === null) return;
  try {
    const st = await api("/api/walk", Object.assign({}, p, {confirm: word}));
    applyStatus(st);
    startPolling();
  } catch (e) {
    toast(e.message, "err");
  }
}

async function stopWalk() {
  try { applyStatus(await api("/api/stop", {})); } catch (e) { toast(e.message, "err"); }
}

function applyStatus(st) {
  walking = st.busy;
  const pill = $("state");
  pill.className = "pill " + st.state;
  pill.textContent = st.state;
  let stage = st.stage ? st.stage : "";
  if (st.state === "running" && st.stage === "walking") stage = "walking, cycle " + Math.max(1, st.cycle) + ", tick " + st.tick;
  $("stageText").textContent = stage;
  $("walkMsg").textContent = st.message || "";
  const log = $("log");
  log.textContent = st.log.join("\n");
  log.scrollTop = log.scrollHeight;
  updateWalkButton();
  return st;
}

function startPolling() {
  clearInterval(pollTimer);
  pollTimer = setInterval(async () => {
    try {
      const st = applyStatus(await api("/api/status"));
      if (!st.busy) { clearInterval(pollTimer); pollTimer = null; }
    } catch (e) { /* server restarting; keep trying */ }
  }, 500);
}

async function init() {
  config = await api("/api/config");
  buildControls();
  $("hw").onchange = updateWalkButton;
  $("walk").onclick = walkOnRobot;
  $("stop").onclick = stopWalk;
  $("play").onclick = togglePlay;
  $("scrub").oninput = (e) => { playing = false; $("play").textContent = "Play"; clearInterval(playTimer); setFrame(parseInt(e.target.value, 10)); };
  window.addEventListener("resize", () => { if (plan) drawAll(); });
  const st = applyStatus(await api("/api/status"));
  if (st.busy) startPolling();
  runPlan();
}

init().catch(e => toast("Could not load the page: " + e.message, "err"));
</script>
</body>
</html>
"""


if __name__ == "__main__":
    sys.exit(main())
