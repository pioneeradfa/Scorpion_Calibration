#!/usr/bin/env python3
#=================================================================================
# Walk panel with obstacle avoidance from the top-mounted LiDAR (LIDAR D300 DTOF developer kit,
# the LD19 protocol: UART 230400 baud, 47-byte packets, 360 degrees, about 10 rotations per second).
#
# This is walk_gui.py with the LiDAR added. Everything in walk_gui stays the same (gait, speed,
# steps, cycles, direction, ICC curves, plan, gait picture, Stop, walking stand). New:
#   - LiDAR decoder (packet format and CRC-8 from the LD19 protocol),
#   - an obstacle check before every servo tick, against the robot's real footprint,
#   - a LiDAR card on the page: live scan, zones, stop/warn distances, health, optional turn-away,
#   - a simulated LiDAR (--simulate-lidar) with a box you can move, for the preview and the tests.
#
# Safety rules (always on):
#   - No walk starts unless the LiDAR is sending fresh data, and the way is clear.
#   - During a walk, a blocked zone or LiDAR data older than 0.5 s stops the walk and the
#     robot returns to the walking stand.
#   - Simulated LiDAR never lets the robot walk.
# Turn-away (optional, off by default, forward and backward walks only): the walk stops at the
# walking stand, the robot turns in place away from the obstacle, stands again and checks the
# path before walking on. At most MAX_AVOID_TURNS times per walk.
#
# Usage
#   python3 walk_lidar.py --simulate-lidar                          # no hardware: simulated scan, no walking
#   python3 walk_lidar.py --lidar-port /dev/ttyUSB0 --host 0.0.0.0   # the real LiDAR on the Pi
#   python3 walk_lidar.py --lidar-port /dev/ttyUSB0 --lidar-check 5  # print what the LiDAR sees, then exit
#=================================================================================

import argparse
import collections
import json
import math
import struct
import sys
import threading
import time
from http.server import ThreadingHTTPServer

import icc_turn as it
import scorpion_gait as sg
import walk_gui as wg
from walk import GLIDE_MS

#---------------------------------------------------------------------------------
# LD19 / D300 packet (47 bytes, little endian)
#   0 header 0x54 | 1 VerLen 0x2C | 2-3 speed deg/s | 4-5 start angle (0.01 deg)
#   6-41 12 x (distance uint16 mm, intensity uint8) | 42-43 end angle (0.01 deg)
#   44-45 timestamp ms | 46 CRC-8 over bytes 0-45 (poly 0x4D, init 0, no reflection, no final xor)
# The CRC parameters come from the LD19 protocol sources (not yet checked on the D300 hardware).

PACKET_LEN = 47
HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACKET = 12
CRC_POLY = 0x4D
BAUD = 230400

MAX_RANGE_MM = 12000
MIN_RANGE_MM = 150        # closer than this to the LiDAR is the robot itself (mast, body): ignored
SCAN_WINDOW_S = 0.3       # scan points older than this are dropped (about 3 rotations)
STALE_S = 0.5             # no packet for this long: the LiDAR is not working, the walk stops
MARGIN_MM = 50            # clearance beside the legs
SPIN_MARGIN_MM = 60       # turning in place sweeps the legs: clearance beyond the footprint
MAX_AVOID_TURNS = 4       # per walk
AVOID_TURN_CYCLES = 3     # gait cycles per turn-away (about 55 degrees at the default stride)

# Robot footprint from the leg geometry (the leg feet reach further than the body)
FOOTPRINT_MM = max(math.hypot(sg.LEGS[n].neutral[0], sg.LEGS[n].neutral[1]) for n in sg.LEG_ORDER)
HALF_WIDTH_MM = max(abs(sg.LEGS[n].neutral[1]) for n in sg.LEG_ORDER) + MARGIN_MM
DEFAULT_STOP_MM = round(FOOTPRINT_MM + 150)
DEFAULT_WARN_MM = DEFAULT_STOP_MM + 250


#---------------------------------------------------------------------------------
# CRC-8 and packet decoding

def _crc_table():
    table = []
    for i in range(256):
        c = i
        for _ in range(8):
            c = (((c << 1) ^ CRC_POLY) if (c & 0x80) else (c << 1)) & 0xFF
        table.append(c)
    return table


_CRC_TABLE = _crc_table()


def crc8(data):
    c = 0
    for b in data:
        c = _CRC_TABLE[c ^ b]
    return c


def decode_packet(pkt):
    """47-byte packet -> list of (angle_deg, distance_mm, intensity). The CRC must be checked first."""
    start = struct.unpack_from("<H", pkt, 4)[0] / 100.0
    end = struct.unpack_from("<H", pkt, 42)[0] / 100.0
    if end < start:
        end += 360.0
    step = (end - start) / (POINTS_PER_PACKET - 1)
    out = []
    for i in range(POINTS_PER_PACKET):
        dist, inten = struct.unpack_from("<HB", pkt, 6 + 3 * i)
        out.append(((start + step * i) % 360.0, dist, inten))
    return out


def encode_packet(start_deg, end_deg, readings, speed=2152, timestamp=0):
    """Builds a valid packet (simulator and tests). readings: 12 x (distance_mm, intensity)."""
    buf = bytearray([HEADER, VERLEN])
    buf += struct.pack("<H", speed)
    buf += struct.pack("<H", int(round(start_deg * 100)) % 36000)
    for dist, inten in readings:
        buf += struct.pack("<HB", int(dist), int(inten))
    buf += struct.pack("<H", int(round(end_deg * 100)) % 36000)
    buf += struct.pack("<H", timestamp)
    buf.append(crc8(buf))
    return bytes(buf)


class Ld19Parser:
    """Finds packets in a byte stream (any chunking). Counts good packets and CRC errors."""

    def __init__(self):
        self.buf = bytearray()
        self.packets = 0
        self.crc_errors = 0

    def feed(self, data):
        self.buf += data
        points = []
        while True:
            i = self.buf.find(bytes([HEADER]))
            if i < 0:
                self.buf.clear()
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < PACKET_LEN:
                break
            if self.buf[1] != VERLEN:
                del self.buf[0]
                continue
            pkt = bytes(self.buf[:PACKET_LEN])
            if crc8(pkt[:PACKET_LEN - 1]) != pkt[PACKET_LEN - 1]:
                self.crc_errors += 1
                del self.buf[0]
                continue
            del self.buf[:PACKET_LEN]
            self.packets += 1
            points.extend(decode_packet(pkt))
        return points


#---------------------------------------------------------------------------------
# Where the LiDAR sits on the robot

class Mount:
    """LiDAR position in the body frame (mm; +X forward, +Y left), its 0-degree offset and its direction.
    yaw_deg: the LiDAR's 0-degree mark measured from the robot's front (check with --lidar-check).
    direction: +1 if the LiDAR angle increases to the left (counter-clockwise), -1 if to the right."""

    def __init__(self, x=0.0, y=0.0, yaw_deg=0.0, direction=1):
        self.x = float(x)
        self.y = float(y)
        self.yaw_deg = float(yaw_deg)
        self.direction = 1 if direction >= 0 else -1


def to_body(mount, angle_deg, dist_mm):
    th = math.radians(mount.direction * angle_deg + mount.yaw_deg)
    return mount.x + dist_mm * math.cos(th), mount.y + dist_mm * math.sin(th)


#---------------------------------------------------------------------------------
# LiDAR data source: the real sensor, or a simulated one with a movable box

class LidarSource:
    simulated = False

    def __init__(self, mount):
        self.mount = mount
        self.error = None
        self._lock = threading.Lock()
        self._pts = collections.deque()           # (time, angle_deg, distance_mm), in the LiDAR frame
        self._packet_times = collections.deque(maxlen=400)
        self._packets = 0
        self._last_packet_t = None
        self.parser = Ld19Parser()

    def feed(self, data, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            points = self.parser.feed(data)
            new = self.parser.packets - self._packets
            if new > 0:
                self._packets = self.parser.packets
                self._last_packet_t = now
                self._packet_times.extend([now] * new)
            for angle, dist, _ in points:
                if 0 < dist <= MAX_RANGE_MM:
                    self._pts.append((now, angle, dist))
            self._prune(now)

    def _prune(self, now):
        while self._pts and self._pts[0][0] < now - SCAN_WINDOW_S:
            self._pts.popleft()

    def _frame_points(self, now):
        with self._lock:
            self._prune(now)
            return [(a, d) for _, a, d in self._pts]

    def body_points(self, now=None):
        """Recent scan points in the body frame: list of (x, y) in mm, robot's own parts removed."""
        now = time.monotonic() if now is None else now
        return [to_body(self.mount, a, d) for a, d in self._frame_points(now) if d >= MIN_RANGE_MM]

    def health(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            last = self._last_packet_t
            rate = sum(1 for t in self._packet_times if t >= now - 1.0)
            ok = last is not None and (now - last) < STALE_S and self.error is None
            return {"ok": ok, "simulated": False, "packets": self._packets,
                    "crc_errors": self.parser.crc_errors,
                    "age_ms": None if last is None else round(1000 * (now - last)),
                    "rate_hz": rate, "error": self.error}

    def open_serial(self, port, baud=BAUD):
        try:
            import serial  # pyserial (on Raspberry Pi OS: sudo apt install python3-serial)
        except ImportError:
            raise RuntimeError("pyserial is not installed. On the Pi run: sudo apt install python3-serial")
        ser = serial.Serial(port, baud, timeout=0.05)
        threading.Thread(target=self._read_loop, args=(ser,), daemon=True).start()

    def _read_loop(self, ser):
        try:
            while True:
                data = ser.read(512)
                if data:
                    self.feed(data)
        except Exception as exc:  # noqa: BLE001  (shown in the LiDAR card as an error)
            with self._lock:
                self.error = "serial read failed: %s" % exc


def _ray_box(ox, oy, dx, dy, x0, x1, y0, y1):
    """Distance along a ray to an axis-aligned box, or None."""
    tmin, tmax = 0.0, float("inf")
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-12:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        tmin, tmax = max(tmin, t1), min(tmax, t2)
        if tmin > tmax:
            return None
    return tmin if tmin > 0 else None


class SimSource(LidarSource):
    """Simulated scan of one 300 mm box obstacle (body frame). A simulated LiDAR never lets the robot walk."""
    simulated = True
    BOX_SIZE_MM = 300.0

    def __init__(self, mount, box_x=900.0, box_y=0.0):
        super().__init__(mount)
        self.box = [float(box_x), float(box_y)]

    def set_box(self, x, y):
        self.box = [float(x), float(y)]

    def health(self, now=None):
        return {"ok": True, "simulated": True, "packets": 0, "crc_errors": 0,
                "age_ms": 0, "rate_hz": 10, "error": None}

    def _frame_points(self, now):
        h = self.BOX_SIZE_MM / 2.0
        x0, x1 = self.box[0] - h, self.box[0] + h
        y0, y1 = self.box[1] - h, self.box[1] + h
        pts = []
        for a in range(360):
            th = math.radians(self.mount.direction * a + self.mount.yaw_deg)
            t = _ray_box(self.mount.x, self.mount.y, math.cos(th), math.sin(th), x0, x1, y0, y1)
            if t is not None and t <= MAX_RANGE_MM:
                pts.append((float(a), t))
        return pts


#---------------------------------------------------------------------------------
# Obstacle zones: which points matter for the walk that is about to happen

def travel_of(gait):
    """What the gait sweeps: ('spin', None) | ('straight', forward) | ('curve', (icc, radius, forward))."""
    radius = math.hypot(*gait.icc)
    if radius < 1.0:
        return ("spin", None)
    if radius >= it.TURN_RADIUS_LIMIT_MM:
        return ("straight", gait.direction == "CCW")
    # ICC at +Y with CCW (or -Y with CW) moves the body forward (same rule as set_straight)
    forward = (gait.direction == "CCW") == (gait.icc[1] > 0)
    return ("curve", (gait.icc, radius, forward))


def in_zone(x, y, travel):
    kind, arg = travel
    if kind == "spin":
        return True
    if kind == "straight":
        ahead = x if arg else -x
        return ahead > 0 and abs(y) < HALF_WIDTH_MM
    icc, radius, forward = arg
    ahead = x if forward else -x
    return ahead > -HALF_WIDTH_MM and abs(math.hypot(x - icc[0], y - icc[1]) - radius) < HALF_WIDTH_MM


def stop_for(travel, stop_mm):
    """A turn in place only sweeps the legs, so it needs only the spin clearance (not the full stop distance)."""
    if travel[0] == "spin":
        return min(stop_mm, FOOTPRINT_MM + SPIN_MARGIN_MM)
    return stop_mm


def zone_description(travel):
    kind, arg = travel
    if kind == "spin":
        return {"kind": "spin"}
    if kind == "straight":
        return {"kind": "straight", "forward": bool(arg)}
    icc, radius, forward = arg
    return {"kind": "curve", "icc": [icc[0], icc[1]], "radius": radius, "forward": forward}


def assess(points, travel, stop_mm, warn_mm):
    """Checks the zone for the points (body frame). status: clear | warn | blocked."""
    status = "clear"
    nearest = None
    for x, y in points:
        d = math.hypot(x, y)                       # distance from the body centre
        if d > warn_mm or not in_zone(x, y, travel):
            continue
        nearest = d if nearest is None else min(nearest, d)
        if d <= stop_mm:
            status = "blocked"
        elif status == "clear":
            status = "warn"
    return {"status": status, "nearest_mm": None if nearest is None else round(nearest)}


def clearer_side(points, stop_mm):
    """Which way to turn in place to get away from an obstacle ahead: 'left', 'right', or None if both sides are blocked."""
    left = [math.hypot(x, y) for x, y in points if x > 0 and y > 0]
    right = [math.hypot(x, y) for x, y in points if x > 0 and y < 0]
    l_min = min(left, default=float("inf"))
    r_min = min(right, default=float("inf"))
    if max(l_min, r_min) <= stop_mm:
        return None
    return "left" if l_min >= r_min else "right"


#---------------------------------------------------------------------------------
# Walking with the LiDAR gate

class SafetyStop(Exception):
    """Walk stopped by the LiDAR gate. kind: 'blocked' (obstacle) or 'stale' (no LiDAR data)."""

    def __init__(self, kind, text, points=None):
        super().__init__(text)
        self.kind = kind
        self.points = points or []      # the scan points that caused a 'blocked' stop (body frame)


class Settings:
    def __init__(self, stop_mm=DEFAULT_STOP_MM, warn_mm=DEFAULT_WARN_MM, avoid=False):
        self.stop_mm = float(stop_mm)
        self.warn_mm = float(warn_mm)
        self.avoid = bool(avoid)

    def as_dict(self):
        return {"stop_mm": self.stop_mm, "warn_mm": self.warn_mm, "avoid": self.avoid}


class LidarWalker(wg.Walker):
    """walk_gui.Walker plus the LiDAR gate before every tick and the optional turn-away."""

    def __init__(self, source, settings, bus_factory=sg.ServoControlBus):
        super().__init__(bus_factory=bus_factory)
        self.source = source
        self.settings = settings
        self.gait = None                           # the gait being walked (read by the LiDAR card)
        self.lidar_status = "unknown"
        self.lidar_nearest_mm = None

    def status(self):
        s = super().status()
        with self._lock:
            s["lidar_status"] = self.lidar_status
            s["lidar_nearest_mm"] = self.lidar_nearest_mm
        return s

    def _assess_gait(self, gait):
        now = time.monotonic()
        if not self.source.health(now)["ok"]:
            return {"status": "stale", "nearest_mm": None}
        travel = travel_of(gait)
        pts = self.source.body_points(now)
        a = assess(pts, travel, stop_for(travel, self.settings.stop_mm), self.settings.warn_mm)
        a["points"] = pts
        return a

    def _gate(self, gait):
        a = self._assess_gait(gait)
        self._set(lidar_status=a["status"], lidar_nearest_mm=a["nearest_mm"])
        if a["status"] == "stale":
            raise SafetyStop("stale", "LiDAR data is stale or missing: walk stopped")
        if a["status"] == "blocked":
            raise SafetyStop("blocked", "obstacle %d mm from the centre: walk stopped" % a["nearest_mm"],
                             points=a["points"])

    def start(self, p, confirm):
        if confirm is None or str(confirm).strip().lower() not in ("yes", "y"):
            raise wg.ParamError("Type YES to walk on the robot. Nothing was sent.")
        if self.source.simulated:
            raise wg.ParamError("the LiDAR is simulated, so the robot cannot walk. Start with --lidar-port.")
        if not self.source.health()["ok"]:
            raise wg.ParamError("LiDAR is not sending data (check the cable and the port). Walk not started.")
        with wg.POSE_LOCK:
            if self.busy:
                raise wg.ParamError("a walk is already running")
            sg.set_active_pose(p["pose"])
            gait = wg.build_gait(p)
            a = self._assess_gait(gait)
            self._set(lidar_status=a["status"], lidar_nearest_mm=a["nearest_mm"])
            if a["status"] == "blocked":
                raise wg.ParamError("obstacle %d mm from the centre: clear the way first" % a["nearest_mm"])
        return super().start(p, confirm)

    def _send_tick(self, gait, bus, p, legs):
        t0 = time.monotonic()
        self._check_stop()
        self._gate(gait)
        joints = gait.next_joints()
        bus.send(joints, p["tick_ms"], legs)
        self._set(tick=gait.tick)
        if not isinstance(bus, sg.DryRunBus):
            time.sleep(max(0.0, p["tick_ms"] / 1000.0 - (time.monotonic() - t0)))

    def _glide(self, gait, bus, legs):
        self._set(stage="glide to first pose")
        self._check_stop()
        self._gate(gait)
        bus.send(gait.joints_at(0), GLIDE_MS, legs)
        if not isinstance(bus, sg.DryRunBus):
            time.sleep(GLIDE_MS / 1000.0)

    def _turn_away(self, p, side, bus, legs):
        self._set(stage="turning %s away from the obstacle" % side)
        sg.move_to_pose(bus, p["pose"], move_ms=p["pose_ms"], legs=legs)
        spin = wg.build_gait(dict(p, direction=side))
        self._glide(spin, bus, legs)
        for _ in range(AVOID_TURN_CYCLES * spin.steps):
            self._send_tick(spin, bus, p, legs)
        sg.move_to_pose(bus, p["pose"], move_ms=p["pose_ms"], legs=legs)

    def _run(self, p, gait, bus, legs):
        self.gait = gait
        error = ""
        kind = "done"
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

            avoidable = self.settings.avoid and p["direction"] in ("forward", "backward")
            done = 0
            turns = 0
            need_glide = True
            while done < p["cycles"]:
                try:
                    if need_glide:
                        self._glide(gait, bus, legs)
                        need_glide = False
                    self._set(cycle=done + 1, stage="walking")
                    for _ in range(gait.steps):
                        self._send_tick(gait, bus, p, legs)
                    done += 1
                except SafetyStop as exc:
                    if not avoidable or exc.kind != "blocked" or turns >= MAX_AVOID_TURNS:
                        raise
                    side = clearer_side(exc.points, self.settings.stop_mm)
                    if side is None:
                        raise SafetyStop("blocked", "obstacle ahead and no clear side: walk stopped")
                    turns += 1
                    self._note("%s; turning %s (%d/%d)" % (exc, side, turns, MAX_AVOID_TURNS))
                    gait.tick = 0               # the partial cycle is dropped; the walk restarts from the stand
                    self._turn_away(p, side, bus, legs)
                    need_glide = True
            self._note("walk done: %d cycle(s)" % p["cycles"])
        except SafetyStop as exc:
            kind, error = "obstacle", str(exc)
            self._note(error)
        except sg.GaitError as exc:
            if self._stop.is_set():
                kind = "stopped"
            else:
                kind, error = "error", str(exc)
            self._note(str(exc))
        except Exception as exc:  # noqa: BLE001  (report, never hide)
            kind, error = "error", "unexpected error: %s" % exc
            self._note(error)
        finally:
            self._set(stage="returning to walking stand")
            try:
                sg.move_to_pose(bus, p["pose"], move_ms=p["pose_ms"], legs=legs)
                self._note("back at walking stand '%s'" % p["pose"])
            except Exception as exc:  # noqa: BLE001
                error = (error + "; " if error else "") + "could not return to walking stand: %s" % exc
                kind = "error"
                self._note(error)
            with self._lock:
                self.stage = ""
                if kind == "error":
                    self.state, self.message = "error", error
                elif kind == "obstacle":
                    self.state, self.message = "obstacle", error + ". Robot is back at the walking stand."
                elif kind == "stopped":
                    self.state, self.message = "stopped", "stopped by user; back at walking stand"
                else:
                    self.state, self.message = "done", "walk finished"


#---------------------------------------------------------------------------------
# The LiDAR card on the page (added to walk_gui's page)

LIDAR_CARD = r"""
<div class="card" id="lidarCard">
  <h2>LiDAR obstacle check <span class="pill idle" id="lidarPill">no data</span></h2>
  <div class="cards">
    <div class="stat"><b id="lPts">-</b><span>scan points (last 0.3 s)</span></div>
    <div class="stat"><b id="lRate">-</b><span>packets per second</span></div>
    <div class="stat"><b id="lCrc">-</b><span>CRC errors</span></div>
    <div class="stat"><b id="lNear">-</b><span>nearest obstacle in the way (mm)</span></div>
  </div>
  <canvas id="lCanvas" height="340" style="margin-top:12px;width:100%"></canvas>
  <div class="hint">Robot at the centre, front at the top. Red dots = obstacle in the way of the walk, grey = other points. Red dashed circle = stop distance, orange = warn distance. Shaded area = the path the body sweeps.</div>
  <div class="row" style="margin-top:10px">
    <div><label for="lStop">Stop distance (mm)</label><input type="number" id="lStop" min="150" max="3000" step="10"></div>
    <div><label for="lWarn">Warn distance (mm)</label><input type="number" id="lWarn" min="200" max="4000" step="10"></div>
  </div>
  <label class="checkrow"><input type="checkbox" id="lAvoid"> Turn away from obstacles (stop, turn, check again)</label>
  <div class="hint">Stopping is always on: a blocked way stops the walk even when turn-away is off. The distance is from the body centre; the legs reach about <span id="lFoot">-</span> mm.</div>
  <div class="simbox" id="simBox" style="display:none">
    <div class="hint">Simulated LiDAR: the test obstacle is a 300 mm box. Body frame, mm, front = +X, left = +Y. The robot cannot walk while the LiDAR is simulated.</div>
    <div class="row">
      <div><label for="simX">Obstacle X</label><input type="number" id="simX" step="10"></div>
      <div><label for="simY">Obstacle Y</label><input type="number" id="simY" step="10"></div>
    </div>
    <button class="ghost" id="simApply" style="margin-top:8px">Move obstacle</button>
  </div>
</div>
"""

LIDAR_CSS = r"""
<style>
  .pill.obstacle { background:#fdecea; color:#c62828; }
  .simbox { margin-top:10px; border-top:1px dashed #d9dde6; padding-top:8px; }
</style>
"""

LIDAR_JS = r"""
(() => {
  const $L = (id) => document.getElementById(id);
  let view = null;
  let settingsReady = false;

  async function call(path, body) {
    const opt = body === undefined ? {} : {method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(body)};
    const r = await fetch(path, opt);
    const data = await r.json().catch(() => ({}));
    if (!r.ok) throw new Error(data.error || ("HTTP " + r.status));
    return data;
  }

  function drawScan() {
    const cv = $L("lCanvas");
    const dpr = window.devicePixelRatio || 1;
    const w = cv.clientWidth, h = cv.clientHeight || 340;
    if (cv.width !== Math.round(w * dpr) || cv.height !== Math.round(h * dpr)) { cv.width = Math.round(w * dpr); cv.height = Math.round(h * dpr); }
    const g = cv.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.fillStyle = "#fbfcfd"; g.fillRect(0, 0, w, h);
    if (!view) return;
    const range = Math.max(view.warn_mm * 1.3, 800);
    const s = Math.min(w, h) / 2 / range;
    const cx = w / 2, cy = h / 2;
    const X = (y) => cx - y * s, Y = (x) => cy - x * s;      // +X forward = up, +Y left = left
    const z = view.zone, hw = view.half_width_mm;
    g.fillStyle = "rgba(198,40,40,0.08)";
    if (z.kind === "straight" && z.forward) g.fillRect(X(hw), Y(view.warn_mm), 2 * hw * s, view.warn_mm * s);
    else if (z.kind === "straight") g.fillRect(X(hw), cy, 2 * hw * s, view.warn_mm * s);
    else if (z.kind === "curve") {
      const ix = X(z.icc[1]), iy = Y(z.icc[0]);
      g.beginPath();
      g.arc(ix, iy, (z.radius + hw) * s, 0, 6.2832);
      g.arc(ix, iy, Math.max(0, z.radius - hw) * s, 0, 6.2832);
      g.fill("evenodd");
    } else if (z.kind === "spin") {
      g.beginPath(); g.arc(cx, cy, view.warn_mm * s, 0, 6.2832); g.fill();
    }
    g.setLineDash([6, 4]);
    g.strokeStyle = "#c62828"; g.beginPath(); g.arc(cx, cy, view.stop_mm * s, 0, 6.2832); g.stroke();
    g.strokeStyle = "#b26a00"; g.beginPath(); g.arc(cx, cy, view.warn_mm * s, 0, 6.2832); g.stroke();
    g.setLineDash([]);
    g.strokeStyle = "#1d2330"; g.beginPath(); g.arc(cx, cy, 6, 0, 6.2832); g.stroke();
    g.fillStyle = "#6b7385"; g.font = "12px system-ui"; g.textAlign = "center";
    g.fillText("FRONT", cx, 14);
    view.points.forEach(([x, y, inz]) => {
      g.fillStyle = inz ? "#c62828" : "#9aa3b5";
      g.fillRect(X(y) - 1.5, Y(x) - 1.5, 3, 3);
    });
    if (view.sim) {
      const b = view.sim;
      g.strokeStyle = "#2f6fdb"; g.lineWidth = 2;
      g.strokeRect(X(b[1] + 150), Y(b[0] + 150), 300 * s, 300 * s);
      g.lineWidth = 1;
    }
  }

  function renderStats(d) {
    const h = d.health;
    const pill = $L("lidarPill");
    if (h.simulated) { pill.className = "pill running"; pill.textContent = "simulated"; }
    else if (h.ok) { pill.className = "pill done"; pill.textContent = "receiving"; }
    else { pill.className = "pill error"; pill.textContent = h.error ? "error" : "no data"; }
    $L("lPts").textContent = d.points.length;
    $L("lRate").textContent = h.rate_hz;
    $L("lCrc").textContent = h.crc_errors;
    $L("lNear").textContent = d.nearest_mm === null ? "clear" : d.nearest_mm;
    $L("lFoot").textContent = Math.round(d.footprint_mm);
    $L("simBox").style.display = d.simulated ? "block" : "none";
    if (!settingsReady) {
      $L("lStop").value = d.settings.stop_mm;
      $L("lWarn").value = d.settings.warn_mm;
      $L("lAvoid").checked = d.settings.avoid;
      if (d.sim) { $L("simX").value = d.sim[0]; $L("simY").value = d.sim[1]; }
      settingsReady = true;
    }
  }

  async function poll() {
    try {
      view = await call("/api/lidar");
      renderStats(view);
      drawScan();
    } catch (e) { /* try again on the next tick */ }
    setTimeout(poll, 500);
  }

  async function saveSettings() {
    try {
      await call("/api/lidar/settings", {stop_mm: parseFloat($L("lStop").value), warn_mm: parseFloat($L("lWarn").value), avoid: $L("lAvoid").checked});
      settingsReady = false;
    } catch (e) { alert(e.message); }
  }

  ["lStop", "lWarn"].forEach(id => $L(id).addEventListener("change", saveSettings));
  $L("lAvoid").addEventListener("change", saveSettings);
  $L("simApply").addEventListener("click", async () => {
    try { await call("/api/lidar/sim", {x: parseFloat($L("simX").value), y: parseFloat($L("simY").value)}); }
    catch (e) { alert(e.message); }
  });
  poll();
})();
"""


def build_page(base):
    """walk_gui's page with the LiDAR card, styles and script added. Fails loudly if walk_gui's page changed."""
    page = base
    anchors = [
        ('<div class="banner" id="banner"></div>', LIDAR_CARD + '<div class="banner" id="banner"></div>'),
        ("</head>", LIDAR_CSS + "</head>"),
        ("</body>", "<script>" + LIDAR_JS + "</script></body>"),
    ]
    for anchor, new in anchors:
        if page.count(anchor) != 1:
            raise RuntimeError("walk_gui page changed: anchor %r not found once" % anchor)
        page = page.replace(anchor, new, 1)
    return page


BASE_PAGE = wg.PAGE
PAGE = build_page(BASE_PAGE)

# Latest plan settings from the page (for the LiDAR card's zone when idle)
LAST = {"params": None}


def current_travel():
    walker = STATE["walker"]
    with wg.POSE_LOCK:
        if walker.busy and walker.gait is not None:
            return travel_of(walker.gait)
        p = LAST["params"]
        if p is None:
            return ("straight", True)
        sg.set_active_pose(p["pose"])
        return travel_of(wg.build_gait(p))


def lidar_view():
    src, settings = STATE["source"], STATE["settings"]
    now = time.monotonic()
    try:
        travel = current_travel()
    except ValueError:
        travel = ("straight", True)
    pts = src.body_points(now)
    a = assess(pts, travel, stop_for(travel, settings.stop_mm), settings.warn_mm)
    shown = []
    for x, y in pts[:1500]:
        inz = in_zone(x, y, travel) and math.hypot(x, y) <= settings.warn_mm
        shown.append([round(x), round(y), 1 if inz else 0])
    return {
        "health": src.health(now), "points": shown, "zone": zone_description(travel),
        "nearest_mm": a["nearest_mm"], "status": a["status"], "settings": settings.as_dict(),
        "footprint_mm": FOOTPRINT_MM, "half_width_mm": HALF_WIDTH_MM,
        "stop_mm": settings.stop_mm, "warn_mm": settings.warn_mm,
        "simulated": src.simulated, "sim": src.box if src.simulated else None,
    }


class LidarHandler(wg.Handler):
    server_version = "ScorpionWalkLidar/1.0"

    def _body(self):
        # read once: the page's plan/walk routes and our routes both need the parsed body
        if not hasattr(self, "_cached_body"):
            self._cached_body = super()._body()
        return self._cached_body

    def do_GET(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/config":
            cfg = wg.config_payload()
            cfg["lidar"] = {"simulated": STATE["source"].simulated, "settings": STATE["settings"].as_dict(),
                            "footprint_mm": FOOTPRINT_MM}
            self._json(200, cfg)
        elif path == "/api/lidar":
            self._json(200, lidar_view())
        else:
            super().do_GET()

    def do_POST(self):
        path = self.path.split("?", 1)[0]
        if path == "/api/plan":
            try:
                LAST["params"] = wg.parse_params(self._body())
            except (wg.ParamError, ValueError):
                pass                                 # the base route reports the error
            super().do_POST()
            return
        if path not in ("/api/lidar/settings", "/api/lidar/sim"):
            super().do_POST()
            return
        try:
            data = self._body()
            if path == "/api/lidar/settings":
                s = STATE["settings"]
                stop = float(data.get("stop_mm", s.stop_mm))
                warn = float(data.get("warn_mm", s.warn_mm))
                if not (150 <= stop <= 3000):
                    raise wg.ParamError("stop distance must be 150-3000 mm")
                if not (stop < warn <= 4000):
                    raise wg.ParamError("warn distance must be above the stop distance and at most 4000 mm")
                s.stop_mm, s.warn_mm = stop, warn
                s.avoid = bool(data.get("avoid", s.avoid))
                self._json(200, s.as_dict())
            else:
                if not STATE["source"].simulated:
                    raise wg.ParamError("the LiDAR is real: the obstacle cannot be moved")
                x, y = float(data.get("x", 900)), float(data.get("y", 0))
                if not (300 <= x <= 5000 and -2500 <= y <= 2500):
                    raise wg.ParamError("obstacle must be within X 300..5000 and Y -2500..2500 mm")
                STATE["source"].set_box(x, y)
                self._json(200, {"box": STATE["source"].box})
        except (wg.ParamError, ValueError) as exc:
            self._json(400, {"error": str(exc)})


STATE = {"source": None, "settings": None, "walker": None}


def setup(source, settings=None, bus_factory=sg.ServoControlBus):
    """Connects the page and the walker to a LiDAR source (real or simulated). Used by main() and the tests."""
    STATE["source"] = source
    STATE["settings"] = settings or Settings()
    STATE["walker"] = LidarWalker(source, STATE["settings"], bus_factory=bus_factory)
    LAST["params"] = None
    wg.WALKER = STATE["walker"]                 # walk_gui's /api/walk, /api/stop and /api/status use it
    wg.PAGE = PAGE
    return STATE["walker"]


def make_server(host, port):
    return ThreadingHTTPServer((host, port), LidarHandler)


#---------------------------------------------------------------------------------
# LiDAR check (no servos): what the sensor sees, by direction from the robot's front

DIRECTIONS = ["front", "front-left", "left", "rear-left", "rear", "rear-right", "right", "front-right"]


def lidar_check(source, seconds):
    time.sleep(seconds)
    h = source.health()
    print("LiDAR: %s, packets %d, CRC errors %d, rate %d/s, last packet %s ms ago"
          % ("OK" if h["ok"] else "NO DATA", h["packets"], h["crc_errors"], h["rate_hz"], h["age_ms"]))
    if h.get("error"):
        print("error: %s" % h["error"])
    pts = source.body_points()
    print("points in the last %.1f s: %d" % (SCAN_WINDOW_S, len(pts)))
    best = {n: None for n in DIRECTIONS}
    for x, y in pts:
        ang = math.degrees(math.atan2(y, x)) % 360.0            # 0 = front, 90 = left
        name = DIRECTIONS[int(((ang + 22.5) % 360) // 45)]
        d = math.hypot(x, y)
        if best[name] is None or d < best[name]:
            best[name] = d
    print("nearest point by direction (from the body centre, mm):")
    for n in DIRECTIONS:
        print("  %-12s %s" % (n, "-" if best[n] is None else "%.0f" % best[n]))
    print("To calibrate the mount: put a box straight in front of the robot and run this again.")
    print("'front' should show the box. If it shows on another side, change --lidar-yaw or --lidar-dir.")
    return 0 if h["ok"] else 1


def main(argv=None):
    parser = argparse.ArgumentParser(description="Walk panel with LiDAR obstacle avoidance.")
    parser.add_argument("--host", default="127.0.0.1",
                        help="address to listen on (default 127.0.0.1 = this machine only; 0.0.0.0 = the network)")
    parser.add_argument("--port", type=int, default=wg.DEFAULT_PORT)
    parser.add_argument("--lidar-port", default="/dev/ttyUSB0", help="serial port of the LiDAR (default /dev/ttyUSB0)")
    parser.add_argument("--lidar-baud", type=int, default=BAUD)
    parser.add_argument("--lidar-x", type=float, default=0.0, help="LiDAR position, mm forward of the body centre")
    parser.add_argument("--lidar-y", type=float, default=0.0, help="LiDAR position, mm left of the body centre")
    parser.add_argument("--lidar-yaw", type=float, default=0.0,
                        help="LiDAR 0-degree mark measured from the robot front, degrees")
    parser.add_argument("--lidar-dir", type=int, default=1, choices=(1, -1),
                        help="1 = LiDAR angle increases to the left (default), -1 = to the right")
    parser.add_argument("--simulate-lidar", action="store_true",
                        help="no hardware: simulated scan with a movable box (the robot cannot walk)")
    parser.add_argument("--lidar-check", type=float, metavar="SECONDS",
                        help="print what the LiDAR sees for SECONDS, then exit")
    parser.add_argument("--stop-mm", type=float, default=DEFAULT_STOP_MM)
    parser.add_argument("--warn-mm", type=float, default=DEFAULT_WARN_MM)
    args = parser.parse_args(argv)

    mount = Mount(args.lidar_x, args.lidar_y, args.lidar_yaw, args.lidar_dir)
    if args.simulate_lidar:
        source = SimSource(mount)
    else:
        source = LidarSource(mount)
        try:
            source.open_serial(args.lidar_port, args.lidar_baud)
        except Exception as exc:  # noqa: BLE001
            print("Could not open the LiDAR on %s: %s" % (args.lidar_port, exc))
            print("Check the cable and the port, or use --simulate-lidar to try the panel without hardware.")
            return 2

    if args.lidar_check is not None:
        return lidar_check(source, args.lidar_check)

    setup(source, Settings(args.stop_mm, args.warn_mm, avoid=False))
    server = make_server(args.host, args.port)
    where = "SIMULATED LiDAR, walking disabled" if source.simulated else args.lidar_port
    print("Scorpion walk panel with LiDAR: http://%s:%d  (%s, Ctrl+C to quit)" % (args.host, args.port, where))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopping the server. A walk in progress is not interrupted; use Stop first.")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
