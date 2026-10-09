#!/usr/bin/env python3
"""
Hiwonder LSC-24 servo layer for the Scorpion hexapod (Raspberry Pi 5).

WHY THIS FILE EXISTS
--------------------
Every previous version of this project converted angles to pulse widths with

    pulse = 500 + (angle / 180.0) * 2000

That formula is copied from Hiwonder's own ServoControl.py, where it is correct
for their 180-degree servo (HPS-2018: "500~2500us, corresponding to 0~180deg").

This robot uses the HPS-2027, whose datasheet says:

    PWM pulse width range : 500~2500us, corresponding to 0~270deg

So 500-2500us spans 270 degrees of shaft rotation, not 180.  Using /180 makes
every joint travel 1.5x the commanded angle.  This module is the single place
that knows the true conversion.

UNITS POLICY
------------
Everything above this module is expressed in TRUE PHYSICAL SHAFT DEGREES
(0-270).  Pulse widths never leave this file.  Legacy angle tables recorded
through the old /180 mapping must be multiplied by 1.5 to become physical
degrees -- see LEGACY_CALIBRATION_SCALE in ik_module.py.

CHANNEL NUMBERING -- READ THIS BEFORE WIRING
--------------------------------------------
Hiwonder's setPWMServoMove() contains:

    servo_id = 254 if (servo_id < 1 or servo_id > 254) else servo_id

Channel 0 is therefore REJECTED and silently becomes 254, which is the
BROADCAST address.  Commanding "channel 0" moves all 24 servos at once.
This module refuses channel 0 outright.  Valid channels are 1..24.

SERIAL THROUGHPUT -- THE REAL TIMING CONSTRAINT
----------------------------------------------
The LSC-24 speaks LOBOT protocol over TTL serial at 9600 baud (10 bits/byte).

    single servo frame      :  9 bytes  ->  9.4 ms
    18-servo array frame    : 61 bytes  -> 63.5 ms
    24-servo array frame    : 79 bytes  -> 82.3 ms

Sending 18 leg servos one at a time costs 169 ms; one array command costs
63.5 ms.  Always use the array command.  Python-side interpolation at 50 Hz is
impossible at this baud rate -- the STM32 on the board interpolates for you
(its `time` argument accepts up to 30000 ms), so command TARGET + DURATION and
never step in Python.

PIPELINED TIMING
----------------
A frame takes ~63.5 ms to arrive, and the board only starts moving once it has
the whole thing.  If you send a command, sleep for its duration, then send the
next, the board sits idle for 63.5 ms between every segment and the motion
staircases.

Instead: write the frame, then sleep for exactly the commanded duration.  The
next frame then finishes arriving at precisely the moment the previous segment
ends, so segments chain with no gap.  `ServoBus.send_pose()` implements this;
call it in a loop and do not add extra sleeps.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Tuple

# ============================================================
# SERVO CONSTANTS -- Hiwonder HPS-2027 (20kg, 270 degree)
# ============================================================
SERVO_SWEEP_DEGREES = 270.0   # HPS-2027: 500-2500us == 0-270 deg
PULSE_MIN = 500               # us
PULSE_MAX = 2500              # us

DEGREE_MIN = 0.0
DEGREE_MAX = SERVO_SWEEP_DEGREES   # 270.0

# Mechanical safety band.  The HPS-2027 can turn 360 deg when unpowered, but
# its powered travel is 0-270.  Keep a guard band so a rounding error or a bad
# IK result never slams the shaft into an internal stop.
DEGREE_SAFE_MIN = 2.0
DEGREE_SAFE_MAX = 268.0

# ============================================================
# LSC-24 SERIAL TIMING
# ============================================================
BAUD = 9600
BITS_PER_BYTE = 10            # 8N1 = 1 start + 8 data + 1 stop

SINGLE_FRAME_BYTES = 9
ARRAY_FRAME_OVERHEAD = 7      # header(2) + len(1) + cmd(1) + count(1) + time(2)
BYTES_PER_SERVO_IN_ARRAY = 3  # id(1) + pos(2)

MAX_TIME_MS = 30000           # board clamps the duration argument to this


def array_frame_bytes(n_servos: int) -> int:
    return ARRAY_FRAME_OVERHEAD + BYTES_PER_SERVO_IN_ARRAY * n_servos


def tx_seconds(n_bytes: int) -> float:
    """How long a frame takes to clock out at 9600 8N1."""
    return n_bytes * BITS_PER_BYTE / BAUD


def tx_ms(n_servos: int) -> float:
    """Transmission time in ms for an n-servo array command."""
    return tx_seconds(array_frame_bytes(n_servos)) * 1000.0


# ============================================================
# CHANNEL MAP
# ============================================================
# The old robot addressed 24 servos on channels 1-31 with gaps, which does not
# fit a 24-channel board at all: RF was on 29/30/31, the claws on 25/28 and
# LF's coxa on 24.  Six channels were out of range.
#
# This map is contiguous and 1-based.  Legs occupy 1-18 so the gait code can
# command every leg joint in a single array frame.  Tail and claw channels are
# allocated so the wiring plan is complete, but nothing in the walking code
# drives them.
#
# ADJUST THIS TO MATCH YOUR ACTUAL WIRING.  Run `python3 walk.py --identify`
# to step each channel one at a time and confirm the mapping physically.
LEG_CHANNELS: Dict[str, Tuple[int, int, int]] = {
    #        coxa  femur  tibia
    "RF":     (1,    2,     3),
    "RM":     (4,    5,     6),
    "RR":     (7,    8,     9),
    "LF":    (10,   11,    12),
    "LM":    (13,   14,    15),
    "LR":    (16,   17,    18),
}

TAIL_CHANNELS: Tuple[int, int, int, int] = (19, 20, 21, 22)   # not used by the gait
CLAW_CHANNELS: Dict[str, int] = {"right": 23, "left": 24}     # not used by the gait

LEG_ORDER = ["RF", "RM", "RR", "LR", "LM", "LF"]
JOINT_NAMES = ("coxa", "femur", "tibia")

# Tripod groups.  RF+LM+RR and LF+RM+LR alternate so three legs are always down.
TRIPOD_A = ["RF", "LM", "RR"]
TRIPOD_B = ["LF", "RM", "LR"]


def all_leg_channel_ids() -> List[int]:
    """Flat list of the 18 leg channel ids, in a stable order."""
    ids = []
    for leg in LEG_ORDER:
        ids.extend(LEG_CHANNELS[leg])
    return ids


# ============================================================
# CONVERSIONS
# ============================================================
def degree_to_pulse(deg: float) -> int:
    """TRUE physical shaft degrees (0-270) -> pulse width in microseconds.

    This is the corrected mapping.  The legacy /180 version produced 1.5x the
    intended rotation on a 270-degree servo.
    """
    p = PULSE_MIN + (deg / SERVO_SWEEP_DEGREES) * (PULSE_MAX - PULSE_MIN)
    return int(round(max(PULSE_MIN, min(PULSE_MAX, p))))


def pulse_to_degree(pulse: int) -> float:
    """Inverse of degree_to_pulse()."""
    return (pulse - PULSE_MIN) * SERVO_SWEEP_DEGREES / (PULSE_MAX - PULSE_MIN)


def legacy_to_physical(legacy_deg: float) -> float:
    """Convert an angle recorded through the old /180 mapping to true degrees.

    Old tables were tuned by trial and error against pulse = 500 + d/180*2000,
    so a stored "90" actually parked the shaft at 135 physical degrees.
    Multiply by 1.5 to restate those numbers in this module's units.
    """
    return legacy_deg * 1.5


class ServoLimitError(ValueError):
    """Raised when a commanded angle is outside the safe band.

    Deliberately an exception rather than a silent clamp: clamping is what made
    the original IK's unreachable-target failures invisible.
    """


# ============================================================
# BUS
# ============================================================
@dataclass
class ServoBus:
    """Thin wrapper over Hiwonder's ServoControl with correct units and timing.

    Works off the Pi too: if ServoControl cannot be imported the bus runs in
    dry-run mode and records every command instead of sending it.  That makes
    the gait testable before the robot is assembled.
    """

    dry_run: bool = False
    enforce_limits: bool = True
    log: List[dict] = field(default_factory=list)
    _sc: object = field(default=None, repr=False)
    connected: bool = field(default=False, init=False)

    def __post_init__(self) -> None:
        if not self.dry_run:
            try:
                import ServoControl  # type: ignore
                self._sc = ServoControl
                self.connected = True
            except Exception as exc:  # ImportError, serial.SerialException, ...
                print(f"[servo] ServoControl unavailable ({exc.__class__.__name__}: {exc})")
                print("[servo] falling back to DRY RUN - no hardware commands will be sent")
                self.dry_run = True
                self.connected = False

    # ---------- validation ----------
    def check_channel(self, ch: int) -> None:
        if ch == 0:
            raise ServoLimitError(
                "channel 0 is invalid: Hiwonder's setPWMServoMove remaps id<1 to 254, "
                "which is the BROADCAST address and would drive all 24 servos at once"
            )
        if ch < 1 or ch > 24:
            raise ServoLimitError(f"channel {ch} out of range for a 24-channel LSC-24 (valid 1-24)")

    def check_degrees(self, deg: float, where: str = "") -> float:
        if deg != deg:  # NaN
            raise ServoLimitError(f"NaN angle {where}")
        if self.enforce_limits and not (DEGREE_SAFE_MIN <= deg <= DEGREE_SAFE_MAX):
            raise ServoLimitError(
                f"angle {deg:.2f} deg out of safe band "
                f"[{DEGREE_SAFE_MIN}, {DEGREE_SAFE_MAX}] {where}"
            )
        return max(DEGREE_MIN, min(DEGREE_MAX, deg))

    # ---------- output ----------
    def send_pose(self, pose: Dict[int, float], duration_ms: int, sleep_after: bool = True) -> float:
        """Command many channels at once in a single array frame.

        pose         : {channel: true physical degrees}
        duration_ms  : interpolation time handed to the board's STM32
        sleep_after  : sleep for duration_ms so pipelined calls chain seamlessly.
                       Set False only for the final pose of a sequence.

        Returns the seconds actually spent (transmission is pipelined into the
        duration, so this is duration_ms/1000 when sleep_after is True).
        """
        if not pose:
            return 0.0
        duration_ms = max(0, min(MAX_TIME_MS, int(duration_ms)))

        for ch, deg in pose.items():
            self.check_channel(ch)
            self.check_degrees(deg, f"(channel {ch})")

        flat: List[int] = []
        for ch, deg in sorted(pose.items()):
            flat.append(ch)
            flat.append(degree_to_pulse(deg))

        if self.dry_run:
            self.log.append({
                "kind": "array",
                "duration_ms": duration_ms,
                "pose": {ch: round(deg, 2) for ch, deg in sorted(pose.items())},
            })
        else:
            # setPWMServoMoveByArray(servos, servos_count, time)
            self._sc.setPWMServoMoveByArray(flat, len(pose), duration_ms)  # type: ignore[attr-defined]

        if sleep_after:
            # Pipelining: the next frame is written after duration_ms, and its
            # ~63ms transmission then lands exactly as this segment ends.
            time.sleep(duration_ms / 1000.0)
            return duration_ms / 1000.0
        return 0.0

    def send_one(self, channel: int, deg: float, duration_ms: int = 400,
                 sleep_after: bool = True) -> None:
        self.check_channel(channel)
        self.check_degrees(deg, f"(channel {channel})")
        pulse = degree_to_pulse(deg)
        if self.dry_run:
            self.log.append({"kind": "single", "channel": channel,
                             "deg": round(deg, 2), "pulse": pulse,
                             "duration_ms": duration_ms})
        else:
            self._sc.setPWMServoMove(channel, pulse, duration_ms)  # type: ignore[attr-defined]
        if sleep_after:
            time.sleep(duration_ms / 1000.0)

    def min_duration_ms(self, n_servos: int) -> float:
        """Shortest segment duration that can still be pipelined for n servos.

        If a segment is shorter than its own transmission time, frames queue up
        faster than the wire drains them and latency grows without bound.
        """
        return tx_ms(n_servos)


if __name__ == "__main__":
    print("LSC-24 serial budget @ 9600 baud 8N1")
    print(f"  single-servo frame : {SINGLE_FRAME_BYTES} B -> {tx_seconds(SINGLE_FRAME_BYTES)*1000:6.1f} ms")
    for n in (6, 9, 18, 24):
        b = array_frame_bytes(n)
        print(f"  {n:2d}-servo array     : {b} B -> {tx_ms(n):6.1f} ms  "
              f"(max pipelined rate {1000/tx_ms(n):5.1f} Hz)")
    print()
    print("degree <-> pulse (HPS-2027, 270 deg sweep)")
    for d in (0, 45, 90, 135, 180, 225, 270):
        p = degree_to_pulse(d)
        print(f"  {d:3d} deg -> {p:4d} us -> back to {pulse_to_degree(p):6.2f} deg"
              f"   (legacy /180 formula would have sent {int(500 + d/180*2000):4d} us)")
    print()
    bus = ServoBus(dry_run=True)
    bus.send_pose({1: 135.0, 2: 135.0, 3: 135.0}, 500, sleep_after=False)
    print("dry-run log:", bus.log)
    try:
        bus.send_one(0, 90.0)
    except ServoLimitError as e:
        print(f"\nchannel 0 correctly rejected: {e}")
    try:
        bus.send_one(1, 300.0)
    except ServoLimitError as e:
        print(f"out-of-range correctly rejected: {e}")
