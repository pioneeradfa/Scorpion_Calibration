"""LEGACY FILE - kept for the tail and claw routines only.  See the warning below.

WARNING: DO NOT MIX THIS FILE WITH servo_lsc24.py / ik_module.py / tripod_gait.py

This file has its own angle_to_pulse() using the legacy mapping

    pulse = 500 + (angle / 180.0) * 2000

which was copied from Hiwonder's ServoControl.py and is correct for their 180
degree servo (HPS-2018) but WRONG for the HPS-2027 fitted to this robot, whose
datasheet says 500-2500us spans 0-270 degrees.  Every angle in this file
therefore drives the shaft 1.5x further than the number suggests.

All of POSITIONS, TAIL_POSES and CLAW_POSES below were tuned by trial and error
THROUGH that mapping, so the numbers are correct as-is for this file and only
this file.  A stored "90" means 135 physical degrees here and 90 physical
degrees in the new stack.  They are not interchangeable.

Migrating the tail and claw means changing the mapping AND multiplying every
stored angle by 1.5, then re-verifying each pose on hardware - any that already
sat against a mechanical stop would then be commanded past it.  Deliberately not
done; the current work is six-leg walking, in tripod_gait.py.

Known bugs in this file, left alone on purpose:
  - caterpillar_walk() and tripod_walk() never update `base`, so the feet return
    to the same absolute position every cycle and the robot walks in place.  They
    also apply `coxa += forward` to both sides, but increasing raw coxa is
    forward on a right leg and backward on a left leg, so the body twists.
    Both are superseded by tripod_gait.py.
  - move_leg() sends one frame per servo: 18 frames, 169ms at 9600 baud.
  - move_smooth() steps in Python, 21 frames per servo.  Never port it.
  - menu_mode() maps 'c' twice, so claws_close() is unreachable there.
  - keyboard_mode() calls stand() immediately at launch with no arming step.
  - channel numbers 24, 25, 28, 29, 30, 31 do not exist on a 24-channel board.
"""

import ServoControl
from time import sleep
import sys
import termios
import tty

# ============================================================
# CONFIG
# ============================================================
USE_MIRROR = False  # Set True only if claws are perfectly mirrored

# ============================================================
# PIN MAP
# ============================================================
LEG_MAP = {
    "RF": [29, 30, 31],
    "RM": [3, 4, 5],
    "RR": [9, 1, 2],
    "LF": [24, 23, 22],
    "LM": [13, 14, 15],
    "LR": [10, 11, 12],
}
LEG_ORDER = ["RF", "RM", "RR", "LR", "LM", "LF"]

TAIL_PINS = [21, 20, 7, 6]

CLAW_PINS = {
    "right": 28,
    "left": 25
}

# ============================================================
# POSES
# ============================================================
POSITIONS = {
    "stand": {
        "RF": [102, 110, 35],
        "RM": [90, 110, 35],
        "RR": [103, 105, 35],
        "LF": [115, 110, 40],
        "LM": [90, 110, 20],
        "LR": [90, 110, 30],
    },
    "sit": {
        "RF": [102, 80, 60],
        "RM": [90, 80, 60],
        "RR": [101, 75, 60],
        "LF": [115, 80, 65],
        "LM": [90, 80, 45],
        "LR": [90, 80, 55],
    }
}

TAIL_POSES = {
    "neutral":  [90, 90, 60, 60],
    "raised":   [90, 110, 130, 120],
    "curl":     [90, 120, 30, 130],
    "strike":   [90, 70, 50, 10],
    "down":     [90, 130, 150, 140],
}

# ✅ Per-claw calibration (fixes reversed issue)
CLAW_POSES = {
    "open":  {"right": 90,  "left": 90},
    "closed": {"right": 30,  "left": 150},
    "grip":  {"right": 20, "left": 170},
}

# ============================================================
# SMOOTH MOTION
# ============================================================
CLAW_STATE = {
    "left": 90,
    "right": 90
}

def ease_in_out(t):
    return t * t * (3 - 2 * t)

def angle_to_pulse(angle):
    return int(500 + (angle / 180.0) * 2000)

def move_smooth(pin, start, end, duration=400, steps=20):
    for i in range(steps + 1):
        t = i / steps
        eased = ease_in_out(t)
        angle = start + (end - start) * eased
        ServoControl.setPWMServoMove(pin, angle_to_pulse(angle), int(duration / steps))
        sleep(duration / steps / 1000.0)

# ============================================================
# CORE MOVEMENT
# ============================================================
def move(pin, angle, duration=400):
    ServoControl.setPWMServoMove(pin, angle_to_pulse(angle), duration)

def move_leg(leg, angles, duration=400):
    for pin, angle in zip(LEG_MAP[leg], angles):
        move(pin, angle, duration)

def move_all_legs(angles_dict, duration=400):
    for leg in LEG_ORDER:
        move_leg(leg, angles_dict[leg], duration)
    sleep(duration / 1000.0 + 0.05)

def move_tail(angles, duration=400):
    for pin, angle in zip(TAIL_PINS, angles):
        move(pin, angle, duration)
    sleep(duration / 1000.0 + 0.05)

# ============================================================
# CLAWS (SMOOTH + FIXED)
# ============================================================
def get_target_angle(side, pose):
    angle = CLAW_POSES[pose][side]
    if USE_MIRROR and side == "left":
        angle = 180 - angle
    return angle

def move_claw_smooth(side, pose, duration=400):
    pin = CLAW_PINS[side]
    target = get_target_angle(side, pose)
    start = CLAW_STATE[side]

    move_smooth(pin, start, target, duration)
    CLAW_STATE[side] = target

def move_both_claws(pose, duration=400):
    move_claw_smooth("left", pose, duration)
    move_claw_smooth("right", pose, duration)

def claws_open(): move_both_claws("open")
def claws_close(): move_both_claws("closed")
def claws_grip(): move_both_claws("grip")

# ============================================================
# POSTURES
# ============================================================
def stand():
    print("🦂 Standing...")
    move_all_legs(POSITIONS["stand"], 800)
    move_tail(TAIL_POSES["neutral"], 800)

def sit():
    print("🦂 Sitting...")
    move_all_legs(POSITIONS["sit"], 800)
    move_tail(TAIL_POSES["down"], 800)

def belly_touch(steps=15, step_duration=150):
    """Slowly lowers the chassis to the ground. Femur -> 130, Tibia -> 45."""
    print("🦂 Lowering chassis to belly touch ground...")
    
    # We interpolate from the stand pose to ensure a slow, soft descent
    base = POSITIONS["stand"]
    for step in range(1, steps + 1):
        fraction = step / float(steps)
        for leg in LEG_ORDER:
            # Coxa stays the same, Femur smoothly transitions to 130, Tibia to 40
            c_target = base[leg][0]
            f_target = int(base[leg][1] + (130 - base[leg][1]) * fraction)
            t_target = int(base[leg][2] + (45 - base[leg][2]) * fraction)
            
            move_leg(leg, [c_target, f_target, t_target], step_duration)
        sleep(step_duration / 1000.0)
    print("  ✅ Grounded.")

# Tail
def tail_neutral(): move_tail(TAIL_POSES["neutral"], 600)
def tail_raised(): move_tail(TAIL_POSES["raised"], 600)
def tail_curl(): move_tail(TAIL_POSES["curl"], 600)
def tail_strike(): move_tail(TAIL_POSES["strike"], 300)
def tail_down(): move_tail(TAIL_POSES["down"], 600)

# ============================================================
# WALKING
# ============================================================
def caterpillar_walk(steps=3, lift=10, forward=10, speed=300):
    base = {leg: POSITIONS["stand"][leg].copy() for leg in LEG_ORDER}
    for _ in range(steps):
        for leg in LEG_ORDER:
            lifted = base[leg].copy()
            lifted[1] -= lift
            lifted[2] += lift
            move_leg(leg, lifted, speed)

            moved = lifted.copy()
            moved[0] += forward
            move_leg(leg, moved, speed)

            lowered = base[leg].copy()
            lowered[0] += forward
            move_leg(leg, lowered, speed)
            sleep(0.05)

def tripod_walk(steps=4, lift=20, forward=15):
    A = ["RF", "LM", "RR"]
    B = ["LF", "RM", "LR"]
    base = {leg: POSITIONS["stand"][leg].copy() for leg in LEG_ORDER}

    for _ in range(steps):
        for leg in A:
            pos = base[leg].copy()
            pos[1] -= lift; pos[2] += lift; pos[0] += forward
            move_leg(leg, pos, 250)
        sleep(0.3)
        for leg in A: move_leg(leg, base[leg], 250)

        for leg in B:
            pos = base[leg].copy()
            pos[1] -= lift; pos[2] += lift; pos[0] += forward
            move_leg(leg, pos, 250)
        sleep(0.3)
        for leg in B: move_leg(leg, base[leg], 250)

# ============================================================
# ANIMATIONS
# ============================================================
def tail_wave():
    for _ in range(3):
        move_tail([70,110,130,150],300)
        sleep(0.3)
        move_tail([110,110,130,150],300)
        sleep(0.3)
    tail_neutral()

def claw_snap():
    for _ in range(3):
        move_both_claws("open",150)
        move_both_claws("grip",120)
    claws_close()

def strike_pose():
    tail_raised()
    sleep(0.2)
    claws_open()
    sleep(0.4)
    tail_strike()

def defensive_stance():
    sit()
    tail_curl()
    sleep(0.2)
    claws_open()
    sleep(1)
    stand()
    claws_close()

# ============================================================
# INPUT
# ============================================================
def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        
        
def print_controls():
    print("""
╔══════════════════════════════════════════════════════════════╗
║               🦂  SCORPION HEXAPOD CONTROLS                  ║
╚══════════════════════════════════════════════════════════════╝

🚶 Movement:
  w       — Caterpillar walk (3 steps)
  W       — Tripod walk (4 steps)

📐 Posture:
  s       — Stand
  x       — Sit
  b       — Belly touch (Slow lowering)

🦂 Tail:
  1       — Neutral (relaxed)
  2       — Raised (ready)
  3       — Curl (over body)
  4       — Strike! ⚡
  5       — Down (backward)
  6       — Tail wave animation

✂️ Claws:
  o       — Open claws
  c       — Close claws
  g       — Grip
  n       — Claw snap animation

🎪 Tricks:
  t       — Strike pose (threat display)
  d       — Defensive stance

🛑 Quit:
  q       — Sit & Exit
""")

# ============================================================
# MAIN LOOP
# ============================================================

def menu_mode():
    """Text-based menu for testing."""
    print("\n🦂 SCORPION HEXAPOD CONTROLLER")
    print("=" * 60)
    print("Commands:")
    print("  s = stand          x = sit           b = belly touch")
    print("  c = caterpillar    w = tripod walk")
    print("  t = strike pose    d = defensive")
    print("  1-5 = tail poses   o/c/g = claws")
    print("  q = quit\n")
    
    while True:
        cmd = input("Command: ").strip().lower()
        
        if   cmd == 's': stand()
        elif cmd == 'x': sit()
        elif cmd == 'b': belly_touch()
        elif cmd == 'c': caterpillar_walk(steps=3, lift=10, forward=10, speed=300)
        elif cmd == 'w': tripod_walk(steps=4, lift=20, forward=15)
        elif cmd == 't': strike_pose()
        elif cmd == 'd': defensive_stance()
        elif cmd == '1': tail_neutral()
        elif cmd == '2': tail_raised()
        elif cmd == '3': tail_curl()
        elif cmd == '4': tail_strike()
        elif cmd == '5': tail_down()
        elif cmd == '6': tail_wave()
        elif cmd == 'o': claws_open()
        elif cmd == 'c': claws_close()
        elif cmd == 'g': claws_grip()
        elif cmd == 'n': claw_snap()
        elif cmd == 'q':
            print("Shutting down...")
            sit()
            sleep(0.5)
            break
        else:
            print("Unknown command.")


def keyboard_mode():
    """Real-time keyboard control (press keys without Enter)."""
    print_controls()
    print("\n🦂 Initializing...")
    stand()
    claws_close()

    while True:
        key = getch()

        # Movement
        if key == 'w': caterpillar_walk(3,10,10,300)
        elif key == 'W': tripod_walk(4,20,15)

        # Posture
        elif key == 's': stand()
        elif key == 'x': sit()
        elif key == 'b': belly_touch()

        # Tail
        elif key == '1': tail_neutral()
        elif key == '2': tail_raised()
        elif key == '3': tail_curl()
        elif key == '4': tail_strike()
        elif key == '5': tail_down()
        elif key == '6': tail_wave()

        # Claws
        elif key == 'o': claws_open()
        elif key == 'c': claws_close()
        elif key == 'g': claws_grip()
        elif key == 'n': claw_snap()

        # Tricks
        elif key == 't': strike_pose()
        elif key == 'd': defensive_stance()

        # Quit
        elif key == 'q':
            print("\n🛑 Shutting down...")
            sit()
            tail_down()
            claws_close()
            sleep(1.0)
            print("💤 Goodbye!")
            break

# ============================================================
if __name__ == "__main__":
    import sys
    
    # Choose mode based on argument
    if len(sys.argv) > 1 and sys.argv[1] == "--menu":
        menu_mode()
    else:
        keyboard_mode()
