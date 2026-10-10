#!/usr/bin/env python3
#=================================================================================
# Type a body position (x, y, z) and the whole robot body moves there.
# All six feet move together, so the feet stay put on the ground while the body moves.
# Uses scorpion_controller_v2 (IK, servo pulses).
#
# Body position = offset of the body from the stand pose, mm, body frame:
#   x > 0  body moves forward      y > 0  body moves left
#   z > 0  body moves UP           (feet move down relative to the body)
# So "0 0 20" raises the body 20 mm; "0 0 0" is the stand pose.
#
# If any foot cannot reach its target, the whole move is rejected and nothing is sent.
#
# Commands:
#   x y z         move the body to this offset, e.g.  30 0 -10
#   home          return the body to the stand pose (0 0 0)
#   time 800      move time in ms (default 800)
#   show          print the body offset and each foot's position
#   help, q       help / quit
#=================================================================================

import time

import scorpion_controller_v2 as bot

DEFAULT_MOVE_MS = 800

HELP = """Commands:
  x y z         move the body to this offset (mm), e.g.  30 0 -10
  home          return the body to the stand pose
  time 800      move time in ms
  show          print body offset and foot positions
  help, q       help / quit
  (z > 0 raises the body; feet move down relative to it)"""

body = (0.0, 0.0, 0.0)     # current body offset (mm)

def show():
    print("body offset (x, y, z) = (%.1f, %.1f, %.1f) mm" % body)
    for name in bot.LEG_ORDER:
        leg = bot.LEGS[name]
        print("  %s foot (x, y, z) = (%.1f, %.1f, %.1f)   angles [%.1f, %.1f, %.1f]"
              % ((name, leg.x, leg.y, leg.z) + leg.raw))

def move_body(bx, by, bz, move_ms):
    """Feet target = stand foot position minus the body offset.
    Checks every leg first; sends nothing if any leg is out of reach."""
    global body
    targets = {}
    for name in bot.LEG_ORDER:
        leg = bot.LEGS[name]
        tx, ty, tz = leg.base_x - bx, leg.base_y - by, leg.base_z - bz
        raw = bot.ik(leg.side, leg.cal, tx - leg.mount_x, ty - leg.mount_y, tz)
        if raw is None:
            print("rejected: body (%.1f, %.1f, %.1f) puts %s out of reach. Nothing sent."
                  % (bx, by, bz, name))
            return False
        targets[name] = (tx, ty, tz)
    for name, (tx, ty, tz) in targets.items():
        bot.LEGS[name].solve(tx, ty, tz)
    bot.move_legs(move_ms)
    time.sleep(move_ms / 1000.0)
    body = (bx, by, bz)
    show()
    return True

def main():
    move_ms = DEFAULT_MOVE_MS
    print("Body at stand pose. Type 'help' for commands.")
    show()
    while True:
        try:
            line = input("body> ").strip()
        except EOFError:
            break
        if not line:
            continue
        parts = line.split()
        cmd = parts[0].lower()
        try:
            if cmd in ("q", "quit", "exit"):
                break
            elif cmd == "help":
                print(HELP)
            elif cmd == "show":
                show()
            elif cmd == "home":
                move_body(0.0, 0.0, 0.0, move_ms)
            elif cmd == "time" and len(parts) == 2:
                move_ms = int(parts[1])
                if move_ms <= 0:
                    raise ValueError("move time must be positive")
                print("move time:", move_ms, "ms")
            elif len(parts) == 3:
                move_body(*(float(p) for p in parts), move_ms)
            else:
                print("unknown command. Type 'help'.")
        except ValueError as e:
            print("input error:", e)

if __name__ == "__main__":
    main()