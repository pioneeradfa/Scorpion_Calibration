#!/usr/bin/env python3
#=================================================================================
# Type a foot position (x, y, z) and the leg moves there.
# Uses scorpion_controller_v2 (IK, servo pulses). Only the selected leg moves.
#
# Coordinates: body frame, mm. X = forward, Y = left, Z = up (feet have Z < 0).
# Nothing moves until you type a command. Commands:
#   leg RF        select a leg (LF, RF, LM, RM, LR, RR). Default RF.
#   x y z         move the selected leg's foot to (x, y, z), e.g.  120 -280 -65
#   time 800      set the move time in ms (default 800)
#   stand         send the selected leg to its stand position
#   show          print the foot position and servo angles of the selected leg
#   help, q       help / quit
#
# Out-of-reach targets are rejected and nothing is sent to the servos.
#=================================================================================

import sys
import time

import scorpion_controller_v2 as bot

DEFAULT_LEG = "RF"
DEFAULT_MOVE_MS = 800

def show(leg):
    rel = (leg.x - leg.mount_x, leg.y - leg.mount_y, leg.z)
    print("%s  body (x, y, z) = (%.1f, %.1f, %.1f) mm   leg-relative = (%.1f, %.1f, %.1f)"
          % (leg.name, leg.x, leg.y, leg.z, rel[0], rel[1], rel[2]))
    print("     servo angles [coxa, femur, tibia] = (%.1f, %.1f, %.1f) deg" % leg.raw)

def move_to(leg, x, y, z, move_ms):
    """Solve and send one leg. Returns False (and sends nothing) if unreachable."""
    if not leg.solve(x, y, z):
        print("rejected: (%.1f, %.1f, %.1f) is out of reach for %s. Nothing sent." % (x, y, z, leg.name))
        return False
    leg.send(move_ms)
    time.sleep(move_ms / 1000.0)
    show(leg)
    return True

HELP = """Commands:
  leg RF        select a leg (LF, RF, LM, RM, LR, RR)
  x y z         move the selected foot to body-frame (x, y, z) mm, e.g.  120 -280 -65
  time 800      move time in ms
  stand         send the selected leg to its stand position
  show          print foot position and servo angles
  help, q       help / quit"""

def main():
    leg = bot.LEGS[DEFAULT_LEG]
    move_ms = DEFAULT_MOVE_MS
    print("Selected leg:", leg.name, "| move time:", move_ms, "ms. Type 'help' for commands.")
    show(leg)
    while True:
        try:
            line = input("%s> " % leg.name).strip()
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
            elif cmd == "leg" and len(parts) == 2 and parts[1].upper() in bot.LEGS:
                leg = bot.LEGS[parts[1].upper()]
                show(leg)
            elif cmd == "time" and len(parts) == 2:
                move_ms = int(parts[1])
                if move_ms <= 0:
                    raise ValueError("move time must be positive")
                print("move time:", move_ms, "ms")
            elif cmd == "show":
                show(leg)
            elif cmd == "stand":
                move_to(leg, leg.base_x, leg.base_y, leg.base_z, move_ms)
            elif len(parts) == 3:
                x, y, z = (float(p) for p in parts)
                move_to(leg, x, y, z, move_ms)
            else:
                print("unknown command. Type 'help'.")
        except ValueError as e:
            print("input error:", e)

if __name__ == "__main__":
    main()