#!/usr/bin/env python3
#=================================================================================
# Demo: body motion with the v2 controller
# Stands the robot, tilts the body a little, returns it to level, then puts every
# foot back on its base position. Run directly to move the robot.
# Importing this file does nothing.
#=================================================================================

import time

import scorpion_controller_v2 as bot
import scorpion_body_motion as bm

TILT_DEG = 5.0        # keep below about 10 deg (reach limit)
HOLD_S = 1.0          # pause at each pose

def main():
    bot.stand()
    time.sleep(HOLD_S)

    # Each call sets the full attitude (roll, pitch, yaw); unnamed angles are 0.
    bm.set_attitude(roll_deg=TILT_DEG)      # roll only
    time.sleep(HOLD_S)
    bm.set_attitude(pitch_deg=TILT_DEG)     # pitch only (roll returns to 0)
    time.sleep(HOLD_S)
    bm.set_attitude(yaw_deg=TILT_DEG)       # yaw only
    time.sleep(HOLD_S)

    bm.reset_attitude()                     # back to level
    time.sleep(HOLD_S)

    bm.reposition_to_base()                 # feet back to base positions
    print("attitude:", bm.current_attitude())

if __name__ == "__main__":
    main()