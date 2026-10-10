SCORPION HEXAPOD - ROBOT SKILLS REFERENCE
==========================================
Status date: 2026-10-10
Hardware: Raspberry Pi 5 + Hiwonder LSC-32 (32-channel servo controller), library ServoControl
Robot: 6 legs, 3 servos per leg (18 leg servos). Claw and tail servos are NOT used.

Status summary
- Geometry, IK and stand/sit poses: computed and checked in software (round trips exact).
- Controller (scorpion_controller_new_robot.py): compiles; leg output checked with a stub library.
- NOT yet tested on the robot. No walking loop is in the controller file yet.

Known problems are listed in section 10.


1. FILES
--------
  scorpion_controller_new_robot.py  Main controller (geometry, IK, stand, gait engine, moveLegs)
  simple_leg_check.py               Simple 18-servo check (stand, sit, belly touch)
  ik_module.py                      Per-leg IK module with calibration (uses 64.5/64.5/135.6, see 2.4)
  ik_7da.py                         Reference "7.d.a" IK + forward kinematics + round-trip test
  ik_tester.py                      Interactive/all-leg IK tester (type angles or foot positions, save poses)
  right_legs_step1.py               Step-1 output for RF, RM, RR (geometry, FK, IK, swing preview)
  rf_leg_step1.py                   Step-1 output for RF only
  leg_calibration.json              Per-servo calibration (coxa, femur, tibia) for each leg
  docs/                             Step-1 documents and diagrams (some still show old tibia 130.59)


2. DIMENSIONS AND GEOMETRY
--------------------------
2.1 Link lengths (measured from the leg photo)
  L1  Coxa link (coxa axis -> femur joint, horizontal)      64.25 mm   (controller and ik_7da)
  L2  Femur link (femur joint -> knee)                      64.25 mm
  L3  Tibia / Link 3 (knee -> foot tip, straight line)     135.6  mm
  The real leg is a 3-link model. There is no extra link.
  Model values: ik_module.py uses coxa 64.5 and femur 64.5 (not yet updated). Tibia 135.6 is
  used everywhere. The difference is 0.25 mm per link.

2.2 Mount positions (coxa-axis position in the body frame, mm)
  Body frame: X = forward, Y = left (+), Z = up (+). Origin at body centre.
  Leg   X      Y      Side
  LF   +115   +60    Left   (Leg1 in code)
  LM      0   +60    Left   (Leg3)
  LR   -115   +60    Left   (Leg5)
  RF   +115   -60    Right  (Leg2)
  RM      0   -60    Right  (Leg4)
  RR   -115   -60    Right  (Leg6)
  Front/mid/rear spacing 115 mm; left/right spacing 120 mm (+/-60).
  Each coxa axis points straight out from the body (90 deg to the centre line).

2.3 Heights and reach (user measurements)
  Ground clearance: the coxa axis is 6.5 cm above the ground (body height above ground).
  The 13 cm measurement: horizontal distance from the ground point under the coxa axis to the foot tip.
  The foot touches the ground in the stand pose.
  Sketch value 6.8 cm = coxa-axis-to-femur horizontal distance (model uses 6.45 cm).
  Controller Body.height = -64.0 mm (base height of the body origin, mean stand foot height).

2.4 Coordinate frame for each leg (used in IK)
  x = forward, y = outward from the body (sign flipped for right legs in ik_module),
  z = up. The foot must be BELOW the coxa axis (z < 0) for the IK to be valid.
  Horizontal distance from the femur joint: xy_comp = sqrt(x^2 + y^2) - L1.
  Femur-joint-to-foot distance: d = sqrt(xy_comp^2 + z^2).
  Reachable only if |L3 - L2| <= d <= L2 + L3  (here 71.35 mm <= d <= 199.85 mm).

2.5 Joint directions (from the notes and the calibration)
  Coxa: increasing raw angle = counter-clockwise rotation. On right legs CCW = forward;
        on left legs CCW = backward (mirrored mounting).
  Femur: increasing raw angle = leg lifts up.
  Tibia: increasing raw angle = leg curls up.
  Note: the tibia sign is still in doubt (see section 10).


3. SERVOS, PINS AND PULSES
--------------------------
3.1 Pin map (LSC-32 servo IDs), order [coxa, femur, tibia]
  RF [29, 30, 31]    RM [3, 4, 5]     RR [9, 1, 2]
  LF [24, 23, 22]    LM [13, 14, 15]  LR [10, 11, 12]
  All 18 IDs are unique.

3.2 Pulse conversion (joint angle in degrees -> microseconds)
  pulse_us = 500 + (angle / 180) * 2000      (clamped to 500..2500)
  The servo is assumed to be a standard 0-180 deg servo over 500-2500 us.
  Check this with one servo before trusting the full range.

3.3 Commands
  ServoControl.setPWMServoMove(servo_id, pulse_us, duration_ms)
  The controller uses duration 20 ms (DEFAULT_MOVE_TIME_MS). simple_leg_check.py uses 400 ms.
  Serial: /dev/ttyAMA0 at 9600 baud (opened by the ServoControl library).

3.4 Calibration per leg (raw servo degrees at the straight calibration pose)
  Leg  coxa  femur  tibia   pins (coxa/femur/tibia)
  RF    98    88     94     29 / 30 / 31
  RM    90    92     84      3 /  4 /  5
  RR   103    87     90      9 /  1 /  2
  LF   115    93    101     24 / 23 / 22
  LM    90    90     84     13 / 14 / 15
  LR    87    90     91     10 / 11 / 12


4. INVERSE AND FORWARD KINEMATICS
---------------------------------
4.1 Method ("7.d.a" reference equations, as coded in ik_7da.py and Leg.inverse_kinematics)
  Given foot (x, y, z) relative to the coxa axis:
  1. Outward distance and coxa angle:
       right legs: theta = 180 + atan(x / y)      left legs: theta = atan(x / y)
  2. Horizontal distance from the coxa axis:      DFO = sqrt(x^2 + y^2)
  3. Horizontal distance from the femur joint:    xy_comp = DFO - L1
  4. Femur-to-foot distance:                      ext = sqrt(xy_comp^2 + z^2)
  5. Knee interior angle (180 = straight):        psi = acos((L2^2 + L3^2 - ext^2) / (2 L2 L3))
  6. Angle of femur-to-foot line from vertical:   phi_p1 = |atan(xy_comp / z)|
  7. Angle at femur joint:                        phi_p2 = acos((L2^2 + ext^2 - L3^2) / (2 L2 ext))
  8. Femur angle from vertical:                   phi = 180 - phi_p1 - phi_p2
  The literal reference line sqrt(x^2 + y^2 + z^2) for "extension" includes L1 and is wrong.
  phi_p1 = |atan(xy_comp / z)| is valid when xy_comp > 0 and z < 0 (walking targets always are).

4.2 Mapping to raw servo angles
  coxa raw  = cal_coxa + (180 - theta)     right legs
  coxa raw  = cal_coxa - theta             left legs
  femur raw = cal_femur + (90 - phi)       femur elevation from horizontal, up = +
  tibia raw = cal_tibia - (180 - psi)      tibia = cal + TIBIA_SIGN*(180 - knee), TIBIA_SIGN = -1
  Output must be inside 0..180 for every joint (checked in ik_7da and ik_tester).

4.3 Controller version (Leg.inverse_kinematics)
  Uses the same geometry and the same calibration. Differences from 4.1 are only the form of phi
  (controller: phi = atan2(xy_comp, z) - acos(...)) and it reads the calibration from setup_leg().
  Measured agreement with ik_7da on 1,059 random foot targets: worst difference 0.045 deg,
  which is the 1 us rounding of the pulse.

4.4 Forward kinematics (ik_7da.fk_7da)
  Given raw angles, recover the foot position. Used for the stand and sit tables (section 5) and
  for the round-trip checks. Round trips are exact (error 0.0000 deg) for every leg.

4.5 Worked examples (L1 = L2 = 64.25, L3 = 135.6)
  LF stand, foot (0.00, 223.24, -75.41) -> coxa 115.00, femur 110.00, tibia 40.00
  RF stand, foot (16.19, -231.55, -57.54) -> coxa 102.00, femur 110.00, tibia 35.00

4.6 Reach limit (known)
  The 13 cm measurement (foot 130 mm out, z = -65 mm) is NOT reachable in the current model:
  tibia would need -49.3 deg (valid range 0..180). The calibrated stand (radial 232 mm) does not
  match the 13 cm figure. This is an open measurement question (see section 10).


5. STAND AND SIT
----------------
5.1 Stand pose, raw servo degrees [coxa, femur, tibia]
  LF [115, 110, 40]   LM [90, 110, 20]   LR [90, 110, 30]
  RF [102, 110, 35]   RM [90, 110, 35]   RR [103, 105, 35]

5.2 Sit pose, raw servo degrees [coxa, femur, tibia]
  LF [115, 80, 65]    LM [90, 80, 45]    LR [90, 80, 55]
  RF [102, 80, 60]    RM [90, 80, 60]    RR [101, 75, 60]

5.3 Stand foot positions (body frame, mm, from FK with L3 = 135.6)
  Leg   x        y         z        radial from coxa axis
  LF   115.00   283.24   -75.41    223.2
  LM     0.00   282.17   -72.22    222.2
  LR  -126.88   286.65   -66.99    227.0
  RF   131.19  -291.55   -57.54    232.1
  RM     0.00  -301.59   -49.98    241.6
  RR  -115.00  -293.65   -61.75    233.7
  Note: stand foot heights range from -50 to -75 mm. The coxa axis is 65 mm above the ground,
  so a level body needs all feet at about z = -65 mm. The RM foot is about 15 mm higher than
  the mean, and the LF foot is about 11 mm lower. Check the stand pose on the robot for tilt.

5.4 Sit foot positions (body frame, mm)
  LF   115.00   275.81  -116.79     LM     0.00   276.49  -113.50     LR  -126.60   281.42  -108.70
  RF   130.95  -288.09   -99.68     RM     0.00  -296.80   -93.06     RR  -122.95  -287.73  -104.09

5.5 Stand-up (controller standUp)
  1. Set every leg's target to its stand foot position (X/Y/Z_base_position).
  2. Run the IK for all six legs.
  3. Send all 18 servo commands (moveLegs).
  4. Wait 3 s.

5.6 Belly touch (simple_leg_check.belly_touch)
  Starts from stand. In 15 steps of 150 ms each, every leg's femur moves linearly to raw 130
  and tibia to raw 45 (coxa stays at the stand value). This lowers the chassis onto the belly.

5.7 Sit (simple_leg_check.sit)
  Moves all legs to the sit pose over 800 ms. The controller file has no sit() function yet.


6. GAIT CALCULATION
-------------------
6.1 Parameters (class Gaits)
  number_of_steps  N = 30     steps per gait cycle (more steps = slower)
  stride_length    L = 45 mm  maximum horizontal step length (starts at 20 mm until the first cycle ends)
  stride_height    H = 30 mm  maximum lift height
  gaitType         "tripod" (default). Others: double_ripple, ripple_123456, ripple_654321, ripple_135246.
  ICC              [x, y, direction]. Default [0, 10000, "CCW"].
  direction        "CW", "CCW" or "Stop".

6.2 Phase timing (tripod, N = 30, L = 45, H = 30)
  Support (foot on ground, moves in an arc):  support = N/2 = 15 steps
  Lift:   lift  = (N/2) / (L + 2H) * H = 4.29 steps
  Swing (foot in the air, moves to new spot): swing = (N/2) / (L + 2H) * L = 6.43 steps
  Lower:  lower = (N/2) / (L + 2H) * H = 4.29 steps
  Total = 15 + 4.29 + 6.43 + 4.29 = 30.
  Ripple gates (ripple_123456, ripple_654321, ripple_135246), N = 30:
    support = 5N/6 = 25;  lift = lower = (N/6)/(L + 2H) * H = 1.43;  swing = (N/6)/(L + 2H) * L = 2.14
    Total = 25 + 1.43 + 2.14 + 1.43 = 30.
  Double ripple (double_ripple_14_36_52), N = 30:
    support = 2N/3 = 20;  lift = lower = (N/3)/(L + 2H) * H = 2.86;  swing = (N/3)/(L + 2H) * L = 4.29
    Total = 20 + 2.86 + 4.29 + 2.86 = 30.

6.3 Tripod phase offsets (tripod_step_numbers)
  Group A: LF, RM, LR   offset 0      (phase 0 at step 0)
  Group B: RF, LM, RR   offset N/2 = 15
  While group A is in support, group B is in swing, and the reverse. Each step number is
  wrapped to 0..N-1 for every leg.
  Offsets for every gait (fraction of N; leg names as in section 2.2):
    tripod           LF 0,   RF 1/2, LM 1/2, RM 0,   LR 0,   RR 1/2
    ripple_123456    LF 5/6, RF 4/6, LM 3/6, RM 2/6, LR 1/6, RR 0
    ripple_654321    LF 0,   RF 1/6, LM 2/6, RM 3/6, LR 4/6, RR 5/6
    ripple_135246    LF 5/6, RF 2/6, LM 4/6, RM 1/6, LR 3/6, RR 0
    double_ripple    LF 2/3, RM 2/3 | LM 1/3, RR 1/3 | LR 0, RF 0   (pairs move together)
  These match the old robot's Gaits class line for line.

6.4 Leg target calculation (calculate_target_position)
  Support phase (step <= support):
    leg_stride = L * R_leg / R_max          R_leg = leg distance from ICC, R_max = largest of all legs
    alpha = 2 asin(0.5 * leg_stride / R_leg)
    alpha_step = alpha / support
    beta = atan2(Y_dist, X_dist) -/+ alpha_step  (+ for CW, - for CCW)
    target = ICC + R_leg (cos beta, sin beta);  Z = base Z (foot stays on the ground)
  Lift phase: Z += H / lift_steps
  Swing phase:
    leg_stride = L * Rb_leg / Rb_max        Rb = distance from ICC to the leg's base position
    alpha = asin(0.5 * leg_stride / Rb_leg)
    gamma = atan2(base_Y - ICC_y, base_X - ICC_x)
    beta = gamma -/+ alpha (CW/CCW)
    final = ICC + Rb (cos beta, sin beta)
    X, Y: each step moves the foot toward the final point by (step - start)/swing_steps of the
    CURRENT distance to it. This is not a constant-speed straight line. It approaches the final point
    over the swing.
    Angle note: support uses the full angle alpha = 2 asin(...), swing uses the half angle
    alpha = asin(...). This is an open point in the old code ("need to check all this properly").
  Lower phase: Z -= H / lower_steps
  If direction is "Stop", all targets stay put and the feet are lowered to base Z (all_legs_support).

6.5 Step numbers
  Each leg has its own step_number. tripod_step_numbers() advances the shared step number by 1
  each call, wraps it at N, and sets each leg's step_number from its offset.
  The first complete cycle sets one_gait_cycle_flag, after which stride length = 45 mm.

6.6 Walking loop (intended order)
  1. ICC (from the stick or a fixed value)
  2. calculate_leg_positions()          (targets for all six legs)
  3. leg_Inverse_Kinamatics()           (joint angles)
  4. moveLegs()                         (servo commands)
  5. update_step_numbers()              (advance the gait)
  The walking loop is NOT in scorpion_controller_new_robot.py yet (see section 10).


6.7 Check against the old robot's Gaits class (tested in simulation)
--------------------------------------------------------------------
The Gaits class in scorpion_controller_new_robot.py matches the old robot's class: step offsets,
phase lengths, support/swing/lift formulas, change_gait, update_step_numbers and the stride ramp
(20 mm until the first cycle ends, then 45 mm). The only intended changes are standUp() and
all_legs_support() (section 5.5 and section 10).

Simulation (tripod, N = 30, L = 45, H = 30, ICC = [0, 10000, CCW], stub servo library):
  - Per leg: 4 lift ticks (+7 mm each = +28 mm), 6 swing ticks, 4 lower ticks (-7 mm each).
    The lift and lower heights are equal, so the foot returns to ground level.
  - No IK limit hits over 3 cycles.
  - Support moves each foot by about 45 mm along the ICC arc (LF -45 mm in X; RF +19 mm).
  - Start-up transient: after the first cycle the feet are shifted from their stand positions
    in X: LF +21, RF -23, LM -21, RM +22, LR +21, RR -23 mm. The cause is the half/full angle
    mismatch in 6.4. After cycle 2 the positions repeat exactly (checked after cycles 2 and 3), so the
    offset does not grow. Expect the first step to move the body/feet by about 20 mm.
  - Only tripod was simulated. The other gaits have not been simulated.

Python 2 -> 3 note: the old code used "/" on integers, which floored the result (Python 2).
In the controller these are now true divisions. With N = 30 the results are the same. For other N
(for example 25) the phase lengths and step offsets differ, for example N*1/6 = 4 (Py2) but
4.17 (Py3). Use // or keep N a multiple of 6.


7. ICC (INSTANTANEOUS CENTRE OF CURVATURE)
------------------------------------------
7.1 Meaning
  The ICC is the point on the ground around which the body turns. Each leg moves on a circle around
  the ICC, at a speed set by its distance from it. A large ICC distance (for example y = 10000 mm)
  gives almost straight walking. A small distance gives a tight turn.

7.2 Calculations
  For each leg:  X_dist = X_leg - ICC_x,  Y_dist = Y_leg - ICC_y,  R_leg = sqrt(X_dist^2 + Y_dist^2)
  max_ICC_distance = largest R_leg of the six legs.
  The base versions (Rb_leg) use the base position of each leg; max_base_ICC_distance is their maximum.
  The support-phase stride scales with R_leg / max, so the outer legs take longer steps.

7.3 Status in the code
  calculate_ICC() uses placeholder values (0, 100000, "CCW") and does not read the stick.
  calculate_leg_positions() calls ICC_Parallel(), which is NOT defined in the file (see section 10).
  calculate_leg_positions_without_ICC_Parallel() does the same work without that call.


8. CONTROLLER API (scorpion_controller_new_robot.py)
----------------------------------------------------
  set_servo(servo_id, pulse_us)        Clamp to 500..2500 and move the servo (20 ms)
  moveLegs()                           Send the 18 leg servos from each leg's *_Joint values
  leg_Inverse_Kinamatics()             Run the IK for all six legs
  standUp()                            Stand (section 5.5)
  Gaits.tripod_Cycle_Steps(N)          Set phase lengths for tripod
  Gaits.change_gait(speed, N, type)    Put all feet down, reset, set gait
  Gaits.calculate_leg_positions()      Foot targets from the ICC (needs ICC_Parallel, section 10)
  Gaits.update_step_numbers()          Advance the gait by one step
  Gaits.stop()                         Stride 0
  set_eye_channel(...)                 No-op (the eye LEDs have no driver)
  Removed: claws, tail, pincers, PCA9685 drivers, serial driver.


9. TIMING AND LIMITS
--------------------
  Servo serial rate: 9600 baud. Each ServoControl.setPWMServoMove call is one serial frame of about
  10 bytes (about 10 ms). A full 18-servo update therefore takes about 180 ms.
  This limits how often a gait can update. Batching (setPWMServoMoveByArray) would be faster,
  but its sample code clamps positions to 0..1000 and needs checking before use.
  Recommended step rate with the current hardware: a few steps per second, not 50 Hz.
  Joint limits: 0..180 deg per joint (raw). Tibia values near 0 deg are close to the mechanical limit.


10. OPEN ISSUES AND CHECKS
--------------------------
  A. ICC_Parallel() is called by calculate_leg_positions() but is not defined. Any walking call fails
     with NameError. Fix: define it or call calculate_leg_positions_without_ICC_Parallel().
  B. There is no walking loop, no sit() function and no stick input in the controller file.
  C. calculate_ICC() uses placeholder values; the stick mapping is not implemented.
  D. Pulse limits: every servo uses 500..2500 us. The 0 and 180 deg ends must be checked per servo.
  E. Tibia sign: the docstring in ik_module.py says bending raises the tibia value, but the formula
     lowers it (TIBIA_SIGN = -1). The formula is what the calibration fits. Verify on the robot
     with one leg before walking.
  F. ik_module.py still uses 64.5 for L1 and L2 (photo: 64.25). Update it to match ik_7da.py.
  G. The 13 cm reach measurement is not reachable with the current model (tibia -49.3 deg).
     Confirm the measurement or the stand pose.
  H. Stand foot heights differ by about 25 mm (section 5.3). Check the body is level.
  I. Leg photo was not in the workspace; re-upload to confirm stride and lift values.
  J. The stride (45 mm) and lift (30 mm) values have not been tested on the robot.
  K. Only RF has been planned for a first hardware test. Move one leg first, keep the others at stand.
  L. ik_module_notes.md still lists tibia 130.59 in one line (the controller and ik_7da use 135.6).
  M. docs/RF_leg_step1.md and docs/Right_legs_step1.md still show 130.59 in places.
  N. Claw, tail and eye outputs were removed or stubbed as requested; the Body tail maths remains in
     the file but is not sent to any servo.
  O. ICC_Parallel() is not defined in the controller or in the original robot file
     (stand_sit_walk_tail_claw_belly.py). The old robot's Gaits class calls it too, so it was
     never in the repo. Use calculate_leg_positions_without_ICC_Parallel() until it is found.
  P. Start-up transient of about 20 mm on the first cycle (section 6.7). Decide whether the walk
     should start from the stand pose or from the steady-state position.
  Q. The swing angle is half of the support angle (section 6.4). Confirm the intended design
     on the robot before walking.
  R. Python 2 integer division: use // or N = 30 (section 6.7).
  S. Only tripod was simulated. Ripple and double-ripple gaits are not checked.