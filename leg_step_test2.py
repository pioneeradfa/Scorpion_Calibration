#!/usr/bin/env python3
#=================================================================================
# Scorpion hexapod controller - UPDATED FOR NEW ROBOT
# Hardware: Raspberry Pi 5 (Python 3) + Hiwonder LSC-32 32-channel servo controller
#           Servo control library: ServoControl (Hiwonder)
#
#  - Geometry: new robot mounts (front/mid/rear X = +115/0/-115, left/right Y = +/-60),
#    links L1 = 64.25, L2 = 64.25, L3 = 135.6 mm.
#  - Stand foot positions come from the calibrated stand angles (forward kinematics).
#  - Leg IK: 7.d.a reference equations, output converted to raw servo angles with
#    each servo's calibration (values are set in setup_leg()).
#  - Joint angle 0-180 deg -> 500-2500 us for every servo. Check on the robot.
#  - Only the 18 leg servos are used. Claw and tail servos were removed.
#=================================================================================

import time
import math
import sys
import numbers
import ServoControl  # Hiwonder LSC-32 library (opens the serial port itself)

# Old robot modules (not part of this repo). Optional so the file still imports.
try:
    import piMusic
except ImportError:
    piMusic = None
try:
    import MCP3008_ADC
except ImportError:
    MCP3008_ADC = None
try:
    import PS3Controller
except ImportError:
    PS3Controller = None

#=================================================================================
# Servo pins (LSC-32 servo IDs). Each leg: [coxa/hip, femur, tibia]
LEG_MAP = {
    "RF": [29, 30, 31],
    "RM": [3, 4, 5],
    "RR": [9, 1, 2],
    "LF": [24, 23, 22],
    "LM": [13, 14, 15],
    "LR": [10, 11, 12],
}

PULSE_MIN = 500                 # microseconds
PULSE_MAX = 2500                # microseconds
DEFAULT_MOVE_TIME_MS = 20       # time for a servo to reach each new position

def set_servo(servo_id, pulse):
    #Moves one servo. The pulse is clamped to 500-2500 us.
    pulse = int(min(PULSE_MAX, max(PULSE_MIN, round(pulse))))
    ServoControl.setPWMServoMove(servo_id, pulse, DEFAULT_MOVE_TIME_MS)

def set_eye_channel(channel, value):
    #RGB eye LEDs are not servos. Nothing drives them on the LSC-32 yet.
    pass

#Maps a number from one range to another
def map_number(input_variable, min_input, max_input, min_output, max_output):
    return (input_variable - min_input) * (max_output - min_output)/(max_input - min_input) + min_output

#=================================================================================

def Exit_Error(Error_Message):
    print("ERROR: %s" % Error_Message)
    sys.exit(1)

#=================================================================================

class Leg:
    #Link lengths (new robot, measured from the photo)
    L1 = 64.25 #Length of link 1 (S1 to S2) in mm
    L2 = 64.25 #Length of link 2 (S2 to S3) in mm
    L3 = 135.6 #Length of link 3 (S3 to end effector) in mm

    #X, Y, Z Offset for the servo mounting position relitive to the defined origin
    X_Offset = 0.0
    Y_Offset = 0.0
    Z_Offset = 0.0

    #Base X, Y, Z coordinates for the end effector (stand foot position)
    X_base_position = 0.0
    Y_base_position = 0.0
    Z_base_position = -80.0

    #Desired X, Y, Z coordinates for the end effector
    X_Coordinate = 0.0
    Y_Coordinate = 0.0
    Z_Coordinate = -80.0

    #Previous X, Y, Z coordinates
    Last_X = X_base_position
    Last_Y = Y_base_position
    Last_Z = Z_base_position

    previous_X = X_base_position
    previous_Y = Y_base_position
    previous_Z = Z_base_position
    
    #Kinematic limit flag
    limit_Flag = 0
    
    #Joint pulse counts (output of the inverse kinematics)
    Hip_Joint = 0
    Femur_Joint = 0
    Tibia_Joint = 0

    #Raw servo angles at the calibrated stand pose (degrees), from leg_calibration.json
    #Calibration = angle where the leg is straight (belly-down). Set per leg below.
    Cal_coxa = 90.0
    Cal_femur = 90.0
    Cal_tibia = 90.0
    
    #Servo pulse counts (per servo, old robot values - verify on the robot)
    servoMin_S1 = 500
    servoMax_S1 = 2500
    servoMin_S2 = 500
    servoMax_S2 = 2500
    servoMin_S3 = 500
    servoMax_S3 = 2500

    #Side the leg is mounted on
    Leg_Side = "Unknown"

    #Step number the leg is on through its current gait stride profile
    step_number = 0

    #Leg inverse kinematics (reference "7.d.a" equations, calibrated raw servo angles)
    def inverse_kinematics(self):
        #Relate the X, Y, Z coordinates of the end effector to the coxa axis
        #X = forward, Y = left, Z = up (body frame orientation)
        Leg_X_Coordinate = self.X_Coordinate - self.X_Offset
        Leg_Y_Coordinate = self.Y_Coordinate - self.Y_Offset
        Leg_Z_Coordinate = self.Z_Coordinate - self.Z_Offset

        if self.Leg_Side == "Right":
            outward = -Leg_Y_Coordinate     #right legs point to -Y
        elif self.Leg_Side == "Left":
            outward = Leg_Y_Coordinate      #left legs point to +Y
        else:
            print("Leg_Side needs to be either 'Left' or 'Right'. It is currently set to %s" % self.Leg_Side)
            return

        DFOXY = math.sqrt(Leg_X_Coordinate**2 + Leg_Y_Coordinate**2)
        XY_Component = DFOXY - self.L1                      #horizontal distance from the femur joint
        extension = math.sqrt(XY_Component**2 + Leg_Z_Coordinate**2)   #femur joint to foot

        #Reachable only if the foot is below the coxa axis and inside the link limits
        if extension > self.L2 + self.L3 or extension < abs(self.L3 - self.L2) or Leg_Z_Coordinate > 0:
            print("Out of limits")
            self.X_Coordinate = self.Last_X
            self.Y_Coordinate = self.Last_Y
            self.Z_Coordinate = self.Last_Z
            self.limit_Flag = 1
            return

        #Knee interior angle (pi = straight leg)
        psy = math.acos(max(-1.0, min(1.0, (self.L2**2 + self.L3**2 - extension**2)/(2 * self.L2 * self.L3))))
        #Angle of the femur-to-foot line measured from the vertical, minus the femur-to-knee angle
        phip1 = math.atan2(XY_Component, Leg_Z_Coordinate)
        phip2 = math.acos(max(-1.0, min(1.0, (self.L2**2 + extension**2 - self.L3**2)/(2 * self.L2 * extension))))
        phi = phip1 - phip2

        #Raw servo angles (degrees), using this leg's calibration
        yaw_deg = math.degrees(math.atan2(Leg_X_Coordinate, outward))   #coxa swing from straight out
        femur_el_deg = 90.0 - math.degrees(phi)                          #femur elevation, +ve = up
        knee_deg = math.degrees(psy)

        if self.Leg_Side == "Right":
            coxa_raw = self.Cal_coxa + yaw_deg
        else:
            coxa_raw = self.Cal_coxa - yaw_deg
        femur_raw = self.Cal_femur + femur_el_deg
        tibia_raw = self.Cal_tibia - (180.0 - knee_deg)

        if not all(0.0 <= a <= 180.0 for a in (coxa_raw, femur_raw, tibia_raw)):
            print("Out of limits")
            self.X_Coordinate = self.Last_X
            self.Y_Coordinate = self.Last_Y
            self.Z_Coordinate = self.Last_Z
            self.limit_Flag = 1
            return

        self.Last_X = self.X_Coordinate
        self.Last_Y = self.Y_Coordinate
        self.Last_Z = self.Z_Coordinate

        self.Hip_Joint = int(round(map_number(coxa_raw, 0, 180, self.servoMin_S1, self.servoMax_S1)))
        self.Femur_Joint = int(round(map_number(femur_raw, 0, 180, self.servoMin_S2, self.servoMax_S2)))
        self.Tibia_Joint = int(round(map_number(tibia_raw, 0, 180, self.servoMin_S3, self.servoMax_S3)))
        self.limit_Flag = 0

#=================================================================================

class Body:
    roll = 0.0#Body roll about the X
    pitch = 0.0#Body pitch about the Y
    yaw = 0.0#Body yaw about the Z

    previous_roll = 0
    previous_pitch = 0
    previous_yaw = 0
    
    Legs_1_2_X_position = 115.0#Base X distance for legs 1 and 2 from the origin
    Legs_3_4_X_position = 0.0#Base X distance for legs 3 and 4 from the origin
    Legs_5_6_X_position = -115.0#Base X distance for legs 5 and 6 from the origin
    camber = 120.0#Base distance between feet on oposite sides (2 x 60 mm)
    height = -64.0#Base height of the origin (mean stand height of the feet, mm)
    Camera_Angle = 0.0#Angle of the camera relavive to the X axis in the XZ plain
    camera_variable = 40
    pitchToggle = 0

    #Link lengths
    L1 = 64.25 #Length of link 1 in mm
    L2 = 64.25 #Length of link 2 in mm
    L3 = 64.25 #Length of link 3 in mm
    L4 = 64.25 #Length of link 4 in mm
    L5 = 64.25 #Length of link 5 in mm
    LTail_Base = 151.362 #Length of link LTail_Base in mm
    
    #Tail base offset from the origin
    tail_X_Offset = - 182 #X offset
    tail_Y_Offset = 0 #Y offset
    tail_Z_Offset = 1.25 #Z offset
    
    #Servo pulse counts at the base of the tail
    servoMin_S29 = 500
    servoMax_S29 = 2500
    servoMin_S30 = 500
    servoMax_S30 = 2500

    #Tail coordinates
    X_Tail_Coordinate = -240
    Z_Tail_Coordinate = 190
    synch = 0.60#Variable used to ensure both are always mapped to the same value
    
    Last_X = X_Tail_Coordinate
    Last_Z = Z_Tail_Coordinate
    Last_synch = synch

    #RPY initial leg values
    Leg1_X = 0
    Leg1_Y = 0
    Leg1_Z = 0

    Leg2_X = 0
    Leg2_Y = 0
    Leg2_Z = 0

    Leg3_X = 0
    Leg3_Y = 0
    Leg3_Z = 0

    Leg4_X = 0
    Leg4_Y = 0
    Leg4_Z = 0

    Leg5_X = 0
    Leg5_Y = 0
    Leg5_Z = 0

    Leg6_X = 0
    Leg6_Y = 0
    Leg6_Z = 0
    
    #Servos pulse counts in the tail
    #Segment 1
    servoMin_S24 = 500
    servoMax_S24 = 2500
    #Segment 2
    servoMin_S23 = 500
    servoMax_S23 = 2500
    #Segment 3
    servoMin_S22 = 500
    servoMax_S22 = 2500
    #Segment 4
    servoMin_S20 = 500
    servoMax_S20 = 2500
    #Segment 5
    servoMin_S19 = 500
    servoMax_S19 = 2500

    #Tail servo default positions
    Tail_base_S29 = int(round(map_number(synch, -90, 45, servoMax_S29, servoMin_S29)))
    Tail_base_S30 = int(round(map_number(synch, -90, 45, servoMax_S30, servoMin_S30)))

    Tail_Segment_1 = int(round(map_number(45, -90, 90, servoMin_S24, servoMax_S24)))
    Tail_Segment_2 = int(round(map_number(35, -90, 90, servoMin_S23, servoMax_S23)))
    Tail_Segment_3 = int(round(map_number(35, -90, 90, servoMin_S22, servoMax_S22)))
    Tail_Segment_4 = int(round(map_number(35, -90, 90, servoMin_S20, servoMax_S20)))
    Tail_Segment_5 = int(round(map_number(camera_variable, -90, 90, servoMin_S19, servoMax_S19)))

    #Calculates the movement to give the specified body pose
    def kinematics(self, camera_Angle):
        print("Body Kinematics")

    def set_tail_positions(self, TailBase, Segment1, Segment2, Segment3, Segment4, Segment5):
        if all((i >= -90 and i <= 90) for i in(Segment1, Segment2, Segment3, Segment4, Segment5)) and TailBase >= -90 and TailBase <= 45:
            synch = TailBase#Variable used to ensure both are always mapped to the same value
            self.Tail_base_S29 = int(round(map_number(synch, -90, 45, self.servoMax_S29, self.servoMin_S29)))
            self.Tail_base_S30 = int(round(map_number(synch, -90, 45, self.servoMax_S30, self.servoMin_S30)))
            self.Tail_Segment_1 = int(round(map_number(Segment1, -90, 90, self.servoMin_S24, self.servoMax_S24)))
            self.Tail_Segment_2 = int(round(map_number(Segment2, -90, 90, self.servoMin_S23, self.servoMax_S23)))
            self.Tail_Segment_3 = int(round(map_number(Segment3, -90, 90, self.servoMin_S22, self.servoMax_S22)))
            self.Tail_Segment_4 = int(round(map_number(Segment4, -90, 90, self.servoMin_S20, self.servoMax_S20)))
            self.Tail_Segment_5 = int(round(map_number(Segment5, -90, 90, self.servoMin_S19, self.servoMax_S19)))
            self.Camera_Angle = 180 - synch - Segment1 - Segment2 - Segment3 - Segment4 - Segment5
        else:
             Exit_Error("Invalid Tail Servo Position")#Exits the script and prints an error message
    
    def tail_inverse_kinematics(self):
       
        synch = self.synch #need to calculate synch so it will be dynamic #Variable used to ensure both are always mapped to the same value
        
        X_tail_base_offset = math.cos(synch) * self.LTail_Base
        Z_tail_base_offset = math.sin(synch) * self.LTail_Base

        X_target = self.X_Tail_Coordinate - self.tail_X_Offset + X_tail_base_offset
        Z_target = self.Z_Tail_Coordinate - self.tail_Z_Offset - Z_tail_base_offset

        extension1 = math.sqrt(X_target**2 + Z_target**2)
        extension2 = map_number(extension1, 0, self.L1 + self.L2 + self.L3 + self.L4, math.sqrt(self.L1**2 + self.L2**2), self.L1 + self.L2)
        extension3 = map_number(extension1, 0, self.L1 + self.L2 + self.L3 + self.L4,  math.sqrt(self.L3**2 + self.L4**2), self.L3 + self.L4)

        if extension1 >= self.L1 + self.L2 + self.L3 + self.L4 or extension2 >= self.L1 + self.L2 or extension3 >= self.L3 + self.L4:
            print("Tail out of kinematic limits")
            self.X_Tail_Coordinate = self.Last_X 
            self.Z_Tail_Coordinate = self.Last_Z
            self.synch = self.Last_synch
            return 0
        
        sigma = math.acos((self.L3**2 + self.L4**2 - extension2**2)/(2 * self.L3 * self.L4))
        rho = math.acos((self.L1**2 + self.L2**2 - extension2**2)/(2 * self.L1 * self.L2))

        Lambda = math.acos((extension2**2 + extension3**2 - extension1**2)/(2 * extension2 * extension3))
        l = Lambda*180/math.pi
        
        eta = math.acos((extension2**2 + self.L2**2 - self.L1**2)/(2 * extension2 * self.L2))
        theta = math.acos((self.L3**2 + extension3**2 - self.L4**2)/(2 * self.L3 * extension3))
        
        mu = math.atan2(Z_target, -X_target)
        nu = math.acos((self.L1**2 + extension2**2 - self.L2**2)/(2 * self.L1 * extension2))
        xi = math.acos((extension2**2 + extension1**2 - extension3**2)/(2 * extension2 * extension1))
        
        alpha = mu - synch - xi - nu
        beta = 3.141592654 - sigma
        gamma = 3.141592654 - eta - Lambda - theta
        delta = 3.141592654 - rho
        zeta = self.Camera_Angle
        
        if self.synch > -1.570796327 + 0.1 and self.synch < 0.7853981634 - 0.1 and extension1 <= self.L1 + self.L2 + self.L3 + self.L4 -4 and extension1 >= 110 and (self.Z_Tail_Coordinate > 130 or (self.Z_Tail_Coordinate >= 0 and self.X_Tail_Coordinate < self.tail_X_Offset)) and alpha <= 1.570796327:#With in the limits

            self.Last_X = self.X_Tail_Coordinate
            self.Last_Z = self.Z_Tail_Coordinate
            self.Last_synch = self.synch
            
            self.Tail_base_S29 = int(round(map_number(synch, -1.570796327, 0.7853981634, self.servoMax_S29, self.servoMin_S29)))
            self.Tail_base_S30 = int(round(map_number(synch, -1.570796327, 0.7853981634, self.servoMax_S30, self.servoMin_S30)))
            self.Tail_Segment_1 = int(round(map_number(alpha, -1.570796327, 1.570796327, self.servoMin_S24, self.servoMax_S24)))
            self.Tail_Segment_2 = int(round(map_number(beta, -1.570796327, 1.570796327, self.servoMin_S23, self.servoMax_S23)))
            self.Tail_Segment_3 = int(round(map_number(gamma, -1.570796327, 1.570796327, self.servoMin_S22, self.servoMax_S22)))
            self.Tail_Segment_4 = int(round(map_number(delta, -1.570796327, 1.570796327, self.servoMin_S20, self.servoMax_S20)))
            self.Tail_Segment_5 = int(round(map_number(3.141592654 - synch - alpha - beta - gamma - delta - zeta, -1.570796327, 1.570796327, self.servoMin_S19, self.servoMax_S19)))          
            
        else:
            print("Tail out of kinematic limits")
            self.X_Tail_Coordinate = self.Last_X 
            self.Z_Tail_Coordinate = self.Last_Z
            self.synch = self.Last_synch
        
    def set_camber(self, legCamber):
        self.camber = legCamber
        rightCamber = - legCamber / 2
        leftCamber = legCamber / 2
        Leg1.Y_base_position = leftCamber#Sets the base Y coordinate for the leg
        Leg3.Y_base_position = leftCamber#Sets the base Y coordinate for the leg
        Leg5.Y_base_position = leftCamber#Sets the base Y coordinate for the leg
        Leg2.Y_base_position = rightCamber#Sets the base Y coordinate for the leg
        Leg4.Y_base_position = rightCamber#Sets the base Y coordinate for the leg
        Leg6.Y_base_position = rightCamber#Sets the base Y coordinate for the leg

    def set_height(self, legHeight):
        Leg1.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg2.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg3.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg4.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg5.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        Leg6.Z_base_position = legHeight#Sets the base Z coordinate for the leg
        self.height = legHeight#sets the body height

    def set_leg_spacing(self, Xpos_12, Xpos_34, Xpos_56):
        Leg1.X_base_position = Xpos_12#Sets the base X coordinate for the leg
        Leg2.X_base_position = Xpos_12#Sets the base X coordinate for the leg
        Leg3.X_base_position = Xpos_34#Sets the base Z coordinate for the leg
        Leg4.X_base_position = Xpos_34#Sets the base Z coordinate for the leg
        Leg5.X_base_position = Xpos_56#Sets the base Z coordinate for the leg
        Leg6.X_base_position = Xpos_56#Sets the base Z coordinate for the leg

    def PS3_Camera_Angle(self):
        if self.pitchToggle == 0:
            if PS3Controller.ps3.axis24 > 0.15:
                PS3Controller.ps3.axis24 = 0.15
        elif PS3Controller.ps3.axis24 < -0.15:
            PS3Controller.ps3.axis24 = -0.15

        step = map_number(PS3Controller.ps3.axis24, -0.15, 0.15, -3, 3)

        if step > 0 and self.camera_variable > (-90 + step):

            self.camera_variable = self.camera_variable - step
            self.set_tail_positions(40, 45, 30, 30, 30, self.camera_variable)

        if step < 0 and self.camera_variable < (90 + step):
            self.camera_variable = self.camera_variable - step
            self.set_tail_positions(40, 45, 30, 30, 30, self.camera_variable)

    def sting(self):
        #moves servos to make a sting look
        print("sting")

    def leg_translation_to_base_position(self, inverse_Speed):#Only to be used after translations to recenter the body
        
        X_change_step = (Leg1.X_base_position - Leg1.X_Coordinate)/inverse_Speed
        Y_change_step = (Leg1.Y_base_position - Leg1.Y_Coordinate)/inverse_Speed
        Z_change_step = (Leg1.Z_base_position - Leg1.Z_Coordinate)/inverse_Speed

        for i in range(0, inverse_Speed, 1):#for loop moves the leg to the X, Y base positions
            Leg1.X_Coordinate = Leg1.X_Coordinate + X_change_step
            Leg1.Y_Coordinate = Leg1.Y_Coordinate + Y_change_step
            Leg1.Z_Coordinate = Leg1.Z_Coordinate + Z_change_step

            Leg2.X_Coordinate = Leg2.X_Coordinate + X_change_step
            Leg2.Y_Coordinate = Leg2.Y_Coordinate + Y_change_step
            Leg2.Z_Coordinate = Leg2.Z_Coordinate + Z_change_step

            Leg3.X_Coordinate = Leg3.X_Coordinate + X_change_step
            Leg3.Y_Coordinate = Leg3.Y_Coordinate + Y_change_step
            Leg3.Z_Coordinate = Leg3.Z_Coordinate + Z_change_step

            Leg4.X_Coordinate = Leg4.X_Coordinate + X_change_step
            Leg4.Y_Coordinate = Leg4.Y_Coordinate + Y_change_step
            Leg4.Z_Coordinate = Leg4.Z_Coordinate + Z_change_step

            Leg5.X_Coordinate = Leg5.X_Coordinate + X_change_step
            Leg5.Y_Coordinate = Leg5.Y_Coordinate + Y_change_step
            Leg5.Z_Coordinate = Leg5.Z_Coordinate + Z_change_step

            Leg6.X_Coordinate = Leg6.X_Coordinate + X_change_step
            Leg6.Y_Coordinate = Leg6.Y_Coordinate + Y_change_step
            Leg6.Z_Coordinate = Leg6.Z_Coordinate + Z_change_step
            
            leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
            moveLegs() 

        Leg1.X_Coordinate = Leg1.X_base_position
        Leg1.Y_Coordinate = Leg1.Y_base_position
        Leg1.Z_Coordinate = Leg1.Z_base_position

        Leg2.X_Coordinate = Leg2.X_base_position
        Leg2.Y_Coordinate = Leg2.Y_base_position
        Leg2.Z_Coordinate = Leg2.Z_base_position

        Leg3.X_Coordinate = Leg3.X_base_position
        Leg3.Y_Coordinate = Leg3.Y_base_position
        Leg3.Z_Coordinate = Leg3.Z_base_position

        Leg4.X_Coordinate = Leg4.X_base_position
        Leg4.Y_Coordinate = Leg4.Y_base_position
        Leg4.Z_Coordinate = Leg4.Z_base_position

        Leg5.X_Coordinate = Leg5.X_base_position
        Leg5.Y_Coordinate = Leg5.Y_base_position
        Leg5.Z_Coordinate = Leg5.Z_base_position

        Leg6.X_Coordinate = Leg6.X_base_position
        Leg6.Y_Coordinate = Leg6.Y_base_position
        Leg6.Z_Coordinate = Leg6.Z_base_position
            
        leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
        moveLegs()
            
        Gait.reset_step_numbers()
        
    def linear_leg_to_base_position(self, inverse_Speed):
        
        X_change_step1 = (Leg1.X_base_position - Leg1.X_Coordinate)/inverse_Speed
        Y_change_step1 = (Leg1.Y_base_position - Leg1.Y_Coordinate)/inverse_Speed
        Z_change_step1 = (Leg1.Z_base_position - Leg1.Z_Coordinate)/inverse_Speed

        X_change_step2 = (Leg2.X_base_position - Leg2.X_Coordinate)/inverse_Speed
        Y_change_step2 = (Leg2.Y_base_position - Leg2.Y_Coordinate)/inverse_Speed
        Z_change_step2 = (Leg2.Z_base_position - Leg2.Z_Coordinate)/inverse_Speed

        X_change_step3 = (Leg3.X_base_position - Leg3.X_Coordinate)/inverse_Speed
        Y_change_step3 = (Leg3.Y_base_position - Leg3.Y_Coordinate)/inverse_Speed
        Z_change_step3 = (Leg3.Z_base_position - Leg3.Z_Coordinate)/inverse_Speed

        X_change_step4 = (Leg4.X_base_position - Leg4.X_Coordinate)/inverse_Speed
        Y_change_step4 = (Leg4.Y_base_position - Leg4.Y_Coordinate)/inverse_Speed
        Z_change_step4 = (Leg4.Z_base_position - Leg4.Z_Coordinate)/inverse_Speed

        X_change_step5 = (Leg5.X_base_position - Leg5.X_Coordinate)/inverse_Speed
        Y_change_step5 = (Leg5.Y_base_position - Leg5.Y_Coordinate)/inverse_Speed
        Z_change_step5 = (Leg5.Z_base_position - Leg5.Z_Coordinate)/inverse_Speed

        X_change_step6 = (Leg6.X_base_position - Leg6.X_Coordinate)/inverse_Speed
        Y_change_step6 = (Leg6.Y_base_position - Leg6.Y_Coordinate)/inverse_Speed
        Z_change_step6 = (Leg6.Z_base_position - Leg6.Z_Coordinate)/inverse_Speed

        for i in range(0, inverse_Speed, 1):#for loop moves the leg to the X, Y base positions
            Leg1.X_Coordinate = Leg1.X_Coordinate + X_change_step1
            Leg1.Y_Coordinate = Leg1.Y_Coordinate + Y_change_step1
            Leg1.Z_Coordinate = Leg1.Z_Coordinate + Z_change_step1

            Leg2.X_Coordinate = Leg2.X_Coordinate + X_change_step2
            Leg2.Y_Coordinate = Leg2.Y_Coordinate + Y_change_step2
            Leg2.Z_Coordinate = Leg2.Z_Coordinate + Z_change_step2

            Leg3.X_Coordinate = Leg3.X_Coordinate + X_change_step3
            Leg3.Y_Coordinate = Leg3.Y_Coordinate + Y_change_step3
            Leg3.Z_Coordinate = Leg3.Z_Coordinate + Z_change_step3

            Leg4.X_Coordinate = Leg4.X_Coordinate + X_change_step4
            Leg4.Y_Coordinate = Leg4.Y_Coordinate + Y_change_step4
            Leg4.Z_Coordinate = Leg4.Z_Coordinate + Z_change_step4

            Leg5.X_Coordinate = Leg5.X_Coordinate + X_change_step5
            Leg5.Y_Coordinate = Leg5.Y_Coordinate + Y_change_step5
            Leg5.Z_Coordinate = Leg5.Z_Coordinate + Z_change_step5

            Leg6.X_Coordinate = Leg6.X_Coordinate + X_change_step6
            Leg6.Y_Coordinate = Leg6.Y_Coordinate + Y_change_step6
            Leg6.Z_Coordinate = Leg6.Z_Coordinate + Z_change_step6
            
            leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
            moveLegs() 

        Leg1.X_Coordinate = Leg1.X_base_position
        Leg1.Y_Coordinate = Leg1.Y_base_position
        Leg1.Z_Coordinate = Leg1.Z_base_position

        Leg2.X_Coordinate = Leg2.X_base_position
        Leg2.Y_Coordinate = Leg2.Y_base_position
        Leg2.Z_Coordinate = Leg2.Z_base_position

        Leg3.X_Coordinate = Leg3.X_base_position
        Leg3.Y_Coordinate = Leg3.Y_base_position
        Leg3.Z_Coordinate = Leg3.Z_base_position

        Leg4.X_Coordinate = Leg4.X_base_position
        Leg4.Y_Coordinate = Leg4.Y_base_position
        Leg4.Z_Coordinate = Leg4.Z_base_position

        Leg5.X_Coordinate = Leg5.X_base_position
        Leg5.Y_Coordinate = Leg5.Y_base_position
        Leg5.Z_Coordinate = Leg5.Z_base_position

        Leg6.X_Coordinate = Leg6.X_base_position
        Leg6.Y_Coordinate = Leg6.Y_base_position
        Leg6.Z_Coordinate = Leg6.Z_base_position
            
        leg_Inverse_Kinamatics()#Calculates the inverse kinematics for the leg
        moveLegs()
            
        Gait.reset_step_numbers()
        
    def calculate_leg_roll(self, Leg, Y, Z):
        ex = math.sqrt(Y ** 2 + Z ** 2)
        psy = math.atan2(Y, Z)
        phi = psy - self.roll
        Y_roll_change = math.sin(phi) * ex - Leg.Y_Coordinate
        Z_roll_change = math.cos(phi) * ex - Leg.Z_Coordinate
        Leg.Y_Coordinate = Leg.Y_Coordinate + Y_roll_change
        Leg.Z_Coordinate = Leg.Z_Coordinate + Z_roll_change
        
    def calculate_roll(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_Y = Leg1.Y_Coordinate
            self.Leg1_Z = Leg1.Z_Coordinate
            self.Leg2_Y = Leg2.Y_Coordinate
            self.Leg2_Z = Leg2.Z_Coordinate
            self.Leg3_Y = Leg3.Y_Coordinate
            self.Leg3_Z = Leg3.Z_Coordinate
            self.Leg4_Y = Leg4.Y_Coordinate
            self.Leg4_Z = Leg4.Z_Coordinate
            self.Leg5_Y = Leg5.Y_Coordinate
            self.Leg5_Z = Leg5.Z_Coordinate
            self.Leg6_Y = Leg6.Y_Coordinate
            self.Leg6_Z = Leg6.Z_Coordinate

            self.calculate_leg_roll(Leg1, Leg1.Y_Coordinate, Leg1.Z_Coordinate)
            self.calculate_leg_roll(Leg2, Leg2.Y_Coordinate, Leg2.Z_Coordinate)
            self.calculate_leg_roll(Leg3, Leg3.Y_Coordinate, Leg3.Z_Coordinate)
            self.calculate_leg_roll(Leg4, Leg4.Y_Coordinate, Leg4.Z_Coordinate)
            self.calculate_leg_roll(Leg5, Leg5.Y_Coordinate, Leg5.Z_Coordinate)
            self.calculate_leg_roll(Leg6, Leg6.Y_Coordinate, Leg6.Z_Coordinate)
        else:
            self.calculate_leg_roll(Leg1, self.Leg1_Y, self.Leg1_Z)
            self.calculate_leg_roll(Leg2, self.Leg2_Y, self.Leg2_Z)
            self.calculate_leg_roll(Leg3, self.Leg3_Y, self.Leg3_Z)
            self.calculate_leg_roll(Leg4, self.Leg4_Y, self.Leg4_Z)
            self.calculate_leg_roll(Leg5, self.Leg5_Y, self.Leg5_Z)
            self.calculate_leg_roll(Leg6, self.Leg6_Y, self.Leg6_Z)     
        
    def calculate_leg_pitch(self, Leg, X, Z):
        ex = math.sqrt(X ** 2 + Z ** 2)
        psy = math.atan2(Z, X)
        phi = psy - self.pitch

        X_pitch_change = math.cos(phi) * ex - Leg.X_Coordinate
        Z_pitch_change = math.sin(phi) * ex - Leg.Z_Coordinate
        Leg.X_Coordinate = Leg.X_Coordinate + X_pitch_change
        Leg.Z_Coordinate = Leg.Z_Coordinate + Z_pitch_change

    def calculate_pitch(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_X = Leg1.X_Coordinate
            self.Leg1_Z = Leg1.Z_Coordinate
            self.Leg2_X = Leg2.X_Coordinate
            self.Leg2_Z = Leg2.Z_Coordinate
            self.Leg3_X = Leg3.X_Coordinate
            self.Leg3_Z = Leg3.Z_Coordinate
            self.Leg4_X = Leg4.X_Coordinate
            self.Leg4_Z = Leg4.Z_Coordinate
            self.Leg5_X = Leg5.X_Coordinate
            self.Leg5_Z = Leg5.Z_Coordinate
            self.Leg6_X = Leg6.X_Coordinate
            self.Leg6_Z = Leg6.Z_Coordinate

            self.calculate_leg_pitch(Leg1, Leg1.X_Coordinate, Leg1.Z_Coordinate)
            self.calculate_leg_pitch(Leg2, Leg2.X_Coordinate, Leg2.Z_Coordinate)
            self.calculate_leg_pitch(Leg3, Leg3.X_Coordinate, Leg3.Z_Coordinate)
            self.calculate_leg_pitch(Leg4, Leg4.X_Coordinate, Leg4.Z_Coordinate)
            self.calculate_leg_pitch(Leg5, Leg5.X_Coordinate, Leg5.Z_Coordinate)
            self.calculate_leg_pitch(Leg6, Leg6.X_Coordinate, Leg6.Z_Coordinate)
        else:
            self.calculate_leg_pitch(Leg1, self.Leg1_X, self.Leg1_Z)
            self.calculate_leg_pitch(Leg2, self.Leg2_X, self.Leg2_Z)
            self.calculate_leg_pitch(Leg3, self.Leg3_X, self.Leg3_Z)
            self.calculate_leg_pitch(Leg4, self.Leg4_X, self.Leg4_Z)
            self.calculate_leg_pitch(Leg5, self.Leg5_X, self.Leg5_Z)
            self.calculate_leg_pitch(Leg6, self.Leg6_X, self.Leg6_Z)
            
    def calculate_leg_yaw(self, Leg, X, Y):
        ex = math.sqrt(X ** 2 + Y ** 2)
        psy = math.atan2(Y, X)
        phi = psy - self.yaw
        Leg.X_Coordinate = math.cos(phi) * ex
        Leg.Y_Coordinate = math.sin(phi) * ex
        
    def calculate_yaw(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_X = Leg1.X_Coordinate
            self.Leg1_Y = Leg1.Y_Coordinate
            self.Leg2_X = Leg2.X_Coordinate
            self.Leg2_Y = Leg2.Y_Coordinate
            self.Leg3_X = Leg3.X_Coordinate
            self.Leg3_Y = Leg3.Y_Coordinate
            self.Leg4_X = Leg4.X_Coordinate
            self.Leg4_Y = Leg4.Y_Coordinate
            self.Leg5_X = Leg5.X_Coordinate
            self.Leg5_Y = Leg5.Y_Coordinate
            self.Leg6_X = Leg6.X_Coordinate
            self.Leg6_Y = Leg6.Y_Coordinate

            self.calculate_leg_yaw(Leg1, Leg1.X_Coordinate, Leg1.Y_Coordinate)
            self.calculate_leg_yaw(Leg2, Leg2.X_Coordinate, Leg2.Y_Coordinate)
            self.calculate_leg_yaw(Leg3, Leg3.X_Coordinate, Leg3.Y_Coordinate)
            self.calculate_leg_yaw(Leg4, Leg4.X_Coordinate, Leg4.Y_Coordinate)
            self.calculate_leg_yaw(Leg5, Leg5.X_Coordinate, Leg5.Y_Coordinate)
            self.calculate_leg_yaw(Leg6, Leg6.X_Coordinate, Leg6.Y_Coordinate)
        else:
            self.calculate_leg_yaw(Leg1, self.Leg1_X, self.Leg1_Y)
            self.calculate_leg_yaw(Leg2, self.Leg2_X, self.Leg2_Y)
            self.calculate_leg_yaw(Leg3, self.Leg3_X, self.Leg3_Y)
            self.calculate_leg_yaw(Leg4, self.Leg4_X, self.Leg4_Y)
            self.calculate_leg_yaw(Leg5, self.Leg5_X, self.Leg5_Y)
            self.calculate_leg_yaw(Leg6, self.Leg6_X, self.Leg6_Y)
            
    def calculate_leg_RPY(self, Leg, X, Y, Z):
        ex_roll = math.sqrt(Y ** 2 + Z ** 2)
        psy_roll = math.atan2(Y, Z)
        phi_roll = psy_roll - self.roll
        Y_roll_change = math.sin(phi_roll) * ex_roll - Y
        Z_roll_change = math.cos(phi_roll) * ex_roll - Z
        
        ex_pitch = math.sqrt(X ** 2 + Z ** 2)
        psy_pitch = math.atan2(Z, X)
        phi_pitch = psy_pitch - self.pitch
        X_pitch_change = math.cos(phi_pitch) * ex_pitch - X
        Z_pitch_change = math.sin(phi_pitch) * ex_pitch - Z

        ex_yaw = math.sqrt(X ** 2 + Y ** 2)
        psy_yaw = math.atan2(Y, X)
        phi_yaw = psy_yaw - self.yaw
        X_yaw_change = math.cos(phi_yaw) * ex_yaw - X
        Y_yaw_change = math.sin(phi_yaw) * ex_yaw - Y
        
        Leg.X_Coordinate = X + X_pitch_change + X_yaw_change
        Leg.Y_Coordinate = Y + Y_roll_change + Y_yaw_change
        Leg.Z_Coordinate = Z + Z_roll_change + Z_pitch_change

    def calculate_RPY(self):
        if self.roll == 0 and self.pitch == 0 and self.yaw == 0:
            self.Leg1_X = Leg1.X_Coordinate
            self.Leg1_Y = Leg1.Y_Coordinate
            self.Leg1_Z = Leg1.Z_Coordinate
            
            self.Leg2_X = Leg2.X_Coordinate
            self.Leg2_Y = Leg2.Y_Coordinate
            self.Leg2_Z = Leg2.Z_Coordinate
            
            self.Leg3_X = Leg3.X_Coordinate
            self.Leg3_Y = Leg3.Y_Coordinate
            self.Leg3_Z = Leg3.Z_Coordinate
            
            self.Leg4_X = Leg4.X_Coordinate
            self.Leg4_Y = Leg4.Y_Coordinate
            self.Leg4_Z = Leg4.Z_Coordinate
            
            self.Leg5_X = Leg5.X_Coordinate
            self.Leg5_Y = Leg5.Y_Coordinate
            self.Leg5_Z = Leg5.Z_Coordinate
            
            self.Leg6_X = Leg6.X_Coordinate
            self.Leg6_Y = Leg6.Y_Coordinate
            self.Leg6_Z = Leg6.Z_Coordinate

            self.calculate_leg_RPY(Leg1, Leg1.X_Coordinate, Leg1.Y_Coordinate, Leg1.Z_Coordinate)
            self.calculate_leg_RPY(Leg2, Leg2.X_Coordinate, Leg2.Y_Coordinate, Leg2.Z_Coordinate)
            self.calculate_leg_RPY(Leg3, Leg3.X_Coordinate, Leg3.Y_Coordinate, Leg3.Z_Coordinate)
            self.calculate_leg_RPY(Leg4, Leg4.X_Coordinate, Leg4.Y_Coordinate, Leg4.Z_Coordinate)
            self.calculate_leg_RPY(Leg5, Leg5.X_Coordinate, Leg5.Y_Coordinate, Leg5.Z_Coordinate)
            self.calculate_leg_RPY(Leg6, Leg6.X_Coordinate, Leg6.Y_Coordinate, Leg6.Z_Coordinate)
        else:
            self.calculate_leg_RPY(Leg1, self.Leg1_X, self.Leg1_Y, self.Leg1_Z)
            self.calculate_leg_RPY(Leg2, self.Leg2_X, self.Leg2_Y, self.Leg2_Z)
            self.calculate_leg_RPY(Leg3, self.Leg3_X, self.Leg3_Y, self.Leg3_Z)
            self.calculate_leg_RPY(Leg4, self.Leg4_X, self.Leg4_Y, self.Leg4_Z)
            self.calculate_leg_RPY(Leg5, self.Leg5_X, self.Leg5_Y, self.Leg5_Z)
            self.calculate_leg_RPY(Leg6, self.Leg6_X, self.Leg6_Y, self.Leg6_Z)
            
#=================================================================================

class Gaits:
    number_of_steps = 30#Number of steps in the gait cycle (inversly proportional to speed)
    step_number = 0#Current step of the gait cycle
    stride_length = 45.0#Maximum length of each step taken
    stride_height = 30.0#Maximum height of each step taken
    stride_profile = "Rectangular"#Trapize, Semicircular or Rectangular
    gaitType = "tripod"
    ICC = [0, 10000, "CCW"] #instantaneous centre of curvature [x, y "dir"]
    max_ICC_distance = 0#The distance the ICC is from the furthest leg
    max_base_ICC_distance = 0
    one_gait_cycle_flag = 0

    lifting_steps = 0
    lower_steps = 0
    support_steps = 0
    swing_steps = 0

    X_dist1 = 0
    Y_dist1 = 0
    X_dist2 = 0
    Y_dist2 = 0
    X_dist3 = 0
    Y_dist3 = 0
    X_dist4 = 0
    Y_dist4 = 0
    X_dist5 = 0
    Y_dist5 = 0
    X_dist6 = 0
    Y_dist6 = 0
    leg_ICC_distance1 = 0.0
    leg_ICC_distance2 = 0.0
    leg_ICC_distance3 = 0.0
    leg_ICC_distance4 = 0.0
    leg_ICC_distance5 = 0.0
    leg_ICC_distance6 = 0.0

    leg_base_ICC_distance1 = 0.0
    leg_base_ICC_distance2 = 0.0
    leg_base_ICC_distance3 = 0.0
    leg_base_ICC_distance4 = 0.0
    leg_base_ICC_distance5 = 0.0
    leg_base_ICC_distance6 = 0.0

    def stop(self):
        self.stride_length = 0.0#Maximum length of each step taken
        self.stride_height = 0.0#Maximum height of each step taken

    def ripple_123456_step_numbers(self):
        #Correctly scales the step number for each leg
        Leg6var = self.step_number + self.number_of_steps * 0/6
        Leg5var = self.step_number + self.number_of_steps * 1/6
        Leg4var = self.step_number + self.number_of_steps * 2/6
        Leg3var = self.step_number + self.number_of_steps * 3/6
        Leg2var = self.step_number + self.number_of_steps * 4/6
        Leg1var = self.step_number + self.number_of_steps * 5/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0

    def tripod_Cycle_Steps(self, numberOfSteps):
        #Number of steps in each phase of the tripod gait
        self.lifting_steps = ((numberOfSteps/2)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print("lifting_steps = %f" % self.lifting_steps)
        self.lower_steps = ((numberOfSteps/2)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print("lower_steps = %f" % self.lower_steps)
        self.support_steps = numberOfSteps/2
        #print("support_steps = %f" % self.support_steps)
        self.swing_steps = ((numberOfSteps/2)/(self.stride_length + 2 * self.stride_height)) * self.stride_length
        #print("swing_steps = %f" % self.swing_steps)
        self.number_of_steps = numberOfSteps

    def tripod_step_numbers(self):
        if self.ICC[2] == "Stop":
            return 0
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 0/6
        Leg2var = self.step_number + self.number_of_steps * 3/6
        Leg3var = self.step_number + self.number_of_steps * 3/6
        Leg4var = self.step_number + self.number_of_steps * 0/6
        Leg5var = self.step_number + self.number_of_steps * 0/6
        Leg6var = self.step_number + self.number_of_steps * 3/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0

    def ripple_Cycle_Steps(self, numberOfSteps):
        #Number of steps in each phase of the ripple_123456 gait
        self.lifting_steps = ((numberOfSteps/6)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print("lifting_steps = %f" % self.lifting_steps)
        self.lower_steps = ((numberOfSteps/6)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print("lower_steps = %f" % self.lower_steps)
        self.support_steps = numberOfSteps*5/6
        #print("support_steps = %f" % self.support_steps)
        self.swing_steps = ((numberOfSteps/6)/(self.stride_length + 2 * self.stride_height)) * self.stride_length
        #print("swing_steps = %f" % self.swing_steps)
        self.number_of_steps = numberOfSteps

    def ripple_654321_step_numbers(self):
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 0/6
        Leg2var = self.step_number + self.number_of_steps * 1/6
        Leg3var = self.step_number + self.number_of_steps * 2/6
        Leg4var = self.step_number + self.number_of_steps * 3/6
        Leg5var = self.step_number + self.number_of_steps * 4/6
        Leg6var = self.step_number + self.number_of_steps * 5/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0
            
    def ripple_135246_step_numbers(self):
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 5/6
        Leg2var = self.step_number + self.number_of_steps * 2/6
        Leg3var = self.step_number + self.number_of_steps * 4/6
        Leg4var = self.step_number + self.number_of_steps * 1/6
        Leg5var = self.step_number + self.number_of_steps * 3/6
        Leg6var = self.step_number + self.number_of_steps * 0/6

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0
            
    def double_ripple_Cycle_Steps(self, numberOfSteps):
        #Number of steps in each phase of the ripple_123456 gait
        self.lifting_steps = ((numberOfSteps/3)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print("lifting_steps = %f" % self.lifting_steps)
        self.lower_steps = ((numberOfSteps/3)/(self.stride_length + 2 * self.stride_height)) * self.stride_height
        #print("lower_steps = %f" % self.lower_steps)
        self.support_steps = numberOfSteps*2/3
        #print("support_steps = %f" % self.support_steps)
        self.swing_steps = ((numberOfSteps/3)/(self.stride_length + 2 * self.stride_height)) * self.stride_length
        #print("swing_steps = %f" % self.swing_steps)
        self.number_of_steps = numberOfSteps
        
    def double_ripple_14_36_52(self):#sets the step numbers for each leg
        if self.ICC[2] == "Stop":
            return 0
        
        #Correctly scales the step number for each leg
        Leg1var = self.step_number + self.number_of_steps * 2/3
        Leg2var = self.step_number + self.number_of_steps * 0/3
        Leg3var = self.step_number + self.number_of_steps * 1/3
        Leg4var = self.step_number + self.number_of_steps * 2/3
        Leg5var = self.step_number + self.number_of_steps * 0/3
        Leg6var = self.step_number + self.number_of_steps * 1/3

        #Loops the step numbers
        if Leg1var >= self.number_of_steps:
            Leg1var = Leg1var - self.number_of_steps
        if Leg2var >= self.number_of_steps:
            Leg2var = Leg2var - self.number_of_steps
        if Leg3var >= self.number_of_steps:
            Leg3var = Leg3var - self.number_of_steps
        if Leg4var >= self.number_of_steps:
            Leg4var = Leg4var - self.number_of_steps
        if Leg5var >= self.number_of_steps:
            Leg5var = Leg5var - self.number_of_steps
        if Leg6var >= self.number_of_steps:
            Leg6var = Leg6var - self.number_of_steps

        if all((i < self.number_of_steps and i >= 0) for i in(Leg1var, Leg2var, Leg3var, Leg4var, Leg5var, Leg6var)):#Check all step number are in the correct range
             Leg1.step_number = Leg1var
             Leg2.step_number = Leg2var
             Leg3.step_number = Leg3var
             Leg4.step_number = Leg4var
             Leg5.step_number = Leg5var
             Leg6.step_number = Leg6var
        else:
            Exit_Error("Leg step_number out of accepted range")

        self.step_number = self.step_number + 1#Incements the step number

        if self.step_number >= self.number_of_steps:#Resets step number
            self.step_number = 0
            
    def reset_step_numbers(self):
        Leg1.step_number = 0
        Leg2.step_number = 0
        Leg3.step_number = 0
        Leg4.step_number = 0
        Leg5.step_number = 0
        Leg6.step_number = 0
        self.step_number = 0
        
    def reset_Gait_variables(self):
        self.max_ICC_distance = 0.0#The distance the ICC is from the furthest leg

    def calculate_ICC(self):#Reads in from the controller and calculates the ICC
        #print("Calculate the ICC")
        #analog read
        self.ICC[0] = 0 #analog read with some maths
        self.ICC[1] = 100000 #analog read with some maths
        self.ICC[2] = "CCW"

        #Checks the ICC parameters are valid
        if isinstance(self.ICC[0], numbers.Number) == False or isinstance(self.ICC[1], numbers.Number) == False or self.ICC[2] not in("CW", "CCW", "Stop"):
            Exit_Error("Invalid ICC parameters")#Exits the script and prints an error message

    def calculate_leg_ICC_distances(self):#Split this down to 3 function for each pair of legs ready for multi threading
        #print("Calculate the leg to ICC distances")
        self.max_ICC_distance = 0.0

        self.X_dist1 = Leg1.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist1 = Leg1.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance1 = math.sqrt(self.X_dist1**2 + self.Y_dist1**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance1 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance1

        self.X_dist2 = Leg2.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist2 = Leg2.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance2 = math.sqrt(self.X_dist2**2 + self.Y_dist2**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance2 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance2

        self.X_dist3 = Leg3.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist3 = Leg3.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance3 = math.sqrt(self.X_dist3**2 + self.Y_dist3**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance3 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance3

        self.X_dist4 = Leg4.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist4 = Leg4.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance4 = math.sqrt(self.X_dist4**2 + self.Y_dist4**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance4 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance4

        self.X_dist5 = Leg5.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist5 = Leg5.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance5 = math.sqrt(self.X_dist5**2 + self.Y_dist5**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance5 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance5

        self.X_dist6 = Leg6.X_Coordinate - self.ICC[0]#X Distance from ICC to the leg end effector
        self.Y_dist6 = Leg6.Y_Coordinate - self.ICC[1]#Y Distance from ICC to the leg end effector
        self.leg_ICC_distance6 = math.sqrt(self.X_dist6**2 + self.Y_dist6**2)#Distance from ICC to the leg end effector

        if self.leg_ICC_distance6 > self.max_ICC_distance:
            self.max_ICC_distance = self.leg_ICC_distance6

    def calculate_base_leg_ICC_distances(self):#Split this down to 3 function for each pair of legs ready for multi threading
        #print("Calculate the leg to ICC distances")
        self.max_base_ICC_distance = 0.0

        self.leg_base_ICC_distance1 = math.sqrt((Leg1.X_base_position - self.ICC[0])**2 + (Leg1.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance1 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance1

        self.leg_base_ICC_distance2 = math.sqrt((Leg2.X_base_position - self.ICC[0])**2 + (Leg2.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance2 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance2

        self.leg_base_ICC_distance3 = math.sqrt((Leg3.X_base_position - self.ICC[0])**2 + (Leg3.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance3 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance3

        self.leg_base_ICC_distance4 = math.sqrt((Leg4.X_base_position - self.ICC[0])**2 + (Leg4.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance4 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance4

        self.leg_base_ICC_distance5 = math.sqrt((Leg5.X_base_position - self.ICC[0])**2 + (Leg5.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance5 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance5

        self.leg_base_ICC_distance6 = math.sqrt((Leg6.X_base_position - self.ICC[0])**2 + (Leg6.Y_base_position - self.ICC[1])**2)#Distance from ICC to the leg end effector

        if self.leg_base_ICC_distance6 > self.max_base_ICC_distance:
            self.max_base_ICC_distance = self.leg_base_ICC_distance6

    def calculate_leg_positions(self):#Calculates all the legs target positions
        #self.calculate_ICC()#Calculates the ICC position
        ICC_Parallel()#Calculates the ICC position
        self.calculate_leg_ICC_distances()#Calculates the ICC distances from each leg and the furthest distance from a leg
        self.calculate_base_leg_ICC_distances()#Calculates the distances from each leg's base position to the ICC

        self.calculate_target_position(Leg1, self.leg_ICC_distance1, self.leg_base_ICC_distance1, self.X_dist1, self.Y_dist1)#Calculates the target position for the leg
        self.calculate_target_position(Leg3, self.leg_ICC_distance3, self.leg_base_ICC_distance3, self.X_dist3, self.Y_dist3)#Calculates the target position for the leg
        self.calculate_target_position(Leg5, self.leg_ICC_distance5, self.leg_base_ICC_distance5, self.X_dist5, self.Y_dist5)#Calculates the target position for the leg

        self.calculate_target_position(Leg2, self.leg_ICC_distance2, self.leg_base_ICC_distance2, self.X_dist2, self.Y_dist2)#Calculates the target position for the leg
        self.calculate_target_position(Leg4, self.leg_ICC_distance4, self.leg_base_ICC_distance4, self.X_dist4, self.Y_dist4)#Calculates the target position for the leg
        self.calculate_target_position(Leg6, self.leg_ICC_distance6, self.leg_base_ICC_distance6, self.X_dist6, self.Y_dist6)#Calculates the target position for the leg

    def calculate_leg_positions_without_ICC_Parallel(self):
        self.calculate_leg_ICC_distances()#Calculates the ICC distances from each leg and the furthest distance from a leg
        self.calculate_base_leg_ICC_distances()#Calculates the distances from each leg's base position to the ICC

        self.calculate_target_position(Leg1, self.leg_ICC_distance1, self.leg_base_ICC_distance1, self.X_dist1, self.Y_dist1)#Calculates the target position for the leg
        self.calculate_target_position(Leg3, self.leg_ICC_distance3, self.leg_base_ICC_distance3, self.X_dist3, self.Y_dist3)#Calculates the target position for the leg
        self.calculate_target_position(Leg5, self.leg_ICC_distance5, self.leg_base_ICC_distance5, self.X_dist5, self.Y_dist5)#Calculates the target position for the leg

        self.calculate_target_position(Leg2, self.leg_ICC_distance2, self.leg_base_ICC_distance2, self.X_dist2, self.Y_dist2)#Calculates the target position for the leg
        self.calculate_target_position(Leg4, self.leg_ICC_distance4, self.leg_base_ICC_distance4, self.X_dist4, self.Y_dist4)#Calculates the target position for the leg
        self.calculate_target_position(Leg6, self.leg_ICC_distance6, self.leg_base_ICC_distance6, self.X_dist6, self.Y_dist6)#Calculates the target position for the leg


    def calculate_target_position(self, leg_object, leg_ICC_distance, leg_base_ICC_distance, X_dist, Y_dist):
        #print("Calculate the leg target positions")
        X_target = leg_object.X_Coordinate
        Y_target = leg_object.Y_Coordinate
        Z_target = leg_object.Z_Coordinate

        if leg_object.step_number >= 0 and leg_object.step_number <= self.support_steps:#Supporting phase
            leg_stride_length = self.stride_length * leg_ICC_distance / self.max_ICC_distance#Scales stride length to the proportional size for the distance from the ICC
            alpha = 2 * math.asin(0.5 * leg_stride_length / leg_ICC_distance)
            alpha_step = alpha/self.support_steps
            beta = math.atan2(Y_dist, X_dist)

            if self.ICC[2] == "CW":
                beta = beta + alpha_step
            elif self.ICC[2] == "CCW":
                beta = beta - alpha_step
            elif self.ICC[2] == "Stop":
                return 0

            X_from_ICC = math.cos(beta) * leg_ICC_distance
            Y_from_ICC = math.sin(beta) * leg_ICC_distance

            X_target = X_from_ICC + self.ICC[0]
            Y_target = Y_from_ICC + self.ICC[1]
            Z_target = leg_object.Z_base_position
        elif leg_object.step_number > self.support_steps and leg_object.step_number <= self.support_steps + self.lifting_steps and self.ICC[2] != "Stop":#Lifting phase
            Z_target = leg_object.Z_Coordinate + self.stride_height/self.lifting_steps
        elif leg_object.step_number > self.support_steps + self.lifting_steps and leg_object.step_number <= self.support_steps + self.lifting_steps + self.swing_steps:#Swing phase
            leg_stride_length = self.stride_length * leg_base_ICC_distance / self.max_base_ICC_distance#Scales stride length to the proportional size for the distance from the ICC
            alpha = math.asin(0.5 * leg_stride_length / leg_base_ICC_distance)#need to check all this properly
            gamma = math.atan2(leg_object.Y_base_position - self.ICC[1], leg_object.X_base_position - self.ICC[0])

            if self.ICC[2] == "CW":
                beta = gamma - alpha
            elif self.ICC[2] == "CCW":
                beta = gamma + alpha
            elif self.ICC[2] == "Stop":
                return 0

            X_from_ICC = math.cos(beta) * leg_base_ICC_distance
            Y_from_ICC = math.sin(beta) * leg_base_ICC_distance

            X_final_target = X_from_ICC + self.ICC[0]
            Y_final_target = Y_from_ICC + self.ICC[1]

            X_target = leg_object.X_Coordinate + ((X_final_target - leg_object.X_Coordinate) / self.swing_steps) * (leg_object.step_number - (self.support_steps + self.lifting_steps))
            Y_target = leg_object.Y_Coordinate + ((Y_final_target - leg_object.Y_Coordinate) / self.swing_steps) * (leg_object.step_number - (self.support_steps + self.lifting_steps))

        elif leg_object.step_number > self.support_steps + self.lifting_steps + self.swing_steps and leg_object.step_number <= self.support_steps + self.lifting_steps + self.swing_steps + self.lower_steps  and self.ICC[2] != "Stop":#Lowering phase
            Z_target = leg_object.Z_Coordinate - self.stride_height/self.lower_steps
        elif self.ICC[2] != "Stop":
            print("Calculate target position failed")
            #Exit_Error("Calculate target position failed")

        #Sets the leg desired coordinates to the target coordinates
        leg_object.X_Coordinate = X_target
        leg_object.Y_Coordinate = Y_target
        leg_object.Z_Coordinate = Z_target

    def standUp(self):
        #Moves all legs to their stand foot positions (from the calibrated stand pose)
        print("standing up")
        for leg in (Leg1, Leg2, Leg3, Leg4, Leg5, Leg6):
            leg.X_Coordinate = leg.X_base_position
            leg.Y_Coordinate = leg.Y_base_position
            leg.Z_Coordinate = leg.Z_base_position
        leg_Inverse_Kinamatics()
        moveLegs()
        time.sleep(3)

    def leg_to_base_position(self, leg, XYnumber, zNumber):
        Z_change_step = self.stride_height/zNumber
        X_change_step = (leg.X_base_position - leg.X_Coordinate)/XYnumber
        Y_change_step = (leg.Y_base_position - leg.Y_Coordinate)/XYnumber
        
        for i in range(0, zNumber, 1):#for loop raises the leg
            leg.Z_Coordinate = leg.Z_Coordinate + Z_change_step
            leg.inverse_kinematics()#Calculates the inverse kinematics for the leg
            moveLegs()

        for i in range(0, XYnumber, 1):#for loop moves the leg to the X, Y base positions
            leg.X_Coordinate = leg.X_Coordinate + X_change_step
            leg.Y_Coordinate = leg.Y_Coordinate + Y_change_step
            leg.inverse_kinematics()#Calculates the inverse kinematics for the leg
            moveLegs() 

        leg.X_Coordinate = leg.X_base_position
        leg.Y_Coordinate = leg.Y_base_position
                       
        for i in range(zNumber, 0, -1):#for loop lowers the leg
            leg.Z_Coordinate = leg.Z_Coordinate - Z_change_step
            leg.inverse_kinematics()#Calculates the inverse kinematics for the leg
            moveLegs()
            
        self.reset_step_numbers()
        
    def all_legs_support(self, inverse_Speed):
        
        if self.ICC[2] == "Stop":
            zStep = self.stride_height/inverse_Speed
            for i in range (0, inverse_Speed, 1):
                #Each leg is lowered to its own stand height (Z_base_position)
                if Leg1.Z_Coordinate > Leg1.Z_base_position:
                    Leg1.Z_Coordinate = Leg1.Z_Coordinate - zStep
                    leg1flag = 0
                else:
                    leg1flag = 1
                if Leg2.Z_Coordinate > Leg2.Z_base_position:
                    Leg2.Z_Coordinate = Leg2.Z_Coordinate - zStep
                    leg2flag = 0
                else:
                    leg2flag = 1
                if Leg3.Z_Coordinate > Leg3.Z_base_position:
                    Leg3.Z_Coordinate = Leg3.Z_Coordinate - zStep
                    leg3flag = 0
                else:
                    leg3flag = 1
                if Leg4.Z_Coordinate > Leg4.Z_base_position:
                    Leg4.Z_Coordinate = Leg4.Z_Coordinate - zStep
                    leg4flag = 0
                else:
                    leg4flag = 1
                if Leg5.Z_Coordinate > Leg5.Z_base_position:
                    Leg5.Z_Coordinate = Leg5.Z_Coordinate - zStep
                    leg5flag = 0
                else:
                    leg5flag = 1
                if Leg6.Z_Coordinate > Leg6.Z_base_position:
                    Leg6.Z_Coordinate = Leg6.Z_Coordinate - zStep
                    leg6flag = 0
                else:
                    leg6flag = 1
                    
                if leg1flag == 0 or leg2flag == 0 or leg3flag == 0 or leg4flag == 0 or  leg5flag == 0 or leg6flag == 0:     
                    leg_Inverse_Kinamatics()#Calculate inverse kinematics
                    moveLegs()#Moves the leg servos
                else:
                    break
                
    def reposition_legs_to_base(self, inverse_speed):
        self.all_legs_support(inverse_speed)
        self.leg_to_base_position(Leg1, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg2, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg3, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg4, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg5, inverse_speed, inverse_speed)
        self.leg_to_base_position(Leg6, inverse_speed, inverse_speed)
        
    def legs_to_base_position(self, inverse_speed, LegA = "None", LegB = "None", LegC = "None"):
        self.all_legs_support(5)
    
        if not (LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6 or "None"):
            return 0

        if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
            LegA_X_step = (LegA.X_base_position - LegA.X_Coordinate)/inverse_speed
            LegA_Y_step = (LegA.Y_base_position - LegA.Y_Coordinate)/inverse_speed
            LegA_Z_step = self.stride_height/inverse_speed
            
        if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:
            LegB_X_step = (LegB.X_base_position - LegB.X_Coordinate)/inverse_speed
            LegB_Y_step = (LegB.Y_base_position - LegB.Y_Coordinate)/inverse_speed
            LegB_Z_step = self.stride_height/inverse_speed
            
        if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
            LegC_X_step = (LegC.X_base_position - LegC.X_Coordinate)/inverse_speed
            LegC_Y_step = (LegC.Y_base_position - LegC.Y_Coordinate)/inverse_speed
            LegC_Z_step = self.stride_height/inverse_speed
        
        for i in range(0, inverse_speed, 1):#for loop raises the leg
            if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
                LegA.Z_Coordinate = LegA.Z_Coordinate + LegA_Z_step
                LegA.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:  
                LegB.Z_Coordinate = LegB.Z_Coordinate + LegA_Z_step
                LegB.inverse_kinematics()#Calculates the inverse kinematics for the leg

            if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
                LegC.Z_Coordinate = LegC.Z_Coordinate + LegA_Z_step
                LegC.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            moveLegs()

        for i in range(0, inverse_speed, 1):#for loop moves the leg to the X, Y base positions
            if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
                LegA.X_Coordinate = LegA.X_Coordinate + LegA_X_step
                LegA.Y_Coordinate = LegA.Y_Coordinate + LegA_Y_step
                LegA.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:  
                LegB.X_Coordinate = LegB.X_Coordinate + LegB_X_step
                LegB.Y_Coordinate = LegB.Y_Coordinate + LegB_Y_step
                LegB.inverse_kinematics()#Calculates the inverse kinematics for the leg

            if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
                LegC.X_Coordinate = LegC.X_Coordinate + LegC_X_step
                LegC.Y_Coordinate = LegC.Y_Coordinate + LegC_Y_step 
                LegC.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            moveLegs()

        if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
            LegA.X_Coordinate = LegA.X_base_position
            LegA.Y_Coordinate = LegA.Y_base_position
        
        if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:
            LegB.X_Coordinate = LegB.X_base_position
            LegB.Y_Coordinate = LegB.Y_base_position
            
        if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
            LegC.X_Coordinate = LegC.X_base_position
            LegC.Y_Coordinate = LegC.Y_base_position
                       
        for i in range(inverse_speed, 0, -1):#for loop lowers the leg
            if LegA == Leg1 or LegA == Leg2 or LegA == Leg3 or LegA == Leg4 or LegA == Leg5 or LegA == Leg6:
                LegA.Z_Coordinate = LegA.Z_Coordinate - LegA_Z_step
                LegA.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            if LegB == Leg1 or LegB == Leg2 or LegB == Leg3 or LegB == Leg4 or LegB == Leg5 or LegB == Leg6:  
                LegB.Z_Coordinate = LegB.Z_Coordinate - LegA_Z_step
                LegB.inverse_kinematics()#Calculates the inverse kinematics for the leg

            if LegC == Leg1 or LegC == Leg2 or LegC == Leg3 or LegC == Leg4 or LegC == Leg5 or LegC == Leg6:
                LegC.Z_Coordinate = LegC.Z_Coordinate - LegA_Z_step
                LegC.inverse_kinematics()#Calculates the inverse kinematics for the leg
                
            moveLegs()
            
        self.reset_step_numbers()

    def tripod_legs_to_base_position(self, inverse_speed):
        self.legs_to_base_position(inverse_speed, LegA = Leg1, LegB = Leg4, LegC = Leg5)
        self.legs_to_base_position(inverse_speed, LegA = Leg2, LegB = Leg3, LegC = Leg6)
        
    def double_ripple_legs_to_base_position(self, inverse_speed):
        self.legs_to_base_position(inverse_speed, LegA = Leg1, LegB = Leg4)
        self.legs_to_base_position(inverse_speed, LegA = Leg3, LegB = Leg6)
        self.legs_to_base_position(inverse_speed, LegA = Leg5, LegB = Leg2)
        
    def update_step_numbers(self):
        if self.gaitType == "tripod":
            self.tripod_step_numbers()
        elif self.gaitType == "double_ripple":
            self.double_ripple_14_36_52()
        elif self.gaitType == "ripple_123456":
            self.ripple_123456_step_numbers()
        elif self.gaitType == "ripple_654321":
            self.ripple_654321_step_numbers()
        elif self.gaitType == "ripple_135246":
            self.ripple_135246_step_numbers()
        else:
            Exit_Error("Updating step numbers failed. (No matching gait type.)")

        if self.step_number == 0 and self.ICC[2] != "Stop":
            self.one_gait_cycle_flag = 1
            
        if self.one_gait_cycle_flag == 0:
            self.stride_length = 20
        else:
            self.stride_length = 45
            
    def change_gait(self, inverse_Support_Speed, numberOfSteps, gait_Type):
        self.all_legs_support(inverse_Support_Speed)#Puts all the legs on the ground
        self.reset_step_numbers()#Resets all the step numbers
        self.gaitType = gait_Type
        self.number_of_steps = numberOfSteps
        self.one_gait_cycle_flag = 0
        if self.gaitType == "tripod":
            self.tripod_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "double_ripple":
            self.double_ripple_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "ripple_123456":
            self.ripple_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "ripple_654321":
            self.ripple_Cycle_Steps(self.number_of_steps)
        elif self.gaitType == "ripple_135246":
            self.ripple_Cycle_Steps(self.number_of_steps)
        else:
            Exit_Error("Invalid gait type.")
            
    def ripple_135246(self):
        print("ripple_135246 gait")

    def ripple_145236(self):
        print("ripple_145236 gait")

    def catapilar(self):
        print("Catapilar gait")

    def quadrapod(self):
        print("Quadrapod gait")

#=================================================================================

class Eyes:
    rToggle = 0
    gToggle = 0
    bToggle = 0

    def toggleRed(self):
        Red = map_number(self.rToggle, 0, 1, 0, 4095)
        set_eye_channel(0, Red)#Sets the RGB Red
        self.rToggle ^= 1

    def toggleGreen(self):
        Green = map_number(self.gToggle, 0, 1, 0, 4095)
        set_eye_channel(1, Green)#Sets the RGB Green
        self.gToggle ^= 1

    def toggleBlue(self):
        Blue = map_number(self.bToggle, 0, 1, 0, 4095)
        set_eye_channel(2, Blue)#Sets the RGB Blue
        self.bToggle ^= 1

    def custom_eye_colour(self, R, G, B):
        Blue = map_number(B, 0, 255, 0, 4095)
        Green = map_number(G, 0, 255, 0, 4095)
        Red = map_number(R, 0, 255, 0, 4095)

        set_eye_channel(0, Red)#Sets RGB LED Red
        set_eye_channel(1, Green)#Sets RGB LED Green
        set_eye_channel(2, Blue)#Sets RGB LED Blue

    def red(self):#sets eye colour to red
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 0)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

    def green(self):#sets eye colour to green
        set_eye_channel(0, 0)#Sets RGB LED Red
        set_eye_channel(1, 4095)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

    def blue(self):#sets eye colour to blue
        set_eye_channel(0, 0)#Sets RGB LED Red
        set_eye_channel(1, 0)#Sets RGB LED Green
        set_eye_channel(2, 4095)#Sets RGB LED Blue

    def yellow(self):#sets eye colour to yellow
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 4095)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

    def violet(self):#sets eye colour to violet
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 0)#Sets RGB LED Green
        set_eye_channel(2, 4095)#Sets RGB LED Blue

    def turquoise(self):#sets eye colour to violet
        set_eye_channel(0, 0)#Sets RGB LED Red
        set_eye_channel(1, 4095)#Sets RGB LED Green
        set_eye_channel(2, 2048)#Sets RGB LED Blue

    def orange(self):#sets eye colour to orange
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 1024)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

    def purple(self):#sets eye colour to purple
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 0)#Sets RGB LED Green
        set_eye_channel(2, 3000)#Sets RGB LED Blue

    def pink(self):#sets eye colour to pink
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 0)#Sets RGB LED Green
        set_eye_channel(2, 2000)#Sets RGB LED Blue

    def white(self):#sets eye colour to white
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 4095)#Sets RGB LED Green
        set_eye_channel(2, 2250)#Sets RGB LED Blue

    def off(self):#sets eye colour to white
        set_eye_channel(0, 0)#Sets RGB LED Red
        set_eye_channel(1, 0)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

    def green_to_red(self):
        set_eye_channel(0, 0)#Sets RGB LED Red
        set_eye_channel(1, 4095)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

        for i in range(0, 4096, 8):
            set_eye_channel(0, i)#Sets RGB LED Red
            set_eye_channel(1, 4095-i)#Sets RGB LED Green

    def green_to_blue(self):
        set_eye_channel(0, 0)#Sets RGB LED Red
        set_eye_channel(1, 4095)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

        for i in range(0, 4096, 8):
            set_eye_channel(1, 4095-i)#Sets RGB LED Green
            set_eye_channel(2, i)#Sets RGB LED Blue

    def red_to_green(self):
        set_eye_channel(0, 4095)#Sets RGB LED Red
        set_eye_channel(1, 0)#Sets RGB LED Green
        set_eye_channel(2, 0)#Sets RGB LED Blue

        for i in range(0, 4096, 8):
            set_eye_channel(0, 4095-i)#Sets RGB LED Red
            set_eye_channel(1, i)#Sets RGB LED Green

    def change_colour(self, red_start_value, green_start_value, blue_start_value, red_end_value, green_end_value, blue_end_value, speed):
        set_eye_channel(0, red_start_value)#Sets RGB LED Red
        set_eye_channel(1, green_start_value)#Sets RGB LED Green
        set_eye_channel(2, blue_start_value)#Sets RGB LED Blue

        speed_variable = 4096 / speed

        red_step = (red_end_value - red_start_value) / speed_variable
        green_step = (green_end_value - green_start_value) / speed_variable
        blue_step = (blue_end_value - blue_start_value) / speed_variable


        for i in range(0, speed_variable, 1):
            set_eye_channel(0, red_start_value + red_step * i)#Sets RGB LED Red
            set_eye_channel(1, green_start_value + green_step * i)#Sets RGB LED Green
            set_eye_channel(2, blue_start_value + blue_step * i)#Sets RGB LED Blue

#=================================================================================
# LEG SETUP - NEW ROBOT
# Mount positions: front/mid/rear X = +115 / 0 / -115 mm, left Y = +60, right Y = -60
# Stand foot positions (base): from the calibrated stand pose, forward kinematics
#   (leg coordinates in body frame, Z relative to the coxa axis).
# Calibration (Cal_*): straight-leg raw servo angles from leg_calibration.json.
#=================================================================================

def setup_leg(leg, side, x_mount, y_mount, base_x, base_y, base_z, cal, stand):
    leg.Leg_Side = side
    leg.X_Offset = x_mount
    leg.Y_Offset = y_mount
    leg.Z_Offset = 0.0

    leg.X_base_position = base_x
    leg.Y_base_position = base_y
    leg.Z_base_position = base_z

    leg.X_Coordinate = base_x
    leg.Y_Coordinate = base_y
    leg.Z_Coordinate = base_z
    leg.Last_X = base_x
    leg.Last_Y = base_y
    leg.Last_Z = base_z

    leg.Cal_coxa, leg.Cal_femur, leg.Cal_tibia = cal
    leg.Stand_angles = stand

Leg1 = Leg()#Left front (LF)
Leg2 = Leg()#Right front (RF)
Leg3 = Leg()#Left middle (LM)
Leg4 = Leg()#Right middle (RM)
Leg5 = Leg()#Left rear (LR)
Leg6 = Leg()#Right rear (RR)

#Servo pulse limits (per servo, old robot values - verify on the robot)
Leg1.servoMin_S1 = 500; Leg1.servoMax_S1 = 2500
Leg1.servoMin_S2 = 500; Leg1.servoMax_S2 = 2500
Leg1.servoMin_S3 = 500; Leg1.servoMax_S3 = 2500

Leg2.servoMin_S1 = 500; Leg2.servoMax_S1 = 2500
Leg2.servoMin_S2 = 500; Leg2.servoMax_S2 = 2500
Leg2.servoMin_S3 = 500; Leg2.servoMax_S3 = 2500

Leg3.servoMin_S1 = 500; Leg3.servoMax_S1 = 2500
Leg3.servoMin_S2 = 500; Leg3.servoMax_S2 = 2500
Leg3.servoMin_S3 = 500; Leg3.servoMax_S3 = 2500

Leg4.servoMin_S1 = 500; Leg4.servoMax_S1 = 2500
Leg4.servoMin_S2 = 500; Leg4.servoMax_S2 = 2500
Leg4.servoMin_S3 = 500; Leg4.servoMax_S3 = 2500

Leg5.servoMin_S1 = 500; Leg5.servoMax_S1 = 2500
Leg5.servoMin_S2 = 500; Leg5.servoMax_S2 = 2500
Leg5.servoMin_S3 = 500; Leg5.servoMax_S3 = 2500

Leg6.servoMin_S1 = 500; Leg6.servoMax_S1 = 2500
Leg6.servoMin_S2 = 500; Leg6.servoMax_S2 = 2500
Leg6.servoMin_S3 = 500; Leg6.servoMax_S3 = 2500

#Left legs: (side, mount X, mount Y, stand foot X, Y, Z, calibration, stand angles)
setup_leg(Leg1, "Left",  115.0,  60.0, 115.00,  283.24, -75.41, (115, 93, 101), (115, 110, 40))
setup_leg(Leg3, "Left",    0.0,  60.0,   0.00,  282.17, -72.22, (90, 90, 84),   (90, 110, 20))
setup_leg(Leg5, "Left", -115.0,  60.0, -126.88,  286.65, -66.99, (87, 90, 91),   (90, 110, 30))

#Right legs
setup_leg(Leg2, "Right",  115.0, -60.0, 131.19, -291.55, -57.54, (98, 88, 94),   (102, 110, 35))
setup_leg(Leg4, "Right",    0.0, -60.0,   0.00, -301.59, -49.98, (90, 92, 84),   (90, 110, 35))
setup_leg(Leg6, "Right", -115.0, -60.0,-115.00, -293.65, -61.75, (103, 87, 90),  (103, 105, 35))

Gait = Gaits()
body = Body()

def leg_Inverse_Kinamatics():
    #Legs on the Left side
    Leg1.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg3.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg5.inverse_kinematics()#Calculates the inverse kinematics for the leg

    #Legs on the Right side
    Leg2.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg4.inverse_kinematics()#Calculates the inverse kinematics for the leg
    Leg6.inverse_kinematics()#Calculates the inverse kinematics for the leg

#=================================================================================

def moveLegs():#Function to set the leg positions (18 leg servos)
    for leg_name, legs in [("LF", Leg1), ("RF", Leg2), ("LM", Leg3),
                           ("RM", Leg4), ("LR", Leg5), ("RR", Leg6)]:
        coxa, femur, tibia = LEG_MAP[leg_name]
        set_servo(coxa, legs.Hip_Joint)
        set_servo(femur, legs.Femur_Joint)
        set_servo(tibia, legs.Tibia_Joint)