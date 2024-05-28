# -*- coding: utf-8 -*-
"""
Created on Mon May 27 17:12:04 2024

@author: Chris Lew

"""
# Import libraries
import math


# Constant Parameters: 
# TRACKING_TOLERANCE = 0.1; 
# kp = 8 #proportional constant
# ki = 0 #integralError constant
# kd = 0 #derivativeError constant
# ANGLE_OF_GOING_RIGHT = -90; #angle (deg) the robot detects when moving right. Set to -90 if up is 0, CCW is positive.


# DEFAULT_SPEED = 255 #the regular speed the robot runs at
# MAX_SPEED = 255 #exactly 255
# CCW_ANGLE_DIRECTION = 1; #set to 1 if CCW is negative

#phi is the raw angle measurement (deg)
def calcMotorSpeed_PID(x, y, phi, x_desired, y_desired, integralError = 0, derivativeError = 0):
    # Constant Parameters: 
    TRACKING_TOLERANCE = 0.1; #an outside program needs to change the (x_desired,y_desired) 
    kp = 8 #proportional constant
    ki = 0 #integralError constant
    kd = 0 #derivativeError constant
    ANGLE_OF_GOING_RIGHT = -90; #angle (deg) the robot detects when moving right. Set to -90 if up is 0, CCW is positive.
    DEFAULT_SPEED = 255 #the regular speed the robot runs at
    MAX_SPEED = 255 #exactly 255
    MIN_SPEED = -255;
    CCW_ANGLE_DIRECTION = 1; #set to 1 if CCW is negative
    ######
    leftSpeed = DEFAULT_SPEED
    rightSpeed = DEFAULT_SPEED
    dx = x_desired - x;
    dy = y_desired - y;
    angleToDesired = math.degrees(math.atan2(dy, dx)); #
    currentAngle = phi - ANGLE_OF_GOING_RIGHT
    
    #Positive error means need to move CCW
    error = angleToDesired - currentAngle;
    error = CCW_ANGLE_DIRECTION * error;
    
    u = kp * error + ki * integralError + kd * derivativeError
    
    leftSpeed = max(MIN_SPEED, min(MAX_SPEED, leftSpeed - u)) 
    rightSpeed = max(MIN_SPEED, min(MAX_SPEED, rightSpeed + u))
    return (leftSpeed, rightSpeed) #return a tuple, of the new motor speeds



# # Unused Helper Function:
# def calculate_angle_to_desired(x, y, x_desired, y_desired):
#     angleToDesired =  math.atan2(y_desired - y, x_desired - x);
#     return angleToDesired

# # Unused Helper Function, calculate u for 
# def calculateControlOutput_u(error, integralError, derivativeError):
#     integralError += error
#     output = kp * error + ki * integralError + kd * derivativeError
#     #self.prev_error = error
#     return output


r = 10
dAngle = 45
angle = 180
while angle >= -180:
    x_desired = math.cos(angle*math.pi / 180)
    y_desired = math.sin(angle*math.pi / 180)
    out = calcMotorSpeed_PID(0, 0, 0, x_desired, y_desired);
    leftSpeed = out[0]
    rightSpeed = out[1]
    
    #string = "Go to: \t" + str(round(x_desired, 2)) + ", " + str(round(y_desired, 2))
    string = "";
    string2 = " angle = " + str(angle) + ")"
    string3 = "\nMotor Speeds: "
    str4 = str(round(leftSpeed, 2)) + ", " + str(round(rightSpeed, 2))
    print(string+string2+string3+str4 + "\n");
    angle -= dAngle

