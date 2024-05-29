# -*- coding: utf-8 -*-
"""
Created on Mon May 27 17:12:04 2024

@author: Chris Lew

"""
# Import libraries
import math
import numpy as np

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
def calcMotorSpeed_PID(x, y, phi, x_desired, y_desired, CCW_ANGLE_DIRECTION = 1, integralError = 0, derivativeError = 0):
    '''
    Parameters
    ----------
    x : float
        current x position, positive is right.
    y : float
        current y position, positive is up.
    phi : float
        current direction, change CCW_ANGLE_DIRECTION based on positive direction.
    x_desired : float
        Desired x position.
    y_desired : float
        Desired y position.
    CCW_ANGLE_DIRECTION: 1 or -1, default is 1
        Input -1 if rotating CCW decreases angle reading
    integralError : TYPE, optional
        DESCRIPTION. The default is 0.
    derivativeError : TYPE, optional
        DESCRIPTION. The default is 0.

    Returns
    -------
    leftSpeed : TYPE
        DESCRIPTION.
    TYPE
        DESCRIPTION.
    TYPE
        DESCRIPTION.

    '''
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
    integralError += error
    u = kp * error + ki * integralError + kd * derivativeError
    
    leftSpeed = max(MIN_SPEED, min(MAX_SPEED, leftSpeed - u)) 
    rightSpeed = max(MIN_SPEED, min(MAX_SPEED, rightSpeed + u))
    return (leftSpeed, rightSpeed, integralError) #return a tuple, of the new motor speeds


#phi is the raw angle measurement (deg)
def calcMotorSpeed_PID_TupleIn(currentPoseTuple, desiredPositionTuple, CCW_ANGLE_DIRECTION = 1, Y_UP_SENSE = 1, integralError = 0, derivativeError = 0, verbose = False):
    '''Parameters
    ----------
    robotPoseTuple : Tuple of floats
        (x, y, phi) Tuple describing the robot's current position and heading: .
    desiredPointTuple : Tuple of floats
        (x_desired, y_desired) Position of robot's current point it's tracking towards.
    CCW_ANGLE_DIRECTION: 1 or -1, default is 1
        Input -1 if rotating CCW decreases angle reading
    integralError : float, optional
        Integral Error, whatever way you like to calculate. The default is 0.
    derivativeError : float, optional
        Derivative Error, whatever way you like to calculate. The default is 0.

    Returns
    -------
    motorSpeedTuple : Tuple of Floats
        (leftSpeed, rightSpeed) speeds as float vary from -255 to 255.
    '''
    # Process Input
    x = currentPoseTuple[0];
    y = currentPoseTuple[1];
    phi = currentPoseTuple[2];
    x_desired = desiredPositionTuple[0];
    y_desired = desiredPositionTuple[1];
    # Constant Parameters: 
    kp = 1.5 #proportional constant
    ki = 0.0 #integralError constant
    kd = 0.0 #derivativeError constant
    ANGLE_OF_GOING_RIGHT = -90.0; #angle (deg) the robot detects when moving right. Set to -90 if up is 0, CCW is positive.
    DEFAULT_SPEED = 255 #the regular speed the robot runs at
    MAX_SPEED = 255.0 #exactly 255
    MIN_SPEED = -255.0;
    ######

    # Same code as before
    dx = x_desired - x;
    dy = y_desired - y;
    if (Y_UP_SENSE == -1):
        dy = -dy;
    
    leftSpeed = DEFAULT_SPEED
    rightSpeed = DEFAULT_SPEED
    angleToDesired = math.degrees(math.atan2(dy, dx)); #
    currentAngle = phi - ANGLE_OF_GOING_RIGHT
    
    #Positive error means need to move CCW
    error = angleToDesired - currentAngle;
    error = CCW_ANGLE_DIRECTION * error;
    integralError += error;
    
    u = kp * error + ki * integralError + kd * derivativeError

    leftSpeedRAW = leftSpeed - u
    rightSpeedRAW = rightSpeed + u    
    leftSpeed = max(MIN_SPEED, min(MAX_SPEED, leftSpeed - u)) 
    rightSpeed = max(MIN_SPEED, min(MAX_SPEED, rightSpeed + u))
    motorSpeedTuple = (leftSpeed, rightSpeed)
    
    if (verbose):
        str1 = "Motor PID Verbose Output:"
        # Define a tuple
        my_tuple = (1, 2, 3, 4, 5)
        
        # Convert the tuple elements to strings and join them with commas
        result = ', '.join(str(x) for x in my_tuple)
        str1 += "\n\tError = " + str(error) + " degrees"
        str1 += "\n\tCCW direction sense: " + str(CCW_ANGLE_DIRECTION)
        str1 += "\n\tControl u \t= " + str(u)
        str1 += "\n\tLeft/Right \t= " + str(leftSpeed) + "/" + str(rightSpeed)
        str1 += "\n\tRaw Speed \t= "+ str(leftSpeedRAW) + "/" + str(rightSpeedRAW)
        print(str1)
    return motorSpeedTuple #return a tuple, of the new motor speeds

#phi is the raw angle measurement (deg)
def calcMotorSpeed_PID_np(currentPoseNPArray, desiredPositionNPArray, CCW_ANGLE_DIRECTION = 1, Y_UP_SENSE = 1, integralError = 0, derivativeError = 0):
    '''Parameters
    ----------
    robotPoseTuple : (2x1 np array)
        (x, y, phi) Tuple describing the robot's current position and heading: .
    desiredPointTuple : (2x1 np array)
        (x_desired, y_desired) Position of robot's current point it's tracking towards.
    CCW_ANGLE_DIRECTION: 1 or -1, default is 1
        Input -1 if rotating CCW decreases angle reading
    integralError : float, optional
        Integral Error, whatever way you like to calculate. The default is 0.
    derivativeError : float, optional
        Derivative Error, whatever way you like to calculate. The default is 0.

    Returns
    -------
    motorSpeedTuple : Tuple of Floats
        (leftSpeed, rightSpeed) speeds as float vary from -255 to 255.
    '''
    # Process Input
    x = currentPoseNPArray[0];
    y = currentPoseNPArray[1];
    phi = currentPoseNPArray[2];
    x_desired = desiredPositionNPArray[0];
    y_desired = desiredPositionNPArray[1];
    # Constant Parameters: 
    kp = 8 #proportional constant
    ki = 0 #integralError constant
    kd = 0 #derivativeError constant
    ANGLE_OF_GOING_RIGHT = -90; #angle (deg) the robot detects when moving right. Set to -90 if up is 0, CCW is positive.
    DEFAULT_SPEED = 255 #the regular speed the robot runs at
    MAX_SPEED = 255 #exactly 255
    MIN_SPEED = -255;
    ######

    # Same code as before
    dx = x_desired - x;
    dy = y_desired - y;
    if (Y_UP_SENSE == -1):
        dy = -dy;
    
    leftSpeed = DEFAULT_SPEED
    rightSpeed = DEFAULT_SPEED
    angleToDesired = math.degrees(math.atan2(dy, dx)); #
    currentAngle = phi - ANGLE_OF_GOING_RIGHT
    
    #Positive error means need to move CCW
    error = angleToDesired - currentAngle;
    error = CCW_ANGLE_DIRECTION * error;
    integralError += error;
    
    u = kp * error + ki * integralError + kd * derivativeError
    
    leftSpeed = max(MIN_SPEED, min(MAX_SPEED, leftSpeed - u)) 
    rightSpeed = max(MIN_SPEED, min(MAX_SPEED, rightSpeed + u))
    motorSpeedTuple = (leftSpeed, rightSpeed)
    return motorSpeedTuple #return a tuple, of the new motor speeds

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


# Tester Code
r = 10
dAngle = 15
angle = 180
while angle >= -180:
    x_desired = math.cos(angle*math.pi / 180)
    y_desired = math.sin(angle*math.pi / 180)
    #out = calcMotorSpeed_PID(0, 0, 0, x_desired, y_desired);
    out = calcMotorSpeed_PID_TupleIn((0,0, 0), (x_desired, y_desired), verbose = True)
    leftSpeed = out[0]
    rightSpeed = out[1]
    
    #string = "Go to: \t" + str(round(x_desired, 2)) + ", " + str(round(y_desired, 2))
    string = "";
    string2 = " angle = " + str(angle) + ")"
    string3 = "\nMotor Speeds: "
    str4 = str(round(leftSpeed, 2)) + ", " + str(round(rightSpeed, 2))
    print(string+string2+string3+str4 + "");
    angle -= dAngle

