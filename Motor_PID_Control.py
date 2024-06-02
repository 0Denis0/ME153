# -*- coding: utf-8 -*-
"""
Created on Mon May 27 17:12:04 2024

@author: Chris Lew

"""
# Import libraries
import math
import numpy as np
import matplotlib.pyplot as plt
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
# def calcMotorSpeed_PID(x, y, phi, x_desired, y_desired, CCW_ANGLE_DIRECTION = 1, integralError = 0, derivativeError = 0):
#     '''
#     Parameters
#     ----------
#     x : float
#         current x position, positive is right.
#     y : float
#         current y position, positive is up.
#     phi : float
#         current direction, change CCW_ANGLE_DIRECTION based on positive direction.
#     x_desired : float
#         Desired x position.
#     y_desired : float
#         Desired y position.
#     CCW_ANGLE_DIRECTION: 1 or -1, default is 1
#         Input -1 if rotating CCW decreases angle reading
#     integralError : TYPE, optional
#         DESCRIPTION. The default is 0.
#     derivativeError : TYPE, optional
#         DESCRIPTION. The default is 0.

#     Returns
#     -------
#     leftSpeed : TYPE
#         DESCRIPTION.
#     TYPE
#         DESCRIPTION.
#     TYPE
#         DESCRIPTION.

#     '''
#     # Constant Parameters: 
#     TRACKING_TOLERANCE = 0.1; #an outside program needs to change the (x_desired,y_desired) 
#     kp = 8 #proportional constant
#     ki = 0 #integralError constant
#     kd = 0 #derivativeError constant
#     ANGLE_OF_GOING_RIGHT = -90; #angle (deg) the robot detects when moving right. Set to -90 if up is 0, CCW is positive.
#     DEFAULT_SPEED = 255 #the regular speed the robot runs at
#     MAX_SPEED = 255 #exactly 255
#     MIN_SPEED = -255;
#     CCW_ANGLE_DIRECTION = 1; #set to 1 if CCW is negative
#     ######
#     leftSpeed = DEFAULT_SPEED
#     rightSpeed = DEFAULT_SPEED
#     dx = x_desired - x;
#     dy = y_desired - y;
#     angleToDesired = math.degrees(math.atan2(dy, dx)); #
#     currentAngle = phi - ANGLE_OF_GOING_RIGHT
    
#     #Positive error means need to move CCW
#     error = angleToDesired - currentAngle;
#     error = CCW_ANGLE_DIRECTION * error;
#     integralError += error
#     u = kp * error + ki * integralError + kd * derivativeError
    
#     leftSpeed = max(MIN_SPEED, min(MAX_SPEED, leftSpeed - u)) 
#     rightSpeed = max(MIN_SPEED, min(MAX_SPEED, rightSpeed + u))
#     return (leftSpeed, rightSpeed, integralError) #return a tuple, of the new motor speeds


#phi is the raw angle measurement (deg)
def calcMotorSpeed_PID_TupleIn(currentPoseTuple, desiredPositionTuple, CCW_ANGLE_DIRECTION = -1, Y_UP_SENSE = -1, integralError = 0, derivativeError = 0, verbose = False, howVerbose = 0):
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
        (leftSpeed, rightSpeed, integralError) speeds as float vary from -255 to 255.
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
    ANGLE_OF_GOING_RIGHT = 90.0; #angle (deg) the robot detects when moving right. Set to -90 if up is 0, CCW is positive.
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
    # error = angleToDesired - currentAngle;
    # error = CCW_ANGLE_DIRECTION * error;
    
    
    error = calcAngleError(currentPoseTuple, desiredPositionTuple, CCW_ANGLE_DIRECTION, Y_UP_SENSE, ANGLE_OF_GOING_RIGHT)
    integralError += error;
    
    u = kp * error + ki * integralError + kd * derivativeError

    leftSpeedRAW = leftSpeed - u
    rightSpeedRAW = rightSpeed + u    
    leftSpeed = max(MIN_SPEED, min(MAX_SPEED, leftSpeed - u)) 
    rightSpeed = max(MIN_SPEED, min(MAX_SPEED, rightSpeed + u))
    motorSpeedTuple = (leftSpeed, rightSpeed, integralError)
    
    if (verbose and howVerbose > 0):
        str1 = "Motor PID Verbose Output:"
        if (howVerbose % 100 >= 2):
            str1 += "\n\tRaw Current Pose \t(x,y,phi) = " + str(currentPoseTuple)
            str1 += "\n\tRaw Desired Pos\t(x_des,y_des) = " + str(desiredPositionTuple)
            str1 += "\n\tCalculated current angle = \t" + str(round(currentAngle,3)) + " degrees"
        if (howVerbose % 100 >= 1):
            # Define a tuple
            str1 += "\n\tError \t\t= " + str(round(error, 3)) + " degrees"
            str1 += "\n\tCCW direction sense = " + str(CCW_ANGLE_DIRECTION)
            str1 += "\n\tControl u \t = " + str(round(u, 4))
            str1 += "\n\tRaw Speed \t = "+ str(round(leftSpeedRAW, 3)) + "\t/ " + str(round(rightSpeedRAW, 3))
        str1 += "\n\tOutput Speed = " + str(round(leftSpeed, 2)) + "\t/ " + str(round(rightSpeed, 2))
        if (howVerbose >= 100):
            print("Warning: Plotting every time can be slow")
            plot_vectors(currentPoseTuple, desiredPositionTuple, y_direction= Y_UP_SENSE, CCW_direction=CCW_ANGLE_DIRECTION, ANGLE_OF_GOING_RIGHT=ANGLE_OF_GOING_RIGHT)
        
        print(str1)

    return motorSpeedTuple #return a tuple, of the new motor speeds


def calcAngleError(currentPoseTuple, desiredPositionTuple, CCW_ANGLE_DIRECTION = -1, Y_UP_SENSE = -1, ANGLE_OF_GOING_RIGHT = 0, howVerbose = 0):
    x = currentPoseTuple[0];
    y = currentPoseTuple[1];
    phi = currentPoseTuple[2];
    x_desired = desiredPositionTuple[0];
    y_desired = desiredPositionTuple[1];
    # Same code as before
    dx = x_desired - x;
    dy = y_desired - y;
    if (Y_UP_SENSE == -1):
        dy = -dy;
    
    print("dx = "+ str(dx))
    print("dy = " + str(dy))
    
    angleToDesired = math.degrees(math.atan2(dy, dx)); #
    currentAngle = phi - ANGLE_OF_GOING_RIGHT    
    
    if not (Y_UP_SENSE == -1 and CCW_ANGLE_DIRECTION == -1):
        # print("Warning: If y is down, CCW angles will be negative already. Don't also set CCW_ANGLE_DIRECTION to -1, unless you want CCW to be actually be positive")
        print("might need to check ")
    
    #Positive error means need to move CCW
    error = angleToDesired - currentAngle;
    error = CCW_ANGLE_DIRECTION * error;
    if (error > 180):
        error = error - 360
    elif (error < -180):
        error = error + 360
    return error

def plot_vectors(vector, point_T, origin=(0, 0), y_direction=-1, CCW_direction=-1, ANGLE_OF_GOING_RIGHT=0, x_lim=None, y_lim=None, arrowLength = 1, plotString = None):
    x, y, angle = vector
    x_t, y_t = point_T
    x_0, y_0 = origin

    # Adjust the angle based on the direction
    if CCW_direction == 1:
        # Counter-clockwise
        adjusted_angle = ANGLE_OF_GOING_RIGHT + angle
    elif CCW_direction == -1:
        # Clockwise
        adjusted_angle = ANGLE_OF_GOING_RIGHT - angle
    else:
        raise ValueError("CCW_direction should be either 1 (CCW) or -1 (CW)")

    if (arrowLength is None):
        arrowLength = np.linalg.norm(np.array((x,y)) - np.array(origin))/4
        
        
    # Convert angle to radians for trigonometric functions
    adjusted_angle_rad = np.deg2rad(adjusted_angle)

    # Calculate the arrow components
    arrow_dx = arrowLength*np.cos(adjusted_angle_rad)
    arrow_dy = arrowLength*np.sin(adjusted_angle_rad)

    # Adjust coordinates relative to origin
    x = x + x_0
    y = y + y_0
    x_t = x_t + x_0
    y_t = y_t + y_0

    # Plot the origin
    plt.plot(x_0, y_0, 'k+', markersize=10)  # Black plus at the origin

    # Plot the point
    plt.plot(x, y, 'ro')  # Point at (x, y)

    # Plot the vector as an arrow
    plt.arrow(x, y, arrow_dx, arrow_dy, head_width=0.1*arrowLength, head_length=0.2*arrowLength, fc='r', ec='r')

    # Plot the point_T
    plt.plot(x_t, y_t, 'go')  # Point_T at (x_t, y_t)

    # Draw a dashed line between (x, y) and (x_t, y_t)
    plt.plot([x, x_t], [y, y_t], 'k--')  # Dashed line in black ('k--')

    # Set the aspect of the plot to be equal
    plt.gca().set_aspect('equal', adjustable='box')
    
    # Adjust y-axis direction
    if y_direction == -1:
        plt.gca().invert_yaxis()

    # Set the x and y limits if provided
    if x_lim is not None:
        plt.xlim(x_lim)
    if y_lim is not None:
        plt.ylim(y_lim)
        
        
    # Ensure the plot includes the arrow's head
    x_arrow_end = x + arrow_dx
    y_arrow_end = y + arrow_dy
    if x_lim is None:
        x_lim = (min(x_0, x, x_t, x_arrow_end) - 1, max(x_0, x, x_t, x_arrow_end) + 1)
    if y_lim is None:
        y_lim = (min(y_0, y, y_t, y_arrow_end) - 1, max(y_0, y, y_t, y_arrow_end) + 1)


        
    # Adding grid, labels, and title for clarity
    plt.grid(True)
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    if plotString is not None:
        plt.title(plotString)
    else:
        plt.title('Vector and Point Plot')
    
    # Show the plot
    plt.show()
    
def testErrorCalc(dx, dy, x_Robot, y_Robot, theta_Robot, howVerbose = 0, xlim = None, ylim = None, origin_test = (0,0), plotString_test = None, arrowLength_test = 1, y_direction_=-1, CCW_direction_=-1):
    x_test = x_Robot
    y_test = y_Robot
    theta_test = theta_Robot
    origin_ = np.array((0,0))
    robotPose_1 = np.array((x_test, y_test, theta_test))
    trackingPosition_1 = np.array((x_test+dx, y_test+dy))
    error = calcAngleError((x_Robot,y_Robot, theta_Robot), (x_Robot+dx,y_Robot+dy ))
    
    if (howVerbose >= 1):
        print("Plotted")    
        if not (y_direction_ == -1 and CCW_direction_ == -1):
            plotString_test = plotString_test + " COORDS?"
        str2 = ""
        if y_direction_ == -1:
            str2 += "y down"
        else:
            str2 += "y up (wtf?)"
        if CCW_direction_ == -1:
            str2 += ", CW positive"
        else:
            str2 += ", CCW positive (wtf?)"
        if (howVerbose >= 2):
            plotString_test = plotString_test + " e=" + str(round(error, 2)) + " deg, " + str2
        plot_vectors(robotPose_1, trackingPosition_1, x_lim = xlim, y_lim = ylim, origin = origin_, plotString = plotString_test, arrowLength=arrowLength_test, y_direction=-1, CCW_direction=-1)
        str1 = "Error = " + str(error) + "\n"
        print(str1)
    return error


# # Example usage:
# vector = (1, 2, 30)  # x, y, angle (degrees)
# point_T = (3, 4)     # x_t, y_t
# y_direction = -1      
# CCW_direction = 1    
# ANGLE_OF_GOING_RIGHT = 0  # 0 degrees is the reference angle pointing to the right

# plot_vectors(vector, point_T, (0,0), y_direction, CCW_direction, ANGLE_OF_GOING_RIGHT)
# print("Done")
# Coordinate system: x is right, y is down, 0 deg is right, CW is positive
# Test 1, track directly to the right
dx = 3
dy = 1
x_test = 0
y_test = 0
theta_test = 0

print("Test 1:")
testErrorCalc(dx, dy, x_test, y_test, theta_test, howVerbose = 1, plotString_test = "Test 1 ")


print("Test 2:")
testErrorCalc(dx=-100, dy=10,x_Robot = 100, y_Robot = 100, theta_Robot = -90, howVerbose = 1, plotString_test = "test 2 ", arrowLength_test= 100)


print("Test 3:")
testErrorCalc(dx=-100, dy=-10,x_Robot = 100, y_Robot = 100, theta_Robot = -90, howVerbose = 2, plotString_test = "test 3 ", arrowLength_test= 100)

print("Test 4:")
testErrorCalc(dx=100, dy=10,x_Robot = 100, y_Robot = 100, theta_Robot = -90, howVerbose = 2, plotString_test = "test 4 ", arrowLength_test= 100)
print("Test 5:")
testErrorCalc(dx=100, dy=100,x_Robot = 100, y_Robot = 100, theta_Robot = -90, howVerbose = 2, plotString_test = "test 5 ", arrowLength_test= 100)


print("Test 5:")
testErrorCalc(dx=100, dy=100,x_Robot = 100, y_Robot = 100, theta_Robot = -90, howVerbose = 2, plotString_test = "test 5 ", arrowLength_test= 100)
# npA = np.array((1, 2, 3))
# npB = np.array((7, 9))
# # Tester Code
# r = 10
# dAngle = 15
# angle = 180




# while angle >= -180:
#     x_desired = math.cos(angle*math.pi / 180)
#     y_desired = math.sin(angle*math.pi / 180)
#     #out = calcMotorSpeed_PID(0, 0, 0, x_desired, y_desired);
#     out = calcMotorSpeed_PID_TupleIn(npA, npB, verbose = True, howVerbose = 2)
#     leftSpeed = out[0]
#     rightSpeed = out[1]
    
#     #string = "Go to: \t" + str(round(x_desired, 2)) + ", " + str(round(y_desired, 2))
#     string = "";
#     string2 = " angle = " + str(angle) + ")"
#     string3 = "\nMotor Speeds: "
#     str4 = str(round(leftSpeed, 2)) + ", " + str(round(rightSpeed, 2))
#     print(string+string2+string3+str4 + "");
#     angle -= dAngle
    
    
    







