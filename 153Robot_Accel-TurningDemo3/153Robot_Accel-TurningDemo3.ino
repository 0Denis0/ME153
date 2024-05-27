/*
    MPU6050 Triple Axis Gyroscope & Accelerometer. Pitch & Roll Accelerometer Example.
    Read more: http://www.jarzebski.pl/arduino/czujniki-i-sensory/3-osiowy-zyroskop-i-akcelerometr-mpu6050.html
    GIT: https://github.com/jarzebski/Arduino-MPU6050
    Web: http://www.jarzebski.pl
    (c) 2014 by Korneliusz Jarzebski
*/
//Include libraries
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// PID control parameters
const float k_p = 8;
const float k_i = 1;
const float k_d = 0;
const float k_p_turning = 1;
const float k_i_turning = 0;
const float k_d_turning = 0;
float angleReferenceGlobal = 0;
const short CCW_isNegative = -1; //set -1 if CCW is negative
const short ANGLE_IS_UP = 0; //what angle the robot receives when goes upward


// Define motor control pins for the left motor
const int enableLeftPin = 11;
const int input1LeftPin = 13;
const int input2LeftPin = 12;

// Define motor control pins for the right motor
const int enableRightPin = 10;
const int input3RightPin = 9;
const int input4RightPin = 8;

// Define potentiometer pin
const int potPin = A0;

// Define button pins
const int motorToggleButtonPin = 3;
const int directionFlipButtonPin = 2;

// Define variable for speed coefficient
int speedCoefficient = 1;

// Variable to keep track of motor state
bool motorsOn = true;

// Variable to keep track of motor direction
int motorDirection = 1; // 1 for forward, -1 for reverse

long iteration = 0; // iteration of the loop() 


long integralError = 0;

void setup() {
  // Set motor control pins as outputs
  pinMode(enableLeftPin, OUTPUT);
  pinMode(input1LeftPin, OUTPUT);
  pinMode(input2LeftPin, OUTPUT);
  pinMode(enableRightPin, OUTPUT);
  pinMode(input3RightPin, OUTPUT);
  pinMode(input4RightPin, OUTPUT);
  
  // Set button pins as inputs
  pinMode(motorToggleButtonPin, INPUT_PULLUP);
  pinMode(directionFlipButtonPin, INPUT_PULLUP);

  Serial.begin(115200);

  Serial.println("Initialize MPU6050");

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
}

void loop() {
  unsigned long time_ms; //time in milliseconds since program started
  time_ms = millis();
  int leftSpeed;
  int rightSpeed;
  float previousError = 0;
  float leftMotorMultiplier = 1;
  float rightMotorMultiplier = 1;
  int angleToTheRight = -90;
  int newAngle = angleToTheRight;
  int angleMult = 1;
  int numCrossedBoard = 0;

  int timeToTravelAcross = 5*1000;//time (ms)to travel right/left the width of the board
  int timeToTravelDown = 3*1000; //time (ms) to travel down by the width of the eraser
  
  while (numCrossedBoard < 3){
    Go_Straight_PID(timeToTravelAcross, newAngle);
    Rotate_CCW_PID(-90*angleMult);
    newAngle = newAngle - 90*angleMult;
    Go_Straight_PID(timeToTravelDown, newAngle);
    Rotate_CCW_PID(-90*angleMult);
    angleMult *= -1;
    newAngle = angleToTheRight+180;
    if(newAngle > 180){
      newAngle -= 360;
    }
    numCrossedBoard++;
    Serial.print("Crossed Board");
    Serial.print(numCrossedBoard);
    Serial.println(" times");
  }
  

  

  
// Print Output
  Serial.print(" Pitch = ");
  Serial.print(getPitch());
  Serial.print(" Roll = ");
  Serial.print(getRoll());
  Serial.println();
  int userInput1 = -1;
  while (userInput1 != 2){
    Serial.println("Type 2 to restart");
    int userInput1 = Serial.parseInt();
  }
}

void Go_Straight_PID(int duration_ms, float directionAngle){
  //This is a blocking method. It controls the motors for duration_ms.
  //duration_ms is the duration of going in a straight line, in milliseconds
  //directionAngle is the desired angle you want to go to. 0 deg is normal to accelerometer
  
  long timeStart = millis();
  const int defaultSpeed = 50;
  float leftSpeed, rightSpeed; 
  float r = directionAngle; //angle in degrees. Up is 0 degrees. Counter Clockwise angle is positive
  const float MIN_CONTROL = 0.8;
  float this_integralError = 0;
  float e;
  float u_speedControl;
  while (millis() < timeStart + duration_ms){
    e = r - getRoll(); //Error is Positive if need to go more CCW
    u_speedControl = k_p*e+ k_i*this_integralError;
    u_speedControl *= CCW_isNegative; //multiply by negative 1 if CCW is negative
    leftSpeed = defaultSpeed;
    rightSpeed = defaultSpeed;
    if (u_speedControl > MIN_CONTROL) {
      rightSpeed = rightSpeed - u_speedControl;
    }
    else if (u_speedControl < MIN_CONTROL){
      leftSpeed = leftSpeed - u_speedControl;
    }
    if (leftSpeed < 0) {leftSpeed = 0;}
    if (leftSpeed > 255) {leftSpeed = 255;}
    if (rightSpeed < 0) {rightSpeed = 0;}
    if (rightSpeed > 255){rightSpeed = 255;}
    setMotorSpeeds(leftSpeed, rightSpeed);
    Serial.print(leftSpeed);
    Serial.print(" / ");
    Serial.println(rightSpeed);
  }
}

void Rotate_CCW_PID(float directionAngle){
  long timeStart = millis();
  float startAngle = getRoll();
  float leftSpeed, rightSpeed = 0; 
  float r = directionAngle; //angle in degrees. Up is 0 degrees. Counter Clockwise angle is positive
  const float MIN_ERROR = 0.1;
  float this_integralError = 0;
  float e;
  float u_speedControl;
  bool notDoneTurning = true;
  while (notDoneTurning){
    e = r - getRoll();
    u_speedControl = k_p*e+ k_i*this_integralError;
    u_speedControl *= CCW_isNegative; //multiply by negative 1 if CCW is negative
    leftSpeed = 0;
    rightSpeed = 0;
    if (u_speedControl > 0){ //Reference is CCW of current angle
      rightSpeed += u_speedControl;
      leftSpeed -= u_speedControl;
    }
    else if (u_speedControl < 0){
      leftSpeed += u_speedControl;
      rightSpeed -= u_speedControl;
    
    }
    if (leftSpeed < 0) {leftSpeed = 0;}
    if (leftSpeed > 255) {leftSpeed = 255;}
    if (rightSpeed < 0) {rightSpeed = 0;}
    if (rightSpeed > 255){rightSpeed = 255;}
    setMotorSpeeds(leftSpeed, rightSpeed);
    Serial.print(leftSpeed);
    Serial.print(" / ");
    Serial.println(rightSpeed);
    if (e < MIN_ERROR){
      notDoneTurning = false;
    }
    this_integralError += e;
  }//end while loop
  float timeTaken = (millis() - timeStart)/1000.0;
  float angleChanged = getRoll() - startAngle;
  Serial.print("Angle rotated by (degrees): ");
  Serial.println(angleChanged);
  Serial.print("Time taken to rotate (seconds): ");
  Serial.println(timeTaken);
}//end Rotate_CCW_PID() function



//void Perform_PID_control(int defaultSpeed, float error){//don't use
//  // For now, I'm only implementing Proportional control
//  // Assume angles increase in CCW direction
//  // Hyperparameters
//  const float MIN_CONTROL = 2; //minimum control to change speed in motors, unitless
//  float leftSpeed, rightSpeed = defaultSpeed; 
//  float k_p = 2;
//  float k_i, k_d = 0;
//
//  //Control input to the motor speed, u
//  float  u_speedControl = k_p*error;
//
//  if (u_speedControl > MIN_CONTROL) {
//    rightSpeed = rightSpeed - u_speedControl;
//  }
//  else if (u_speedControl < MIN_CONTROL){
//    leftSpeed = leftSpeed - u_speedControl;
//  }
//
//  //Do a lazy, inaccurate speed check. Better would to map() the speeds from (0,255)
//  if (leftSpeed < 0) {leftSpeed = 0;}
//  if (leftSpeed > 255) {leftSpeed = 255;}
//  if (rightSpeed < 0) {rightSpeed = 0;}
//  if (rightSpeed > 255){rightSpeed = 255;}
//  
//  setMotorSpeeds(leftSpeed, rightSpeed);
//}


// Function to set motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Set left motor direction
  if (leftSpeed >= 0) {
    digitalWrite(input1LeftPin, HIGH);
    digitalWrite(input2LeftPin, LOW);
  } else {
    digitalWrite(input1LeftPin, LOW);
    digitalWrite(input2LeftPin, HIGH);
    leftSpeed = -leftSpeed;
  }
  
  // Set right motor direction
  if (rightSpeed >= 0) {
    digitalWrite(input3RightPin, HIGH);
    digitalWrite(input4RightPin, LOW);
  } else {
    digitalWrite(input3RightPin, LOW);
    digitalWrite(input4RightPin, HIGH);
    rightSpeed = -rightSpeed;
  }
  
  // Set motor speeds
  analogWrite(enableLeftPin, leftSpeed);
  analogWrite(enableRightPin, rightSpeed);
}


float getPitch(){
  Vector normAccel = mpu.readNormalizeAccel();  
  float pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  return pitch;
}

float getRoll(){
    Vector normAccel = mpu.readNormalizeAccel();
    float roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
    return roll;
}
