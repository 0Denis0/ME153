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
  
  //Read the accel-orientation
  
  // Read normalized values 
  Vector normAccel = mpu.readNormalizeAccel();

  // Calculate Pitch & Roll
//  int pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
//  int roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;

  float pitch = -(atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI;
  float roll = (atan2(normAccel.YAxis, normAccel.ZAxis)*180.0)/M_PI;
  float leftMotorMultiplier = 1;
  float rightMotorMultiplier = 1;

  // Read motor toggle button state
  if (digitalRead(motorToggleButtonPin) == LOW) {
    motorsOn = !motorsOn; // Toggle motor state
    delay(200); // Debounce delay
  }
  
  // Read direction flip button state
  if (digitalRead(directionFlipButtonPin) == LOW) {
    motorDirection *= -1; // Flip motor direction
    delay(200); // Debounce delay
  }

  
// Print Output
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  Serial.println();


  
  //PID control:
  const float rightReference = -90;
  const short CCW_isNegative = -1;
  float r = rightReference; //angle in degrees. Up is 0 degrees. Counter Clockwise angle is positive
  float e = r - roll;
  Perform_PID_control(255, e);


//  if (r > 180){ r -= 360;}
//  if (r < -180){ r += 360;}
  
//  float u = k_p*e + k_i*integralError;


//  if (CCW_isNegative*roll > r + 2){
//    leftSpeed  -= abs(u);
//    if (hardTurning){
//      rightSpeed += abs(u);
//      Serial.println("Hard turning Left");
//      }
//  }
//  else if (CCW_isNegative*roll < r - 2){
//    rightSpeed -= abs(u);
//    if (hardTurning){
//      leftSpeed  += abs(u);
//      Serial.println("Hard turning Right");
//      }
//  }

  integralError += e;
  //Simple Path, not working yet
//  if (time_ms < 1000*5){
//    r = rightReference - 90;
//    hardTurning = true;
//    Serial.println(" first 5 s");
//  }else if (time_ms < 1000*7){
//    r = rightReference - 180;
//    hardTurning = false;
//    Serial.println(" next 5 s");
//  
  
  Serial.print("Motor speeds L/R: ");
  Serial.print(leftSpeed);
  Serial.print("/");
  Serial.println(rightSpeed);
  delay(100);
}

void Perform_PID_control(int defaultSpeed, float error){
  // For now, I'm only implementing Proportional control
  // Assume angles increase in CCW direction
  // Hyperparameters
  const float MIN_CONTROL = 2; //minimum control to change speed in motors, unitless
  float leftSpeed, rightSpeed = defaultSpeed; 
  float k_p = 2;
  float k_i, k_d = 0;

  //Control input to the motor speed, u
  float  u_speedControl = k_p*error;

  if (u_speedControl > MIN_CONTROL) {
    rightSpeed = rightSpeed - u_speedControl;
  }
  else if (u_speedControl < MIN_CONTROL){
    leftSpeed = leftSpeed - u_speedControl;
  }

  //Do a lazy, inaccurate speed check. Better would to map() the speeds from (0,255)
  if (leftSpeed < 0) {leftSpeed = 0;}
  if (leftSpeed > 255) {leftSpeed = 255;}
  if (rightSpeed < 0) {rightSpeed = 0;}
  if (rightSpeed > 255){rightSpeed = 255;}
  
  setMotorSpeeds(leftSpeed, rightSpeed);
}


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
