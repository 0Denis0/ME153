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


  



  //Old code:
  // Read the potentiometer value
  int potValue = analogRead(potPin);
  // Example: Hardcoded speeds
  leftSpeed = speedCoefficient * motorDirection * leftMotorMultiplier; // Speed for the left motor (0-255)
  rightSpeed = speedCoefficient * motorDirection * rightMotorMultiplier; // Speed for the right motor (0-255)
    
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

//  leftSpeed = speedCoefficient * motorDirection * leftMotorMultiplier; // Speed for the left motor (0-255)
//  rightSpeed = speedCoefficient * motorDirection * rightMotorMultiplier; // Speed for the right motor (0-255)
  
// Print Output
  Serial.print(" Pitch = ");
  Serial.print(pitch);
  Serial.print(" Roll = ");
  Serial.print(roll);
  
  Serial.println();

  
  //PID control:
  float r = 60; //angle in degrees. from +x. Clockwise angle is positive
  float e = r - roll;
  
  
  float k_p = 8;
  float u = k_p*e + 0.0000*integralError;


  leftSpeed = 255*motorDirection;
  rightSpeed = 255*motorDirection;

  if (roll > r + 2){
    leftSpeed  -= abs(u);
    rightSpeed += abs(u);

    Serial.print("\t\tTurning Left by u= ");
    Serial.print(u);
    Serial.println();
  }
  else if (roll < r - 2){
    rightSpeed -= abs(u);
    leftSpeed  += abs(u);

    Serial.print("\t\tTurning Right by u= ");
    Serial.print(u);
    Serial.println();
  }
  if (leftSpeed < 0) {leftSpeed = 0;}
  if (leftSpeed > 255) {leftSpeed = 255;}
  if (rightSpeed < 0) {rightSpeed = 0;}
  if (rightSpeed > 255){rightSpeed = 255;}

  integralError += e;
//  leftSpeed = 255;
//  rightSpeed = 255;
  setMotorSpeeds(leftSpeed, rightSpeed);

  Serial.print("Motor speeds L/R: ");
  Serial.print(leftSpeed);
  Serial.print("/");
  Serial.println(rightSpeed);
  delay(10);
}
