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
    setMotorSpeeds(255, 255);
    Serial.print("Roll = ");
    Serial.print(getRoll());
    Serial.print(" | Pitch = ");
    Serial.println(getPitch());
    delay(50);
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
