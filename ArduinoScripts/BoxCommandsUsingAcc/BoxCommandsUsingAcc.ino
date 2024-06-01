const float HorizontalSpeed = 1.2 * 1.1732 / 100 * 1 / 1000;    //[m/ms]
const float DownwardSpeed = 1.1 * 1.1732 / 100 * 1 / 1000;      //[m/ms]
const float UpwardSpeed = 1.1732 / 100 * 1 / 1000;              //[m/ms]
const float WheelDiameter = 31.1 / 1000;                        //[m]
const float BoxHeight = 0.9;                                    //[m] ~3ft
const float BoxWidth = 0.9;                                     //[m] ~3ft
const float RobotFrameLength = 150 / 1000;                      //[m]
const float RobotFrameWidth = 150 / 1000;                       //[m]
const float netDDistPerCycle = RobotFrameWidth - 22.77 / 1000;  //[M]
const float DiameterW2W = 125 / 1000;                           //[m]
// Global variables
float lastTime = 0.0;           // For time tracking
float integralErrorRoll = 0.0;  // For integral term in PID control for yaw
float lastErrorRoll = 0.0;      // For derivative term in PID control for yaw
float roll = 0.0;               // Current yaw angle
float desiredRoll = 0.0;        // Desired yaw angle
float RollError = 0;
#define pi 3.1415926535897932384626433832795

const float time4Width = (BoxWidth / HorizontalSpeed)/8.5;
const float time4Spin = 10000;  //[ms]
const float DownwardVelocity = netDDistPerCycle / (time4Width + time4Spin);
const float time4dd = BoxHeight / DownwardSpeed;

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

Adafruit_MPU6050 mpu;

// Define motor control pins for the left motor
const int enableLeftPin = 11;
const int input1LeftPin = 13;
const int input2LeftPin = 12;

// Define motor control pins for the right motor
const int enableRightPin = 10;
const int input3RightPin = 9;
const int input4RightPin = 8;

// Define variables for controlling motor speed and direction
int leftSpeed = 0;
int rightSpeed = 0;

void setup() {
  // Set motor control pins as outputs
  pinMode(enableLeftPin, OUTPUT);
  pinMode(input1LeftPin, OUTPUT);
  pinMode(input2LeftPin, OUTPUT);
  pinMode(enableRightPin, OUTPUT);
  pinMode(input3RightPin, OUTPUT);
  pinMode(input4RightPin, OUTPUT);

  // Enable the motors
  digitalWrite(enableLeftPin, 255);
  digitalWrite(enableRightPin, 255);

  digitalWrite(input1LeftPin, HIGH);
  digitalWrite(input2LeftPin, LOW);
  digitalWrite(input3RightPin, HIGH);
  digitalWrite(input4RightPin, LOW);


  // Initialize serial communication
  Serial.begin(9600);
  // Initialize the I2C communication
  Wire.begin();
  // Initialize the MPU6050 sensor
 Serial.begin(9600);
  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  // set accelerometer range to +-8G
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  // set gyro range to +- 500 deg/s
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  // set filter bandwidth to 21 Hz
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(100);
}

// Filter coefficient
const float alpha = 0.5;

void loop() {
  unsigned long timetotal = millis();
  Serial.print("timetotal: ");
  Serial.println(timetotal);
  if (millis()-timetotal < time4dd) {
    // Horizontal Motion While loop
    Serial.println("Starting Horizontal Motion");
    unsigned long timetotal = millis();
    while (millis() - timetotal < time4Width) {
      // Horizontal Motion
      Serial.println("Horizontal Motion");
      // Get new sensor events with the readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Roll Acceleratiion
      float accelRoll =  atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
      float gyroRollRate =  g.gyro.x * 180 / PI;

      // Timing
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTime) / 1000;
      lastTime = currentTime;

      // Calculate roll angle (angle of rotation around the z-axis)
      roll =  alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accelRoll;

      // Desired roll angle
      float desiredRoll = 90.0;  // Adjust this as needed

      // Calculate roll error
      float rollError = desiredRoll - roll;

      // PID parameters for yaw
      float k_p_roll = 8.0;  // Proportional gain for yaw
      float k_i_roll = 8.0;  // Integral gain for yaw
      float k_d_roll = 8.0;  // Derivative gain for yaw

      // Integral term
      integralErrorRoll += RollError * dt;

      // Derivative term
      float derivativeErrorRoll = (rollError - lastErrorRoll) / dt;
      lastErrorRoll = rollError;

      // PID control for yaw
      float rollCorrection = k_p_roll * rollError + k_i_roll * integralErrorRoll + k_d_roll * derivativeErrorRoll;

      // Adjust motor speeds based on the yaw correction
      leftSpeed = 255 + rollCorrection;
      rightSpeed = 255 - rollCorrection;

      // Apply constraints to motor speeds
      if (leftSpeed < 0) leftSpeed = 0;
      if (leftSpeed > 255) leftSpeed = 255;
      if (rightSpeed < 0) rightSpeed = 0;
      if (rightSpeed > 255) rightSpeed = 255;

      // Set motor speeds
      analogWrite(enableLeftPin, leftSpeed);
      analogWrite(enableRightPin, rightSpeed);

      // Print motor speeds and yaw angle for debugging
      Serial.print("Left speed: ");
      Serial.print(leftSpeed);
      Serial.print(", Right speed: ");
      Serial.print(rightSpeed);
      Serial.print(", Roll: ");
      Serial.println(roll);

      // Delay for stability
      delay(10);
    }


    // Spin Motion
    Serial.println("Starting Spin Motion");
    while (abs(roll - (-90)) > 2) {
      Serial.println("Spin Motion");
      // Get new sensor events with the readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Roll Acceleratiion
     float accelRoll =  atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
      float gyroRollRate =  g.gyro.x * 180 / PI;

      // Timing
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTime) / 1000;
      lastTime = currentTime;

      // Calculate roll angle (angle of rotation around the z-axis)
      roll =  alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accelRoll;

      // Set motor speeds
      analogWrite(enableLeftPin, 200);
      analogWrite(enableRightPin, 0);

      // Print motor speeds and yaw angle for debugging
      Serial.print("Left speed: ");
      Serial.print(255);
      Serial.print(", Right speed: ");
      Serial.print(0*rightSpeed);
      Serial.print(", Roll: ");
      Serial.println(roll);

      // Delay for stability
      delay(10);
    }
    Serial.println("Starting Horizontal Motion");
    // Horizontal Motion While loop
    unsigned long timetotal2 = millis();
    while (millis() - timetotal2 < time4Width) {
      // Horizontal Motion
      Serial.println("Starting Horizontal Motion");
      // Get new sensor events with the readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Roll Acceleratiion
      float accelRoll =  atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
      float gyroRollRate =  g.gyro.x * 180 / PI;

      // Timing
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTime) / 1000;
      lastTime = currentTime;

      // Calculate roll angle (angle of rotation around the z-axis)
      roll =  alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accelRoll;

      // Desired roll angle
      float desiredRoll = -90.0;  // Adjust this as needed

      // Calculate roll error
      float rollError = desiredRoll - roll;

      // PID parameters for yaw
      float k_p_roll = 8.0;  // Proportional gain for yaw
      float k_i_roll = 8.0;  // Integral gain for yaw
      float k_d_roll = 8.0;  // Derivative gain for yaw

      // Integral term
      integralErrorRoll += RollError * dt;

      // Derivative term
      float derivativeErrorRoll = (rollError - lastErrorRoll) / dt;
      lastErrorRoll = rollError;

      // PID control for yaw
      float rollCorrection = k_p_roll * rollError + k_i_roll * integralErrorRoll + k_d_roll * derivativeErrorRoll;

      // Adjust motor speeds based on the yaw correction
      leftSpeed = 255 + rollCorrection;
      rightSpeed = 255 - rollCorrection;

      // Apply constraints to motor speeds
      if (leftSpeed < 0) leftSpeed = 0;
      if (leftSpeed > 255) leftSpeed = 255;
      if (rightSpeed < 0) rightSpeed = 0;
      if (rightSpeed > 255) rightSpeed = 255;

      // Set motor speeds
      analogWrite(enableLeftPin, leftSpeed);
      analogWrite(enableRightPin, rightSpeed);

      // Print motor speeds and yaw angle for debugging
      Serial.print("Left speed: ");
      Serial.print(leftSpeed);
      Serial.print(", Right speed: ");
      Serial.print(rightSpeed);
      Serial.print(", Roll: ");
      Serial.println(roll);

      // Delay for stability
      delay(10);
    }


    // 2nd Spin Motion
    Serial.println("Starting Spin Motion Again");
    while (abs(roll - 90) > 2) {

      // Get new sensor events with the readings
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      // Roll Acceleratiion
      float accelRoll =  atan2(a.acceleration.y, a.acceleration.z) * 180 / PI;
      float gyroRollRate =  g.gyro.x * 180 / PI;

      // Timing
      unsigned long currentTime = millis();
      float dt = (currentTime - lastTime) / 1000;
      lastTime = currentTime;

      // Calculate roll angle (angle of rotation around the z-axis)
      roll =  alpha * (roll + gyroRollRate * dt) + (1 - alpha) * accelRoll;

      // Desired yaw angle
      float desiredRoll = 90;  // Adjust this as needed

      // Set motor speeds
      analogWrite(enableLeftPin, 0*leftSpeed);
      analogWrite(enableRightPin, 200);

      // Print motor speeds and yaw angle for debugging
      Serial.print("Left speed: ");
      Serial.print(0*leftSpeed);
      Serial.print(", Right speed: ");
      Serial.print(255);
      Serial.print(", Roll: ");
      Serial.println(roll);

      // Delay for stability
      delay(10);
    }
  }
  else {
    // Add any other code you want to run if millis() >= time4dd
    Serial.println("Loop Done");
    // Perform other tasks or stop the motors
    analogWrite(enableLeftPin, 0);
    analogWrite(enableRightPin, 0);
  }
}
