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
int speedCoefficient = 0;

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
}

void loop() {
  // Read the potentiometer value
  int potValue = analogRead(potPin);
  
  // Map the potentiometer value to a speed coefficient (0-255)
  speedCoefficient = map(potValue, 0, 1023, 0, 255);
  
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
  
  // If motors are on, set speeds according to potentiometer
  if (motorsOn) {
    // Demonstration of turning
    // Full speed for 3 seconds
    setMotorSpeeds(255, 255);
    delay(3000);

    // Left at half speed, right at full speed for 2 seconds
    setMotorSpeeds(127, 255);
    delay(2000);

    // Left stops, right continues at full speed for 1 second
    setMotorSpeeds(0, 255);
    delay(1000);

    // Right stops, left at full speed for 2 seconds
    setMotorSpeeds(255, 0);
    delay(2000);
  } else {
    // Turn off motors
    setMotorSpeeds(0, 0);
  }
  
  iteration += 1;
}
