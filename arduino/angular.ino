#include <AccelStepper.h>

// Define pins for each stepper motor
#define stepPin1 53 // Front Left (53, 51)
#define dirPin1 51
#define stepPin2 48 // Front Right (48, 46)
#define dirPin2 46
#define stepPin3 49 // Rear Right (49, 47)
#define dirPin3 47
#define stepPin4 52 // Rear Left (52, 50)
#define dirPin4 50

// Constants
const int stepsPerRevolution = 500;
const int gearRatio = 5; // Gear ratio 1/5 means output speed = motor speed / 5
const float wheelDiameter = 15.0; // Wheel diameter in cm
const int rotationSpeed = 500; // Rotation speed in steps per second

// Initialize stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4);

void setup() {
  Serial.begin(9600); // Initialize serial communication

  // Set maximum speed (steps per second)
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
  stepper3.setMaxSpeed(1000);
  stepper4.setMaxSpeed(1000);

  Serial.println("Enter 'Z' to set the robot to zero position.");
  Serial.println("Enter 'A' followed by an angle (-360 to 360) to rotate the robot.");
  Serial.println("Enter 'M' followed by speed (cm/s) and duration (s) to move the robot.");
}

void loop() {
  if (Serial.available()) {
    char command = Serial.read();

    if (command == 'Z' || command == 'z') {
      // Set robot to zero position
      stepper1.setCurrentPosition(0);
      stepper2.setCurrentPosition(0);
      stepper3.setCurrentPosition(0);
      stepper4.setCurrentPosition(0);
      Serial.println("Robot set to zero position.");
    } else if (command == 'A' || command == 'a') {
      // Read the angle value
      float angle = Serial.parseFloat();
      if (angle >= -360 && angle <= 360) {
        // Calculate the number of steps based on the angle
        int steps = (angle / 360.0) * stepsPerRevolution * gearRatio;
        
        // Set the rotation direction based on the angle
        int direction = (angle >= 0) ? 1 : -1;
        
        // Set the speeds for clockwise or counterclockwise rotation
        stepper1.setSpeed(direction * rotationSpeed);
        stepper2.setSpeed(direction * rotationSpeed);
        stepper3.setSpeed(direction * rotationSpeed);
        stepper4.setSpeed(direction * rotationSpeed);
        
        // Move the stepper motors to the target position
        stepper1.move(steps);
        stepper2.move(steps);
        stepper3.move(steps);
        stepper4.move(steps);
        
        // Run the stepper motors until they reach the target position
        while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0 || 
               stepper3.distanceToGo() != 0 || stepper4.distanceToGo() != 0) {
          stepper1.run();
          stepper2.run();
          stepper3.run();
          stepper4.run();
        }
        
        Serial.print("Robot rotated by ");
        Serial.print(angle);
        Serial.println(" degrees.");
      } else {
        Serial.println("Invalid angle. Please enter a value between -360 and 360.");
      }
    } else if (command == 'M' || command == 'm') {
      // Read the speed and duration values
      float speed = Serial.parseFloat();
      float duration = Serial.parseFloat();
      if (duration > 0) {
        // Convert speed from cm/s to steps per second
        int sps = (abs(speed) / (PI * wheelDiameter)) * stepsPerRevolution * gearRatio;

        // Set the speeds based on the direction
        if (speed > 0) {
          stepper1.setSpeed(-sps); // Front Left
          stepper2.setSpeed(sps);  // Front Right
          stepper3.setSpeed(sps);  // Rear Right
          stepper4.setSpeed(-sps); // Rear Left
        } else {
          stepper1.setSpeed(sps);  // Front Left
          stepper2.setSpeed(-sps); // Front Right
          stepper3.setSpeed(-sps); // Rear Right
          stepper4.setSpeed(sps);  // Rear Left
        }

        Serial.print("Moving robot with speed: ");
        Serial.print(speed);
        Serial.print(" cm/s, Duration: ");
        Serial.print(duration);
        Serial.println(" seconds.");

        // Start timing
        unsigned long startTime = millis();
        bool isRunning = true;
        // Run the stepper motors for the specified duration
        while (isRunning) {
          stepper1.runSpeed();
          stepper2.runSpeed();
          stepper3.runSpeed();
          stepper4.runSpeed();
          // Stop the motors after the run duration
          if (millis() - startTime >= duration * 1000) {
            stepper1.setSpeed(0);
            stepper2.setSpeed(0);
            stepper3.setSpeed(0);
            stepper4.setSpeed(0);
            isRunning = false;
            Serial.println("Robot stopped after the specified duration.");
          }
        }
      } else {
        Serial.println("Invalid speed or duration values.");
      }
    } else {
      Serial.println("Invalid command.");
    }
  }
}
