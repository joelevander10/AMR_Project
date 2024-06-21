#include <AccelStepper.h>

// Define pins for each stepper motor
#define stepPin1 53 // Front Left
#define dirPin1 51
#define stepPin2 48 // Front Right
#define dirPin2 46
#define stepPin3 49 // Rear Right
#define dirPin3 47
#define stepPin4 52 // Rear Left
#define dirPin4 50

// Initialize stepper motors
AccelStepper stepper1(AccelStepper::DRIVER, stepPin1, dirPin1);
AccelStepper stepper2(AccelStepper::DRIVER, stepPin2, dirPin2);
AccelStepper stepper3(AccelStepper::DRIVER, stepPin3, dirPin3);
AccelStepper stepper4(AccelStepper::DRIVER, stepPin4, dirPin4);

void setup() {
  Serial.begin(9600); // Initialize serial communication
  
  // Set maximum speed and acceleration for each stepper motor
  stepper1.setMaxSpeed(500);
  stepper1.setAcceleration(500);
  stepper2.setMaxSpeed(500);
  stepper2.setAcceleration(500);
  stepper3.setMaxSpeed(500);
  stepper3.setAcceleration(500);
  stepper4.setMaxSpeed(500);
  stepper4.setAcceleration(500);
}

void loop() {
  static char inputBuffer[32];
  static int inputIndex = 0;
  
  // Check if there is incoming serial data
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if (incomingByte == '\n') {
      inputBuffer[inputIndex] = '\0'; // Null-terminate the input string
      
      // Split input string into speed values
      int speeds[4];
      char* token = strtok(inputBuffer, " ");
      for (int i = 0; i < 4 && token != NULL; i++) {
        speeds[i] = atoi(token);
        token = strtok(NULL, " ");
      }
      
      // Set the speed for each stepper motor
      stepper1.setSpeed(speeds[0]);
      stepper2.setSpeed(speeds[1]);
      stepper3.setSpeed(speeds[2]);
      stepper4.setSpeed(speeds[3]);
      
      // Reset the input buffer index
      inputIndex = 0;
    } else {
      // Add the incoming byte to the input buffer
      inputBuffer[inputIndex++] = incomingByte;
      // Prevent buffer overflow
      if (inputIndex >= sizeof(inputBuffer) - 1) {
        inputIndex = sizeof(inputBuffer) - 1;
      }
    }
  }
  
  // Run the stepper motors
  stepper1.runSpeed();
  stepper2.runSpeed();
  stepper3.runSpeed();
  stepper4.runSpeed();
}
