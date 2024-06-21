#include <AccelStepper.h>

// Define pins for each stepper motor
#define stepPin1 53 //Front Left (53, 51)
#define dirPin1 51
#define stepPin2 48 //Front Right (48, 46)
#define dirPin2 46
#define stepPin3 49 //Rear Right (49, 47)
#define dirPin3 47
#define stepPin4 52 //Rear Left (52, 50)
#define dirPin4 50

// Constants
const int stepsPerRevolution = 200;
const int gearRatio = 5; // Gear ratio 1/5 means output speed = motor speed / 5

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

  // Set initial speed
  stepper1.setSpeed(0);
  stepper2.setSpeed(0);
  stepper3.setSpeed(0);
  stepper4.setSpeed(0);

  Serial.println("Masukkan kecepatan dalam RPM untuk masing-masing stepper motor (0-1000 RPM) dan durasi putaran dalam milidetik:");
}

void loop() {
  static char inputBuffer[32];
  static int inputIndex = 0;

  // Check if there is incoming serial data
  while (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if (incomingByte == '\n') {
      inputBuffer[inputIndex] = '\0'; // Null-terminate the input string

      // Split input string into speed values and run duration
      int speeds[4];
      unsigned long runDuration;
      char* token = strtok(inputBuffer, " ");
      for (int i = 0; i < 4 && token != NULL; i++) {
        speeds[i] = atoi(token);
        token = strtok(NULL, " ");
      }
      if (token != NULL) {
        runDuration = atol(token);
      } else {
        runDuration = 1000; // Default run duration if not provided
      }

      // Convert RPM to steps per second (sps)
      int sps1 = (speeds[0] ) ;
      int sps2 = (speeds[1] ) ;
      int sps3 = (speeds[2] ) ;
      int sps4 = (speeds[3] ) ;

      // Set the speed for each stepper motor
      stepper1.setSpeed(sps1);
      stepper2.setSpeed(sps2);
      stepper3.setSpeed(sps3);
      stepper4.setSpeed(sps4);

      // Display the set speeds
      Serial.print("SPEEDS ");
      Serial.print(sps1);
      Serial.print(" ");
      Serial.print(sps2);
      Serial.print(" ");
      Serial.print(sps3);
      Serial.print(" ");
      Serial.println(sps4);

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
        if (millis() - startTime >= runDuration) {
          stepper1.setSpeed(0);
          stepper2.setSpeed(0);
          stepper3.setSpeed(0);
          stepper4.setSpeed(0);
          isRunning = false;
          Serial.println("Motor berhenti setelah beberapa detik.");
        }
      }

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
}
