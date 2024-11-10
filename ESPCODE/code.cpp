#include <ESP32Servo.h>  // Include the ESP32Servo library

// Define servo motors
Servo baseMotor;
Servo link1Motor;
Servo link2Motor;
Servo gripperMotor;

// Define pins for the motors
const int basePin = 25;      // Base motor pin
const int link1Pin = 26;     // Link1 motor pin
const int link2Pin = 32;     // Link2 motor pin
const int gripperPin = 33;   // Gripper motor pin

// Speed control variables
int delayTime = 15;  // Default delay time (Medium speed)

void setup() {
  // Start the serial communication
  Serial.begin(115200);
  
  // Attach servos to their respective pins
  baseMotor.attach(basePin);
  link2Motor.attach(link2Pin);
  link1Motor.attach(link1Pin);
  gripperMotor.attach(gripperPin);

  // Set all servos to initial position (90 degrees)
  baseMotor.write(90);
  link2Motor.write(90);
  link1Motor.write(90);
  gripperMotor.write(90);

  delay(2000);  // Wait for servo initialization
}

void loop() {
  // Check if there's data from the serial port
  if (Serial.available() > 0) {
    // Read the incoming command
    String command = Serial.readStringUntil('\n');

    // Parse the command
    int separatorIndex = command.indexOf(':');
    String part = command.substring(0, separatorIndex);
    String value = command.substring(separatorIndex + 1);

    // Speed control command
    if (part == "speed") {
      if (value == "slow") {
        delayTime = 50;
      } else if (value == "medium") {
        delayTime = 25;
      } else if (value == "fast") {
        delayTime = 0;
      }
    } 
    // Servo control command
    else {
      int targetAngle = value.toInt();
      int currentAngle = 0;

      // Control the motors based on the part
      if (part == "base") {
        currentAngle = baseMotor.read();
        // Move the base motor gradually
        moveMotorSmoothly(baseMotor, currentAngle, targetAngle);
      } 

      else if (part == "link2") {
        currentAngle = link2Motor.read();
        // Move the link2 motor gradually
        moveMotorSmoothly(link2Motor, currentAngle, targetAngle);
      }

      else if (part == "link1") {
        currentAngle = link1Motor.read();
        // Move the link1 motor gradually
        moveMotorSmoothly(link1Motor, currentAngle, targetAngle);
      }
      
      else if (part == "gripper") {
        currentAngle = gripperMotor.read();
        // Move the gripper motor gradually
        moveMotorSmoothly(gripperMotor, currentAngle, targetAngle);
      }
    }
  }
}

// Function to move motor smoothly by 1 degree at a time
void moveMotorSmoothly(Servo &motor, int currentAngle, int targetAngle) {
  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      motor.write(angle);
      delay(delayTime);  // Apply delay for smooth movement
    }
  } else if (targetAngle < currentAngle) {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      motor.write(angle);
      delay(delayTime);  // Apply delay for smooth movement
    }
  }
}
