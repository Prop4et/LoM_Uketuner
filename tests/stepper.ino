// Include the AccelStepper Library
#include <AccelStepper.h>

// Define pin connections
const int dirPin = 13;
const int stepPin = 12;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance
AccelStepper myStepper(motorInterfaceType, stepPin, dirPin);

void setup() {
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  myStepper.setMaxSpeed(600);
  myStepper.setAcceleration(150);
  myStepper.setSpeed(300);
  myStepper.moveTo(10000);
}

void loop() {
  // Change direction once the motor reaches target position
  // Move the motor one step
  myStepper.run();
}
