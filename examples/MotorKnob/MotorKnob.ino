/*
 * MotorKnob
 *
 * A stepper motor follows the turns of a potentiometer
 * (or other sensor) on analog input 0.
 *
 * https://docs.arduino.cc/learn/electronics/stepper-motors
 * This example code is in the public domain.
 */

#include <Stepper.h>

// change this to the number of steps on your motor
#define STEPS 100

// create an instance of the Stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 4, 7, 5, 8);

// the previous reading from the analog input
int previous = 0;

void setup() {
  // set the speed of the motor to 15 RPMs
  stepper.setSpeedRpm(15);
}

void loop() {
  // get the sensor value
  int val = analogRead(0);

  // move a number of steps equal to the change in the
  // sensor reading
  stepper.step(val - previous);
  while(stepper.move()) {
    delay(2);
  }

  // remember the previous value of the sensor
  previous = val;
}
