/*
 Interruptible Three-phase Stepper Motor Control - one revolution

 This program drives a three-phase unipolar stepper motor.
 The motor is attached to GPIO pins 17, 18, 19 of a Heltec wifi kit 32 development board
 (modify for your Arduino or whatever)

 The motor should revolve one revolution in one direction, then
 one revolution in the other direction.

 A potentiometer is used to set speed (like the MotorKnob example)
 If you select a slow speed (like 1), it can take a long time to step

 If you don't want to wait for a cycle to finish itself, you can
 "interrupt" it by pressing the pushbutton -- the ISR calls
 Stepper::interrupt() which sets a private boolean "INTERRUPTED"
 value in your Stepper, which will cause the while() loop in Stepper::step()
 to exit instead of running a long time.  The user code will then call
 Stepper::clear_interrupt(), which resets the INTERRUPTED boolean, in your Stepper

Output should look like this:
  config pushbutton ==> Done
  Attach interrupt...  ==> Done
  Setting stepper speed... ==> Done
  setup() complete
  clockwise, speed: 1  button PRESSED
  counterclockwise, speed: 23  button NOT pressed
  clockwise, speed: 23  button NOT pressed
  counterclockwise, speed: 24  button NOT pressed
  clockwise, speed: 11  button PRESSED
  counterclockwise, speed: 8  button NOT pressed
  ...

 Created 11 Mar. 2007;  Modified 30 Nov. 2009 by Tom Igoe
 Modified 14 Dec 2022 by Joe Brendler
 */

#include <Arduino.h>
#include <Stepper.h>

#define pot A0
#define pushbutton 37

const int stepsPerRevolution = 200; // change this to fit the number of steps per revolution for your motor
long speed = 50;                    // analog input reading (in rpm)

struct buttonInterruptLine
{
  const uint8_t PIN;         // gpio pin number
  volatile uint32_t numHits; // number of times fired
  volatile bool PRESSED;     // boolean logical; is triggered now
};
buttonInterruptLine buttonPress = {pushbutton, 0, false};

// initialize the three-phase stepper library on GPIO
Stepper myStepper(stepsPerRevolution, 17, 18, 19);

/*------------------------------------------------------------------------------
   handle_button_interrupt() - set flag and call stepper interrupt method
  ------------------------------------------------------------------------------*/
void IRAM_ATTR handle_button_interrupt()
{
  buttonPress.PRESSED = true;
  myStepper.interrupt(); // method sets a flag
}

void setup()
{
  // initialize the serial port:
  Serial.begin(115200);

  // initialize analog input
  pinMode(pot, INPUT);

  // Configure function pushbutton interrupt pin
  Serial.print("config pushbutton");
  pinMode(buttonPress.PIN, INPUT_PULLDOWN);
  Serial.println(" ==> Done");
  Serial.print("Attach interrupt... ");
  attachInterrupt(buttonPress.PIN, handle_button_interrupt, FALLING);
  Serial.println(" ==> Done");
  // set stepper speed
  Serial.print("Setting stepper speed...");
  // set the speed
  myStepper.setSpeed(speed);
  Serial.println(" ==> Done");
  Serial.println("setup() complete");
}

void loop()
{
  // read speed from 12-bit ADC input (map 1-100; stepper doesn't like speed=0)
  speed = (long)(map(analogRead(pot), 0, 4095, 1, 100));
  myStepper.setSpeed(speed);
  Serial.printf("clockwise, speed: %d ", speed);
  // step one revolution in one direction:
  myStepper.step(stepsPerRevolution);
  if (buttonPress.PRESSED)
    Serial.println(" button PRESSED");
  else
    Serial.println(" button NOT pressed");
  if (buttonPress.PRESSED)
  {
    buttonPress.PRESSED = false;
    myStepper.clear_interrupt();
  }
  delay(500);

  // read speed from 12-bit ADC input (map 1-100; stepper doesn't like speed=0)
  speed = (long)(map(analogRead(pot), 0, 4095, 1, 100));
  myStepper.setSpeed(speed);
  Serial.printf("counterclockwise, speed: %d ", speed);
  // step one revolution in the other direction:
  myStepper.step(-stepsPerRevolution);
  if (buttonPress.PRESSED)
    Serial.println(" button PRESSED");
  else
    Serial.println(" button NOT pressed");
  if (buttonPress.PRESSED)
  {
    buttonPress.PRESSED = false;
    myStepper.clear_interrupt();
  }
  delay(500);
}
