/*
Stepper Motor Control - revolutions with microstepping

This program drives a bipolar stepper motor in microstepping mode.
The motor is attached to digital pins D1-D4 of an ESP-8266.
D3, D4 are connected to the direction pins, D1 and D2 are connetec to the PWM/ENABLE pins of the L293D H bridge chip.

The motor swings the same number of steps clockwise and back.


Created 17 Mar. 2017
by Attila Kovács

*/

#include "Stepper.h"

#define DIR_MOTOR_A D3
#define DIR_MOTOR_B D4
#define PWM_MOTOR_A D1
#define PWM_MOTOR_B D2

const int stepsPerRevolution = 48;  // change this to fit the number of steps per revolution
const int microSteps = 8;			// valid values are 2, 4, 8
int revolutions = 2;				// number of revoluations
int stepsToSwing = microSteps * stepsPerRevolution * revolutions; //number of steps to do
int motorRpm = 100;					// motor speed


// stepper motor with 8 microstpesinitialized, pin D3, D4 are connected to the H bridge chip inputs,
// D1 (PWM_MOTOR_A) and D2 (PWM_MOTOR_B) are connected to the enable pin of the H bridge chip
Stepper myStepper(stepsPerRevolution, true, microSteps, DIR_MOTOR_A, DIR_MOTOR_B, PWM_MOTOR_A, PWM_MOTOR_B);

// driving without microsteps
// DO NOT forget to adjust the number of steps to do, since there is no microstepping used. (microSteps = 1;)
//Stepper myStepper(stepsPerRevolution, D3, D4);


void setup() {	
	// Set the PWM/enable pins.
	// Only needed in the case of full step drive. Not needed for microstep mode.
	pinMode(PWM_MOTOR_A, OUTPUT);
	pinMode(PWM_MOTOR_B, OUTPUT);
	digitalWrite(PWM_MOTOR_A, HIGH);	
	digitalWrite(PWM_MOTOR_B, HIGH);

	// set the speed of the motor
	myStepper.setSpeed(motorRpm);
	// initialize the serial port:
	Serial.begin(250000);
}


void loop() {
	// Swing clockwise.
	Serial.println("clockwise");
	myStepper.step(stepsToSwing);	
	myStepper.off();
	yield();
	delay(500);
	yield();

	Serial.println("counterclockwise");
	// Swing to the other direction.
	myStepper.step(-stepsToSwing);
	yield();
	myStepper.off();
	delay(500);
	yield();
}
