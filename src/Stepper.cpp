/*
 * Stepper.cpp - Stepper library for Wiring/Arduino - Version 1.2.0
 *
 * Original library        (0.1)   by Tom Igoe.
 * Two-wire modifications  (0.2)   by Sebastian Gassner
 * Combination version     (0.3)   by Tom Igoe and David Mellis
 * Bug fix for four-wire   (0.4)   by Tom Igoe, bug fix from Noah Shibley
 * High-speed stepping mod         by Eugene Kozlenko
 * Timer rollover fix              by Eugene Kozlenko
 * Five phase five wire    (1.1.0) by Ryan Orendorff
 * Microstepping on bipolar(1.2.0) by Attila Kovács
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 * Drives a unipolar, bipolar, or five phase stepper motor.
 *
 * When wiring multiple stepper motors to a microcontroller, you quickly run
 * out of output pins, with each motor requiring 4 connections.
 *
 * By making use of the fact that at any time two of the four motor coils are
 * the inverse of the other two, the number of control connections can be
 * reduced from 4 to 2 for the unipolar and bipolar motors.
 *
 * A slightly modified circuit around a Darlington transistor array or an
 * L293 H-bridge connects to only 2 microcontroler pins, inverts the signals
 * received, and delivers the 4 (2 plus 2 inverted ones) output signals
 * required for driving a stepper motor. Similarly the Arduino motor shields
 * 2 direction pins may be used.
 *
 * The sequence of control signals for 5 phase, 5 control wires is as follows:
 *
 * Step C0 C1 C2 C3 C4
 *    1  0  1  1  0  1
 *    2  0  1  0  0  1
 *    3  0  1  0  1  1
 *    4  0  1  0  1  0
 *    5  1  1  0  1  0
 *    6  1  0  0  1  0
 *    7  1  0  1  1  0
 *    8  1  0  1  0  0
 *    9  1  0  1  0  1
 *   10  0  0  1  0  1
 *
 * The sequence of control signals for 4 control wires is as follows:
 *
 * Step C0 C1 C2 C3
 *    1  1  0  1  0
 *    2  0  1  1  0
 *    3  0  1  0  1
 *    4  1  0  0  1
 *
 * The sequence of controls signals for 2 control wires is as follows
 * (columns C1 and C2 from above):
 *
 * Step C0 C1
 *    1  0  1
 *    2  1  1
 *    3  1  0
 *    4  0  0
 *
 * The circuits can be found at
 *
 * http://www.arduino.cc/en/Reference/Stepper
 */

#include "Arduino.h"
#include "Stepper.h"

 /*
 * Setting the static members arrays to store the 1/2, 1/4, 1/8 microstepping sine, cosine tables
 */
int const Stepper::microstepping_1_2[4][2][2] = {
	{ { 0, 100 },{ 71, 71 } },
	{ { 100, 0 },{ 71, -71 } },
	{ { 0, -100 },{ -71, -71 } },
	{ { -100, 0 },{ -71, 71 } }
};

int const Stepper::microstepping_1_4[4][4][2] = {
	{ { 0, 100 },{ 38, 92 },{ 71, 71 },{ 92, 38 } },
	{ { 100, 0 },{ 92, -38 },{ 71, -71 },{ 38, -92 } },
	{ { 0, -100 },{ -38, -92 },{ -71, -71 },{ -92, -38 } },
	{ { -100, 0 },{ -92, 38 },{ -71, 71 },{ -38, 92 } }
};

int const Stepper::microstepping_1_8[4][8][2] = {
	{ { 0, 100 },{ 20, 98 },{ 38, 92 },{ 56, 83 },{ 71, 71 },{ 83, 56 },{ 92, 38 },{ 98, 20 }, },
	{ { 100, 0 },{ 98, -20 },{ 92, -38 },{ 83, -56 },{ 71, -71 },{ 56, -83 },{ 38, -92 },{ 20, -98 } },
	{ { -0, -100 },{ -20, -98 },{ -38, -92 },{ -56, -83 },{ -71, -71 },{ -83, -56 },{ -92, -38 },{ -98, -20 }, },
	{ { -100, 0 },{ -98, 20 },{ -92, 38 },{ -83, 56 },{ -71, 71 },{ -56, 83 },{ -38, 92 },{ -20, 98 } }
};

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->number_of_steps = number_of_steps; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;

  // setup the pins on the microcontroller:
  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);

  // When there are only 2 pins, set the others to 0:
  this->motor_pin_3 = 0;
  this->motor_pin_4 = 0;
  this->motor_pin_5 = 0;

  // pin_count is used by the stepMotor() method:
  this->pin_count = 2;
}

/*
 * two-wire + PWM constructor
 * Sets which wires should control the motor direction and PWM signals,
 * number of steps per full rotation and the micro steps per step.
 */
Stepper::Stepper(int number_of_steps, bool micro_stepping, int number_of_micro_steps,
									int motor_pin_1, int motor_pin_2,
									int motor_pwm_pin_1, int motor_pwm_pin_2)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->number_of_steps = number_of_steps; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;

  // Pins for the PWM signals
  this->motor_pwm_pin_1 = motor_pwm_pin_1;
  this->motor_pwm_pin_2 = motor_pwm_pin_2;
  

  // setup the pins on the microcontroller:
  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);
  pinMode(this->motor_pwm_pin_1, OUTPUT);
  pinMode(this->motor_pwm_pin_2, OUTPUT);

  // set microstepping mode
  this->micro_stepping = micro_stepping;
  this->number_of_micro_steps = number_of_micro_steps;

  // When there are only 2 pins, set the others to 0:
  this->motor_pin_3 = 0;
  this->motor_pin_4 = 0;
  this->motor_pin_5 = 0;

  // pin_count is used by the stepMotor() method:
  this->pin_count = 2;
}


/*
 *   constructor for four-pin version
 *   Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                      int motor_pin_3, int motor_pin_4)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->number_of_steps = number_of_steps; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;
  this->motor_pin_3 = motor_pin_3;
  this->motor_pin_4 = motor_pin_4;

  // setup the pins on the microcontroller:
  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);
  pinMode(this->motor_pin_3, OUTPUT);
  pinMode(this->motor_pin_4, OUTPUT);

  // When there are 4 pins, set the others to 0:
  this->motor_pin_5 = 0;

  // pin_count is used by the stepMotor() method:
  this->pin_count = 4;
}


/*
* four-wire + PWM constructor
* Sets which wires should control the motor direction and PWM signals,
* number of steps per full rotation and the micro steps per step.
*/
Stepper::Stepper(int number_of_steps, bool micro_stepping, int number_of_micro_steps,
									int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4,
									int motor_pwm_pin_1, int motor_pwm_pin_2)
{
	this->step_number = 0;    // which step the motor is on
	this->direction = 0;      // motor direction
	this->last_step_time = 0; // time stamp in us of the last step taken
	this->number_of_steps = number_of_steps; // total number of steps for this motor

											 // Arduino pins for the motor control connection:
	this->motor_pin_1 = motor_pin_1;
	this->motor_pin_2 = motor_pin_2;
	this->motor_pin_3 = motor_pin_3;
	this->motor_pin_4 = motor_pin_4;

	// Pins for the PWM signals
	this->motor_pwm_pin_1 = motor_pwm_pin_1;
	this->motor_pwm_pin_2 = motor_pwm_pin_2;


	// setup the pins on the microcontroller:
	pinMode(this->motor_pin_1, OUTPUT);
	pinMode(this->motor_pin_2, OUTPUT);
	pinMode(this->motor_pin_3, OUTPUT);
	pinMode(this->motor_pin_4, OUTPUT);

	pinMode(this->motor_pwm_pin_1, OUTPUT);
	pinMode(this->motor_pwm_pin_2, OUTPUT);

	// set microstepping mode
	this->micro_stepping = micro_stepping;
	this->number_of_micro_steps = number_of_micro_steps;

	// When there are only 2 pins, set the others to 0:
	this->motor_pin_3 = 0;
	this->motor_pin_4 = 0;
	this->motor_pin_5 = 0;

	// pin_count is used by the stepMotor() method:
	this->pin_count = 2;
}


/*
 *   constructor for five phase motor with five wires
 *   Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
                                      int motor_pin_3, int motor_pin_4,
                                      int motor_pin_5)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->number_of_steps = number_of_steps; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;
  this->motor_pin_3 = motor_pin_3;
  this->motor_pin_4 = motor_pin_4;
  this->motor_pin_5 = motor_pin_5;

  // setup the pins on the microcontroller:
  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);
  pinMode(this->motor_pin_3, OUTPUT);
  pinMode(this->motor_pin_4, OUTPUT);
  pinMode(this->motor_pin_5, OUTPUT);

  // pin_count is used by the stepMotor() method:
  this->pin_count = 5;
}

/*
 * Sets the speed in revs per minute
 */
void Stepper::setSpeed(long whatSpeed)
{
  this->step_delay = 60L * 1000L * 1000L / this->number_of_steps / whatSpeed;
  if (this->micro_stepping) {
	  this->micro_step_delay = step_delay / number_of_micro_steps;
	  // the PWM signal frqequency is proprtinal to the RPM
	  analogWriteFreq(whatSpeed * 100);
  }
  else
    this->micro_step_delay = 0;
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void Stepper::step(int steps_to_move)
{
  int steps_left = abs(steps_to_move);  // how many steps to take

  // determine direction based on whether steps_to_mode is + or -:
  if (steps_to_move > 0) { this->direction = 1; }
  if (steps_to_move < 0) { this->direction = 0; }


  // if no microstepping is set
  if (!this->micro_stepping)
  {
    // decrement the number of steps, moving one step each time:
    while (steps_left > 0)
    {
      unsigned long now = micros();
      // move only if the appropriate delay has passed:
      if (now - this->last_step_time >= this->step_delay)
      {
        // get the timeStamp of when you stepped:
        this->last_step_time = now;
        // increment or decrement the step number,
        // depending on direction:
        if (this->direction == 1)
        {
          this->step_number++;
          if (this->step_number == this->number_of_steps) {
            this->step_number = 0;
          }
        }
        else
        {
          if (this->step_number == 0) {
            this->step_number = this->number_of_steps;
          }
          this->step_number--;
        }
        // decrement the steps left:
        steps_left--;
        // step the motor to step number 0, 1, ..., {3 or 10}
        if (this->pin_count == 5)
          stepMotor(this->step_number % 10);
        else
          stepMotor(this->step_number % 4);
      }
    }
  }
  
  // if microstepping is used
  else 
  {
      // decrement the number of steps, moving one step each time:
    while (steps_left > 0)
    {
      unsigned long now = micros();
      // move only if the appropriate delay has passed:
      if (now - this->last_step_time >= this->micro_step_delay)
      {
        // get the timeStamp of when you stepped:
        this->last_step_time = now;
          
        // increment or decrement the whole step number,
        // depending on direction:
        if (this->direction == 1)
        {
          this->micro_step_number++;
          // if there is whole step
          if (this->micro_step_number == this->number_of_micro_steps) {
            this->step_number++;
            if (this->step_number == this->number_of_steps) {
              this->step_number = 0;
            }
            this->micro_step_number = 0; //there was a whole step, microstep is reset to 0
          }
        }
        else
        {
          // if there is whole step
          if (this->micro_step_number == 0) {
            if (this->step_number == 0) {
              this->step_number = this->number_of_steps;
            }
            this->step_number--;
            this->micro_step_number = this->number_of_micro_steps;
          }
          this->micro_step_number--;
        }
        
        // decrement the steps left:
		
        steps_left--;
        
        // step the motor to step number 0, 1, ..., {3 or 10}
        if (this->pin_count == 2 || this->pin_count == 4)
			microStepMotor(this->step_number % 4, this->micro_step_number);
      }
    }
  } 
}

/*
* Moves the motor forward or backwards in microsteps.
*/
void Stepper::microStepMotor(int this_step, int this_micro_step)
{
	//yield() might be needed at slow RPM and/or many steps on an ESP8266
	//yield(); 
	int coil1value;
	int coil2value;
	
	switch (this->number_of_micro_steps) {
	case 2:
		coil1value = microstepping_1_2[this_step][this_micro_step][0];
		coil2value = microstepping_1_2[this_step][this_micro_step][1];
		break;
	case 4:
		coil1value = microstepping_1_4[this_step][this_micro_step][0];
		coil2value = microstepping_1_4[this_step][this_micro_step][1];
		break;
	case 8:
		coil1value = microstepping_1_8[this_step][this_micro_step][0];
		coil2value = microstepping_1_8[this_step][this_micro_step][1];
		break;
	}

	// set the correct PWM on the enable pin
	analogWrite(motor_pwm_pin_1, (abs(coil1value) * PWMRANGE) / 100);
	analogWrite(motor_pwm_pin_2, (abs(coil2value) * PWMRANGE) / 100);

	if (this->pin_count == 2) {
		if (coil1value > 0)
			digitalWrite(motor_pin_1, HIGH);
		else
			digitalWrite(motor_pin_1, LOW);

		if (coil2value > 0)
			digitalWrite(motor_pin_2, HIGH);
		else
			digitalWrite(motor_pin_2, LOW);
	}
	else if (this->pin_count == 4) {
		if (coil1value > 0) {
			digitalWrite(motor_pin_1, HIGH);
			digitalWrite(motor_pin_2, LOW);
		}
		else {
			digitalWrite(motor_pin_1, LOW);
			digitalWrite(motor_pin_2, HIGH);
		}

		if (coil2value > 0) {
			digitalWrite(motor_pin_3, HIGH);
			digitalWrite(motor_pin_4, LOW);
		}
		else {
			digitalWrite(motor_pin_3, LOW);
			digitalWrite(motor_pin_4, HIGH);
		}
	}
}

/*
 * Moves the motor forward or backwards.
 */
void Stepper::stepMotor(int thisStep)
{
	//yield() might be needed at slow RPM and/or many steps on an ESP8266
  //yield(); 
  if (this->pin_count == 2) {
    switch (thisStep) {
      case 0:  // 01
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
      break;
      case 1:  // 11
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, HIGH);
      break;
      case 2:  // 10
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
      break;
      case 3:  // 00
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
      break;
    }
  }
  if (this->pin_count == 4) {
    switch (thisStep) {
      case 0:  // 1010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 1:  // 0110
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
      break;
      case 2:  //0101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
      case 3:  //1001
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
      break;
    }
  }

  if (this->pin_count == 5) {
    switch (thisStep) {
      case 0:  // 01101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 1:  // 01001
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 2:  // 01011
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 3:  // 01010
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 4:  // 11010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, HIGH);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 5:  // 10010
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, LOW);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 6:  // 10110
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, HIGH);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 7:  // 10100
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, LOW);
        break;
      case 8:  // 10101
        digitalWrite(motor_pin_1, HIGH);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
      case 9:  // 00101
        digitalWrite(motor_pin_1, LOW);
        digitalWrite(motor_pin_2, LOW);
        digitalWrite(motor_pin_3, HIGH);
        digitalWrite(motor_pin_4, LOW);
        digitalWrite(motor_pin_5, HIGH);
        break;
    }
  }
}


/*
off() turns of the motor by clearing motor pins to by clearing pwm/enable pins:
*/
void Stepper::off(void)
{
	// if microstepping is used, the motor can be turned off by clearing the PWM pins (even in the case of 2 wire configuration)
	if (this->micro_stepping) {
		analogWrite(motor_pwm_pin_1, 0);
		analogWrite(motor_pwm_pin_2, 0);
	}
	else if (this->pin_count == 2) {
		//This is not possible with 2 wire configuration without using  the PWM/enable pins.
		//The two wire stepper (full step mode) should be initialized with the PWM pins, to be able to switch off the motor.
		//Or using the microstepping mode, the PWM/enable pins are known.
	}
	else if (this->pin_count == 4) {
		digitalWrite(motor_pin_1, LOW);
		digitalWrite(motor_pin_2, LOW);
		digitalWrite(motor_pin_3, LOW);
		digitalWrite(motor_pin_4, LOW);
	}
	else if (this->pin_count == 5) {
		digitalWrite(motor_pin_1, LOW);
		digitalWrite(motor_pin_2, LOW);
		digitalWrite(motor_pin_3, LOW);
		digitalWrite(motor_pin_4, LOW);
		digitalWrite(motor_pin_5, LOW);
	}
}

/*
  version() returns the version of the library:
*/
int Stepper::version(void)
{
  return 5;
}
