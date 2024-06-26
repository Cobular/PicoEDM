/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"
#include "Wire.h"
#include "AccelStepper.h"

#define EN_PIN    14 								// LOW: Driver enabled. HIGH: Driver disabled
#define STEP_PIN  13 								// Step on rising edge
#define DIR_PIN 12

#define CURRENT_PIN 26

// #include <TMC2208Stepper.h>							// Include library
// TMC2208Stepper driver = TMC2208Stepper(&Serial2);	// Create driver and use

int counter = 0;

u_int32_t rolling_avg = 0;


AccelStepper stepper = AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

u_int32_t sat_subu32b(u_int32_t x, u_int32_t y)
{
	u_int32_t res = x - y;
	res &= -(res <= x);
	
	return res;
}


void setup()
{
  Serial.begin(115200);


  analogReadResolution(12);


  stepper.setEnablePin(EN_PIN);
  stepper.setPinsInverted(false, false, true);


	// pinMode(EN_PIN, OUTPUT);
  // pinMode(STEP_PIN, OUTPUT);
  // pinMode(DIR_PIN, OUTPUT);
	// digitalWrite(EN_PIN, LOW);							// Enable driver

  stepper.setCurrentPosition(0);
  stepper.setMaxSpeed(2000);
  stepper.setAcceleration(4000);

  stepper.enableOutputs();

  stepper.setSpeed(5000);
}

void loop()
{
  // Serial.println(stepper.currentPosition());

  long pos = stepper.currentPosition();
  if (pos > 10000) {
    stepper.setSpeed(-2000);
  }

  if (pos < 3000) {
    stepper.setSpeed(2000);
  }

  // stepper.moveTo(200);
  stepper.runSpeed();

  // rolling_avg = rolling_avg * 0.99 + 0.01 * analogRead(CURRENT_PIN);

  // Ranges from about 3100 to about 3500
  uint32_t current_ma = sat_subu32b(analogRead(CURRENT_PIN), 3120) * 13;

  if (current_ma =)

 	// digitalWrite(STEP_PIN, HIGH); // Step
  // delay(delay_time);
  // digitalWrite(STEP_PIN, LOW); // Step
  // delay(delay_time);
  // // delayMicroseconds(500);
  // // counter += 1;

  // // if (counter == 5000) {
  // //    	digitalWrite(DIR_PIN, !digitalRead(DIR_PIN)); // Step
  // //     counter = 0;
  // // }
	// // delay(1);
}

