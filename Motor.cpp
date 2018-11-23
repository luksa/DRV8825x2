#include "Motor.h"

#define DEBUG_LOG false

void Motor::begin(short rpm, int accel){
	this->rpm = rpm;
	this->accel = accel;
}

short Motor::getRPM() {
	return rpm;
}

void Motor::setRPM(short rpm) {
	this->rpm = rpm;
}

short Motor::getAcceleration() {
	return accel;
}

void Motor::setAcceleration(int accel) {
	this->accel = accel;
}

void Motor::enable() {
    digitalWrite(enablePin, LOW);
}

void Motor::disable(){
    digitalWrite(enablePin, HIGH);
}

void Motor::rotate(long deg){
    move(calcStepsForRotation(deg));
}

long Motor::calcStepsForRotation(long deg){
	return deg * motorSteps / 360;
}

#define ACCELERATING 0
#define CRUISING 1
#define BRAKING 2

void Motor::move(long steps){
	short dirState = steps >= 0 ? HIGH : LOW;
	digitalWrite(dirPin, dirState);

	long totalSteps = abs(steps);

	unsigned long cruisePulseLength = 60000000L / ((long)rpm * motorSteps);
	unsigned long cruiseStepsPerSecond = (long)rpm * motorSteps / 60;


	long stepsToCruise = cruiseStepsPerSecond * cruiseStepsPerSecond / (2 * accel);
	long stepsToBrake = stepsToCruise;	// = stepsToCruise * accel / decel


	bool cruiseSpeedAchieved = true;
	if (totalSteps < stepsToCruise + stepsToBrake){
		// cannot reach max speed, will need to brake early
		stepsToCruise = totalSteps / 2; // = steps * decel / (accel + decel);
		stepsToBrake = totalSteps - stepsToCruise;
		cruiseSpeedAchieved = false;
	}

#ifdef DEBUG_LOG
	Serial.println();
	Serial.println("rpm:          " + String(rpm));
	Serial.println("rps:          " + String((float)rpm / 60));
	Serial.println("motorSteps:   " + String(motorSteps));
	Serial.println("accel:        " + String(accel));
	Serial.println("cruisePulseLength:    " + String(cruisePulseLength) + " us");
	Serial.println("cruiseStepsPerSecond: " + String(cruiseStepsPerSecond));
	Serial.println("totalSteps:    " + String(totalSteps));
	Serial.println("stepsToCruise: " + String(stepsToCruise));
	Serial.println("stepsToBrake:  " + String(stepsToBrake));

	short debugEntries = 10;
	short halfDebugEntries = debugEntries/2;
	long stepNumbers[3][debugEntries];
	long remSteps[3][debugEntries];
	long actualTimes[3][debugEntries];
	long desiredTimes[3][debugEntries];
	long processingTimes[3][debugEntries];

	for (short state=0; state<3; state++) {
		for (short i=0; i<debugEntries; i++) {
			stepNumbers[state][i] = 0;
			remSteps[state][i] = 0;
			actualTimes[state][i] = 0;
			desiredTimes[state][i] = 0;
			processingTimes[state][i] = 0;
		}
	}

	unsigned long moveStartTime = micros();
	unsigned long previousTime = moveStartTime;
#endif

	float millionTimesSqrtOf2DivA = 1000000.0 * sqrt(2.0/(float)accel);
	float sqrtOfCurrStep = 0.0;

	for (long step=0; step<totalSteps; step++) {
		unsigned long pulseStartTime = micros();

#ifdef DEBUG_LOG
		unsigned long microsSincePrevPulse = pulseStartTime - previousTime;
		unsigned long microsSinceStart = pulseStartTime - moveStartTime;
#endif

		digitalWrite(stepPin, HIGH);
		delayMicroseconds(stepHighMin);
		digitalWrite(stepPin, LOW);


#ifdef DEBUG_LOG
		short state;
		unsigned long internalStep;
		unsigned long totalInternalSteps;
#endif

		unsigned long desiredPulseLength;

		long stepsRemaining = totalSteps - 1 - step;

		if (step < stepsToCruise) {
			// accelerating
			float sqrtOfNextStep = sqrt((float)step+1.0);
			desiredPulseLength = (long)(millionTimesSqrtOf2DivA * (sqrtOfNextStep - sqrtOfCurrStep));
			sqrtOfCurrStep = sqrtOfNextStep;
		} else if (step >= totalSteps - stepsToBrake) {
			// braking
			float sqrtOfNextStep = sqrt((float)stepsRemaining-1.0);
			desiredPulseLength = (long)(millionTimesSqrtOf2DivA * (sqrtOfCurrStep - sqrtOfNextStep));
			sqrtOfCurrStep = sqrtOfNextStep;
		} else {
			// cruising
			desiredPulseLength = cruisePulseLength;
		}

#ifdef DEBUG_LOG
		if (step < stepsToCruise) {
			state = ACCELERATING;
			internalStep = step;
			totalInternalSteps = stepsToCruise;
		} else if (step >= totalSteps - stepsToBrake) {
			state = BRAKING;
			internalStep = step - (totalSteps - stepsToBrake);
			totalInternalSteps = stepsToBrake;
		} else {
			state = CRUISING;
			internalStep = step - stepsToCruise;
			totalInternalSteps = totalSteps - stepsToCruise - stepsToBrake;
		}

		previousTime = pulseStartTime;

		if (internalStep < halfDebugEntries || internalStep >= totalInternalSteps - halfDebugEntries) {
			short pos = internalStep < halfDebugEntries ? internalStep : (internalStep - totalInternalSteps + debugEntries);
			stepNumbers[state][pos] = step;
			remSteps[state][pos] = stepsRemaining;
			actualTimes[state][pos] = microsSincePrevPulse;
			desiredTimes[state][pos] = desiredPulseLength;
		}
#endif

		unsigned long processingTime = micros() - pulseStartTime;

#ifdef DEBUG_LOG
		if (internalStep < halfDebugEntries || internalStep >= totalInternalSteps - halfDebugEntries) {
			short pos = internalStep < halfDebugEntries ? internalStep : (internalStep - totalInternalSteps + debugEntries);
			processingTimes[state][pos] = processingTime;
		}
#endif

		if (desiredPulseLength > processingTime) {
			delayMicroseconds(desiredPulseLength - processingTime);
		}
	}

#ifdef DEBUG_LOG
	for (short state=0; state<3; state++) {
		if (state == CRUISING && !cruiseSpeedAchieved) {
			continue;
		}
		for (int i=0; i<debugEntries; i++) {
			Serial.print(padRight(String(stepNumbers[state][i]), 8));
			Serial.print(F("remain: "));
			Serial.print(padRight(String(remSteps[state][i]), 8));
			Serial.print(F("since prev pulse: "));
			Serial.print(padRight(String(actualTimes[state][i]), 12));
			Serial.print(F("processing: "));
			Serial.print(padRight(String(processingTimes[state][i]), 12));
			Serial.print(F("desired: "));
			Serial.print(padRight(String(desiredTimes[state][i]), 12));
			Serial.println();
			if (i == halfDebugEntries-1) {
				Serial.println("...");
			}
		}
		Serial.println(F("======================"));
	}
#endif

}

String padRight(String str, int width) {
	short padWidth = width - str.length();
	String p = "";
    for (int i=0; i<padWidth; i++) {
    	p = p + " ";
    }
    return str + p;
}
