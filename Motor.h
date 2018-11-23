

#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>

String padRight(String str, int width);


class Motor {
protected:

    short motorSteps;           // motor microsteps per revolution (e.g. 200*4)

	short enablePin;
	short dirPin;
	short stepPin;

    short rpm = 0;
    int accel = 0;		// rpm per second

    // tWH(STEP) pulse duration, STEP high, min value (1.9us)
    static const int stepHighMin = 2;
    // tWL(STEP) pulse duration, STEP low, min value (1.9us)
    static const int stepLowMin = 2;
    // tWAKE wakeup time, nSLEEP inactive to STEP (1000us)
    static const int wakeupTime = 1700;

    long calcStepsForRotation(long deg);

public:
    Motor(short steps, short dirPin, short stepPin, short enablePin)
    :motorSteps(steps), dirPin(dirPin), stepPin(stepPin), enablePin(enablePin)
    {};

    void begin(short rpm, int accel);

    short getRPM();
    void setRPM(short rpm);
    short getAcceleration();
    void setAcceleration(int accel);

    void enable();
    void disable();

    void move(long steps);
    void rotate(long deg);

};
#endif // MOTOR_H
