

#ifndef DRV8825x2_H
#define DRV8825x2_H

#include <Arduino.h>
#include "Motor.h"

class DRV8825x2 {
protected:

    Motor* motor1;
    Motor* motor2;


public:
    DRV8825x2(Motor& m1, Motor& m2)
    :motor1(&m1), motor2(&m2)
    {};

    void enable();
    void disable();

    void rotate(long deg1, long deg2);

};
#endif // DRV8825x2_H
