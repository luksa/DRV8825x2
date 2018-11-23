#include "DRV8825x2.h"

#define FOREACH_MOTOR(action) {motor1->action; motor2->action;}

void DRV8825x2::enable() {
    FOREACH_MOTOR(enable());
}

void DRV8825x2::disable(){
    FOREACH_MOTOR(disable());
}

void DRV8825x2::rotate(long deg1, long deg2){
    motor1->rotate(deg1);
    motor2->rotate(deg2);
}


