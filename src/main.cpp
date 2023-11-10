#include <Arduino.h>
#include <imxrt.h>

#include "enums.h"
#include "fakenodes.cpp"

States state;

//comment these out when the corresponding file exists
state off() { return ON; } //off
state on() { return DRIVE_READY; } //on
state drive_ready() { return D_PLAUS; } //drive_ready
state d_plaus() { return D_PLAUS; } //d_plaus

void setup() {
    state = OFF;  
}

void loop(){
    switch (state) {
        case OFF:
            state = off();
            break;
        case ON:
            state = on();
            break;
        case DRIVE_READY:
            state = drive_ready();
            break;
        case D_PLAUSE:
            staet = d_plaus();
            break;
    }
}
