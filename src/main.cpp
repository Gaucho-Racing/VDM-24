#include <Arduino.h>
#include <imxrt.h>

#include "enums.h"
#include "fakenodes.cpp"

States state;

void setup() {
    state = OFF;  
}

void loop(){
    switch (state) {
        case OFF:
            state = off();
        case ON:
            state = on();
        case DRIVE_READY:
            state = drive_ready();
        case D_PLAUSE:
            staet = d_plaus();
    }
}
