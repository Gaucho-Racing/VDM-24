#include <Arduino.h>
#include <imxrt.h>

#include "main.h"
#include "fakenodes.cpp"

State state;

//comment these out when the corresponding file exists
State off(iCANflex &Car) { return ON; } //off
State on(iCANflex &Car) { return DRIVE_READY; } //on
State drive_ready(iCANflex &Car) { return D_PLAUS; } //drive_ready
State d_plaus(iCANflex &Car) { return D_PLAUS; } //d_plaus

void setup() {
    state = OFF;  
}

void loop(){
    switch (state) {
        case OFF:
            state = off(Car);
            break;
        case ON:
            state = on(Car);
            break;
        case DRIVE_READY:
            state = drive_ready(Car);
            break;
        case D_PLAUS:
            state = d_plaus(Car);
            break;
    }

    //test
}
