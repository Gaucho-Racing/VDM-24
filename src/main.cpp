#include <Arduino.h>
#include <imxrt.h>

#include "main.h"
#include "fakenodes.cpp"
#include "iCANflex.h"


State state;


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
