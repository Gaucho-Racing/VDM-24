#include <Arduino.h>
#include <imxrt.h>

#include "main.h"
#include "fakenodes.cpp"
#include "iCANflex.h"
#include "machine.h"
#include "critical_errors.h"



volatile State state;
volatile State prevState;
volatile bool (*errorCheck)(void); 

State sendToError(volatile State currentState, volatile bool (*erFunc)(void)) {
   errorCheck = erFunc; 
   prevState = currentState; 
   return ERROR;
}


void loop(){
    switch (state) {
        case OFF:
            state = off(Car, switches);
            break;
        case ON:
            if ((state = on(Car, switches)) == ERROR) sendToError(ON, &startupCriticalError);
            break;
        case DRIVE_READY:
            state = drive_ready(Car, switches); 
            break;
        case DRIVE:
            state = drive(Car, switches);
            break;
        case ERROR:
            state = error(Car, switches, prevState, errorCheck);
            break;
        // case TESTING;
    }

    //test
}

void setup() {
    Serial.begin(9600);
    Car.begin();


    // set all the switchboard pins to  digital read inputs



    // set state  
    state = OFF;  
}

