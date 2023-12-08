#include <Arduino.h>
#include <imxrt.h>
#include "main.h"
#include "iCANflex.h"
#include "machine.h"



volatile State state;
volatile State prevState;
volatile bool (*errorCheck)(iCANflex& Car); 
bool BSE_APPS_violation = false;

State sendToError(volatile State currentState, volatile bool (*erFunc)(iCANflex& Car)) {
   errorCheck = erFunc; 
   prevState = currentState; 
   return ERROR;
}

volatile bool currentLimitSafe(iCANflex& Car) {
    return (Car.DTI.getDCCurrent() > 575); // based on below current limit
}

volatile bool currentLimitExceeded(iCANflex& Car) {
    return (Car.DTI.getDCCurrent() > 600); // taken from last year's implementation
}

void currentLimitExceeded_ISR() {
    Car.sendDashError(106);
    state = sendToError(state, currentLimitSafe);
}

void loop(){
    if (currentLimitExceeded(Car)) { 
        NVIC_TRIGGER_IRQ(1); // pin number is filler
    }
    switch (state) {
        case OFF:
            state = off(Car, switches);
            break;
        case ON:
            if ((state = on(Car, switches)) == ERROR) sendToError(ON, &ECU_Startup_Rejection);
            break;
        case DRIVE_READY:
            state = drive_ready(Car, switches, BSE_APPS_violation); 
            break;
        case DRIVE:
            state = drive(Car, switches, BSE_APPS_violation);
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

    attachInterruptVector(1, &currentLimitExceeded_ISR); // pin number is filler

    // set all the switchboard pins to  digital read inputs



    // set state  
    state = OFF;  
}

