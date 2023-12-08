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

void motorTempHigh_ISR() {
    Car.sendDashError(5); // send placeholder error byte code to dash
    State = sendToError(state, motorTempHighExitCondition(Car));
}

volatile bool motorTempHighExitCondition(iCANflex& car) {
    if Car.getMotorTemp() < 55 {
        return false;
    }
    return true;
}

volatile bool motorTempHighEntryCondition(iCANflex& Car) {
    if (Car.getMotorTemp() >= 60) {
        return true;
    }
    return false;
}

void loop(){
    if(motorTempHighEntry(Car)) {
        NVIC_TRIGGER_IRQ(3); //placeholder pin number 3
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

    attachInterruptVector(3, &motorTempHigh_ISR); //placeholder pin number 3


    // set all the switchboard pins to  digital read inputs



    // set state  
    state = OFF;  
}

