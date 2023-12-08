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

void canReceiveFailure_ISR() {
    Car.sendDashError(100);
    state = sendToError(state, canReceiveFailure(Car));
}

const int canFailureThreshold = 100; // msec

volatile bool canReceiveFailure(iCANflex& Car) {
    return 
        Car.Inverter.getAge() > canFailureThreshold ||
        Car.VDM.getAge() > canFailureThreshold ||
        Car.Wheel.getAge() > canFailureThreshold ||
        Car.GPS.getAge() > canFailureThreshold ||
        Car.Pedals.getAge() > canFailureThreshold ||
        Car.ACU.getAge() > canFailureThreshold ||
        Car.BCM.getAge() > canFailureThreshold ||
        Car.Dash.getAge() > canFailureThreshold ||
        Car.Energy_Meter.getAge() > canFailureThreshold;
}

void loop(){
    if (canReceiveFailure(Car)) {NVIC_TRIGGER_IRQ(10);}

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
    attachInterruptVector(10, &canReceiveFailure_ISR);

    Serial.begin(9600);
    Car.begin();


    // set all the switchboard pins to  digital read inputs



    // set state  
    state = OFF;  
}

