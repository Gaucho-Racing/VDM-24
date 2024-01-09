#include <Arduino.h>
#include <imxrt.h>
#include "main.h"
#include "iCANflex.h"
#include "machine.h"

volatile State state;
volatile State prevState;
volatile bool (*errorCheck)(iCANflex& Car); 
bool BSE_APPS_violation = false;

<<<<<<< HEAD
//comment these out when the corresponding file exists
//State off(iCANflex &Car, switchboard& switches) { return ON; } //off
State on(iCANflex &Car, switchboard& switches) { return DRIVE; } //on
//State drive(iCANflex &Car, switchboard& switches) { return D_PLAUS; } //drive (it's DRIVE READY in the state diagram)
//State d_plaus(iCANflex &Car, switchboard& switches) { return D_PLAUS; } //d_plaus

State sendToError(volatile State currentState, volatile bool (*erFunc)(void)) {
=======
State sendToError(volatile State currentState, volatile bool (*erFunc)(iCANflex& Car)) {
>>>>>>> 7fe39df7806d331731358492bbb6ca63ddd5245b
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
            if ((state = on(Car, switches)) == ERROR) sendToError(ON, &ECU_Startup_Rejection);
            break;
<<<<<<< HEAD
        case DRIVE:
            state = drive(Car, switches); 
            break;
        case D_PLAUS:
            state = d_plaus(Car, switches);
=======
        case DRIVE_READY:
            state = drive_ready(Car, switches, BSE_APPS_violation); 
            break;
        case DRIVE:
            state = drive(Car, switches, BSE_APPS_violation);
>>>>>>> 7fe39df7806d331731358492bbb6ca63ddd5245b
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
    //Car.begin(); //car.begin??


    // set all the switchboard pins to  digital read inputs



    // set state  
    state = OFF;  
}

