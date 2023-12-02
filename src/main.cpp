#include <Arduino.h>
#include <imxrt.h>

//#include "fakenodes.cpp"
#include "iCANflex.h"
#include "machine.h"
#include "critical_errors.h"

volatile State state;
volatile State prevState;
volatile bool (*errorCheck)(void); 

//comment these out when the corresponding file exists
//State off(iCANflex &Car, switchboard& switches) { return ON; } //off
State on(iCANflex &Car, switchboard& switches) { return DRIVE; } //on
//State drive(iCANflex &Car, switchboard& switches) { return D_PLAUS; } //drive (it's DRIVE READY in the state diagram)
//State d_plaus(iCANflex &Car, switchboard& switches) { return D_PLAUS; } //d_plaus

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
        case DRIVE:
            state = drive(Car, switches); 
            break;
        case D_PLAUS:
            state = d_plaus(Car, switches);
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

