#include "machine.h"


State off(iCANflex& Car, vector<int>& switches) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);    

    if(switches[0] && !switches[1]) { return ON; }
    return OFF;
}   



State on(iCANflex& Car, vector<int>& switches) { // ON is when PRECHARGING BEGINS
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    //activate relay to TS
    //activate Precharge through ACU;

    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2.0;
    float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2.0;
    // switch 1 turned off
    if(!switches[0]){
        return OFF;
    }
    // Stay here if any startup error is detected or switch 2 is not yet on
    else if(!switches[1] ||  ECU_Startup_Rejection(Car)) {
        return ON;
    }
    // if switch 2 is on and no startup errors, run more systems checks and go to drive ready
    else if(switches[1] && brake > 0.05 && throttle < 0.05) {
        if(Critical_Systems_Fault(Car)) return ERROR;
        Warning_Systems_Fault(Car);
        // start powertrain cooling
        // WAIT FOR PRECHARGE COMPLETE SIGNAL FROM ACU!!!!!!
        // play RTD sound
        return DRIVE_READY;
    } 
    else {
        return ON;
    }
}




State drive_ready(iCANflex& Car, vector<int>& switches, bool& BSE_APPS_violation) { // PRECHARGING MUST BE COMPLETE BEFORE ENTERING THIS STATE
    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(0);
    //start cooling system and all that 

    // switch 1 turned off 
    if(!switches[0]) { return OFF;}
    // switch 2 turned off while 1 is on
    else if(!switches[1]) { return ON;}

    float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2.0;
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2.0;
   
    if(BSE_APPS_violation) {
        if(throttle < 0.05) {
            // violation exit condition, reset violation and return to DRIVE_READY
            BSE_APPS_violation = false;
            return DRIVE_READY;
        }
        // else loop back into DRIVE_READY state with Violation still true
    }
    // only if no violation, and throttle is pressed, go to DRIVE
    if(!BSE_APPS_violation && throttle > 0.05) {
        return DRIVE;
    }

    return DRIVE_READY;
}




float motorOut(float throttle, iCANflex& car, vector<int>& switches) {
    const int HIGH_PWR_R_CURRENT = 50;
    const int LOW_PWR_R_CURRENT = 25; 
    // regen curve is neg on 0 -delta from -const variable on steering angle and battery level
    // change these to a continuous power curve later^^^
    return switches[3] ? HIGH_PWR_R_CURRENT *throttle: LOW_PWR_R_CURRENT*throttle; 
}

State drive(iCANflex& Car, vector<int>& switches, bool& BSE_APPS_violation) {
    if(!switches[0]) return OFF;
    if(!switches[1]) return ON;

    float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2;
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2;
    
    // TODO: NEEDS WORK for actual calculation
    bool APPS_GRADIENT_FAULT = false;
    if(APPS_GRADIENT_FAULT) { return OFF; }

    // set violation condtion, and return to DRIVE_READY, cutting motor power. 
    if(brake > 0.05 && throttle > 0.25) {
        BSE_APPS_violation = true;
        return DRIVE_READY;
    }

    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(motorOut(throttle, Car, switches));
    
    return DRIVE;
}

State error(iCANflex& Car, vector<int>& switches, State prevState, volatile bool (*errorCheck)(iCANflex& c)) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);

    
    if(!switches[0]) return OFF;
    if(!switches[1]) return ON;

    

    if(errorCheck(Car)) {
        return ERROR;
    }
    else {
        float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2;
        if(throttle < 0.05) {
            return prevState;
        }
        return ERROR;
    }
}
