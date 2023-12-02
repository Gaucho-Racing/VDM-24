#include "machine.h"


State off(iCANflex& Car, switchboard& switches) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);    


    if(switches.drive_enable && !switches.drive_engage) { return ON; }
    return OFF;
}   



State on(iCANflex& Car, switchboard& switches) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);

    // switch 1 turned off
    if(!switches.drive_enable){
        return OFF;
    }
    // Stay here if any startup error is detected or switch 2 is not yet on
    else if(!switches.drive_engage ||  ECU_Startup_Rejection(Car, switches)) {
        return ON;
    }
    // if switch 2 is on and no startup errors, run more systems checks and go to drive ready
    else if(switches.drive_engage) {
        if(Critical_Systems_Fault(Car, switches)) return ERROR;
        Warning_Systems_Fault(Car, switches);
        // play RTD sound
        return DRIVE_READY;
    } 
    else {
        return ON;
    }
}




State drive_ready(iCANflex& Car, switchboard& switches, bool& BSE_APPS_violation) {
    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(0);
    //start cooling system and all that jazz

    // switch 1 turned off 
    if(!switches.drive_enable) { return OFF;}
    // switch 2 turned off while 1 is on
    else if(!switches.drive_engage) { return ON;}

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




float motorOut(float throttle, iCANflex& car, switchboard& switches) {
    const int HIGH_PWR_R_CURRENT = 50;
    const int LOW_PWR_R_CURRENT = 25; 
    // change these to a continuous power curve later^^^
    return switches.pwr_lvl ? HIGH_PWR_R_CURRENT *throttle: LOW_PWR_R_CURRENT*throttle; 
}

State drive(iCANflex& Car, switchboard& switches, bool& BSE_APPS_violation) {
    if(!switches.drive_enable) return OFF;
    if(!switches.drive_engage) return ON;

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

State error(iCANflex& Car, switchboard& switches, State prevState, volatile bool (*errorCheck)(iCANflex& c, switchboard& s)) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);

    
    if(!switches.drive_enable) return OFF;
    if(!switches.drive_engage) return ON;

    

    if(errorCheck(Car, switches)) {
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
