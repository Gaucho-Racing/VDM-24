#include <Arduino.h>
#include <imxrt.h>
#include "machine.h"

bool reject_on = true;

State off(iCANflex& Car, const vector<int>& switches) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);    
   
    reject_on = switches[1];
    if (switches[0] && !reject_on) return ON;
    if (!switches[0] && !switches[1]) reject_on = false;
    return OFF;
}  

// ON is when PRECHARGING BEGINS
State ts_precharge(iCANflex& Car) { 
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);

    if( ECU_Startup_Rejection(Car)) {
        return TS_PRECHARGE;
    }
    else {
        if(Critical_Systems_Fault(Car)) return ERROR;
        Warning_Systems_Fault(Car);
        // start powertrain cooling
        // WAIT FOR PRECHARGE COMPLETE SIGNAL FROM ACU!!!!!!
        // play RTD sound
        return RTD_0TQ;
    } 
    return TS_PRECHARGE;
}



 // PRECHARGING MUST BE COMPLETE BEFORE ENTERING THIS STATE
State drive_ready(iCANflex& Car, const vector<int>& switches, bool& BSE_APPS_violation) {
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
        // SEND CAN WARNING TO DASH
        if(throttle < 0.05) {
            // violation exit condition, reset violation and return to DRIVE_READY
            BSE_APPS_violation = false;
            return RTD_0TQ;
        }  
    }
    // else loop back into RTD state with Violation still true
    return RTD_0TQ;
}

float requested_torque(iCANflex& Car, float throttle, int rpm) {
    // z = np.clip((x - (1-x)*(x + b)*((y/5500.0)**p)*k )*100, 0, 100)
    float tq_percent = (throttle-(1-throttle)*(throttle+TORQUE_PROFILE_B)*pow(rpm/REV_LIMIT, TORQUE_PROFILE_P)*TORQUE_PROFILE_K);
    return tq_percent*MAX_MOTOR_CURRENT;
}

State drive_torque(iCANflex& Car, bool& BSE_APPS_violation) {
    float a1 = Car.PEDALS.getAPPS1();
    float a2 = Car.PEDALS.getAPPS2();
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2;
    
    // APPS GRADIENT VIOLATION
    if(abs(a1 - (2*a2)) > 0.1){
        // send an error message on the dash that APPS blew up
        return DRIVE_NULL;
    } 
    // APPS BSE VIOLATION
    if((brake > 0.05 && a1 > 0.25)) {
        BSE_APPS_violation = true;
        return DRIVE_NULL;
    }
    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(requested_torque(Car, throttle, Car.DTI.getERPM()/10.0));
    float power = Car.ACU1.getAccumulatorVoltage() * Car.DTI.getDCCurrent();

    return DRIVE_TORQUE;
}

float requested_regenerative_torque(iCANflex& Car, float brake, int rpm) {
    // if(rpm > 500 && brake > 0.05) return Car.ACU1.getMaxChargeCurrent();
    // else return 0;
    return 0;
}
=======
>>>>>>> 30f8389 (cdr)

State drive_regen(iCANflex& Car, bool& BSE_APPS_violation, Mode mode){
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2;
<<<<<<< HEAD
    float throttle = Car.PEDALS.getAPPS1();
    if(throttle > 0.05) return DRIVE_TORQUE;
    if(brake < 0.05) return DRIVE_NULL;
=======
    
    // APPS GRADIENT VIOLATION
    if(abs(Car.PEDALS.getAPPS1() - (2*Car.PEDALS.getAPPS2())) > 0.1)  return RTD_0TQ;

    // APPS BSE VIOLATION
    if(brake > 0.05 && a1 > 0.25) {
        BSE_APPS_violation = true;
        return RTD_0TQ;
    }


    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(requested_torque(Car, throttle, Car.DTI.getERPM()/10.0));
    
    return DRIVE;
}

State error(iCANflex& Car, const vector<int>& switches, State prevState, volatile bool (*errorCheck)(iCANflex& c)) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    
    if(!switches[0]) return OFF;
    //if(!switches[1]) return ON;

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
