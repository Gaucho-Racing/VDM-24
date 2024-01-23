#include <Arduino.h>
#include <imxrt.h>
#include "machine.h"


/*

GLV_ON STATE

THIS STATE IS ACTIVE WHEN THE MICROCONTROLLER IS POWERED DUE TO THE 
GLV MASTER SWITCH BEING TURNED. THIS STATE IS RESPONSIBLE FOR THE IDLE 
STATE OF THE VEHICLE DYNAMICS MODULE. 

THIS STATE IS RESPONSIBLE FOR THE FOLLOWING:
    - SETTING THE DRIVE ENABLE TO 0
    - SETTING THE MOTOR CURRENT TO 0
    - WAITING FOR THE TS ACTIVE SWITCH TO BE PRESSED
    - RECONFIGURING THE TUNE PARAMETERS THROUGH THE CAN DEVICE
*/



bool reject_on = true;

State off(iCANflex& Car, const vector<int>& switches) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);    
   
    reject_on = switches[1];
    if (switches[0] && !reject_on) return ON;
    if (!switches[0] && !switches[1]) reject_on = false;
    return OFF;
}  


/*

TS_PRECHARGE STATE

THIS STATE IS ACTIVE WHEN THE TS ACTIVE SWITCH IS PRESSED. THIS STATE IS ENTERED
AFTER THE TS ACTIVE BUTTON ON THE DASH PANEL IS PRESSED AND A PING IS RECIEVED BY THE 
VDM. ONCE THIS HAPPENS THE VEHICLE MUST BEGIN THE PRECHARGING AND WAIT FOR A SIGNAL FROM
THE ACU THAT THE PRECHARGING IS COMPLETE BEFORE THE RTD BUTTON CAN BE PRESSED. 

THIS STATE IS RESPONSIBLE FOR THE FOLLOWING:
    - SETTING THE DRIVE ENABLE TO 0
    - SETTING THE MOTOR CURRENT TO 0
    - ACTING AS AN INTERMEDIARY STATE BETWEEN GLV_ON AND RTD_0TQ
    - WAITING FOR THE PRECHARGE COMPLETE SIGNAL FROM THE ACU
    - PLAYING THE RTD SOUND WHEN THE RTD BUTTON IS PRESSED
    - SENDING A CAN MESSAGE TO THE DASH TO INDICATE THAT THE VEHICLE IS READY TO DRIVE
    - PERFORMING A COMPLETE SYSTEMS CHECK BEFORE THE VEHICLE IS READY TO DRIVE

*/




State ts_precharge(iCANflex& Car) { 
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);

    if(/*rtd pressed but rtd_brake_fault*/ false){
        // can message for dash
        return TS_PRECHARGE;
    }
    else if(/*can message for rtd button + brake*/ false) {
        if(SystemsCheck::critical_sys_fault(Car)) return ERROR;
        SystemsCheck::warn_sys_fault(Car);
        // WAIT FOR PRECHARGE COMPLETE SIGNAL FROM ACU!!!!!!
        // play RTD sound
        return RTD_0TQ;
    } 
    return TS_PRECHARGE;
}



 // PRECHARGING MUST BE COMPLETE BEFORE ENTERING THIS STATE
State rtd_0tq(iCANflex& Car, bool& BSE_APPS_violation) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    //start cooling system and all that 

    // switch 1 turned off 
    if(!switches[0]) { return OFF;}
    // switch 2 turned off while 1 is on
    else if(!switches[1]) { return ON;}

    float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2.0;
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2.0;
    
    // only if no violation, and throttle is pressed, go to DRIVE
    if(!BSE_APPS_violation && throttle > 0.05) return DRIVE_TORQUE;

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


/*
DRIVE_TORQUE STATE

THIS STATE IS RESPONSIBLE FOR THE VEHICLE DYNAMICS WHEN THE DRIVER IS REQUESTING TORQUE FROM THE MOTOR.
THE TORQUE IS CALCULATED THROUGH THE STANDARD EQUATION DEFINED BELOW. 
Z = X-(1-X)(X+B)(Y^P)K  0 <= Z <= 1 (CLIPPED)
X IS THROTTLE 0 TO 1
Y IS RPM LOAD 0 TO 1
B IS OFFSET 0 TO 1 
K IS MULTIPLIER 0 TO 1
P IS STEEPNESS 0 TO 5

THE CONSTANTS B, K, AND P ARE DEFINED THROUGHT THE ECU MAP IN THE SD CARD OR THE REFLASH OVER CAN.
THIS VALUE OF Z IS APPLIED TO THE MAX CURRENT SET AND WILL BE THE DRIVER REQUESTED TORQUE. 
THIS IS FOR A GENERALLY SMOOTHER TORQUE PROFILE AND DRIVABILITY.


THE DRIVE_TORQUE STATE IS ALSO RESPONSIBLE FOR CHECKING THE APPS AND BSE FOR VIOLATIONS AS WELL AS 
THE GRADIENTS OF THE TWO APPS SIGNALS TO MAKE SURE THAT THEY ARE NOT COMPROMISED. 
*/


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
    if(abs(a1 - (2*a2)) > 0.1){
        // send an error message on the dash
        return RTD_0TQ;
    } 
    // APPS BSE VIOLATION
    if(brake > 0.05 && a1 > 0.25) {
        BSE_APPS_violation = true;
        return RTD_0TQ;
    }


    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(requested_torque(Car, throttle, Car.DTI.getERPM()/10.0));
    
    return DRIVE;
}


/*
ERROR STATE

THIS STATE WILL HANDLE ERRORS THAT OCCUR DURING THE OPERATION OF THE VEHICLE.
THIS STATE WILL BE ENTERED WHENEVER A CRITICAL SYSTEMS FAILURE OCCURS OR WHEN THE
DRIVER REQUESTS TO STOP THE VEHICLE.

THE VEHICLE REMAINS IN THIS STATE UNTIL THE VIOLATION IS RESOLVED 

*/


State error(iCANflex& Car, volatile bool (*errorCheck)(iCANflex& c)) {
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
