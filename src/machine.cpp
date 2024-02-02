#include <Arduino.h>
#include <imxrt.h>
#include "machine.h"
#include "sstream"





State ecu_flash(iCANflex& Car) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    // flash the ecu
    Serial.println("Initializing SD Card...");
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println("CRITICAL FAULT: PLEASE INSERT SD CARD CONTAINING ECU FLASH TUNE");
        Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");
        return ERROR;
    }
    else{
        Serial.println("SD INITIALIZATION SUCCESSFUL");
        File ecu_tune;
        ecu_tune = SD.open("gr24.txt");
        if(ecu_tune){
            Serial.print("Reading ECU FLASH....");
            String tune;
            while(ecu_tune.available()){
                Serial.print(".");
                tune += (char)ecu_tune.read();
            }
            ecu_tune.close();
            Serial.println("");

            const stringstream iss(tune.c_str()); // const so put into FLASH MEMORY
            // read in torque profiles, regen profiles, and traction profiles
            Serial.println("ECU FLASH COMPLETE. GR24 TUNE DOWNLOADED.");

        }
        else {
            Serial.println("CRITICAL FAULT: ERROR OPENING GR24 ECU TUNE");
            Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");
            return ERROR;
        }
    }  
    return GLV_ON;
}

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
    // WRITE TO THE SOFTWARE OK PIN

<<<<<<< HEAD
=======
    // wait for the TS ACTIVE button to be pressed
>>>>>>> 49bd2fd (rebase)
    return GLV_ON;
}  


/*

PRECHARGING PROCESS


*/

State ts_precharge(iCANflex& Car) { 
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    // run a system check
    // begin prechargin
    return PRECHARGING;
}

State precharging(iCANflex& Car){
   
    // wait for precharge complete signal
    return PRECHARGING;
}

State precharge_complete(iCANflex& Car){
    return PRECHARGE_COMPLETE;
    
    
    // wait for RTD signal
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
<<<<<<< HEAD
    float tq_percent = (throttle-(1-throttle)*(throttle+TORQUE_PROFILE_B)*pow(rpm/REV_LIMIT, TORQUE_PROFILE_P)*TORQUE_PROFILE_K);
    return tq_percent*MAX_MOTOR_CURRENT;
=======
    float k = TORQUE_PROFILES[THROTTLE_MAPPING].K;
    float p = TORQUE_PROFILES[THROTTLE_MAPPING].P;
    float b = TORQUE_PROFILES[THROTTLE_MAPPING].B;
    float current = TORQUE_PROFILES[THROTTLE_MAPPING].MAX_CURRENT;
    float tq_percent = (throttle-(1-throttle)*(throttle+b)*pow(rpm/REV_LIMIT, p)*k);
    if(tq_percent > 1) tq_percent = 1;
    if(tq_percent < 0) tq_percent = 0;
    return tq_percent*current;
>>>>>>> 49bd2fd (rebase)
}


State drive_torque(iCANflex& Car, bool& BSE_APPS_violation) {
    float a1 = Car.PEDALS.getAPPS1();
    float a2 = Car.PEDALS.getAPPS2();
    float throttle = Car.PEDALS.getThrottle();
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
    if((brake > 0.05 && a1 > 0.25)) {
        BSE_APPS_violation = true;
        return RTD_0TQ;
    }


    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(requested_torque(Car, throttle, Car.DTI.getERPM()/10.0));
    
    return DRIVE;
}



State regen_torque(iCANflex& Car){
    return REGEN_TORQUE;
}

/*
ERROR STATE

THIS STATE WILL HANDLE ERRORS THAT OCCUR DURING THE OPERATION OF THE VEHICLE.
THIS STATE WILL BE ENTERED WHENEVER A CRITICAL SYSTEMS FAILURE OCCURS OR WHEN THE
DRIVER REQUESTS TO STOP THE VEHICLE.

THE VEHICLE REMAINS IN THIS STATE UNTIL THE VIOLATION IS RESOLVED 

*/


State error(iCANflex& Car, volatile bool (*errorCheck)(const iCANflex& c)) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    
    if(!switches[0]) return OFF;
    //if(!switches[1]) return ON;

    if(errorCheck(Car))  return ERROR;
    else {
        float throttle = Car.PEDALS.getAPPS1();
        if(throttle < 0.05) {
            return GLV_ON;
        }
        return ERROR;
    }
    
}
