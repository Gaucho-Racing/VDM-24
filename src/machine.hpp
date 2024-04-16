#ifndef MACHINE
#define MACHINE

#include <Arduino.h>
#include <imxrt.h>
#include <sstream>
#include <iostream>
#include <SD.h>
#include "tune.hpp"
#include "unordered_set"

using namespace std;

enum State {ECU_FLASH, GLV_ON, TS_PRECHARGE, PRECHARGING, PRECHARGE_COMPLETE, DRIVE_NULL, DRIVE_TORQUE, DRIVE_REGEN, ERROR};
enum Mode {TESTING, LAUNCH, ENDURANCE, AUTOX, SKIDPAD, ACC, PIT};

/*

STARTUP STAGE 1:
ECU FLASH
When the Car is turned on, the Main ECU will read in the ECU flash from the SD card.
This will be the first state that the car will enter.
This is essential for the car to operate as the ECU flash contains the 
torque profiles, regen profiles, and traction control profiles.
*/
State ecu_flash(iCANflex& Car) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    // flash the ecu
    return ECU_FLASH;

}

/*
STARTUP STAGE 2:    
GLV ON
When the grounded low voltage system is turned on, the microcontroller has power, 
but the motor controller is not enabled. This is the second state that the car will enter
after the ECU Flash is complete. Here it waits for the TS ACTIVE button to be pressed.
*/
State glv_on(iCANflex& Car) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);    

    // wait for the TS ACTIVE button to be pressed
    return TS_PRECHARGE;
    // return GLV_ON;
}  


/*
STARTUP STAGE 3: PRECHARGING
When the TS ACTIVE button is pressed, the car will enter the precharging state.
This is the third state that the car will enter after the GLV_ON state.
The precharging is essential for the car to operate as it allows the voltage to build up
in the motor controller before the car can be driven.
PRECHARGING is broken into 3 stages for ACU responses and communication
*/

// -- PRECHARGING STAGE 1 
State ts_precharge(iCANflex& Car) { 
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    // begin precharging by sendign signal to ACU
    // wait for signal back
    // if dont get signal back 
    return PRECHARGING;
}
// -- PRECHARGING STAGE 2
State precharging(iCANflex& Car){
   
    // wait for precharge complete signal
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    return PRECHARGE_COMPLETE;
}

// -- PRECHARGING STAGE 3
State precharge_complete(iCANflex& Car){
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);

    // wait for RTD signal
    return DRIVE_NULL;  
}

/*
STARTUP STAGE 4:  READY TO DRIVE

READY TO DRIVE SUB STATES
- DRIVE_NULL
- DRIVE_TORQUE
- DRIVE_REGEN

*/
State drive_null(iCANflex& Car, bool& BSE_APPS_violation, Mode mode) {
    // Car.DTI.setDriveEnable(0); // TODO: Make this frequency lower to 100hz
    // Car.DTI.setRCurrent(0);

    // float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2.0;
    // float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2.0;
    
    // // only if no violation, and throttle is pressed, go to DRIVE
    // if(!BSE_APPS_violation && throttle > 0.05) return DRIVE_TORQUE;
    // if(!BSE_APPS_violation && brake > 0.05) return DRIVE_REGEN;

    // if(BSE_APPS_violation) {
    //     // SEND CAN WARNING TO DASH
    //     if(throttle < 0.05) {
    //         // violation exit condition, reset violation and return to DRIVE_READY
    //         BSE_APPS_violation = false;
    //         return DRIVE_NULL;
    //     }  
    // }
    // else loop back into RTD state with Violation still true
    return DRIVE_NULL;
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

THE CONSTANTS B, K, AND P ARE DEFINED THROUGHOUT THE ECU MAP IN THE SD CARD OR THE REFLASH OVER CAN.
THIS VALUE OF Z IS APPLIED TO THE MAX CURRENT SET AND WILL BE THE DRIVER REQUESTED TORQUE. 
THIS IS FOR A GENERALLY SMOOTHER TORQUE PROFILE AND DRIVABILITY.

THE DRIVE_TORQUE STATE IS ALSO RESPONSIBLE FOR CHECKING THE APPS AND BSE FOR VIOLATIONS AS WELL AS 
THE GRADIENTS OF THE TWO APPS SIGNALS TO MAKE SURE THAT THEY ARE NOT COMPROMISED. 
*/


float requested_torque(iCANflex& Car, float throttle, int rpm, Tune& tune, uint8_t power_level, uint8_t throttle_map, uint8_t regen_level) {
    // python calcs: z = np.clip((x - (1-x)*(x + b)*((y/5500.0)**p)*k )*100, 0, 100)
    TorqueProfile tp = tune.getTorqueProfile(throttle_map);
    float k = tp.K;
    float p = tp.P;
    float b = tp.B;
    float max_current = tune.getPowerSetting(power_level);
    float tq_percent = (throttle-(1-throttle)*(throttle+b)*pow(rpm/tune.revLimit(), p)*k);
    if(tq_percent > 1) tq_percent = 1; // clipping
    if(tq_percent < 0) tq_percent = 0;
    return tq_percent*max_current;
}


State drive_torque(iCANflex& Car, bool& BSE_APPS_violation, Mode mode, Tune& tune) {

    // float a1 = Car.PEDALS.getAPPS1();
    // float a2 = Car.PEDALS.getAPPS2();
    // float throttle = a1; // TODO: FIX
    // float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2;
    
    // // APPS GRADIENT VIOLATION
    // if(abs(a1 - (2*a2)) > 0.1){
    //     // send an error message on the dash that APPS blew up
    //     return DRIVE_NULL;
    // } 
    // // APPS BSE VIOLATION
    // if((brake > 0.05 && a1 > 0.25)) {
    //     BSE_APPS_violation = true;
    //     return DRIVE_NULL;
    // }
    // Car.DTI.setDriveEnable(1);
    // Car.DTI.setRCurrent(requested_torque(Car, throttle, Car.DTI.getERPM()/10.0, tune, ));
    // // float power = Car.ACU1.getAccumulatorVoltage() * Car.DTI.getDCCurrent();

    return DRIVE_TORQUE;
}

float requested_regenerative_torque(iCANflex& Car, float brake, int rpm) {
    // if(rpm > 500 && brake > 0.05) return Car.ACU1.getMaxChargeCurrent();
    // else return 0;
    return 0;
}

State drive_regen(iCANflex& Car, bool& BSE_APPS_violation, Mode mode, Tune& tune, uint8_t regen_level){
    // float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2;
    // float throttle = Car.PEDALS.getAPPS1();
    // if(throttle > 0.05) return DRIVE_TORQUE;
    // if(brake < 0.05) return DRIVE_NULL;

    // float rpm = Car.DTI.getERPM()/10.0;
    // Car.DTI.setDriveEnable(1);
    // Car.DTI.setRCurrent(-1 * requested_regenerative_torque(Car, brake, rpm) * tune.getRegenSetting(regen_level));
    return DRIVE_REGEN;
}

/*
ERROR STATE

THIS STATE WILL HANDLE ERRORS THAT OCCUR DURING THE OPERATION OF THE VEHICLE.
THIS STATE WILL BE ENTERED WHENEVER A CRITICAL SYSTEMS FAILURE OCCURS OR WHEN THE
DRIVER REQUESTS TO STOP THE VEHICLE.

THE VEHICLE REMAINS IN THIS STATE UNTIL THE VIOLATION IS RESOLVED 

*/


State error(iCANflex& Car, bool (*errorCheck)(const iCANflex& c), unordered_set<bool (*)(const iCANflex& c)>& active_faults){
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    if(errorCheck(Car))  return ERROR;
    else {
        active_faults.erase(errorCheck);
        return GLV_ON; // gets sent back to error from main() if there are more in the hashset from main
    }
    
}

#endif