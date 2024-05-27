#ifndef MACHINE_H
#define MACHINE_H
#include "VehicleTuneController.h"
#include "global.h"
#include "vehicle.h"
#include "SystemsCheck.h"
#include "network.h"

/*
   ______________  ____________   __  ______   ________  _______   ________
  / ___/_  __/   |/_  __/ ____/  /  |/  /   | / ____/ / / /  _/ | / / ____/
  \__ \ / / / /| | / / / __/    / /|_/ / /| |/ /   / /_/ // //  |/ / __/   
 ___/ // / / ___ |/ / / /___   / /  / / ___ / /___/ __  // // /|  / /___   
/____//_/ /_/  |_/_/ /_____/  /_/  /_/_/  |_\____/_/ /_/___/_/ |_/_____/   
    ____  ____  __________  ___  ______________  _   __
  / __ \/ __ \/ ____/ __ \/   |/_  __/  _/ __ \/ | / /
 / / / / /_/ / __/ / /_/ / /| | / /  / // / / /  |/ / 
/ /_/ / ____/ /___/ _, _/ ___ |/ / _/ // /_/ / /|  /  
\____/_/   /_____/_/ |_/_/  |_/_/ /___/\____/_/ |_/   
                                                                                                                            
*/




/*

STARTUP STAGE 1:
ECU FLASH
When the Car is turned on, the Main ECU will read in the ECU flash from the SD card.
This will be the first state that the car will enter.
This is essential for the car to operate as the ECU flash contains the 
torque profiles, regen profiles, and traction control profiles.
*/





State ecu_flash(VehicleTuneController* t) {
    // Car.DTI.setDriveEnable(0);
    // Car.DTI.setRCurrent(0);
    // flash the ecu
    t->readSDCard();
    Serial.println("ECU Flash Complete");
    return GLV_ON;

}

/*
STARTUP STAGE 2:    
GLV ON
When the grounded low voltage system is turned on, the microcontroller has power, 
but the motor controller is not enabled. This is the second state that the car will enter
after the ECU Flash is complete. Here it waits for the TS ACTIVE button to be pressed.
*/
State glv_on( Vehicle& Car) {
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        Car.times.setLastDTIMessage(millis());
    }
    // wait for the TS ACTIVE button to be pressed
    // return TS_PRECHARGE;
    return GLV_ON;
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
State ts_precharge(Vehicle& Car) { 
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        Car.times.setLastDTIMessage(millis());
    }
    if(Car.ACU1.getPrecharging()){
        return PRECHARGING;
    }
    else if(Car.ACU1.getPrechargeDone()){
        Car.ACU1.resetPrechargeDone();
        return PRECHARGE_COMPLETE;
    }
    return TS_PRECHARGE;
}
// -- PRECHARGING STAGE 2
State precharging(Vehicle& Car){
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        Car.times.setLastDTIMessage(millis());
    }

    if(Car.ACU1.getPrechargeDone()) return PRECHARGE_COMPLETE;
    return PRECHARGING;
}

// -- PRECHARGING STAGE 3
State precharge_complete(Vehicle& Car){
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        Car.times.setLastDTIMessage(millis());
    }
    // wait for RTD signal
    return PRECHARGE_COMPLETE;  
}


/*
STARTUP STAGE 4:  READY TO DRIVE

READY TO DRIVE SUB STATES


*/

bool (*errorCheck)(VehicleTuneController&); // global function pointer to error causing the ISR
State sendToError(bool (*erFunc)(VehicleTuneController& tune)) {
   errorCheck = erFunc; 
   return ERROR;
}


float getThrottle1(uint16_t a1, VehicleTuneController& tune){
    float throttle =  1.0 - ((a1 - tune.getAPPSFloor1()*1.0)/(tune.getAPPSZero1()-tune.getAPPSFloor1()));
    if (throttle < 0.05) return 0;
    else if (throttle > 1 && throttle < 1.1) return 1;
    else if (throttle > 1.1) return 0;
    else return throttle;
}

float getThrottle2(uint16_t a2,  VehicleTuneController& tune){
    float throttle =  1.0 - ((a2 - tune.getAPPSFloor2()*1.0)/(tune.getAPPSZero2()-tune.getAPPSFloor2()));
    if (throttle < 0.05) return 0;
    else if (throttle > 1 && throttle < 1.1) return 1;
    else if (throttle > 1.1) return 0;
    else return throttle;
}

State drive_standby(bool& BSE_APPS_violation, VehicleTuneController& tune, Vehicle& Car) {
    
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        Car.times.setLastDTIMessage(millis());
    }

    float throttle = getThrottle1(Car.PEDALS.getAPPS1(), tune);
    // ! CHANGE TO REAL BSE
    float brake = analogRead(BSE_HIGH);
    // only if no violation, and throttle is pressed, go to DRIVE

    if(!BSE_APPS_violation && throttle > 0.05) return DRIVE_ACTIVE;
    if(!BSE_APPS_violation && throttle == 0 && brake > 500 && Car.DTI.getERPM() > 250) return DRIVE_REGEN;//TODO: Fix

    if(BSE_APPS_violation) {
        // SEND CAN WARNING TO DASH
        sendDashPopup(0x01, 3);
        if(throttle < 0.05) {
            // violation exit condition, reset violation and return to DRIVE_READY
            BSE_APPS_violation = false;
            return DRIVE_STANDBY;
        }  
    }
    // else loop back into RTD state with Violation still true
    return DRIVE_STANDBY;
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
State drive_active(bool& BSE_APPS_violation, VehicleTuneController& tune, Vehicle& Car, SWSettings& settings, Mode& mode, float tc_multiplier){
    float throttle = getThrottle1(Car.PEDALS.getAPPS1(), tune);
    float a2 = getThrottle2(Car.PEDALS.getAPPS2(), tune);
    float brake = analogRead(BSE_HIGH); 
    // ! CHANGE TO REAL BSE
    if (throttle < 0.05) return DRIVE_STANDBY;
    // ACCELERATOR GRADIENT PLAUSIBILITY VIOLATION
    if(abs(throttle-a2) > 0.1){
        sendDashPopup(0x02, 3);
        return DRIVE_STANDBY;
    } 
    // APPS X BSE VIOLATION
    if((brake > 500 && throttle > 0.25)) {
        sendDashPopup(0x01, 1);
        BSE_APPS_violation = true;
        return DRIVE_STANDBY; // Put car into neutral state, no engine power
    }
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setDriveEnable(1);
        // TORQUE MAPPING FOR DRIVING AND STABILITY VIA NONLINEAR THROTTLE CONTROL
        // THROTTLE CURVE EQUATION: z = np.clip((x - (1-x)*(x + b)*((y/5500.0)**p)*k )*100, 0, 100) 
        TorqueProfile tp = tune.getActiveTorqueProfile(settings.throttle_map);
        float k = tp.K;
        float p = tp.P;
        float b = tp.B;
        float rpm = Car.DTI.getERPM()/10.0;
        float torque_multiplier = (throttle-(1-throttle)*(throttle+b)*pow(rpm/tune.revLimit(), p)*k);
        if(torque_multiplier > 1) torque_multiplier = 1; // clipping
        if(torque_multiplier < 0) torque_multiplier = 0;
        float r_current = torque_multiplier*100;
        if(settings.throttle_map == LINEAR_TORQUE) r_current = throttle*100;
        if(mode == DYNAMIC_TC) r_current *= tc_multiplier;
        Car.DTI.setRCurrent(r_current);
        Car.times.setLastDTIMessage(millis());
    }
    return DRIVE_ACTIVE; // stay in the drive state
}


State drive_regen(bool& BSE_APPS_violation, VehicleTuneController& tune, SWSettings& settings, Vehicle& Car, SystemsCheck* sysCheck){
    if(settings.regen_level == REGEN_OFF) return DRIVE_STANDBY;

    float brake = analogRead(BSE_HIGH); 
    // ! CHANGE TO REAL BSE
    float throttle = getThrottle1(Car.PEDALS.getAPPS1(), tune);
    if(throttle > 0.05) return DRIVE_ACTIVE;
    if(brake < 500) return DRIVE_STANDBY;

    float rpm = Car.DTI.getERPM()/10.0;
    
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setDriveEnable(1);
        // Do this one in AMPS instead of Relative Current
        // 30A Max Regen, 15A Continuous/RMS
        float accumulator_input_amps = 0; 
        float steering_angle = 0; //TODO: in nodes
        // must be > 5 kph
        bool regen_ok = Car.ACU1.getSOC() < 85 && rpm > 250 && brake > 0.05 && throttle == 0 && abs(steering_angle) < tune.getMaxRegenSteeringAngle();
        bool max_regen_ok = regen_ok && rpm > tune.getRegenDumpMinRPM() && brake > 0.75 && !sysCheck->warn_battery_temp(tune, Car) && !sysCheck->limit_battery_temp(tune, Car);
        if(max_regen_ok) {// make sure revs are high enough for significant backemf
            // only for hard braking requests, dump energy back into accumulator
            accumulator_input_amps = tune.getRegenDumpAmps();
        } else if(regen_ok){
            // 250 - 2000 RPM interpolate from 0 to 15A based on RPM
            // 2000 - 3000 RPM is 15A 
            if (rpm < tune.getRegenRMSMaxRPM()) accumulator_input_amps = 15*(rpm-250)/(tune.getRegenRMSMaxRPM()-250);
            else accumulator_input_amps = tune.getRegenRMSAmps(); // TODO: Put this interpolation diagram in the tuning software
        } else {
            accumulator_input_amps = 0;
        }

        Car.DTI.setBrakeCurrent(-1 * accumulator_input_amps * tune.getActiveRegenPower(settings.regen_level));
        Car.times.setLastDTIMessage(millis());
    }
    return DRIVE_REGEN;
}

/*
ERROR STATE

THIS STATE WILL HANDLE ERRORS THAT OCCUR DURING THE OPERATION OF THE VEHICLE.
THIS STATE WILL BE ENTERED WHENEVER A CRITICAL SYSTEMS FAILURE OCCURS OR WHEN THE
DRIVER REQUESTS TO STOP THE VEHICLE.

THE VEHICLE REMAINS IN THIS STATE UNTIL ALL VIOLATIONS ARE RESOLVED 

*/
State error(VehicleTuneController& t, bool (*errorCheck)(VehicleTuneController& t), std::unordered_set<bool (*)(VehicleTuneController& t)>& active_faults, Vehicle& Car){
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        Car.times.setLastDTIMessage(millis());
    }

    if(errorCheck(t))  return ERROR;
    else {
        active_faults.erase(errorCheck);
        return GLV_ON; // gets sent back to error from main() if there are more in the hashset from main
    }
    
}

State ts_discharge_off(Vehicle& Car){
    if(millis() - Car.times.getLastDTIMessage() > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        Car.times.setLastDTIMessage(millis());
    }
    if(Car.ACU1.getTSVoltage() < 60) return GLV_ON;
    return TS_DISCHARGE_OFF;

}

#endif