#include "systems_check.h"

bool SystemsCheck::rtd_brake_fault(const iCANflex& Car) {
    if (Car.PEDALS.getAPPS1() > 0.05 || 
        Car.PEDALS.getAPPS2() > 0.05 ||
        Car.PEDALS.getBrakePressureF() <= 0.05 || 
        Car.PEDALS.getBrakePressureR() <= 0.05) {
        Serial.println("ECU STARTUP REJECTION: HOLD BRAKES");
        // send error code to dash
        return true;
    }
    return false;
}

bool SystemsCheck::critical_sys_fault(const iCANflex& Car){
    return false; 
    //implement later
    Serial.println("CRITICAL SYSTEMS FAULT");
}

bool SystemsCheck::warn_sys_fault(const iCANflex& Car){
    Serial.println("NON CRITICAL ERROR CODES");
    return false;
}

bool SystemsCheck::critical_can_failure(const iCANflex& Car){
    return 
        Car.DTI.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ECU.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.PEDALS.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ACU1.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.BCM1.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ENERGY_METER.getAge() > SystemsCheck::CAN_MS_THRESHOLD;
}

bool SystemsCheck::warn_can_failure(const iCANflex& Car){
    return 
        Car.WFL.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WFR.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WRL.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WRR.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.DASHBOARD.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.GPS1.getAge() > SystemsCheck::CAN_MS_THRESHOLD;
}   


bool SystemsCheck::AMS_fault(const iCANflex& Car){
    if(analogRead(AMS_OK_PIN) < 310) return true; // and send a can light thing
    else if(analogRead(AMS_OK_PIN) > 730 && analogRead(AMS_OK_PIN) < 760) return false;
    return true;
}
bool SystemsCheck::IMD_fault(const iCANflex& Car){
    if(analogRead(IMD_OK_PIN) < 310) return true; // and can light thing
    else if(analogRead(IMD_OK_PIN) > 730 && analogRead(IMD_OK_PIN) < 760) return false;
    return true;
}
bool SystemsCheck::BSPD_fault(const iCANflex& Car){
    if(analogRead(BSPD_OK_PIN) < 310) return true;
    else if(analogRead(BSPD_OK_PIN) > 730 && analogRead(BSPD_OK_PIN) < 760) return false;
    return true;
}
bool SystemsCheck::SDC_opened(const iCANflex& Car){
    return false; // TODO: implement
}

