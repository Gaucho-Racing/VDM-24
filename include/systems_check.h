#ifndef SYSTEMS_CHECK
#define SYSTEMS_CHECK

#include "machine.h"

//BSE and APPS check for input at startup ONLY
static volatile bool ECU_Startup_Rejection(iCANflex& Car) {
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

static volatile bool Critical_Systems_Fault(iCANflex& Car) {
    return false; //implement later
    Serial.println("CRITICAL SYSTEMS FAULT");
}

static volatile void Warning_Systems_Fault(iCANflex& Car) {
    Serial.println("NON CRITICAL ERROR CODES");
}
const int CAN_MS_THRESHOLD = 100; // msec


// // CAN RECIEVE FAILURES
// volatile bool CRITICAL_CAN_FAILURE(iCANflex& Car) {
//     return 
//         Car.DTI.getAge() > CAN_MS_THRESHOLD ||
//         Car.ECU.getAge() > CAN_MS_THRESHOLD ||
//         Car.PEDALS.getAge() > CAN_MS_THRESHOLD ||
//         Car.ACU1.getAge() > CAN_MS_THRESHOLD ||
//         Car.BCM1.getAge() > CAN_MS_THRESHOLD ||
//         Car.ENERGY_METER.getAge() > CAN_MS_THRESHOLD;
// }

// volatile bool NON_CRITICAL_CAN_FAILURE(iCANflex& Car){
//     return 
//         Car.WFL.getAge() > CAN_MS_THRESHOLD ||
//         Car.WFR.getAge() > CAN_MS_THRESHOLD ||
//         Car.WRL.getAge() > CAN_MS_THRESHOLD ||
//         Car.WRR.getAge() > CAN_MS_THRESHOLD ||
//         Car.DASHBOARD.getAge() > CAN_MS_THRESHOLD ||
//         Car.GPS1.getAge() > CAN_MS_THRESHOLD;
// }   

#endif
