#include "machine.h"
#include "sstream"




volatile State state;
volatile bool (*errorCheck)(const iCANflex& Car); 
bool BSE_APPS_violation = false;

State sendToError(volatile bool (*erFunc)(const iCANflex& Car)) {
   errorCheck = erFunc; 
   return ERROR;
}

volatile bool motorTempHighExitCondition(iCANflex& Car) {
    if (Car.DTI.getMotorTemp() < 55) {
        return false;
    }
    return true;
}

volatile bool motorTempHighEntryCondition(iCANflex& Car) {
    if (Car.DTI.getMotorTemp() >= 60) {
        return true;
    }
    return false;
}

void motorTempHigh_ISR() {
    Car.sendDashError(5); // send placeholder error byte code to dash
    state = sendToError(state, motorTempHighExitCondition);
}

const int canFailureThreshold = 100; // msec

volatile bool canReceiveFailure(iCANflex& Car) {
    return 
        Car.DTI.getAge() > canFailureThreshold ||
        Car.ECU.getAge() > canFailureThreshold ||
        Car.WFL.getAge() > canFailureThreshold ||
        Car.WFR.getAge() > canFailureThreshold ||
        Car.WRL.getAge() > canFailureThreshold ||
        Car.WRR.getAge() > canFailureThreshold ||
        Car.GPS1.getAge() > canFailureThreshold ||
        Car.PEDALS.getAge() > canFailureThreshold ||
        Car.ACU1.getAge() > canFailureThreshold ||
        Car.BCM1.getAge() > canFailureThreshold ||
        Car.DASHBOARD.getAge() > canFailureThreshold ||
        Car.ENERGY_METER.getAge() > canFailureThreshold;
}

void canReceiveFailure_ISR() {
    Car.sendDashError(100);
    state = sendToError(state, canReceiveFailure);
}

volatile bool currentLimitSafe(iCANflex& Car) {
    return (Car.DTI.getDCCurrent() > 575); // based on below current limit
}

volatile bool currentLimitExceeded(iCANflex& Car) {
    return (Car.DTI.getDCCurrent() > 600); // taken from last year's implementation
}

void currentLimitExceeded_ISR() {
    Car.sendDashError(106);
    state = sendToError(state, currentLimitSafe);
}

volatile bool shutdown_pinned(iCANflex& Car) {
    return (bool)digitalRead(shutdown_pin);
}

void shutdown_pinned_ISR() {
    Car.sendDashError(150);
    state = sendToError(OFF, shutdown_pinned);
}

void loop(){
    // read bspd, ams, and imd pins as analog   

    SystemsCheck::hardware_system_critical(*Car);

    if(active_faults.size()) state = sendToError(*active_faults.begin());
    
    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);

    // read in settings from Steering Wheel
    THROTTLE_MAPPING = 0; // read from can
    REGEN_LEVEL = 0;
    TRACTION_MODE = 0;



    // boolean indicating the SDC is open (example: ESTOP Pressed) 

    // STATE MACHINE OPERATION
    switch (state) {
        case ECU_FLASH:
            state = ecu_flash(*Car);
            break;
        case GLV_ON: // GLV ON
            state = glv_on(Car);
            break;
        case TS_PRECHARGE:
            state = ts_precharge(Car);
            break;
        case PRECHARGING:
            state = precharging(*Car);
            break;
        case PRECHARGE_COMPLETE:
            state = precharge_complete(*Car);
            break;
        case RTD_0TQ:
            state = rtd_0tq(Car, BSE_APPS_violation); 
            break;
        case DRIVE_TORQUE:
            state = drive_torque(Car, BSE_APPS_violation);
            break;
        case REGEN_TORQUE:
            state = regen_torque(*Car);
            break;
        case ERROR:
            state = error(*Car, errorCheck);
            break;
        case ERROR_RESOLVED:
            if(active_faults.size() == 0) state = GLV_ON;
            else state = sendToError(*active_faults.begin());
            break;
    }
}

void setup() {
    Car = new iCANflex();
    Serial.begin(9600);
    Serial.println("Waiting for Serial Port to connect");
    while(!Serial) Serial.println("Waiting for Serial Port to connect");
    Serial.println("Connected to Serial Port 9600");


    Car->begin();

     // set state  
    state = GLV_ON; 

    // Read the SD CARD Settings for the ECU TUNE ON STARTUP
    ecu_flash(*Car);
}

