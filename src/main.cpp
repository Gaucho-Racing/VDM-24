#include "machine.h"
#include "sstream"

volatile State state;
bool (*errorCheck)(const iCANflex& Car); 
bool BSE_APPS_violation = false;

State sendToError(bool (*erFunc)(const iCANflex& Car)) {
   errorCheck = erFunc; 
   return ERROR;
}

void loop(){
    // read bspd, ams, and imd pins as analog   

    SystemsCheck::run_system_check(*Car);
    if(active_faults.size()) state = sendToError(*active_faults.begin());

    // read in settings from Steering Wheel
    THROTTLE_MAPPING = 0; // read from can
    REGEN_LEVEL = 0;
    TRACTION_MODE = 0;


    // STATE MACHINE OPERATION
    switch (state) {
        case ECU_FLASH:
            state = ecu_flash(*Car);
            break;
        case GLV_ON: // GLV ON
            state = glv_on(*Car);
            break;
        case TS_PRECHARGE:
            state = ts_precharge(*Car);
            break;
        case PRECHARGING:
            state = precharging(*Car);
            break;
        case PRECHARGE_COMPLETE:
            state = precharge_complete(*Car);
            break;
        case RTD_0TQ:
            state = rtd_0tq(*Car, BSE_APPS_violation); 
            break;
        case DRIVE_TORQUE:
            state = drive_torque(*Car, BSE_APPS_violation);
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



//GLV STARTUP
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

