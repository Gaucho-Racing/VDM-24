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
    // reads bspd, ams, and imd pins as analog   
    SystemsCheck::hardware_system_critical(*Car);


    state = active_faults.size() ?  sendToError(*active_faults.begin()) : GLV_ON;

    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);

    // read in settings from Steering Wheel
    THROTTLE_MAPPING = 0; 
    REGEN_LEVEL = 0;
    PWR_LEVEL = 0;


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
        case DRIVE_NULL:
            state = drive_null(*Car, BSE_APPS_violation); 
            break;
        case DRIVE_TORQUE:
            state = drive_torque(*Car, BSE_APPS_violation);
            break;
        case DRIVE_REGEN:
            state = drive_regen(*Car);
            break;
        case ERROR:
            state = error(*Car, errorCheck);
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
    state = ECU_FLASH; 
}

