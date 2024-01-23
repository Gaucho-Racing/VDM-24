#include "machine.h"


volatile State state;
volatile bool (*errorCheck)(iCANflex& Car); 
bool BSE_APPS_violation = false;

State sendToError(volatile bool (*erFunc)(iCANflex& Car)) {
   errorCheck = erFunc; 
   return ERROR;
}

void loop(){
    // read bspd, ams, and imd pins as analog


   


    // switchboard CAN stuff for moving from glv_on to ts_precharge from the ts_active switch
    // also the brake + rtd switch to move from ts_precharge to rtd_0tq 
    // wait for a ping, use a callback function and pointer to a function
    // will require changing nodes.h.


    // boolean indicating the SDC is open (ex: ESTOP Pressed) 

    // STATE MACHINE OPERATION
    switch (state) {
        case GLV_ON: // GLV ON
            state = glv_on(Car);
            break;
        case TS_PRECHARGE:
            state = ts_precharge(Car);
            break;
        case RTD_0TQ:
            state = rtd_0tq(Car, BSE_APPS_violation); 
            break;
        case DRIVE_TORQUE:
            state = drive_torque(Car, BSE_APPS_violation);
            break;
        case ERROR:
            state = error(Car, errorCheck);
            break;
    }
}



//GLV STARTUP
void setup() {

    Serial.begin(9600);
    Serial.println("Waiting for Serial Port to connect");
    while(!Serial) Serial.println("Waiting for Serial Port to connect");
    Serial.println("Connected to Serial Port 9600");

    Car.begin();

     // set state  
    state = GLV_ON; 

    // Read the SD CARD Settings for the ECU TUNE ON STARTUP
    // Serial.println("Initializing SD Card...");
    // if(!SD.begin(BUILTIN_SDCARD)){
    //     Serial.println("CRITICAL FAULT: PLEASE INSERT SD CARD CONTAINING ECU FLASH TUNE");
    //     Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");

    //     state = ERROR;
    // }
    // else{
    //     Serial.println("SD INITIALIZATION SUCCESSFUL");
    //     File ecu_tune;
    //     ecu_tune = SD.open("GR24_FLASH_TUNE.ecu");
    //     if(ecu_tune){
    //         Serial.print("Reading ECU FLASH....");
    //         String tune;
    //         while(ecu_tune.available()){
    //             Serial.print(".");
    //             tune += (char)ecu_tune.read();
    //         }
    //         ecu_tune.close();
    //         Serial.println("");

    //         Serial.println("ECU FLASH COMPLETE. GR24 TUNE DOWNLOADED.");

    //     }
    //     else {
    //         Serial.println("CRITICAL FAULT: ERROR OPENING GR24 ECU TUNE");
    //         Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");
    //         state = ERROR;
    //     }
    // }
    




   
}

