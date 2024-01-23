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
    // .5v is shit -  ADC: 155
    // 3v when almost ok - ADC: 930
    // 2.4v is ok - ADC: 744
    // 1v = 310
    if(analogRead(BSPD_OK_PIN) < 310){

    }
    if(analogRead(AMS_OK_PIN) < 310){
        // send can message for light on dash panel
        state = ERROR;
    }
     else if(analogRead(AMS_OK_PIN) > 730 && analogRead(AMS_OK_PIN) < 760) state = GLV_ON;

    if(analogRead(IMD_OK_PIN) < 310){
        // send can message for light on dash panel
        state = ERROR;
    } 
    else if(analogRead(IMD_OK_PIN) > 730 && analogRead(IMD_OK_PIN) < 760) state = GLV_ON;

    // this coz it exists
    digitalWrite(SOFTWARE_OK_CONTROL_PIN, HIGH);

    // Brake Light Operation
    if(Car.PEDALS.getBrakePressureF() > 0.05 || Car.PEDALS.getBrakePressureR() > 0.05) {
        digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    }
    else {
        digitalWrite(BRAKE_LIGHT_PIN, LOW);
    }

    if(CRITICAL_CAN_FAILURE(Car)) {
        sendToError(&CRITICAL_CAN_FAILURE);
    }

    if(NON_CRITICAL_CAN_FAILURE(Car)){
        // warn on dash
    }


    // switchboard CAN stuff for moving from glv_on to ts_precharge from the ts_active switch
    // also the brake + rtd switch to move from ts_precharge to rtd_0tq 
    // wait for a ping, use a callback function and pointer to a function
    // will require changing nodes.h.


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

    // Read the SD CARD Settings for the ECU TUNE
    Serial.println("Initializing SD Card...");
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println("CRITICAL FAULT: PLEASE INSERT SD CARD CONTAINING ECU FLASH TUNE");
        Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");

        state = ERROR;
    }
    else{
        Serial.println("SD INITIALIZATION SUCCESSFUL");
        File ecu_tune;
        ecu_tune = SD.open("GR24_FLASH_TUNE.ecu");
        if(ecu_tune){
            Serial.print("Reading ECU FLASH....");
            String tune;
            while(ecu_tune.available()){
                Serial.print(".");
                tune += (char)ecu_tune.read();
            }
            ecu_tune.close();
            Serial.println("");

            Serial.println("ECU FLASH COMPLETE. GR24 TUNE DOWNLOADED.");

        }
        else {
            Serial.println("CRITICAL FAULT: ERROR OPENING GR24 ECU TUNE");
            Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");
            state = ERROR;
        }
    }
    




   
}

