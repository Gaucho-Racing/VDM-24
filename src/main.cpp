#include <Arduino.h>
#include <imxrt.h>
#include "main.h"
#include "iCANflex.h"
#include "machine.h"
#include "SD.h"


// volatile stuff and state transition
volatile State state;
volatile State prevState;
volatile bool (*errorCheck)(iCANflex& Car); 
bool BSE_APPS_violation = false;

// ECU TUNE Reads
float PWR_REGEN_MAX;
float BRAKE_BALANCE;
float MAX_MOTOR_CURRENT;
vector<unordered_map<int, float>> THROTTLE_MAPPING(100); // index is throttle position, value[rpm] is the % of max current
unordered_map<float, float> REGEN_TORQUE_MAPPING;
vector<float> LAUNCH_CONTROL_FUNCTION;
float LAUNCH_CONTROL_INTERVAL;

const int REV_LIMITER = 5500;

State sendToError(volatile State currentState, volatile bool (*erFunc)(iCANflex& Car)) {
   errorCheck = erFunc; 
   prevState = currentState; 
   return ERROR;
}


void loop(){
    switch (state) {
        case OFF:
            state = off(Car, switches);
            break;
        case ON:
            if ((state = on(Car, switches)) == ERROR) sendToError(ON, &ECU_Startup_Rejection);
            break;
        case DRIVE_READY:
            state = drive_ready(Car, switches, BSE_APPS_violation); 
            break;
        case DRIVE:
            state = drive(Car, switches, BSE_APPS_violation);
            break;
        case LAUNCH:
            state = launch(Car, switches, BSE_APPS_violation);
            break;
        case ERROR:
            state = error(Car, switches, prevState, errorCheck);
            break;
        // case TESTING;
    }

    //test
}

void setup() {
    Serial.begin(9600);
    Serial.println("Waiting for Serial Port to connect");
    while(!Serial){
        Serial.println("Waiting for Serial Port to connect");
    }
    Serial.println("Connected to Serial Port 9600");

    Car.begin();


     // set state  
    state = OFF; 

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
            string tune;
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

