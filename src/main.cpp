#include "machine.h"
#include "main.h"
#include "comms.hpp"
#include "debug.hpp"


bool (*errorCheck)(const iCANflex& Car); 
bool BSE_APPS_violation = false;

// #define PRINT_LOGS 0x00;


State sendToError(bool (*erFunc)(const iCANflex& Car)) {
   errorCheck = erFunc; 
   return ERROR;
}


void loop(){

    #if defined(PRINT_LOGS) 
        print_status(500);
        print_system_health(500);
        print_pings(500);
    #endif


    if(can1.read(msg)) HandleIncomingMessages(); //
    
    // reads bspd, ams, and imd pins as analog   TODO: Uncomment for actual test bench
    SystemsCheck::hardware_system_critical(*Car, *active_faults);
    SystemsCheck::system_faults(*Car, *active_faults);
    SystemsCheck::system_limits(*Car, *active_limits);
    SystemsCheck::system_warnings(*Car, *active_warnings);

    // Get ping values for all systems
    tryPingReqests({0x10FFE, 0x12FFE, 0xCA, 0x95}, *Car);

    state = active_faults->size() ?  sendToError(*active_faults->begin()) : state;

    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);
   

    // error severity: warning -> limit -> critical

    // TODO: read in settings from Steering Wheel CAN

    switch(power_level) {
        case LIMIT:
            // do something
            break;
        case LOW_PWR:
            // do something
            break;
        case MEDIUM_PWR:
            // do something
            break;
        case HIGH_PWR:
            // do something
            break;
        default:
            // ERROR
            break;
    }
    switch(throttle_map) {
        case LINEAR:
            // do something
            break;
        case TQ_MAP_1:
            // do something
            break;
        case TQ_MAP_2:
            // do something
            break;
        case TQ_MAP_3:
            // do something
            break;
        default:
            // ERROR
            break;
    }
    switch(regen_level){
        case REGEN_0:
            // do something
            break;
        case REGEN_1:
            // do something
            break;
        case REGEN_2:
            // do something
            break;
        case REGEN_3:
            // do something
            break;
        default:
            // ERROR
            break;
    }

    


    power_level = active_limits->size() ? LIMIT : power_level; // limit power in overheat conditions

    
    mode = ENDURANCE; // TODO: Energy management algorithm for endurance

   

    // STATE MACHINE OPERATION

    switch (state) {
        // ERROR
        case ERROR:
            state = error(*Car, errorCheck);
            break;

        // STARTUP 
        case ECU_FLASH:
            state = ecu_flash(*Car);
            break;
        case GLV_ON:
            state = glv_on(*Car);
            break;
     
        // PRECHARGE
        case TS_PRECHARGE:
            state = ts_precharge(*Car);
            break;
        case PRECHARGING:
            state = precharging(*Car);
            break;
        case PRECHARGE_COMPLETE:
            state = precharge_complete(*Car);
            break;
        
        // DRIVE
        case DRIVE_NULL:
            state = drive_null(*Car, BSE_APPS_violation, mode); 
            break;
        case DRIVE_TORQUE:
            state = drive_torque(*Car, BSE_APPS_violation, mode);
            break;
        case DRIVE_REGEN:
            state = drive_regen(*Car, BSE_APPS_violation, mode);
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

    pinMode(SOFTWARE_OK_CONTROL_PIN, OUTPUT);
    pinMode(AMS_OK_PIN, INPUT);
    pinMode(BSPD_OK_PIN, INPUT);
    pinMode(IMD_OK_PIN, INPUT);
    pinMode(BRAKE_LIGHT_PIN, INPUT);    

    setupCAN();

    Car->begin();
    active_faults = new unordered_set<bool (*)(const iCANflex&)>(); 
    active_warnings = new unordered_set<bool (*)(const iCANflex&)>();
    active_limits = new unordered_set<bool (*)(const iCANflex&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();
    for(int i = 0; i < 5; i++) SYS_CHECK_CAN_FRAME[i] = 0x0;

    // set state  
    // state = ECU_FLASH; TODO: Remember to uncomment 
    state = GLV_ON;
}
