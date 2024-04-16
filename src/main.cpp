#include "icanflex.h"
#include "imxrt.h"
#include "Arduino.h"
#include "Nodes.h"
#include "machine.hpp"
#include "comms.hpp"
#include "systems_check.hpp"    
#include "tune.hpp"
#include "debug.hpp"
#include <unordered_set>
#include <cstddef>




// #define PRINT_DBG 0x00;

iCANflex* Car;
Debugger* dbg;
CANComms* comms;
SystemsCheck* sysCheck;
Tune* tune;



// namespace std {
//     template <>
//     struct hash<bool (*)(const iCANflex&)> {
//         size_t operator()(bool (*f)(const iCANflex&)) const noexcept{
//             return reinterpret_cast<size_t>(f);
//         }
//     };
// };

std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_faults;
std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_warnings;
std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_limits;
 





bool (*errorCheck)(const iCANflex& Car, Tune&); 
bool BSE_APPS_violation = false;



State state;
Mode mode;


State sendToError(bool (*erFunc)(const iCANflex& Car, Tune& tune)) {
   errorCheck = erFunc; 
   return ERROR;
}

void loop(){

    #if defined(PRINT_DBG) 
        dbg->print_status(state, mode);
        dbg->print_system_health();
        dbg->print_pings(comms);
    #endif


    if(comms->can1.read(comms->msg)) {
        comms->handleDashPanelInputs(state);    
        comms->handleDriverInputs(*tune);
        comms->handlePings();
    }
    
    // Get ping values for all systems
    comms->tryPingReqests({0x10FFE, 0x12FFE, 0xCA, 0x95}, *Car);

    // reads bspd, ams, and imd pins as analog   TODO: Uncomment for actual test bench
    sysCheck->hardware_system_critical(*Car, *active_faults, tune);
    sysCheck->system_faults(*Car, *active_faults, tune);
    sysCheck->system_limits(*Car, *active_limits, tune);
    sysCheck->system_warnings(*Car, *active_warnings, tune);

    

    state = active_faults->size() ?  sendToError(*active_faults->begin()) : state;

    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);
   
    tune->settings.power_level = active_limits->size() ? LIMIT : tune->settings.power_level; // limit power in overheat conditions

    // error severity: warning -> limit -> critical

    // TODO: read in settings from Steering Wheel CAN
   // state machine operation
    switch (state) {
        // ERROR
        case ERROR:
            state = error(*Car, *tune, errorCheck, *active_faults);
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
        case DRIVE_STANDBY:
            state = drive_standby(*Car, BSE_APPS_violation, mode); 
            break;
        case DRIVE_ACTIVE:
            state = drive_active(*Car, BSE_APPS_violation, mode, *tune);
            break;
        case DRIVE_REGEN:
            state = drive_regen(*Car, BSE_APPS_violation, mode, *tune);
            break;
    }
}




//GLV STARTUP
void setup() {
    Car = new iCANflex();
    Car->begin();

    dbg = new Debugger(500);
    comms = new CANComms(1000000);
    sysCheck = new SystemsCheck();  
    tune = new Tune();


    Serial.begin(9600);
    Serial.println("Waiting for Serial Port to connect");
    while(!Serial) Serial.println("Waiting for Serial Port to connect");
    Serial.println("Connected to Serial Port 9600");

    pinMode(SOFTWARE_OK_CONTROL_PIN, OUTPUT);
    pinMode(AMS_OK_PIN, INPUT);
    pinMode(BSPD_OK_PIN, INPUT);
    pinMode(IMD_OK_PIN, INPUT);
    pinMode(BRAKE_LIGHT_PIN, INPUT);    


    active_faults = new unordered_set<bool (*)(const iCANflex&, Tune&)>(); 
    active_warnings = new unordered_set<bool (*)(const iCANflex&, Tune&)>();
    active_limits = new unordered_set<bool (*)(const iCANflex&, Tune&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();


    // set state  
    // state = ECU_FLASH; TODO: Remember to uncomment 
    state = GLV_ON;
    mode = ENDURANCE; // TODO: Energy management algorithm for endurance

}
