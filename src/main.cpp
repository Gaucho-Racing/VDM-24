#include "icanflex.h"
#include "imxrt.h"
#include "Arduino.h"
#include "Nodes.h"
#include <unordered_set>
#include <cstddef>

#include "machine.hpp"
#include "tune.hpp"
#include "debug.hpp"
#include "systems_check.hpp"
#include "string"


#define PRINT_DBG 0x00;

iCANflex* Car;
// Debugger* dbg;
SystemsCheck* sysCheck;
Tune* tune;

std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_faults;
std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_warnings;
std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_limits;
 
bool (*errorCheck)(const iCANflex& Car, Tune&); 
bool BSE_APPS_violation = false;



State state;
Mode mode;



FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

unsigned long ACU_Ping = 0;
unsigned long Pedals_Ping = 0;
unsigned long DashPanel_Ping = 0;
unsigned long SteeringWheel_Ping = 0;

unsigned long sendTime = 0;
unsigned long lastPrechargeTime = 0;

unsigned long ping() {
    unsigned long newTime = (long)msg.buf[3] + ((long)msg.buf[2] << 8) + ((long)msg.buf[1] << 16) + ((long)msg.buf[0] << 24);
    unsigned long newTime2 = (long)msg.buf[7] + ((long)msg.buf[6] << 8) + ((long)msg.buf[5] << 16) + ((long)msg.buf[4] << 24);
    unsigned long roundTripDelay = ((millis() - newTime) * 1000) + ((micros() - newTime2) % 1000);
    return roundTripDelay;
}


void tryPingReqests(std::vector<uint32_t> request_ids, iCANflex& Car){
    if(millis()-sendTime > 250){
        for(uint32_t request_id : request_ids){
            
            unsigned long mills=millis();
            unsigned long micro=micros();
            byte data[8] = {0x00};
            for(int i=0; i<4; i++){
                data[3-i]=(byte)(mills >> (i*8));
                data[7-i]=(byte)(micro >> (i*8));
            }

            CAN_message_t message;
            message.flags.extended = true;  
            message.id = request_id;
            message.len = 8;
            memcpy(message.buf, data, 8);

            can1.write(message);

            sendTime=millis();
        }
    }   
}
void sendDashboardPopup(int warning_id){
        //TODO:

    }


    void sendVDMInfo(State state, Mode mode, Tune& tune){
       //TODO:
    }

    void handleDriverInputs(Tune& tune){
        if(msg.id == 0x11002){
            tune.settings.power_level = msg.buf[0];
            tune.settings.throttle_map = msg.buf[1];
            tune.settings.regen_level = msg.buf[2];
        }
    }


    void handleDashPanelInputs(State& state){
        if(msg.id == 0x13000 ){
            if(msg.buf[0]){ // TS_ACTIVE
                if(state == GLV_ON){
                    if(millis() - lastPrechargeTime > 5000){
                        state = TS_PRECHARGE;
                        CAN_message_t message;
                        message.flags.extended = true;
                        message.id = 0x66;
                        message.len = 8;
                        message.buf[0] = 1;
                        can1.write(message);
                        lastPrechargeTime = millis();
                    }
                }
            }
            else if(msg.buf[1]){ // TS_OFF
                // shut off car entireley
                CAN_message_t message;
                message.flags.extended = true;
                message.id = 0x66;
                message.len = 8;
                message.buf[0] = 0;
                can1.write(message);
                state = GLV_ON;
            }
            else if(msg.buf[2]) {// RTD_ON
                if(state == PRECHARGE_COMPLETE){
                    state = DRIVE_STANDBY;
                    // play rtd sound
                }
            }
            else if(msg.buf[3]){ // RTD_OFF
                if(state == DRIVE_STANDBY) {
                    state = TS_PRECHARGE;
                }
            }
        }
        
    }

    void handlePings(){
        //PING RESPONSES

        if (msg.id == ACU_Ping_Response) {
            ACU_Ping = ping();
            msg.buf[0] = 1;
            // add the ACU_Ping to the next 4 bytes
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(ACU_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
        if (msg.id == 0xC9) {
            Pedals_Ping = ping();
            msg.buf[0] = 2;
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(Pedals_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
        if (msg.id == 0x10FFF) {
            SteeringWheel_Ping = ping();    
            msg.buf[0] = 3;
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(SteeringWheel_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
        if (msg.id == 0x12FFF) {
            DashPanel_Ping = ping();
            msg.buf[0] = 4;
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(DashPanel_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
    }


// namespace std {
//     template <>
//     struct hash<bool (*)(const iCANflex&)> {
//         size_t operator()(bool (*f)(const iCANflex&)) const noexcept{
//             return reinterpret_cast<size_t>(f);
//         }
//     };
// };


State sendToError(bool (*erFunc)(const iCANflex& Car, Tune& tune)) {
   errorCheck = erFunc; 
   return ERROR;
}
std::unordered_map<State, std::string> state_to_string = {
                {ECU_FLASH, "ECU_FLASH"},
                {GLV_ON, "GLV_ON"},
                {TS_PRECHARGE, "TS_PRECHARGE"},
                {PRECHARGING, "PRECHARGING"},
                {PRECHARGE_COMPLETE, "PRECHARGE_COMPLETE"},
                {DRIVE_STANDBY, "DRIVE_STANDBY"},
                {DRIVE_ACTIVE, "DRIVE_ACTIVE"},
                {DRIVE_REGEN, "DRIVE_REGEN"},
                {ERROR, "ERROR"}
            };
std::unordered_map<Mode, std::string>mode_to_string = {
                {TESTING, "TESTING"},
                {LAUNCH, "LAUNCH"},
                {ENDURANCE, "ENDURANCE"},
                {AUTOX, "AUTOX"},
                {SKIDPAD, "SKIDPAD"},
                {ACC, "ACC"},
                {PIT, "PIT"}
            };  

void loop(){

    // #if defined(PRINT_DBG) 
    //     dbg->print_status(state, mode);
    //     // dbg->print_system_health(active_faults, active_warnings, active_limits);
    //     dbg->print_pings(comms);
    // #endif


    if(can1.read(msg)) {
        handleDashPanelInputs(state);    
        handleDriverInputs(*tune);
        handlePings();
    }
    
    // Get ping values for all systems
    tryPingReqests({0x10FFE, 0x12FFE, 0xCA, 0x95}, *Car);

    // reads bspd, ams, and imd pins as analog   TODO: Uncomment for actual test bench
    // sysCheck->hardware_system_critical(*Car, *active_faults, tune);
    // sysCheck->system_faults(*Car, *active_faults, tune);
    // sysCheck->system_limits(*Car, *active_limits, tune);
    // sysCheck->system_warnings(*Car, *active_warnings, tune);
    #if defined(PRINT_DBG)
        if(millis() % 100 == 0){
        Serial.println("==================================");
        Serial.print("CLOCK TIME: ");
        Serial.println(millis());
        Serial.print("STATE: ");
        State currentState = state;
        Serial.print(state_to_string.find(currentState)->second.c_str());
        Serial.print(" | ");
        Mode currentMode = mode;
        Serial.print("MODE: ");
        Serial.println(mode_to_string.find(currentMode)->second.c_str());
        Serial.println("==================================");
        Serial.println("SYSTEM HEALTH: ");
        Serial.print("Critical Faults: ");
        Serial.println(active_faults->size());
        Serial.print("Limits: ");
        Serial.println(active_limits->size());
        Serial.print("Warnings: ");
        Serial.println(active_warnings->size());
        Serial.println("==================================");
        Serial.println("PING TIME: ");
        Serial.print("ACU Ping: ");
        Serial.println(ACU_Ping);
        Serial.print("Pedals Ping: ");
        Serial.println(Pedals_Ping);
        Serial.print("DashPanel Ping: ");
        Serial.println(DashPanel_Ping);
        Serial.print("SteeringWheel Ping: ");
        Serial.println(SteeringWheel_Ping);
        Serial.print("==================================");
    }
    #endif
    
    
    CAN_message_t message;
    message.flags.extended = true;
    message.id = 0x13001;
    message.len = 8;
    message.buf[0] = 200;
    message.buf[1] = 200;
    message.buf[2] = 200;
    if(millis() % 100 == 0) can1.write(message);


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
            // state = drive_active(*Car, BSE_APPS_violation, mode, *tune, *comms);
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

    // dbg = new Debugger(10);
    sysCheck = new SystemsCheck();  
    tune = new Tune();
    can1.begin();
    can1.setBaudRate(1000000);
    msg.flags.extended = true;

    Serial.begin(9600);
    Serial.println("Waiting for Serial Port to connect");
    while(!Serial) Serial.println("Waiting for Serial Port to connect");
    Serial.println("Connected to Serial Port 9600");

    pinMode(SOFTWARE_OK_CONTROL_PIN, OUTPUT);
    pinMode(AMS_OK_PIN, INPUT);
    pinMode(BSPD_OK_PIN, INPUT);
    pinMode(IMD_OK_PIN, INPUT);
    pinMode(BRAKE_LIGHT_PIN, INPUT);    


    active_faults = new std::unordered_set<bool (*)(const iCANflex&, Tune&)>(); 
    active_warnings = new std::unordered_set<bool (*)(const iCANflex&, Tune&)>();
    active_limits = new std::unordered_set<bool (*)(const iCANflex&, Tune&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();


    // set state  
    // state = ECU_FLASH; TODO: Remember to uncomment 
    state = GLV_ON;
    mode = ENDURANCE; // TODO: Energy management algorithm for endurance

}
