// #ifndef DEBUG_HPP
// #define DEBUG_HPP

// #include <unordered_map>
// #include "machine.hpp"
// #include "string"

// class Debugger {
//     private:
//         int interval;
//         std::unordered_map<State, std::string> state_to_string;
//         std::unordered_map<Mode, std::string> mode_to_string;
//         unsigned long lastDebugTime0;
//         unsigned long lastDebugTime1;
//         unsigned long lastDebugTime2;
//         unsigned long lastDebugTime3;
//     public:            
//         Debugger(int interval){
//             this->interval = interval;
//             state_to_string = {
//                 {ECU_FLASH, "ECU_FLASH"},
//                 {GLV_ON, "GLV_ON"},
//                 {TS_PRECHARGE, "TS_PRECHARGE"},
//                 {PRECHARGING, "PRECHARGING"},
//                 {PRECHARGE_COMPLETE, "PRECHARGE_COMPLETE"},
//                 {DRIVE_STANDBY, "DRIVE_STANDBY"},
//                 {DRIVE_ACTIVE, "DRIVE_ACTIVE"},
//                 {DRIVE_REGEN, "DRIVE_REGEN"},
//                 {ERROR, "ERROR"}
//             };
//             mode_to_string = {
//                 {TESTING, "TESTING"},
//                 {LAUNCH, "LAUNCH"},
//                 {ENDURANCE, "ENDURANCE"},
//                 {AUTOX, "AUTOX"},
//                 {SKIDPAD, "SKIDPAD"},
//                 {ACC, "ACC"},
//                 {PIT, "PIT"}
//             };  
//             lastDebugTime0 = 0;
//             lastDebugTime1 = 0;
//             lastDebugTime2 = 0;
//             lastDebugTime3 = 0;
//         }

//         void print_status(State state, Mode mode){
//             if(millis() - lastDebugTime0 > interval){
//                 Serial.println("==================================");
//                 Serial.print("CLOCK TIME: ");
//                 Serial.println(millis());
//                 Serial.print("STATE: ");
//                 State currentState = state;
//                 Serial.print(state_to_string.find(currentState)->second.c_str());
//                 Serial.print(" | ");
//                 Mode currentMode = mode;
//                 Serial.print("MODE: ");
//                 Serial.println(mode_to_string.find(currentMode)->second.c_str());
//                 Serial.println("==================================");
//                 lastDebugTime0 = millis();
//             }
//         }

//         void print_system_health(std::unordered_set<bool (*)(const iCANflex&, Tune&)> *active_faults, std::unordered_set<bool (*)(const iCANflex&, Tune&)> *active_warnings, std::unordered_set<bool (*)(const iCANflex&, Tune&)> *active_limits) {
//             if(millis() - lastDebugTime1 > interval){
//                 Serial.println("SYSTEM HEALTH: ");
//                 Serial.print("Critical Faults: ");
//                 Serial.println(active_faults->size());
//                 Serial.print("Limits: ");
//                 Serial.println(active_limits->size());
//                 Serial.print("Warnings: ");
//                 Serial.println(active_warnings->size());
//                 Serial.println("==================================");
//                 lastDebugTime1 = millis();
//             }
//         }

//         void print_pings(CANComms* comms){
//             if(millis() - lastDebugTime2 > interval){
//                 Serial.println("PING TIME: ");
//                 Serial.print("ACU Ping: ");
//                 Serial.println(comms->getACU_Ping());
//                 Serial.print("Pedals Ping: ");
//                 Serial.println(comms->getPedals_Ping());
//                 Serial.print("DashPanel Ping: ");
//                 Serial.println(comms->getDashPanel_Ping());
//                 Serial.print("SteeringWheel Ping: ");
//                 Serial.println(comms->getSteeringWheel_Ping());
//                 Serial.print("==================================");
//                 lastDebugTime2 = millis();
//             }
//         }

// };

// #endif