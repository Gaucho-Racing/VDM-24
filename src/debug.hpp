#ifndef DEBUG_HPP
#define DEBUG_HPP

#include <unordered_map>
#include "main.h"
#include "comms.hpp"

static std::unordered_map<State, string> state_to_string = {
    {ECU_FLASH, "ECU_FLASH"},
    {GLV_ON, "GLV_ON"},
    {TS_PRECHARGE, "TS_PRECHARGE"},
    {PRECHARGING, "PRECHARGING"},
    {PRECHARGE_COMPLETE, "PRECHARGE_COMPLETE"},
    {DRIVE_NULL, "DRIVE_NULL"},
    {DRIVE_TORQUE, "DRIVE_TORQUE"},
    {DRIVE_REGEN, "DRIVE_REGEN"},
    {ERROR, "ERROR"}
};

static std::unordered_map<Mode, string> mode_to_string = {
    {TESTING, "TESTING"},
    {LAUNCH, "LAUNCH"},
    {ENDURANCE, "ENDURANCE"},
    {AUTOX, "AUTOX"},
    {SKIDPAD, "SKIDPAD"},
    {ACC, "ACC"},
    {PIT, "PIT"}
};  

static void print_state(int interval){
    if(millis() % interval == 0){
        Serial.println("==================================");
        Serial.print("TIME: ");
        Serial.println(millis());
        Serial.print("STATE: ");
        State currentState = state;
        Serial.print(state_to_string.find(currentState)->second.c_str());
        Serial.print(" | ");
        Mode currentMode = mode;
        Serial.print("MODE: ");
        Serial.println(mode_to_string.find(currentMode)->second.c_str());
        Serial.println("==================================");
    }
}

static void print_system_health(int interval){
    if(millis() % interval == 0){
        Serial.println("SYSTEM HEALTH: ");
        Serial.println("==================================");
        Serial.print("Critical Faults: ");
        Serial.println(active_faults->size());
        Serial.print("Limits: ");
        Serial.println(active_warnings->size());
        Serial.print("Warnings: ");
        Serial.println(active_warnings->size());
        Serial.print("==================================");
    }
}

static void print_pings(int interval){
    if(millis() % interval == 0){
        Serial.println("PING TIME: ");
        Serial.println("==================================");
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
}
#endif