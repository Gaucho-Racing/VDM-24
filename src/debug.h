#ifndef DEBUG_H
#define DEBUG_H
#include "unordered_map"
#include "global.h"
#include "Arduino.h"
#include "vehicle.h"
#include "SystemsCheck.h"
#include "network.h"
#include "VehicleTuneController.h"
#include "unordered_set"
#include "unordered_map"
#include "machine.h"
#include <string>   

unsigned long lastPrintTime = 0;
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
                {STANDARD, "STANDARD"},
                {DYNAMIC_TC, "DYNAMIC_TC"}
}; 
std::unordered_map<int, std::string>node_to_string = {
    {1, "ACU"},
    {2, "Pedals"},
    {3, "Steering Wheel"},
    {4, "Dash Panel"} //TODO: BCM, TCM
};

String vehicleStatus(State state, Mode mode){
    String output = "|                       STATUS:                          |\n";
    output += "| CLOCK: " + String(millis()) + " ms ";
    output += "| STATE: ";
    State currentState = state;
    output += state_to_string.find(currentState)->second.c_str();
    output += " | MODE: ";
    Mode currentMode = mode;
    output += mode_to_string.find(currentMode)->second.c_str();
    output += "     |\n";
    output += "----------------------------------------------------------";
    return output;
}

String vehicleHealth(SystemsCheck* sysCheck, Vehicle* Car){
    String output = "|                      SYSTEM HEALTH:                    |\n";
    output += "| CRITICAL: " + String(Car->active_faults->size()) + " | LIMIT: " + String(Car->active_limits->size()) + " | WARN: " + String(Car->active_warnings->size()) + "          \n| HARDWARE FAULTS: ";
    for(auto e : *Car->active_faults){
        Serial.println("here");
        if(e == sysCheck->AMS_fault) output += "AMS | ";
        if(e == sysCheck->IMD_fault) output += "IMD | ";
        if(e == sysCheck->BSPD_fault) output += "BSPD | ";
        if(e == sysCheck->SDC_opened) output += "SDC ";
    }
    output += "\n ----------------------------------------------------------";
    return output;
}

String vehicleNetwork(){
    String output = "|          NETWORK SPEED: (microseconds)                 |\n";
    if(timeout_nodes.find(ACU_Ping_Response) != timeout_nodes.end()) output += "| ACU: COOKED 💀🔥 \n";
    else output += "| ACU: " + String(ping_response_times[ACU_Ping_Response]) + "\n" ;
    if(timeout_nodes.find(Pedals_Ping_Response) != timeout_nodes.end()) output += "| Pedals: COOKED 💀🔥 \n";
    else output += "| Pedals: " + String(ping_response_times[Pedals_Ping_Response]) + "\n" ;
    if(timeout_nodes.find(Steering_Wheel_Ping_Response) != timeout_nodes.end()) output += "| Steering: COOKED 💀🔥 \n";
    else output += "| Steering: " + String(ping_response_times[Steering_Wheel_Ping_Response]) + "\n" ;
    if(timeout_nodes.find(Dash_Panel_Ping_Response) != timeout_nodes.end()) output += "| DashPanel: COOKED 💀🔥 \n";
    else output += "| DashPanel: " + String(ping_response_times[Dash_Panel_Ping_Response]) + "  \n";
    output += "----------------------------------------------------------";
    return output;
}

String vehicleSettings(SWSettings settings){
    String output = "|                     VEHICLE SETTINGS:                  |\n";
    output += "| POWER LEVEL: ";
    if (settings.power_level == LIMIT) output += "LIMIT     ";
    else if (settings.power_level == LOW_PWR) output += "LOW       ";
    else if (settings.power_level == MID_PWR) output += "MID       ";
    else if (settings.power_level == HIGH_PWR) output += "HIGH      ";
    output += "\n| THROTTLE MAP: ";
    if (settings.throttle_map == LINEAR_TORQUE) output += "LINEAR    ";
    else if (settings.throttle_map == TORQUE_MAP_1) output += "MAP 1     ";
    else if (settings.throttle_map == TORQUE_MAP_2) output += "MAP 2     ";
    else if (settings.throttle_map == TORQUE_MAP_3) output += "MAP 3     ";
    output += "\n| REGEN LEVEL: ";
    if (settings.regen_level == REGEN_OFF) output += "OFF       \n";
    else if (settings.regen_level == REGEN_LOW) output += "LOW       \n";
    else if (settings.regen_level == REGEN_MID) output += "MID       \n";
    else if (settings.regen_level == REGEN_HIGH) output += "HIGH      \n";
    output += "----------------------------------------------------------";
    return output;
}

String vehiclePowerData(VehicleTuneController* tune, Vehicle* Car, SWSettings settings){
    String output = "|                     POWER DATA:                        |\n";
    int raw1 = Car->PEDALS.getAPPS1();
    output += "| APPS1: RAW: " + String(raw1) + ", SCALED: " + String(getThrottle1(raw1, *tune)) + "               \n";
    int raw2 = Car->PEDALS.getAPPS2();
    output += "| APPS2: RAW: " + String(raw2) + ", SCALED: " + String(getThrottle2(raw2, *tune)) + "               \n";
    output += "| BSE: " + String(analogRead(BSE_HIGH)) + "               \n";
    output += "| INVERTER CURRENT LIMIT: " + String(tune->getPowerLevelsData()[settings.power_level] )+ " A            \n";
    output += "| POWER DRAW: " + String(Car->DTI.getACCurrent() * Car->ACU1.getTSVoltage()) + "W                          \n";
    output += "----------------------------------------------------------\n";
    return output;
}

void printDebug(State state, Mode mode, SystemsCheck* sysCheck, SWSettings settings, VehicleTuneController* tune, Vehicle* Car){
    if(millis() - lastPrintTime > 1000/DEBUG_PRINT_FREQUENCY){
        Serial.println("----------------------------------------------------------");
        Serial.println("|                     GR24 EV VEHICLE DEBUG              |");
        Serial.println("----------------------------------------------------------");
        Serial.println(vehicleStatus(state, mode));
        Serial.println(vehicleHealth(sysCheck, Car));
        Serial.println(vehicleNetwork());
        Serial.println(vehicleSettings(settings));
        Serial.println(vehiclePowerData(tune, Car, settings));
        lastPrintTime = millis();
    }
}
#endif