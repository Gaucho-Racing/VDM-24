#ifndef MAIN
#define MAIN

#include <vector>
#include <Arduino.h>
#include <string>
#include <vector>
#include <imxrt.h>
#include <unordered_map>
#include <unordered_set>
#include "iCANflex.h"
#include "SD.h"
#include "systems_check.h" 

using namespace std;

// le car
static iCANflex* Car;

// PIN DEFINITIONS
const uint8_t SOFTWARE_OK_CONTROL_PIN = 41;
const uint8_t BRAKE_LIGHT_PIN = 4;
const uint8_t BSPD_OK_PIN = 19;
const uint8_t IMD_OK_PIN = 20;
const uint8_t AMS_OK_PIN = 21;


struct TorqueProfile{
    float MAX_CURRENT;
    float K;
    float P;
    float B;
    TorqueProfile(float k, float p, float b): K(k), P(p), B(b){}
    TorqueProfile(){}
};


// ECU TUNE READS
// Read in Torque profiles from the SD card
static vector<TorqueProfile> TORQUE_PROFILES(4);
static const float REV_LIMIT = 5500.0;

// STEERING WHEEL SETTINGS
static uint8_t THROTTLE_MAPPING; // 0-3
static uint8_t REGEN_LEVEL; // 0-3
static uint8_t PWR_LEVEL; // 0 - 3
static uint8_t TC_LEVEL; // 0 - 3

static byte SYS_CHECK_CAN_FRAME[5];   
/*
5 bytes of 8 bits:
[can warn][can failure][AMS][IMD][BSPD][SDC][][]
[warn motor][limit motor][crit motor][warn batt][limit batt][crit batt][rev limit][] temps motor and battery, Revs
[warn water][limit water][crit water][warn mcu][limit mcu][crit mcu][TCM Status][] // water temp DTI temp, TCM
[][][][][][][][] 
[][][][][][][][] 
*/
static void SEND_SYS_CHECK_FRAMES(){ // TODO:
    Serial.println("SENDING SYS CHECK FRAMES");
    for(int i = 0; i < 5; i++){
        Serial.println(SYS_CHECK_CAN_FRAME[i]);
    }
}

// all active detected errors
// TODO: maybe make this a heap to prioritize errors
static unordered_set<bool (*)(const iCANflex&)> active_faults;
static unordered_set<bool (*)(const iCANflex&)> active_warnings;
static unordered_set<bool (*)(const iCANflex&)> active_limits;

enum State {ECU_FLASH, GLV_ON, TS_PRECHARGE, PRECHARGING, PRECHARGE_COMPLETE, DRIVE_NULL, DRIVE_TORQUE, DRIVE_REGEN, ERROR};
enum Mode {TESTING, LAUNCH, ENDURANCE, AUTOX, SKIDPAD, ACC, PIT};

#endif