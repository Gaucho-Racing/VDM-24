#ifndef MAIN
#define MAIN

#include <vector>
#include <Arduino.h>
#include <string>
#include <vector>
#include <imxrt.h>
#include <unordered_map>
#include <queue>
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


// TORQUE MAP PROFILES 
const uint8_t LINEAR = 0;
const uint8_t TQ_MAP_1 = 1;
const uint8_t TQ_MAP_2 = 2;
const uint8_t TQ_MAP_3 = 3;

// POWER LEVELS
const uint8_t LIMIT = 0;
const uint8_t LOW_PWR = 1;
const uint8_t MEDIUM_PWR = 2;
const uint8_t HIGH_PWR = 3;

struct TorqueProfile{
    float K;
    float P;
    float B;
    TorqueProfile(float k, float p, float b): K(k), P(p), B(b){}
    TorqueProfile(){}
};


// ECU TUNE READS TODO: Initialize in SD card read
static vector<TorqueProfile> TORQUE_PROFILES(4); // see above
static vector<float> POWER_LEVELS(4); // max current value in Amperes
static vector<float> REGEN_LEVELS(4); // percentile value 0 to 100

static const float REV_LIMIT = 5500.0;

// STEERING WHEEL SETTINGS
static uint8_t throttle_map; // 0-3
static uint8_t regen_level; // 0-3
static uint8_t power_level; // 0 - 3

/*
5 bytes of 8 bits:
[can warn][can failure][AMS][IMD][BSPD][SDC][][]
[warn motor][limit motor][crit motor][warn batt][limit batt][crit batt][rev limit][] temps motor and battery, Revs
[warn water][limit water][crit water][warn mcu][limit mcu][crit mcu][TCM Status][] // water temp DTI temp, TCM
[][][][][][][][] 
[][][][][][][][] 
*/
static byte SYS_CHECK_CAN_FRAME[5];   
static void SEND_SYS_CHECK_FRAMES(iCANflex& Car);

// all active detected errors
// // function pointer hash function
namespace std {
    template <>
    struct hash<bool (*)(const iCANflex&)> {
        size_t operator()(bool (*f)(const iCANflex&)) const {
            return reinterpret_cast<size_t>(f);
        }
    };
}

static unordered_set<bool (*)(const iCANflex&)> *active_faults;
static unordered_set<bool (*)(const iCANflex&)> *active_warnings;
static unordered_set<bool (*)(const iCANflex&)> *active_limits;




enum State {ECU_FLASH, GLV_ON, TS_PRECHARGE, PRECHARGING, PRECHARGE_COMPLETE, DRIVE_NULL, DRIVE_TORQUE, DRIVE_REGEN, ERROR};
enum Mode {TESTING, LAUNCH, ENDURANCE, AUTOX, SKIDPAD, ACC, PIT};

#endif