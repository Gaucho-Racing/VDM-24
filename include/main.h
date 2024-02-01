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
const int SOFTWARE_OK_CONTROL_PIN = 41;
const int BRAKE_LIGHT_PIN = 4;
const int BSPD_OK_PIN = 19;
const int IMD_OK_PIN = 20;
const int AMS_OK_PIN = 21;


struct TorqueProfile{
    float MAX_CURRENT;
    float K;
    float P;
    float B;
    TorqueProfile(float k, float p, float b): K(k), P(p), B(b){}
    TorqueProfile(){}
};


// ECU TUNE READS
static vector<TorqueProfile> TORQUE_PROFILES(4);
static float REV_LIMIT = 5500.0;

// STEERING WHEEL SETTINGS
static int THROTTLE_MAPPING;
static int REGEN_LEVEL;
static int TRACTION_MODE;


// all active detected errors
static unordered_set<bool (*)(const iCANflex&)> active_faults;


enum State {ECU_FLASH, GLV_ON, TS_PRECHARGE, PRECHARGING, PRECHARGE_COMPLETE, RTD_0TQ, DRIVE_TORQUE, REGEN_TORQUE, ERROR, ERROR_RESOLVED};

static unordered_map<State, string> stateToString = {
    {ECU_FLASH, "ECU_FLASH"},
    {GLV_ON, "ON"},
    {TS_PRECHARGE, "TS_PRECHARGE"},
    {RTD_0TQ, "RTD_0TQ"},
    {DRIVE_TORQUE, "DRIVE_TORQUE"},
    {REGEN_TORQUE, "REGEN_TORQUE"},
    {ERROR, "ERROR"},
    {ERROR_RESOLVED, "ERROR_RESOLVED"}
};


#endif