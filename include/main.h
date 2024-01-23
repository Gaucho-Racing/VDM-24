#ifndef MAIN
#define MAIN

#include <vector>
#include <Arduino.h>
#include <string>
#include <vector>
#include <imxrt.h>
#include <unordered_map>
#include "iCANflex.h"
#include "SD.h"
#include "systems_check.h" 

using namespace std;

// PIN DEFINITIONS
const int SOFTWARE_OK_CONTROL_PIN = 41;
const int BRAKE_LIGHT_PIN = 4;
const int BSPD_OK_PIN = 19;
const int IMD_OK_PIN = 20;
const int AMS_OK_PIN = 21;


enum State {GLV_ON, TS_PRECHARGE, RTD_0TQ, DRIVE_TORQUE, ERROR};

static unordered_map<State, string> stateToString = {
    {GLV_ON, "ON"},
    {TS_PRECHARGE, "TS_PRECHARGE"},
    {RTD_0TQ, "RTD_0TQ"},
    {DRIVE_TORQUE, "DRIVE_TORQUE"},
    {ERROR, "ERROR"},
};
static iCANflex* Car;

// this is temporary until we get CAN
static unordered_map<string, int> switches = {
    {"TS_ACTIVE", 0},
    {"RTD_SWITCH", 0}
};
// ECU TUNE READS
static float MAX_MOTOR_CURRENT;
static float TORQUE_PROFILE_K;
static float TORQUE_PROFILE_P;
static float TORQUE_PROFILE_B;

static float REV_LIMIT = 5500.0;



#endif