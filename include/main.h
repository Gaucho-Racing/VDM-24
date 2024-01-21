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

enum State {OFF, ON, DRIVE_READY, DRIVE, ERROR};
const int SHUTDOWN_PIN = 41;
static unordered_map<State, string> stateToString = {
    {OFF, "OFF"},
    {ON, "ON"},
    {DRIVE, "DRIVE"},
    {DRIVE_READY, "DRIVE_READY"},
    {ERROR, "ERROR"},
};
static iCANflex Car;
static vector<int> switches(10);
// ECU TUNE READS
static float MAX_MOTOR_CURRENT;
static float TORQUE_PROFILE_K;
static float TORQUE_PROFILE_P;
static float TORQUE_PROFILE_B;

static float REV_LIMIT = 5500.0;





#endif