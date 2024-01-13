#ifndef MAIN
#define MAIN

#include <vector>
#include <Arduino.h>
#include <string>
#include <vector>
#include <imxrt.h>
#include "iCANflex.h"
#include <unordered_map>
#include "systems_check.h" 


using namespace std;

// OFF
// ON // TURN CAR ON, (START PRECHARGE) RUN ERROR CHECKS
// DRIVE_READY // CAR IS READY TO DRIVE
// DRIVE // DRIVE
// ERROR // ERROR
// TESTING // TESTING
 
enum State {OFF, ON, DRIVE_READY, DRIVE, ERROR, TESTING, LAUNCH};
static unordered_map<State, string> stateToString = {
    {OFF, "OFF"},
    {ON, "ON"},
    {DRIVE_READY, "DRIVE_READY"},
    {DRIVE, "DRIVE"},
    {ERROR, "ERROR"},
    {TESTING, "TESTING"}
};
static iCANflex Car;
static vector<int> switches(10);






#endif