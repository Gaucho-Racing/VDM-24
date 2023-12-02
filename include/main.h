#ifndef MAIN
#define MAIN

#include <vector>
#include <Arduino.h>
#include <string>
#include <imxrt.h>
#include "iCANflex.h"
#include <unordered_map>
#include "systems_check.h" 


using namespace std;

// OFF
// ON // TURN CAR ON, RUN ERROR CHECKS
// DRIVE_READY // WAIT FOR DRIVER TO PRESS THROTTLE
// DRIVE // DRIVE
// ERROR // ERROR
// TESTING // TESTING
 
enum State {OFF, ON, DRIVE_READY, DRIVE, ERROR, TESTING};
static unordered_map<State, string> stateToString = {
    {OFF, "OFF"},
    {ON, "ON"},
    {DRIVE_READY, "DRIVE_READY"},
    {DRIVE, "DRIVE"},
    {ERROR, "ERROR"},
    {TESTING, "TESTING"}
};
iCANflex Car = iCANflex();
switchboard switches;


struct switchboard{
  int drive_enable; //1
  int drive_engage; //2
  int traction_control;
  int launch_control;
  int fan_override;
  int regen;
  
  double pwr_lvl;
  double brake_balance;
  int race_mode;

  // this is for all digital read switchboard pins
  // we will use this later when we order switches and encoders.


};

#endif