#ifndef MAIN
#define MAIN

#include <vector>
#include <Arduino.h>
#include <string>
#include <imxrt.h>
#include "iCANflex.h"
#include <unordered_map>


using namespace std;

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
  int drive_enable;
  int drive_engage;
  int traction_control;
  int fan_override;
  int regen;

  double brake_balance;
  int race_mode;

  // this is for all digital read switchboard pins
  // we will use this later when we order switches and encoders.


};

#endif