#ifndef MAIN
#define MAIN

#include <vector>
#include <string>
#include "icanflex.h"
#include "Nodes.h"

using namespace std;

enum State {OFF, ON, DRIVE_READY, D_PLAUS, ERROR, TESTING};
static vector<string> statesString = {"OFF", "ON", "DRIVE_READY", "D_PLAUS", "ERROR", "TESTING"};
iCANflex Car = iCANflex();

static string getTextForEnum( int enumVal ) {
  return statesString[enumVal];
}

#endif