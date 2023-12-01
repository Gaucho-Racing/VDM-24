#ifndef MAIN
#define MAIN

#include <vector>
#include <string>
#include "Nodes.h"

using namespace std;

enum State {OFF, ON, DRIVE, DRIVE_READY, ERROR, TESTING};
static vector<string> statesString = {"OFF", "ON", "DRIVE_READY", "DRIVE", "ERROR", "TESTING"};

static string getTextForEnum( int enumVal )
{
  return statesString[enumVal];
}


#endif