#ifndef MAIN
#define MAIN

#include <vector>
#include <string>

using namespace std;

enum State {OFF, ON, ON_READY, DRIVE, ERROR, TESTING};
static vector<string> statesString = {"OFF", "ON", "ON_READY", "DRIVE", "ERROR", "TESTING"};

static string getTextForEnum( int enumVal )
{
  return statesString[enumVal];
}


#endif