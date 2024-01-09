#ifndef MACHINE
#define MACHINE

#include "main.h"

State off(iCANflex& Car, vector<int>& switches);
State on(iCANflex& Car, vector<int>& switches);
State drive_ready(iCANflex& Car, vector<int>& switches, bool& BSE_APPS_violation);
State drive(iCANflex& Car, vector<int>& switches, bool& BSE_APPS_violation);
State error(iCANflex& Car, vector<int>& switches, State prevState, volatile bool (*errorCheck)(iCANflex& c));
State testing(iCANflex& Car);

#endif