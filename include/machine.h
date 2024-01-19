#ifndef MACHINE
#define MACHINE

#include "main.h"

State off(iCANflex& Car, const vector<int>& switches);
State on(iCANflex& Car, const vector<int>& switches);
State drive_ready(iCANflex& Car, const vector<int>& switches, bool& BSE_APPS_violation);
State drive(iCANflex& Car, const vector<int>& switches, bool& BSE_APPS_violation);
State error(iCANflex& Car, const vector<int>& switches, State prevState, volatile bool (*errorCheck)(iCANflex& c));
State launch(iCANflex& Car, const vector<int>& switches, bool& BSE_APPS_violation);
State testing(iCANflex& Car);

#endif