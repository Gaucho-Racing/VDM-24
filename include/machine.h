#ifndef STUBS
#define STUBS

#include "main.h"

State off(iCANflex& Car, switchboard& switches);
State on(iCANflex& Car, switchboard& switches);
State drive_ready(iCANflex& Car, switchboard& switches, bool& BSE_APPS_violation);
State drive(iCANflex& Car, switchboard& switches, bool& BSE_APPS_violation);
State error(iCANflex& Car, switchboard& switches, State prevState, volatile bool (*errorCheck)(iCANflex& c, switchboard& s));
State testing(iCANflex& Car, switchboard& switches);

#endif