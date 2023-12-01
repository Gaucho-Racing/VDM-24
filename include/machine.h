#ifndef STUBS
#define STUBS

#include "main.h"

State off(iCANflex& Car, switchboard& switches);
State on(iCANflex& Car, switchboard& switches);
State drive_ready(iCANflex& Car, switchboard& switches);
State drive(iCANflex& Car, switchboard& switches);
State error(iCANflex& Car, switchboard& switches, State prevState, volatile bool (*errorCheck)(void));
State testing(iCANflex& Car, switchboard& switches);

#endif