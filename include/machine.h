#ifndef MACHINE
#define MACHINE

#include "main.h"

State glv_on(iCANflex& Car);
State ts_precharge(iCANflex& Car);
State rtd_0tq(iCANflex& Car, bool& BSE_APPS_violation);
State drive_torque(iCANflex& Car, bool& BSE_APPS_violation);
State error(iCANflex& Car, volatile bool (*errorCheck)(iCANflex& c));

#endif