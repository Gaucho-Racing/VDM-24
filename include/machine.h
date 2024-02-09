#ifndef MACHINE
#define MACHINE

#include "main.h"
#include "systems_check.h"

State ecu_flash(iCANflex& Car);
State glv_on(iCANflex& Car);
State ts_precharge(iCANflex& Car);
State precharging(iCANflex& Car);
State precharge_complete(iCANflex& Car);
State drive_null(iCANflex& Car, bool& BSE_APPS_violation);
State drive_torque(iCANflex& Car, bool& BSE_APPS_violation);
State drive_regen(iCANflex& Car);
State error(iCANflex& Car, bool (*errorCheck)(const iCANflex& c));

float requested_regenerative_torque(iCANflex& Car, float brake, int rpm);
float requested_torque(iCANflex& Car, float throttle, int rpm);

#endif                              


