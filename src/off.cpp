#include <Arduino.h>
#include <imxrt.h>
#include "machine.h"

State off(iCANflex& Car, switchboard& switches) {
    /*
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    bool reject_on = false;
    if (switches.drive_engage) reject_on = true;
    if(switches.on && !switches.drive_engage && !reject_on) {
        return ON;
    }
    if (!switches.on && !switches.drive_engage) reject_on = false;
    return OFF;
    */
    return OFF;
}   
