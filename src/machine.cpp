#include "machine.h"

State off(iCANflex& Car, switchboard& switches) {
    Car.DTI.setDriveEnable(0);
    Car.DTI.setRCurrent(0);
    bool reject_on = false;
    if (switches.drive_engage) reject_on = true;
    if(switches.on && !switches.drive_engage && !reject_on) {
        return ON;
    }
    if (!switches.on && !switches.drive_engage) reject_on = false;
    return OFF;

}   

float motorOut(float throttle, iCANflex& car, switchboard& switches) {
    const int HIGH_PWR_R_CURRENT = 50;
    const int LOW_PWR_R_CURRENT = 25; 
    // change these to a continuous power curve later^^^
    return switches.pwr_lvl ? HIGH_PWR_R_CURRENT *throttle: LOW_PWR_R_CURRENT*throttle; 
}

State drive(iCANflex& Car, switchboard& switches) {
    if(Car.PEDALS.getAPPS1() < 0.05 || Car.PEDALS.getAPPS2() < 0.05) return DRIVE;

    float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2;

    Car.DTI.setDriveEnable(1);
    Car.DTI.setRCurrent(motorOut(throttle, Car, switches));
    
    return DRIVE;
}
