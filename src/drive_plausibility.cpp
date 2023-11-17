#include <Arduino.h>
#include <imxrt.h>
#include <main.h>
#include <fakenodes.h>

// float throttle = 0.03;
// float ADPS_implausibility = 0.1;

int drive_plausibility(){

    // cut_power();

    if( FAKENODES::throttle() < 0.05 && FAKENODES::adps_implausibility() < 0.1)
        return DRIVE_READY;

    return 0;
}