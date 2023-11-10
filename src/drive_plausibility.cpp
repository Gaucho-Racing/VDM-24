#include <Arduino.h>
#include <imxrt.h>
#include <main.h>

float throttle = 0.03;
float ADPS_implausibility = 0.1;

int drive_plausibility(){

    // cut power
    //cut_power();

    if(throttle < 0.05 && ADPS_implausibility < 0.1)
        return DRIVE_READY;

    return 0;
}