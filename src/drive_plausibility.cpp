#include <Arduino.h>
#include <imxrt.h>
#include <main.h>
//#include <fakenodes.h>

int d_plaus(iCANflex &Car){
    /*
    needs more logic.
    change all fakenodes to actual nodes.
    */

    // set current to 0;

    if( FAKENODES::throttle() < 0.05 && FAKENODES::adps_implausibility() < 0.1)
        return DRIVE_READY;

    return 0;
}