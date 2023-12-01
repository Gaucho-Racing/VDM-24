#include <Arduino.h>
#include <imxrt.h>
#include <main.h>
#include <fakenodes.h>

int drive(){

    // if on and brake and drive
    if(FAKENODES::on_switch() && FAKENODES::brake_pressed() && FAKENODES::ready_to_drive()){
        return DRIVE_READY;
    }
    
    // if car is not on
    if(!FAKENODES::on_switch()){
        return OFF;
    }

    return 0;
}