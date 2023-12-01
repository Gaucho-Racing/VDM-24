#include <Arduino.h>
#include <imxrt.h>
#include <main.h>
#include <fakenodes.h>

int drive(){

    // if on and brake and drive
    //Fix units for brake pressure
    //on_switch and ready_to_drive are still fake nodes
    if(FAKENODES::on_switch() && 
       Car.Pedals.getBrakePressureF() > 0.0 &&
       Car.Pedals.getBrakePressureR() > 0.0 &&
       FAKENODES::ready_to_drive())
    {
        return DRIVE_READY;
    }
    

    // if car is not on
    //on_switch is still a fake node
    if(!FAKENODES::on_switch()){
        return OFF;
    }

    return 0;
}