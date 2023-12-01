// off.cpp
#include <Arduino.h>
#include <imxrt.h>
#include <main.h>
#include <fakenodes.h>

States off(){

    
    if(FAKENODES::on() && !FAKENODES::drive()) { //there should also be a check for wrong, but we are ignoring that for now
            return ON;
        }

}