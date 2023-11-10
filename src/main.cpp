#include <Arduino.h>
#include <imxrt.h>
#include<main.h>

volatile State state;
volatile State prevState;

void setup(){
    
}

void loop(){
    switch(state){
        case OFF:
            //state = off();
            break;

        case DRIVE:
            //state = drive();
            break;
    }
}
