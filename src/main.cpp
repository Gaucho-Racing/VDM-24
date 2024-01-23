#include "machine.h"
#include "main.h"

volatile State state;
volatile State prevState;
volatile bool (*errorCheck)(iCANflex& Car); 
bool BSE_APPS_violation = false;



State sendToError(volatile State currentState, volatile bool (*erFunc)(iCANflex& Car)) {
   errorCheck = erFunc; 
   return ERROR;
}

volatile bool motorTempHighExitCondition(iCANflex& Car) {
    if (Car.DTI.getMotorTemp() < 55) {
        return false;
    }
    return true;
}

volatile bool motorTempHighEntryCondition(iCANflex& Car) {
    if (Car.DTI.getMotorTemp() >= 60) {
        return true;
    }
    return false;
}

void motorTempHigh_ISR() {
    Car.sendDashError(5); // send placeholder error byte code to dash
    state = sendToError(state, motorTempHighExitCondition);
}

const int canFailureThreshold = 100; // msec

volatile bool canReceiveFailure(iCANflex& Car) {
    return 
        Car.DTI.getAge() > canFailureThreshold ||
        Car.ECU.getAge() > canFailureThreshold ||
        Car.WFL.getAge() > canFailureThreshold ||
        Car.WFR.getAge() > canFailureThreshold ||
        Car.WRL.getAge() > canFailureThreshold ||
        Car.WRR.getAge() > canFailureThreshold ||
        Car.GPS1.getAge() > canFailureThreshold ||
        Car.PEDALS.getAge() > canFailureThreshold ||
        Car.ACU1.getAge() > canFailureThreshold ||
        Car.BCM1.getAge() > canFailureThreshold ||
        Car.DASHBOARD.getAge() > canFailureThreshold ||
        Car.ENERGY_METER.getAge() > canFailureThreshold;
}

void canReceiveFailure_ISR() {
    Car.sendDashError(100);
    state = sendToError(state, canReceiveFailure);
}

volatile bool currentLimitSafe(iCANflex& Car) {
    return (Car.DTI.getDCCurrent() > 575); // based on below current limit
}

volatile bool currentLimitExceeded(iCANflex& Car) {
    return (Car.DTI.getDCCurrent() > 600); // taken from last year's implementation
}

void currentLimitExceeded_ISR() {
    Car.sendDashError(106);
    state = sendToError(state, currentLimitSafe);
}

volatile bool shutdown_pinned(iCANflex& Car) {
    return (bool)digitalRead(shutdown_pin);
}

void shutdown_pinned_ISR() {
    Car.sendDashError(150);
    state = sendToError(OFF, shutdown_pinned);
}

void loop(){
    // read bspd, ams, and imd pins as analog
    // .5v is shit -  ADC: 155
    // 3v when almost ok - ADC: 930
    // 2.4v is ok - ADC: 744
    // 1v = 310
    if(analogRead(BSPD_OK_PIN) < 310){

    }
    if(analogRead(AMS_OK_PIN) < 310){
        // send can message for light on dash panel
        state = ERROR;
    }
     else if(analogRead(AMS_OK_PIN) > 730 && analogRead(AMS_OK_PIN) < 760) state = GLV_ON;

    if(analogRead(IMD_OK_PIN) < 310){
        // send can message for light on dash panel
        state = ERROR;
    } 
    else if(analogRead(IMD_OK_PIN) > 730 && analogRead(IMD_OK_PIN) < 760) state = GLV_ON;

    // this coz it exists
    digitalWrite(SOFTWARE_OK_CONTROL_PIN, HIGH);

    // Brake Light Operation
    if(Car.PEDALS.getBrakePressureF() > 0.05 || Car.PEDALS.getBrakePressureR() > 0.05) {
        digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    }
    else {
        digitalWrite(BRAKE_LIGHT_PIN, LOW);
    }

    if(CRITICAL_CAN_FAILURE(Car)) {
        sendToError(&CRITICAL_CAN_FAILURE);
    }

    if(NON_CRITICAL_CAN_FAILURE(Car)){
        // warn on dash
    }


    // switchboard CAN stuff for moving from glv_on to ts_precharge from the ts_active switch
    // also the brake + rtd switch to move from ts_precharge to rtd_0tq 
    // wait for a ping, use a callback function and pointer to a function
    // will require changing nodes.h.


    if(motorTempHighEntryCondition(Car)) {
        NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT2);
    }

    if (canReceiveFailure(Car)) { NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT1); }

    if (currentLimitExceeded(Car)) { 
        NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT3);
    }

    if (shutdown_pinned(Car)) { NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT0); }

    switch (state) {
        case OFF:
            state = off(Car, switches);
            break;
        case ON:
            state = on(Car, switches);
            break;
        case DRIVE_READY:
            state = drive_ready(Car, switches, BSE_APPS_violation); 
            break;
        case DRIVE:
            state = drive(Car, switches, BSE_APPS_violation);
            break;
        case ERROR:
            state = error(Car, errorCheck);
            break;
    }
}

void setup() {
    Car = new iCANflex();
    Serial.begin(9600);
    Serial.println("Waiting for Serial Port to connect");
    while(!Serial) Serial.println("Waiting for Serial Port to connect");
    Serial.println("Connected to Serial Port 9600");

    Car.begin();

    attachInterruptVector(IRQ_GPIO1_INT2, &motorTempHigh_ISR); //placeholder pin number 3
    NVIC_ENABLE_IRQ(IRQ_GPIO1_INT2);
    attachInterruptVector(IRQ_GPIO1_INT1, &canReceiveFailure_ISR);
    NVIC_ENABLE_IRQ(IRQ_GPIO1_INT1);
    attachInterruptVector(IRQ_GPIO1_INT3, &currentLimitExceeded_ISR); // pin number is filler
    NVIC_ENABLE_IRQ(IRQ_GPIO1_INT3);
    attachInterruptVector(IRQ_GPIO1_INT0, &shutdown_pinned_ISR); 
    NVIC_ENABLE_IRQ(IRQ_GPIO1_INT0);

     // set state  
    state = GLV_ON; 

    // Read the SD CARD Settings for the ECU TUNE
    Serial.println("Initializing SD Card...");
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println("CRITICAL FAULT: PLEASE INSERT SD CARD CONTAINING ECU FLASH TUNE");
        Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");

        state = ERROR;
    }
    else{
        Serial.println("SD INITIALIZATION SUCCESSFUL");
        File ecu_tune;
        ecu_tune = SD.open("GR24_FLASH_TUNE.ecu");
        if(ecu_tune){
            Serial.print("Reading ECU FLASH....");
            String tune;
            while(ecu_tune.available()){
                Serial.print(".");
                tune += (char)ecu_tune.read();
            }
            ecu_tune.close();
            Serial.println("");




            Serial.println("ECU FLASH COMPLETE. GR24 TUNE DOWNLOADED.");

        }
        else {
            Serial.println("CRITICAL FAULT: ERROR OPENING GR24 ECU TUNE");
            Serial.println("MOVING STATE TO ERROR: ECU RESTART REQUIRED");
            state = ERROR;
        }
    }
    




   
}

