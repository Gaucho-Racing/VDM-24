#include "machine.h"
#include "main.h"

volatile State state;
volatile Mode mode;
bool (*errorCheck)(const iCANflex& Car); 
bool BSE_APPS_violation = false;

// ECU TUNE Reads
float PWR_REGEN_MAX;
float BRAKE_BALANCE;
float MAX_MOTOR_CURRENT;
vector<unordered_map<int, float>> THROTTLE_MAPPING(100); // index is throttle position, value[rpm] is the % of max current
unordered_map<float, float> REGEN_TORQUE_MAPPING;
vector<float> LAUNCH_CONTROL_FUNCTION;
float LAUNCH_CONTROL_INTERVAL;

const int REV_LIMITER = 5500;

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

    if(motorTempHighEntryCondition(Car)) {
        NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT2);
    }

    if (canReceiveFailure(Car)) { NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT1); }

    if (currentLimitExceeded(Car)) { 
        NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT3);
    }

    if (shutdown_pinned(Car)) { NVIC_TRIGGER_IRQ(IRQ_GPIO1_INT0); }

    switch (state) {
        // ERROR
        case ERROR:
            state = error(*Car, errorCheck);
            break;

        // STARTUP 
        case ECU_FLASH:
            state = ecu_flash(*Car);
            break;
        case GLV_ON:
            state = glv_on(*Car);
            break;
     
        // PRECHARGE
        case TS_PRECHARGE:
            state = ts_precharge(*Car);
            break;
        case PRECHARGING:
            state = precharging(*Car);
            break;
        case PRECHARGE_COMPLETE:
            state = precharge_complete(*Car);
            break;
        
        // DRIVE
        case DRIVE_NULL:
            state = drive_null(*Car, BSE_APPS_violation, mode); 
            break;
        case LAUNCH:
            state = launch(Car, switches, BSE_APPS_violation);
            break;
        case ERROR:
            state = error(Car, switches, prevState, errorCheck);
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
    state = OFF; 

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
            string tune;
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

