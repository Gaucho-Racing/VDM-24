#include "machine.h"
#include "main.h"

volatile State state;
volatile Mode mode;
bool (*errorCheck)(const iCANflex& Car); 
bool BSE_APPS_violation = false;

State sendToError(bool (*erFunc)(const iCANflex& Car)) {
   errorCheck = erFunc; 
   return ERROR;
}

static void SEND_SYS_CHECK_FRAMES(){ // TODO:
    Serial.println("SENDING SYS CHECK FRAMES");
    for(int i = 0; i < 5; i++){
        Serial.print(millis());
        Serial.println(SYS_CHECK_CAN_FRAME[i]);
    }
} 

// string maps for testing purposes
// -------------------------------------------------------------------------------
static unordered_map<State, string> state_to_string = {
    {ECU_FLASH, "ECU_FLASH"},
    {GLV_ON, "GLV_ON"},
    {TS_PRECHARGE, "TS_PRECHARGE"},
    {PRECHARGING, "PRECHARGING"},
    {PRECHARGE_COMPLETE, "PRECHARGE_COMPLETE"},
    {DRIVE_NULL, "DRIVE_NULL"},
    {DRIVE_TORQUE, "DRIVE_TORQUE"},
    {DRIVE_REGEN, "DRIVE_REGEN"},
    {ERROR, "ERROR"}
};

static unordered_map<Mode, string> mode_to_string = {
    {TESTING, "TESTING"},
    {LAUNCH, "LAUNCH"},
    {ENDURANCE, "ENDURANCE"},
    {AUTOX, "AUTOX"},
    {SKIDPAD, "SKIDPAD"},
    {ACC, "ACC"},
    {PIT, "PIT"}
};  

// -------------------------------------------------------------------------------

void loop(){

    Serial.println("-----------------");
    State currentState = state;
    Serial.print(state_to_string.find(currentState)->second.c_str());
    Serial.print(" | ");
    Mode currentMode = mode;
    Serial.println(mode_to_string.find(currentMode)->second.c_str());
    Serial.print("Faults: ");
    Serial.println(active_faults->size());
    Serial.print("Limits: ");
    Serial.println(active_warnings->size());
    Serial.print("Warnings: ");
    Serial.println(active_warnings->size());

    // reads bspd, ams, and imd pins as analog   TODO: Uncomment for actual test bench
    // SystemsCheck::hardware_system_critical(*Car, *active_faults);
    // SystemsCheck::system_faults(*Car, *active_faults);
    // SystemsCheck::system_limits(*Car, *active_limits);
    // SystemsCheck::system_warnings(*Car, *active_warnings);


    // SEND_SYS_CHECK_FRAMES();
    
    delay(500);
    
    state = active_faults->size() ?  sendToError(*active_faults->begin()) : state;
    // Serial.println(millis());
    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);
   

    // error severity: warning -> limit -> critical

    // read in settings from Steering Wheel
    throttle_map = 0; 
    regen_level = 0;
    power_level = 0;

    power_level = active_limits->size() ? LIMIT : power_level; // limit power in overheat conditions

    
    mode = ENDURANCE; // TODO: Energy management algorithm for endurance

   

    // STATE MACHINE OPERATION
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
        case DRIVE_TORQUE:
            state = drive_torque(*Car, BSE_APPS_violation, mode);
            break;
        case DRIVE_REGEN:
            state = drive_regen(*Car, BSE_APPS_violation, mode);
            break;
    }
}



//GLV STARTUP
void setup() {
    Car = new iCANflex();
    Serial.begin(9600);
    Serial.println("Waiting for Serial Port to connect");
    while(!Serial) Serial.println("Waiting for Serial Port to connect");
    Serial.println("Connected to Serial Port 9600");

    Car->begin();
    active_faults = new unordered_set<bool (*)(const iCANflex&)>(); 
    active_warnings = new unordered_set<bool (*)(const iCANflex&)>();
    active_limits = new unordered_set<bool (*)(const iCANflex&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();
    for(int i = 0; i < 5; i++) SYS_CHECK_CAN_FRAME[i] = 0x0;

    // set state  
    state = ECU_FLASH; 
    // state = GLV_ON;
}
