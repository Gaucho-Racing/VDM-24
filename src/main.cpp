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

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

void sendPingRequest(std::vector<uint32_t> request_ids, iCANflex& Car){
    if(millis()-sendTime > 500){
        for(uint32_t request_id : request_ids){

            unsigned long mills=millis();
            unsigned long micro=micros();
            byte data[8] = {0x00};
            for(int i=0; i<4; i++){
                data[3-i]=(byte)(mills >> (i*8));
                data[7-i]=(byte)(micro >> (i*8));
            }

            CAN_message_t message;
            message.flags.extended = true;  
            message.id = request_id;
            message.len = 8;
            memcpy(message.buf, data, 8);

            can1.write(message);

            sendTime=millis();
        
        }
  }
}

unsigned long ping() {
    unsigned long newTime = (long)msg.buf[3] + ((long)msg.buf[2] << 8) + ((long)msg.buf[1] << 16) + ((long)msg.buf[0] << 24);
    unsigned long newTime2 = (long)msg.buf[7] + ((long)msg.buf[6] << 8) + ((long)msg.buf[5] << 16) + ((long)msg.buf[4] << 24);
    unsigned long roundTripDelay = ((millis() - newTime) * 1000) + ((micros() - newTime2) % 1000);
    return roundTripDelay;
}

void handlePingResponses() {
    // Handle incoming CAN messages
    if(msg.id == 0x13000){
        // print the message
        Serial.print("Received on 0x13000: ");
        for(int i=0; i<msg.len; i++){
            Serial.print(msg.buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    if(msg.id == 0x13001){
        // print the message
        Serial.print("Received on 0x13001: ");
        for(int i=0; i<msg.len; i++){
            Serial.print(msg.buf[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
    }
    if (msg.id == ACU_Ping_Response) {
        ACU_Ping = ping();
        Serial.println("ACU Ping: ");
        Serial.println(ACU_Ping);
        msg.buf[0] = 1;
        // add the ACU_Ping to the next 4 bytes
        for(int i=0; i<4; i++){
            msg.buf[4-i]=(byte)(ACU_Ping >> (i*8));
        }
        msg.len = 8;
        msg.id = VDM_Ping_Values;
        can1.write(msg);
    }
    if (msg.id == 0xC9) {
        Pedals_Ping = ping();
        Serial.println("Pedals Ping: ");
        Serial.println(Pedals_Ping);
        msg.buf[0] = 2;
        for(int i=0; i<4; i++){
            msg.buf[4-i]=(byte)(Pedals_Ping >> (i*8));
        }
        msg.len = 8;
        msg.id = VDM_Ping_Values;
        can1.write(msg);
    }
    if (msg.id == 0x10FFF) {
        SteeringWheel_Ping = ping();    
        Serial.println("Steering Wheel Ping: ");
        Serial.println(SteeringWheel_Ping);
        msg.buf[0] = 3;
        for(int i=0; i<4; i++){
            msg.buf[4-i]=(byte)(SteeringWheel_Ping >> (i*8));
        }
        msg.len = 8;
        msg.id = VDM_Ping_Values;
        can1.write(msg);
    }
    if (msg.id == 0x12FFF) {
        DashPanel_Ping = ping();
        Serial.println("Dash Panel Ping: ");
        Serial.println(DashPanel_Ping);
        msg.buf[0] = 4;
        for(int i=0; i<4; i++){
            msg.buf[4-i]=(byte)(DashPanel_Ping >> (i*8));
        }
        msg.len = 8;
        msg.id = VDM_Ping_Values;
        can1.write(msg);
    }
    
    
}

void setupCAN() {
  can1.begin();
  can1.setBaudRate(1000000); // Set CAN baud rate to 500 kbps
//   can1.onReceive(handlePingResponses); // Set callback for received messages
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
    if(can1.read(msg)) {
        handlePingResponses();
    }
    // if(millis() % 1000 == 0){
    //     Serial.println("-----------------");
    //     Serial.println(millis());
    //     State currentState = state;
    //     Serial.print(state_to_string.find(currentState)->second.c_str());
    //     Serial.print(" | ");
    //     Mode currentMode = mode;
    //     Serial.println(mode_to_string.find(currentMode)->second.c_str());
    //     Serial.print("Faults: ");
    //     Serial.println(active_faults->size());
    //     Serial.print("Limits: ");
    //     Serial.println(active_warnings->size());
    //     Serial.print("Warnings: ");
    //     Serial.println(active_warnings->size());
    // }
    

    // reads bspd, ams, and imd pins as analog   TODO: Uncomment for actual test bench
    // SystemsCheck::hardware_system_critical(*Car, *active_faults);
    // SystemsCheck::system_faults(*Car, *active_faults);
    // SystemsCheck::system_limits(*Car, *active_limits);
    // SystemsCheck::system_warnings(*Car, *active_warnings);

    // Get ping values for all systems

    sendPingRequest({0x10FFE, 0x12FFE, 0xCA, 0x95}, *Car);



    state = active_faults->size() ?  sendToError(*active_faults->begin()) : state;

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

    pinMode(SOFTWARE_OK_CONTROL_PIN, OUTPUT);
    pinMode(AMS_OK_PIN, INPUT);
    pinMode(BSPD_OK_PIN, INPUT);
    pinMode(IMD_OK_PIN, INPUT);
    pinMode(BRAKE_LIGHT_PIN, INPUT);    

    setupCAN();

    Car->begin();
    active_faults = new unordered_set<bool (*)(const iCANflex&)>(); 
    active_warnings = new unordered_set<bool (*)(const iCANflex&)>();
    active_limits = new unordered_set<bool (*)(const iCANflex&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();
    for(int i = 0; i < 5; i++) SYS_CHECK_CAN_FRAME[i] = 0x0;

    // set state  
    // state = ECU_FLASH; 
    state = GLV_ON;
}
