#include "icanflex.h"
#include "imxrt.h"
#include "Arduino.h"
#include "Nodes.h"
#include <unordered_set>
#include <cstddef>

#include "machine.hpp"
#include "tune.hpp"
#include "debug.hpp"
#include "systems_check.hpp"
#include "string"


#define PRINT_DBG 0x00;

enum State {ECU_FLASH, GLV_ON, TS_PRECHARGE, PRECHARGING, PRECHARGE_COMPLETE, DRIVE_STANDBY, DRIVE_ACTIVE, DRIVE_REGEN, ERROR};
enum Mode {TESTING, LAUNCH, ENDURANCE, AUTOX, SKIDPAD, ACC, PIT};

iCANflex* Car;
SystemsCheck* sysCheck;
Tune* tune;

std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_faults;
std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_warnings;
std::unordered_set<bool (*)(const iCANflex&, Tune& t)> *active_limits;
bool (*errorCheck)(const iCANflex& Car, Tune&); 

bool BSE_APPS_violation = false;
State state;
Mode mode;

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
CAN_message_t msg;

unsigned long ACU_Ping = 0;
unsigned long Pedals_Ping = 0;
unsigned long DashPanel_Ping = 0;
unsigned long SteeringWheel_Ping = 0;
unsigned long sendTime = 0;
unsigned long lastPrechargeTime = 0;
unsigned long lastDTIMessage = 0;


//GLV STARTUP
void setup() {
    Car = new iCANflex();
    Car->begin();
    

    // dbg = new Debugger(10);
    sysCheck = new SystemsCheck();  
    tune = new Tune();
    can1.begin();
    can1.setBaudRate(1000000);
    msg.flags.extended = true;

    Serial.begin(115200);

    pinMode(SOFTWARE_OK_CONTROL_PIN, OUTPUT);
    pinMode(AMS_OK_PIN, INPUT);
    pinMode(BSPD_OK_PIN, INPUT);
    pinMode(IMD_OK_PIN, INPUT);
    pinMode(BRAKE_LIGHT_PIN, INPUT);    


    active_faults = new std::unordered_set<bool (*)(const iCANflex&, Tune&)>(); 
    active_warnings = new std::unordered_set<bool (*)(const iCANflex&, Tune&)>();
    active_limits = new std::unordered_set<bool (*)(const iCANflex&, Tune&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();


    // set state  
    // state = ECU_FLASH; TODO: Remember to uncomment 
    state = GLV_ON;
    mode = ENDURANCE; // TODO: Energy management algorithm for endurance

}



// ----------------------------- CONTROLLER AREA NETWORK -----------------------------
unsigned long ping() {
    unsigned long newTime = (long)msg.buf[3] + ((long)msg.buf[2] << 8) + ((long)msg.buf[1] << 16) + ((long)msg.buf[0] << 24);
    unsigned long newTime2 = (long)msg.buf[7] + ((long)msg.buf[6] << 8) + ((long)msg.buf[5] << 16) + ((long)msg.buf[4] << 24);
    unsigned long roundTripDelay = ((millis() - newTime) * 1000) + ((micros() - newTime2) % 1000);
    return roundTripDelay;
}


void tryPingReqests(std::vector<uint32_t> request_ids, iCANflex& Car){
    if(millis()-sendTime > 20){
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
    void sendDashboardPopup(int warning_id){
        //TODO:

    }


    void sendVDMInfo(State state, Mode mode, Tune& tune){
       //TODO:
    }

    void handleDriverInputs(Tune& tune){
        if(msg.id == 0x11002){
            tune.settings.power_level = msg.buf[0];
            tune.settings.throttle_map = msg.buf[1];
            tune.settings.regen_level = msg.buf[2];
        }
    }


    void handleDashPanelInputs(State& state){
        if(msg.id == 0x13000 ){
            if(msg.buf[0]){ // TS_ACTIVE
                if(state == GLV_ON){
                    if(millis() - lastPrechargeTime > 5000){
                        state = TS_PRECHARGE;
                        CAN_message_t message;
                        message.flags.extended = true;
                        message.id = 0x66;
                        message.len = 8;
                        message.buf[0] = 1;
                        can1.write(message);
                        lastPrechargeTime = millis();
                    }
                }
            }
            else if(msg.buf[1]){ // TS_OFF
                // shut off car entireley
                CAN_message_t message;
                message.flags.extended = true;
                message.id = 0x66;
                message.len = 8;
                message.buf[0] = 0;
                can1.write(message);
                state = GLV_ON;
            }
            else if(msg.buf[2]) {// RTD_ON
                if(state == PRECHARGE_COMPLETE){
                    state = DRIVE_STANDBY;
                    // play rtd sound
                }
            }
            else if(msg.buf[3]){ // RTD_OFF
                if(state == DRIVE_STANDBY) {
                    state = TS_PRECHARGE;
                }
            }
        }
        
    }

    void handlePings(){
        //PING RESPONSES

        if (msg.id == ACU_Ping_Response) {
            ACU_Ping = ping();
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
            msg.buf[0] = 4;
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(DashPanel_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
    }


// namespace std {
//     template <>
//     struct hash<bool (*)(const iCANflex&)> {
//         size_t operator()(bool (*f)(const iCANflex&)) const noexcept{
//             return reinterpret_cast<size_t>(f);
//         }
//     };
// };


State sendToError(bool (*erFunc)(const iCANflex& Car, Tune& tune)) {
   errorCheck = erFunc; 
   return ERROR;
}


// ----------------------------- DEBUG STUFF -----------------------------


std::unordered_map<State, std::string> state_to_string = {
                {ECU_FLASH, "ECU_FLASH"},
                {GLV_ON, "GLV_ON"},
                {TS_PRECHARGE, "TS_PRECHARGE"},
                {PRECHARGING, "PRECHARGING"},
                {PRECHARGE_COMPLETE, "PRECHARGE_COMPLETE"},
                {DRIVE_STANDBY, "DRIVE_STANDBY"},
                {DRIVE_ACTIVE, "DRIVE_ACTIVE"},
                {DRIVE_REGEN, "DRIVE_REGEN"},
                {ERROR, "ERROR"}
            };
std::unordered_map<Mode, std::string>mode_to_string = {
                {TESTING, "TESTING"},
                {LAUNCH, "LAUNCH"},
                {ENDURANCE, "ENDURANCE"},
                {AUTOX, "AUTOX"},
                {SKIDPAD, "SKIDPAD"},
                {ACC, "ACC"},
                {PIT, "PIT"}
            };  

void printStatus(){
    if(millis() % 100 == 0){
        Serial.println("==================================");
        Serial.print("CLOCK TIME: ");
        Serial.println(millis());
        Serial.print("STATE: ");
        State currentState = state;
        Serial.print(state_to_string.find(currentState)->second.c_str());
        Serial.print(" | ");
        Mode currentMode = mode;
        Serial.print("MODE: ");
        Serial.println(mode_to_string.find(currentMode)->second.c_str());
        Serial.println("==================================");
        Serial.println("SYSTEM HEALTH: ");
        Serial.print("Critical Faults: ");
        Serial.println(active_faults->size());
        Serial.print("Limits: ");
        Serial.println(active_limits->size());
        Serial.print("Warnings: ");
        Serial.println(active_warnings->size());
        Serial.println("==================================");
        Serial.println("PING TIME: ");
        Serial.print("ACU Ping: ");
        Serial.println(ACU_Ping);
        Serial.print("Pedals Ping: ");
        Serial.println(Pedals_Ping);
        Serial.print("DashPanel Ping: ");
        Serial.println(DashPanel_Ping);
        Serial.print("SteeringWheel Ping: ");
        Serial.println(SteeringWheel_Ping);
        Serial.print("==================================");
    }
}



// ----------------------------- STATE MACHINE FUNCTIONS -----------------------------


/*

STARTUP STAGE 1:
ECU FLASH
When the Car is turned on, the Main ECU will read in the ECU flash from the SD card.
This will be the first state that the car will enter.
This is essential for the car to operate as the ECU flash contains the 
torque profiles, regen profiles, and traction control profiles.
*/
State ecu_flash(iCANflex& Car) {
    // Car.DTI.setDriveEnable(0);
    // Car.DTI.setRCurrent(0);
    // flash the ecu
    return ECU_FLASH;

}

/*
STARTUP STAGE 2:    
GLV ON
When the grounded low voltage system is turned on, the microcontroller has power, 
but the motor controller is not enabled. This is the second state that the car will enter
after the ECU Flash is complete. Here it waits for the TS ACTIVE button to be pressed.
*/
State glv_on(iCANflex& Car) {
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }
    // wait for the TS ACTIVE button to be pressed
    // return TS_PRECHARGE;
    return GLV_ON;
}  


/*
STARTUP STAGE 3: PRECHARGING
When the TS ACTIVE button is pressed, the car will enter the precharging state.
This is the third state that the car will enter after the GLV_ON state.
The precharging is essential for the car to operate as it allows the voltage to build up
in the motor controller before the car can be driven.
PRECHARGING is broken into 3 stages for ACU responses and communication
*/

// -- PRECHARGING STAGE 1 
State ts_precharge(iCANflex& Car) { 
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    // begin precharging by sendign signal to ACU
    //TODO: Precharge stuff
    if(Car.ACU1.getPrecharging()){
        return PRECHARGING;
    }
    return TS_PRECHARGE;
}
// -- PRECHARGING STAGE 2
State precharging(iCANflex& Car){
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    if(Car.ACU1.getPrechargeDone()) return PRECHARGE_COMPLETE;
    return PRECHARGE_COMPLETE;
}

// -- PRECHARGING STAGE 3
State precharge_complete(iCANflex& Car){
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }
    // wait for RTD signal
    return PRECHARGE_COMPLETE;  
}

/*
STARTUP STAGE 4:  READY TO DRIVE

READY TO DRIVE SUB STATES
- DRIVE_NULL
- DRIVE_TORQUE
- DRIVE_REGEN

*/
State drive_standby(iCANflex& Car, bool& BSE_APPS_violation) {
    
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    float throttle = (Car.PEDALS.getAPPS1() + Car.PEDALS.getAPPS2())/2.0;
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2.0;
    
    // only if no violation, and throttle is pressed, go to DRIVE
    if(!BSE_APPS_violation && throttle > 0.05) return DRIVE_ACTIVE;
    if(!BSE_APPS_violation && brake > 0.05) return DRIVE_REGEN;

    if(BSE_APPS_violation) {
        // SEND CAN WARNING TO DASH
        if(throttle < 0.05) {
            // violation exit condition, reset violation and return to DRIVE_READY
            BSE_APPS_violation = false;
            return DRIVE_STANDBY;
        }  
    }
    // else loop back into RTD state with Violation still true
    return DRIVE_STANDBY;
}


/*
DRIVE_TORQUE STATE

THIS STATE IS RESPONSIBLE FOR THE VEHICLE DYNAMICS WHEN THE DRIVER IS REQUESTING TORQUE FROM THE MOTOR.
THE TORQUE IS CALCULATED THROUGH THE STANDARD EQUATION DEFINED BELOW. 
Z = X-(1-X)(X+B)(Y^P)K  0 <= Z <= 1 (CLIPPED)
X IS THROTTLE 0 TO 1
Y IS RPM LOAD 0 TO 1
B IS OFFSET 0 TO 1 
K IS MULTIPLIER 0 TO 1
P IS STEEPNESS 0 TO 5

THE CONSTANTS B, K, AND P ARE DEFINED THROUGHOUT THE ECU MAP IN THE SD CARD OR THE REFLASH OVER CAN.
THIS VALUE OF Z IS APPLIED TO THE MAX CURRENT SET AND WILL BE THE DRIVER REQUESTED TORQUE. 
THIS IS FOR A GENERALLY SMOOTHER TORQUE PROFILE AND DRIVABILITY.

THE DRIVE_TORQUE STATE IS ALSO RESPONSIBLE FOR CHECKING THE APPS AND BSE FOR VIOLATIONS AS WELL AS 
THE GRADIENTS OF THE TWO APPS SIGNALS TO MAKE SURE THAT THEY ARE NOT COMPROMISED. 
*/


float requested_torque(iCANflex& Car, float throttle, int rpm, Tune& tune) {
    // python calcs: z = np.clip((x - (1-x)*(x + b)*((y/5500.0)**p)*k )*100, 0, 100)
    TorqueProfile tp = tune.getActiveTorqueProfile();
    float k = tp.K;
    float p = tp.P;
    float b = tp.B;
    float max_current = tune.getActiveCurrentLimit();
    float torque_multiplier = (throttle-(1-throttle)*(throttle+b)*pow(rpm/tune.revLimit(), p)*k);
    if(torque_multiplier > 1) torque_multiplier = 1; // clipping
    if(torque_multiplier < 0) torque_multiplier = 0;
    return torque_multiplier*max_current;
}


State drive_active(iCANflex& Car, bool& BSE_APPS_violation, Tune& tune) {

    float a1 = Car.PEDALS.getAPPS1();
    float a2 = Car.PEDALS.getAPPS2();
    float throttle = a1; // TODO: FIX
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2;
    
    // APPS GRADIENT VIOLATION
    if(abs(a1 - (2*a2)) > 0.1){
        // TODO: send an error message on the dash that APPS blew up
        // comms.sendDashboardPopup(0x01);
        return DRIVE_STANDBY;
    } 
    // APPS BSE VIOLATION
    if((brake > 0.05 && a1 > 0.25)) {
        BSE_APPS_violation = true;
        return DRIVE_STANDBY;
    }
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setDriveEnable(1);
        Car.DTI.setRCurrent(requested_torque(Car, throttle, Car.DTI.getERPM()/10.0, tune));
        // float power = Car.ACU1.getAccumulatorVoltage() * Car.DTI.getDCCurrent();
        lastDTIMessage = millis();
    }
    
    return DRIVE_ACTIVE;
}

float requested_regenerative_torque(iCANflex& Car, float brake, int rpm) {
    // if(rpm > 500 && brake > 0.05) return Car.ACU1.getMaxChargeCurrent();
    // else return 0;
    return 0;
}

State drive_regen(iCANflex& Car, bool& BSE_APPS_violation, Tune& tune){
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2; // TODO: Change to AnalogRead from pin
    float throttle = Car.PEDALS.getAPPS1();
    if(throttle > 0.05) return DRIVE_ACTIVE;
    if(brake < 0.05) return DRIVE_STANDBY;

    float rpm = Car.DTI.getERPM()/10.0;
    
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setDriveEnable(1);
        Car.DTI.setRCurrent(-1 * requested_regenerative_torque(Car, brake, rpm) * tune.getActiveRegenPower());
        lastDTIMessage = millis();
    }
    return DRIVE_REGEN;
}

/*
ERROR STATE

THIS STATE WILL HANDLE ERRORS THAT OCCUR DURING THE OPERATION OF THE VEHICLE.
THIS STATE WILL BE ENTERED WHENEVER A CRITICAL SYSTEMS FAILURE OCCURS OR WHEN THE
DRIVER REQUESTS TO STOP THE VEHICLE.

THE VEHICLE REMAINS IN THIS STATE UNTIL THE VIOLATION IS RESOLVED 

*/


State error(iCANflex& Car, Tune& t, bool (*errorCheck)(const iCANflex& c, Tune& t), std::unordered_set<bool (*)(const iCANflex& c, Tune& t)>& active_faults){
    if(millis() - lastDTIMessage > 10){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    if(errorCheck(Car, t))  return ERROR;
    else {
        active_faults.erase(errorCheck);
        return GLV_ON; // gets sent back to error from main() if there are more in the hashset from main
    }
    
}




// ----------------------------- MAIN LOOP -----------------------------

void loop(){
    
    // Serial.println(Car->ACU1.getPrecharging());
    if(can1.read(msg)) {
        handleDashPanelInputs(state);    
        handleDriverInputs(*tune);
        handlePings();
    }
    
    // Get ping values for all systems
    tryPingReqests({0x10FFE, 0x12FFE, 0xCA, 0x95}, *Car);
   
    // reads bspd, ams, and imd pins as analog   TODO: Uncomment for actual test bench
    sysCheck->hardware_system_critical(*Car, *active_faults, tune);
    sysCheck->system_faults(*Car, *active_faults, tune);
    sysCheck->system_limits(*Car, *active_limits, tune);
    sysCheck->system_warnings(*Car, *active_warnings, tune);
    #if defined(PRINT_DBG)
        printStatus();
    #endif
    

    state = active_faults->size() ?  sendToError(*active_faults->begin()) : state;

    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);
   
    tune->settings.power_level = active_limits->size() ? LIMIT : tune->settings.power_level; // limit power in overheat conditions

    // error severity: warning -> limit -> critical

   // state machine operation
    switch (state) {
        // ERROR
        case ERROR:
            state = error(*Car, *tune, errorCheck, *active_faults);
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
        case DRIVE_STANDBY:
            state = drive_standby(*Car, BSE_APPS_violation); 
            break;
        case DRIVE_ACTIVE:
            // state = drive_active(*Car, BSE_APPS_violation, mode, *tune, *comms);
            break;
        case DRIVE_REGEN:
            state = drive_regen(*Car, BSE_APPS_violation, *tune);
            break;
    }
    
}




