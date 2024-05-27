
#include "imxrt.h"
#include "Arduino.h"
//#include "Nodes.h"
#include <unordered_set>
#include <cstddef>
#include "SD.h"
#include <sstream>
#include <string>
#include <unordered_map>    
#include <vector>
#include "string"
#include "network.h"
#include "vehicle.h"
#include "machine.h"
#include "global.h"
#include "VehicleTuneController.h"
#include "SystemsCheck.h"

/*
    _   _______________       ______  ____  __ __
   / | / / ____/_  __/ |     / / __ \/ __ \/ //_/
  /  |/ / __/   / /  | | /| / / / / / /_/ / ,<   
 / /|  / /___  / /   | |/ |/ / /_/ / _, _/ /| |  
/_/ |_/_____/ /_/    |__/|__/\____/_/ |_/_/ |_|  
                                                 
*/


FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_primary;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_data;
CAN_message_t msg;
CAN_message_t msg2;
Vehicle *Car = new Vehicle(can_primary, can_data);
SystemsCheck* sysCheck = new SystemsCheck();
VehicleTuneController* tune = new VehicleTuneController();
State state;
Mode mode;
SWSettings settings;



/*
  ________  ___   _______   ________
 /_  __/ / / / | / /  _/ | / / ____/
  / / / / / /  |/ // //  |/ / / __  
 / / / /_/ / /|  // // /|  / /_/ /  
/_/  \____/_/ |_/___/_/ |_/\____/   
   _____ _______________________   _____________
  / ___// ____/_  __/_  __/  _/ | / / ____/ ___/
  \__ \/ __/   / /   / /  / //  |/ / / __ \__ \ 
 ___/ / /___  / /   / / _/ // /|  / /_/ /___/ / 
/____/_____/ /_/   /_/ /___/_/ |_/\____//____/  
                                                
                                    



/*
A class to store the vehicle's performance tune and settings. 
This includes the power limits, torque mappings, regen settings, error limits, and other vehicle parameters.
The tune should be initialized via SD card and can be modified in other functions using the set methods. 
*/



/*
Reads the SD card in the Microcontroller to initialize the Vehicles Parameters and Performance VehicleTuneController with Race Presets.

*/



/*
   _______  _____________________  ___   __  ___________    __  ________  __
  / ___/\ \/ / ___/_  __/ ____/  |/  /  / / / / ____/   |  / / /_  __/ / / /
  \__ \  \  /\__ \ / / / __/ / /|_/ /  / /_/ / __/ / /| | / /   / / / /_/ / 
 ___/ /  / /___/ // / / /___/ /  / /  / __  / /___/ ___ |/ /___/ / / __  /  
/____/  /_//____//_/ /_____/_/  /_/  /_/ /_/_____/_/  |_/_____/_/ /_/ /_/   
    __________  ____  ____  ____     ________  ________________ _______
   / ____/ __ \/ __ \/ __ \/ __ \   / ____/ / / / ____/ ____/ //_/ ___/
  / __/ / /_/ / /_/ / / / / /_/ /  / /   / /_/ / __/ / /   / ,<  \__ \ 
 / /___/ _, _/ _, _/ /_/ / _, _/  / /___/ __  / /___/ /___/ /| |___/ / 
/_____/_/ |_/_/ |_|\____/_/ |_|   \____/_/ /_/_____/\____/_/ |_/____/  
                                                                                                                                                 
*/










/*
   ________    ____  ____  ___    __ 
  / ____/ /   / __ \/ __ )/   |  / / 
 / / __/ /   / / / / __  / /| | / /  
/ /_/ / /___/ /_/ / /_/ / ___ |/ /___
\____/_____/\____/_____/_/  |_/_____/
*/

 // error severity: warning -> limit -> critical

// A class to statically check for system faults and warnings and gives dynamic CAN frames using bit masking. 


// iCANflex* Car; // Controller Area Network Object for the Vehicles CAN Bus


// THIS HASH FUNCTION DOES NOT WORK: DO NOT USE
// namespace std { 
//     template <>
//     struct hash<bool (*)(VehicleTuneController& t)> {
//         size_t operator()(bool (*f)(VehicleTuneController& t)) const noexcept{
//             return reinterpret_cast<size_t>(f);
//         }
//     };
// };

std::unordered_set<bool (*)(VehicleTuneController& t)> *active_faults; // hashset of active system faults as function pointers
std::unordered_set<bool (*)(VehicleTuneController& t)> *active_warnings; // hashset of active system warnings as function pointers
std::unordered_set<bool (*)(VehicleTuneController& t)> *active_limits; // hashset of active system limits as function pointers
bool (*errorCheck)(VehicleTuneController&); // global function pointer to error causing the ISR
State sendToError(bool (*erFunc)(VehicleTuneController& tune)) {
   errorCheck = erFunc; 
   return ERROR;
}


bool BSE_APPS_violation = false;







// green means press, red means dont press



#define SERIAL_BUFFER_SIZE 256;






/*
  __________  ___   ____________________  _   __   
 /_  __/ __ \/   | / ____/_  __/  _/ __ \/ | / /   
  / / / /_/ / /| |/ /     / /  / // / / /  |/ /    
 / / / _, _/ ___ / /___  / / _/ // /_/ / /|  /     
/_/ /_/ |_/_/  |_\____/ /_/ /___/\____/_/ |_/      
    __________  _   ____________  ____  __ 
  / ____/ __ \/ | / /_  __/ __ \/ __ \/ / 
 / /   / / / /  |/ / / / / /_/ / / / / /  
/ /___/ /_/ / /|  / / / / _, _/ /_/ / /___
\____/\____/_/ |_/ /_/ /_/ |_|\____/_____/
                                                                                            

*/
float Kp = 0.2;   // Proportional gain
float Ki = 0.02;  // Integral gain
float Kd = 0.1;   // Derivative gain

// MOTOR MODEL
const float L_INDUCTANCE = 225.5e-6;
const float R_RESISTANCE = 0.01548;
const float KB_BACK_EMF = 0.6870;
const float KM_TORQUE_CONSTANT = 0.6870;
const float J_INERTIA = 0.0421;
const float B_DAMPING = 3.97e-6;
const float FC = 0.0;
const float VEHICLE_MASS = 300;
const float WHEEL_RADIUS = 0.1778;
const float G = 9.81;
const float Q_W_BALANCE_FACTOR = 0.5;
const float GEAR_RATIO = 3.42;


// System variables
float tc_multiplier = 1;
float loss, previousLoss = 0;
float integral = 0;
float derivative;
float pidOutput;
unsigned long lastTractionCompute = 0;

// Constants for performance thresholds
const float SLIP_THRESHOLD = 0.1;  // Threshold for initiating corrective action

// Function to dynamically adjust PID gains based on driving conditions
void adjustPIDGains(float slipRatio) {
    if (slipRatio > SLIP_THRESHOLD) {
        // Increase gains for high slip scenarios
        Kp = 0.3;
        Ki = 0.03;
        Kd = 0.15;
    } else {
        // Reset to normal gains if slip is under control
        Kp = 0.2;
        Ki = 0.02;
        Kd = 0.1;
    }
}

// Calculate slip ratio
float calculateSlipRatio(float referenceSpeed, float actualSpeed) {
    if (referenceSpeed == 0) return 0;
    return (actualSpeed - referenceSpeed) / referenceSpeed;
}

// Main traction control function
void computeTractionControl() {
    if (millis() - lastTractionCompute > 1000 / TRACTION_CONTROL_FREQENCY) {
        float rearLeftWheelSpeed = Car->WRL.getWheelSpeed();
        float rearRightWheelSpeed = Car->WRR.getWheelSpeed();
        float frontLeftWheelSpeed = Car->WFL.getWheelSpeed();
        float frontRightWheelSpeed = Car->WFR.getWheelSpeed();

        float averageRearWheelSpeed = (rearLeftWheelSpeed + rearRightWheelSpeed) / 2;
        float averageFrontWheelSpeed = (frontLeftWheelSpeed + frontRightWheelSpeed) / 2;

        float slipRatio = calculateSlipRatio(averageFrontWheelSpeed, averageRearWheelSpeed);

        // Adjust PID gains dynamically based on slip ratio
        adjustPIDGains(slipRatio);

        // Compute loss and PID output
        loss = slipRatio;
        integral += loss;
        derivative = loss - previousLoss;
        pidOutput = Kp * loss + Ki * integral + Kd * derivative;
        previousLoss = loss;

        tc_multiplier = 1.0 - constrain(pidOutput, 0.0, 1.0);
        // Update system control loop timing
        lastTractionCompute = millis(); 

        // Optionally log or display the PID parameters and multiplier for tuning and monitoring
        // Serial.print("Slip Ratio: "); Serial.println(slipRatio);
        // Serial.print("PID Output: "); Serial.println(pidOutput);
        // Serial.print("Throttle Multiplier: "); Serial.println(tc_multiplier);
    }
}



/*
    ____  __________  __  ________
   / __ \/ ____/ __ )/ / / / ____/
  / / / / __/ / __  / / / / / __  
 / /_/ / /___/ /_/ / /_/ / /_/ /  
/_____/_____/_____/\____/\____/   
    __    ____  _________________   ________
   / /   / __ \/ ____/ ____/  _/ | / / ____/
  / /   / / / / / __/ / __ / //  |/ / / __  
 / /___/ /_/ / /_/ / /_/ // // /|  / /_/ /  
/_____/\____/\____/\____/___/_/ |_/\____/   
                                            
*/

unsigned long lastPrintTime = 0;
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
                {STANDARD, "STANDARD"},
                {DYNAMIC_TC, "DYNAMIC_TC"}
}; 
std::unordered_map<int, std::string>node_to_string = {
    {1, "ACU"},
    {2, "Pedals"},
    {3, "Steering Wheel"},
    {4, "Dash Panel"} //TODO: BCM, TCM
};


String vehicleStatus(){
    String output = "|                       STATUS:                          |\n";
    output += "| CLOCK: " + String(millis()) + " ms ";
    output += "| STATE: ";
    State currentState = state;
    output += state_to_string.find(currentState)->second.c_str();
    output += " | MODE: ";
    Mode currentMode = mode;
    output += mode_to_string.find(currentMode)->second.c_str();
    output += "     |\n";
    output += "----------------------------------------------------------";
    return output;
}

String vehicleHealth(){
    String output = "|                      SYSTEM HEALTH:                    |\n";
    output += "| CRITICAL: " + String(active_faults->size()) + " | LIMIT: " + String(active_limits->size()) + " | WARN: " + String(active_warnings->size()) + "          \n| HARDWARE FAULTS: ";
    for(auto e : *active_faults){
        Serial.println("here");
        if(e == sysCheck->AMS_fault) output += "AMS | ";
        if(e == sysCheck->IMD_fault) output += "IMD | ";
        if(e == sysCheck->BSPD_fault) output += "BSPD | ";
        if(e == sysCheck->SDC_opened) output += "SDC ";
    }
    output += "\n ----------------------------------------------------------";
    return output;
}

String vehicleNetwork(){
    String output = "|          NETWORK SPEED: (microseconds)                 |\n";
    if(timeout_nodes.find(ACU_Ping_Response) != timeout_nodes.end()) output += "| ACU: COOKED 💀🔥 \n";
    else output += "| ACU: " + String(ping_response_times[ACU_Ping_Response]) + "\n" ;
    if(timeout_nodes.find(Pedals_Ping_Response) != timeout_nodes.end()) output += "| Pedals: COOKED 💀🔥 \n";
    else output += "| Pedals: " + String(ping_response_times[Pedals_Ping_Response]) + "\n" ;
    if(timeout_nodes.find(Steering_Wheel_Ping_Response) != timeout_nodes.end()) output += "| Steering: COOKED 💀🔥 \n";
    else output += "| Steering: " + String(ping_response_times[Steering_Wheel_Ping_Response]) + "\n" ;
    if(timeout_nodes.find(Dash_Panel_Ping_Response) != timeout_nodes.end()) output += "| DashPanel: COOKED 💀🔥 \n";
    else output += "| DashPanel: " + String(ping_response_times[Dash_Panel_Ping_Response]) + "  \n";
    output += "----------------------------------------------------------";
    return output;
}

String vehicleSettings(){
    String output = "|                     VEHICLE SETTINGS:                  |\n";
    output += "| POWER LEVEL: ";
    if (settings.power_level == LIMIT) output += "LIMIT     ";
    else if (settings.power_level == LOW_PWR) output += "LOW       ";
    else if (settings.power_level == MID_PWR) output += "MID       ";
    else if (settings.power_level == HIGH_PWR) output += "HIGH      ";
    output += "\n| THROTTLE MAP: ";
    if (settings.throttle_map == LINEAR_TORQUE) output += "LINEAR    ";
    else if (settings.throttle_map == TORQUE_MAP_1) output += "MAP 1     ";
    else if (settings.throttle_map == TORQUE_MAP_2) output += "MAP 2     ";
    else if (settings.throttle_map == TORQUE_MAP_3) output += "MAP 3     ";
    output += "\n| REGEN LEVEL: ";
    if (settings.regen_level == REGEN_OFF) output += "OFF       \n";
    else if (settings.regen_level == REGEN_LOW) output += "LOW       \n";
    else if (settings.regen_level == REGEN_MID) output += "MID       \n";
    else if (settings.regen_level == REGEN_HIGH) output += "HIGH      \n";
    output += "----------------------------------------------------------";
    return output;
}

String vehiclePowerData(){
    String output = "|                     POWER DATA:                        |\n";
    int raw1 = Car->PEDALS.getAPPS1();
    output += "| APPS1: RAW: " + String(raw1) + ", SCALED: " + String(getThrottle1(raw1, *tune)) + "               \n";
    int raw2 = Car->PEDALS.getAPPS2();
    output += "| APPS2: RAW: " + String(raw2) + ", SCALED: " + String(getThrottle2(raw2, *tune)) + "               \n";
    output += "| BSE: " + String(analogRead(BSE_HIGH)) + "               \n";
    output += "| INVERTER CURRENT LIMIT: " + String(tune->getPowerLevelsData()[settings.power_level] )+ " A            \n";
    output += "| POWER DRAW: " + String(Car->DTI.getACCurrent() * Car->ACU1.getTSVoltage()) + "W                          \n";
    output += "----------------------------------------------------------\n";
    return output;
}

void printDebug(){
    if(millis() - lastPrintTime > 1000/DEBUG_PRINT_FREQUENCY){
        // Serial.println("----------------------------------------------------------");
        // Serial.println("|                     GR24 EV VEHICLE DEBUG              |");
        // Serial.println("----------------------------------------------------------");
        // Serial.println(vehicleStatus());
        Serial.println(vehicleHealth());
        // Serial.println(vehicleNetwork());
        // Serial.println(vehicleSettings());
        // Serial.println(vehiclePowerData());
        lastPrintTime = millis();
    }
}



/*
    __  ______    _____   __   ____  ____  ____  __________  ___    __  ___
   /  |/  /   |  /  _/ | / /  / __ \/ __ \/ __ \/ ____/ __ \/   |  /  |/  /
  / /|_/ / /| |  / //  |/ /  / /_/ / /_/ / / / / / __/ /_/ / /| | / /|_/ / 
 / /  / / ___ |_/ // /|  /  / ____/ _, _/ /_/ / /_/ / _, _/ ___ |/ /  / /  
/_/  /_/_/  |_/___/_/ |_/  /_/   /_/ |_|\____/\____/_/ |_/_/  |_/_/  /_/                                                                              
    _______  __ ______________  ________________  _   __
   / ____/ |/ // ____/ ____/ / / /_  __/  _/ __ \/ | / /
  / __/  |   // __/ / /   / / / / / /  / // / / /  |/ / 
 / /___ /   |/ /___/ /___/ /_/ / / / _/ // /_/ / /|  /  
/_____//_/|_/_____/\____/\____/ /_/ /___/\____/_/ |_/   
*/



//GLV STARTUP
void setup() {
    //Car = new Vehicle(can_primary, can_data);
    sysCheck = new SystemsCheck();  
    tune = new VehicleTuneController();
    can_primary.begin();
    can_primary.setBaudRate(1000000);
    msg.flags.extended = 1;
    can_data.begin();
    can_data.setBaudRate(1000000);

    Serial.begin(115200);

    pinMode(SOFTWARE_OK_CONTROL_PIN, OUTPUT);
    pinMode(AMS_OK_PIN, INPUT);
    pinMode(BSPD_OK_PIN, INPUT);
    pinMode(IMD_OK_PIN, INPUT);
    pinMode(BRAKE_LIGHT_PIN, OUTPUT);
    pinMode(BSE_HIGH, INPUT);    
    pinMode(CURRENT_SIGNAL, INPUT);


    active_faults = new std::unordered_set<bool (*)(VehicleTuneController&)>(); 
    active_warnings = new std::unordered_set<bool (*)(VehicleTuneController&)>();
    active_limits = new std::unordered_set<bool (*)(VehicleTuneController&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();


    // set state  
    state = ECU_FLASH;
    //! FOR TEST ONLY - DELETE LATER
    // state = DRIVE_STANDBY;
    mode = STANDARD; 

    // ! FOR MOTOR TEST BENCH ONLY
    // ! UNCOMMENT FOR NOMINAL VEHICLE OPERATION
    tune->setPowerLevelData(LIMIT, 0);
    tune->setPowerLevelData(LOW_PWR,2);
    tune->setPowerLevelData(MID_PWR, 20);
    tune->setPowerLevelData(HIGH_PWR, 50);

    settings.power_level = LOW_PWR;
    Car->DTI.setMaxCurrent(tune->getActiveCurrentLimit(settings.power_level));
    
}



// MAIN LOOP
void loop(){
    printDebug();
    

    // System Checks
    // ! SYSTEM CHECKS ARE SUPRESSED FOR MOTOR TEST BENCH
    // ! UNCOMMENT FOR NOMINAL VEHICLE OPERATION
    // sysCheck->hardware_system_critical(*active_faults, tune); 
    // sysCheck->system_faults(*active_faults, tune);
    // sysCheck->system_limits(*active_limits, tune);
    // sysCheck->system_warnings(*active_warnings, tune);
    
    state = active_faults->size() ?  sendToError(*active_faults->begin()) : state;
    
    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);
    settings.power_level = active_limits->size() ? LIMIT : settings.power_level; // limit power in overheat conditions


    // if(settings.power_level == LIMIT) sendDashPopup(0xA, 5);
    // Serial.println(analogRead(IMD_OK_PIN)*3.3/(1024.0));

    // AMS and IMD LEDs and Dash LEDs
    bool AMS_led = active_faults->find(sysCheck->AMS_fault) != active_faults->end();
    bool IMD_led = active_faults->find(sysCheck->IMD_fault) != active_faults->end();
    TSState = (state == GLV_ON) ? GREEN : RED;
    RTDState = (state == PRECHARGE_COMPLETE) ? GREEN : RED;
    sendDashLED(AMS_led, IMD_led, TSState, RTDState, *Car); // ! Latching?


    // send outgoing CAN Messages
    tryPingRequests({Pedals_Ping_Request, Steering_Wheel_Ping_Request, Dash_Panel_Ping_Request, ACU_Ping_Request}, *Car);
    checkPingTimeout();
    sendPingValues(*Car); 
    sendVDMInfo(sysCheck); 
    
    // Serial.println(Car->ACU1.getSDCVoltage());

    if(can_primary.read(msg)){
        Car->DTI.receive(msg.id, msg.buf);
        Car->ECU.receive(msg.id, msg.buf);
        Car->PEDALS.receive(msg.id, msg.buf);
        Car->ACU1.receive(msg.id, msg.buf);
        Car->TCM1.receive(msg.id, msg.buf);
        Car->DASHBOARD.receive(msg.id, msg.buf);
        Car->ENERGY_METER.receive(msg.id, msg.buf);
        Car->STEERING_WHEEL.receive(msg.id, msg.buf);
        // process incoming CAN Messages    
        handleDashPanelInputs(msg, state, *Car);   
        handleDriverInputs(*tune, settings, msg);
        handlePingResponse(msg);
        handleECUTuning(*tune);
    }
    if(can_data.read(msg2)){
        Car->WFL.receive(msg.id, msg.buf);
        Car->WFR.receive(msg.id, msg.buf);
        Car->WRL.receive(msg.id, msg.buf);
        Car->WRR.receive(msg.id, msg.buf);
        Car->GPS1.receive(msg.id, msg.buf);
    }

    // traction control
    if(mode == DYNAMIC_TC) computeTractionControl();
    if(tc_multiplier < 1) sendDashPopup(0x07, 1);

    // brake light
    if(analogRead(BSE_HIGH) > 500) digitalWrite(BRAKE_LIGHT_PIN, HIGH);
    else digitalWrite(BRAKE_LIGHT_PIN, LOW);

    // state machine operation
    switch (state) {
        // ERROR
        case ERROR:
            state = error(*tune, errorCheck, *active_faults, *Car);
            break;

        // STARTUP 
        case ECU_FLASH:
            state = ecu_flash(tune);
            break;
        case GLV_ON:
            state = glv_on(*Car);
            break;
     
        // PRECHARGE PROCESS
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
            state = drive_standby(BSE_APPS_violation, *tune, *Car); 
            break;
        case DRIVE_ACTIVE:
            state = drive_active(BSE_APPS_violation, *tune, *Car, settings, mode, tc_multiplier);
            break;
        case DRIVE_REGEN:
            state = drive_regen(BSE_APPS_violation, *tune, settings, *Car, sysCheck);
            break;
        // turning off TS
        case TS_DISCHARGE_OFF:
            state = ts_discharge_off(*Car);
    }




    // FOR PRELIMINARY MOTOR TEST BENCH ONLY
    //     if(millis() %10 == 0){

    //     if(motor_on){
    //         Car->DTI.setMaxCurrent(MAX_AMPS);
    //         Car->DTI.setDriveEnable(1);
    //         Car->DTI.setRCurrent(-1*incomingValue/9.0* 100);
    //     }
    //     else {
    //         Car->DTI.setRCurrent(0);
    //         Car->DTI.setDriveEnable(0);
    //     }
    // }
    // if (Serial.available()) {
    //     // Read the incoming byte
    //     incomingValue = Serial.parseInt();
    //     // Perform an action based on the value received (example)
    //     if (incomingValue >= 2 && incomingValue <= 7) {
    //         Serial.print("Requesting -");
    //         Serial.print(incomingValue/7.0 * MAX_AMPS);
    //         Serial.println(" Amps");
    //         motor_on = true;
    //     } else if(incomingValue == 1){
    //         motor_on = false;
    //         Serial.println("Stopping Motor");
    //     }
    // }

    
    
}



