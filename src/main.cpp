
#include "imxrt.h"
#include "Arduino.h"
#include <unordered_set>
#include <cstddef>
#include <sstream>
#include <string>
#include <unordered_map>    
#include <vector>
#include "string"
#include "vehicle.h"
#include "global.h"

#include "SystemsCheck.h"
#include "debug.h"


//varaible declaration
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_primary;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_data;
CAN_message_t msg;
CAN_message_t msg2;
Vehicle *Car = new Vehicle(can_primary, can_data);
SystemsCheck* sysCheck = new SystemsCheck();
State state;
Mode mode;
SWSettings settings;

//ask nikk


bool BSE_APPS_violation = false;
#define SERIAL_BUFFER_SIZE 256;


//no clue about this
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

//GLV STARTUP
void setup() {
    //Car = new Vehicle(can_primary, can_data);
    //sysCheck = new SystemsCheck();  
    //tune = new VehicleTuneController();
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





    // set state  
    state = ECU_FLASH;
    //! FOR TEST ONLY - DELETE LATER
    // state = DRIVE_STANDBY;
    mode = STANDARD; 

    // ! FOR MOTOR TEST BENCH ONLY
    // ! UNCOMMENT FOR NOMINAL VEHICLE OPERATION
    Car->tune->setPowerLevelData(LIMIT, 0);
    Car->tune->setPowerLevelData(LOW_PWR,2);
    Car->tune->setPowerLevelData(MID_PWR, 20);
    Car->tune->setPowerLevelData(HIGH_PWR, 50);

    settings.power_level = HIGH_PWR;
    Car->DTI.setMaxCurrent(Car->tune->getActiveCurrentLimit(settings.power_level));
    
}



// MAIN LOOP
void loop(){
    printDebug(state, mode, sysCheck, settings, Car->tune, Car);
    

    // System Checks
    // ! SYSTEM CHECKS ARE SUPRESSED FOR MOTOR TEST BENCH
    // ! UNCOMMENT FOR NOMINAL VEHICLE OPERATION
    // sysCheck->hardware_system_critical(*active_faults, Car->tune); 
    // sysCheck->system_faults(*active_faults, Car->tune);
    // sysCheck->system_limits(*active_limits, Car->tune);
    // sysCheck->system_warnings(*active_warnings, Car->tune);
    
    state = Car->active_faults->size() ?  sendToError(*Car->active_faults->begin()) : state;
    
    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);
    settings.power_level = Car->active_limits->size() ? LIMIT : settings.power_level; // limit power in overheat conditions


    // if(settings.power_level == LIMIT) sendDashPopup(0xA, 5);
    // Serial.println(analogRead(IMD_OK_PIN)*3.3/(1024.0));

    // AMS and IMD LEDs and Dash LEDs
    bool AMS_led = Car->active_faults->find(sysCheck->AMS_fault) != Car->active_faults->end();
    bool IMD_led = Car->active_faults->find(sysCheck->IMD_fault) != Car->active_faults->end();
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
        handleDriverInputs(*Car->tune, settings, msg);
        handlePingResponse(msg);
        handleECUTuning(*Car->tune);
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
            state = error(*Car->tune, errorCheck, *Car->active_faults, *Car);
            break;

        // STARTUP 
        case ECU_FLASH:
            state = ecu_flash(Car->tune);
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
            state = drive_standby(BSE_APPS_violation, *Car->tune, *Car); 
            break;
        case DRIVE_ACTIVE:
            state = drive_active(BSE_APPS_violation, *Car->tune, *Car, settings, mode, tc_multiplier);
            break;
        case DRIVE_REGEN:
            state = drive_regen(BSE_APPS_violation, *Car->tune, settings, *Car, sysCheck);
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



