#ifndef GLOBAL_H
#define GLOBAL_H

#define LINEAR_TORQUE 0
#define TORQUE_MAP_1 1
#define TORQUE_MAP_2 2
#define TORQUE_MAP_3 3

// POWER LEVELS
#define LIMIT  0
#define LOW_PWR 1
#define MID_PWR 2
#define HIGH_PWR  3

// REGEN LEVELS
#define REGEN_OFF 0
#define REGEN_LOW 1
#define REGEN_MID 2
#define REGEN_HIGH 3

#define SOFTWARE_OK_CONTROL_PIN 41
#define BRAKE_LIGHT_PIN 4
#define BSPD_OK_PIN 19
#define IMD_OK_PIN 20
#define AMS_OK_PIN 21
#define BSE_HIGH A12
#define CURRENT_SIGNAL A13

#define DTI_COMM_FREQUENCY 100 // Hz
#define PING_REQ_FREQENCY 1 // Hz
#define PING_VALUE_SEND_FREQENCY 10 // Hz
#define VDM_INFO_SEND_FREQENCY 10 // Hz
#define TRACTION_CONTROL_FREQENCY 100 // Hz
#define DEBUG_PRINT_FREQUENCY 4 // Hz
#define DASH_PANEL_LED_FREQUENCY 20 // Hz    
#define PING_TIMEOUT 3000000 // microseconds 




enum State {ECU_FLASH, GLV_ON, TS_PRECHARGE, TS_DISCHARGE_OFF, PRECHARGING, PRECHARGE_COMPLETE, DRIVE_STANDBY, DRIVE_ACTIVE, DRIVE_REGEN, ERROR};
enum Mode {STANDARD, DYNAMIC_TC};

// A struct to represent the current settings of the vehicle determined by the position of inputs on Steering Wheel.
struct SWSettings {
    uint8_t power_level; // 0 - 3
    uint8_t throttle_map; // 0-3
    uint8_t regen_level; // 0-3
    SWSettings(){}  
}; 

enum Color {RED, GREEN, OFF};

#endif