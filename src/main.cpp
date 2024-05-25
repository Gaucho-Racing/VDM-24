
#include "imxrt.h"
#include "Arduino.h"
#include "Nodes.h"
#include <unordered_set>
#include <cstddef>
#include "SD.h"
#include <sstream>
#include <string>
#include <unordered_map>    
#include <vector>
#include "string"




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
Inverter DTI = Inverter(22, can_primary);
VDM ECU = VDM(can_primary, can_data);
Wheel WFL = Wheel(can_data, WHEEL_FL);
Wheel WFR = Wheel(can_data, WHEEL_FR);
Wheel WRL = Wheel(can_data, WHEEL_RL);
Wheel WRR = Wheel(can_data, WHEEL_RR);
GPS GPS1 = GPS(can_data);
Pedals PEDALS = Pedals(can_primary);
ACU ACU1 = ACU(can_primary);
TCM TCM1 = TCM(can_data);
Dash DASHBOARD = Dash(can_primary);
Energy_Meter ENERGY_METER = Energy_Meter(can_primary);
SteeringWheel STEERING_WHEEL = SteeringWheel(can_primary);
byte IMU[3][8] = {0x00};
int i = 0;



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
                                                
                                    
*/
// TORQUE MAPPING PROFILES 
const uint8_t LINEAR_TORQUE = 0;
const uint8_t TORQUE_MAP_1 = 1;
const uint8_t TORQUE_MAP_2 = 2;
const uint8_t TORQUE_MAP_3 = 3;

// POWER LEVELS
const uint8_t LIMIT = 0;
const uint8_t LOW_PWR = 1;
const uint8_t MID_PWR = 2;
const uint8_t HIGH_PWR = 3;

// REGEN LEVELS
const uint8_t REGEN_OFF = 0;
const uint8_t REGEN_LOW = 1;
const uint8_t REGEN_MID = 2;
const uint8_t REGEN_HIGH = 3;

struct TorqueProfile{
    float K; // multiplier
    float P; // steepness
    float B; // offset
    TorqueProfile(float k, float p, float b): K(k), P(p), B(b){}
    TorqueProfile(){}
};



/*
Reads the SD card in the Microcontroller to initialize the Vehicles Parameters and Performance Tune with Race Presets.

@param TorqueProfilesData: a reference to vector of 4 TorqueProfile objects
@param PowerLevelsData: a reference to vector of 4 floats representing the current limits for each power level
@param RegenLevelsData: a reference to vector of 4 floats representing the regen levels for each regen level
@param t: a Tune object reference for initializing other vehicle parameters
*/
// void readSDCard(std::vector<TorqueProfile>& TorqueProfilesData, std::vector<float>& PowerLevelsData, std::vector<float>& RegenLevelsData, Tune& t){
//     Serial.println("Initializing SD Card...");
//             while(!SD.begin(BUILTIN_SDCARD)){
//                 Serial.println("Waiting for SD Card to initialize...");
//             }
            
//             Serial.println("SD INITIALIZATION SUCCESSFUL");
//             File ecu_tune;
//             ecu_tune = SD.open("gr24.ecu");
//             Serial.print("Reading ECU FLASH....");
//             String tune;
//             while(ecu_tune.available()){
//                 Serial.print("..");
//                 tune += (char)ecu_tune.read(); 
//             }
//             Serial.println(tune.length());
//             ecu_tune.close();
//             Serial.println("");

//             std::stringstream iss(tune.c_str());
//             // read in torque profiles, regen profiles, and traction profiles
//             for(int i = 0; i < 4; i++){
//                 float k, p, b;
//                 iss >> k >> p >> b;
//                 TorqueProfilesData[i] = TorqueProfile(k, p, b);
//             }   
//             Serial.println("TORQUE PROFILES INITIALIZED");
//             for(int i = 0; i < 4; i++){
//                 float cmax;
//                 iss >> cmax;
//                 PowerLevelsData[i] = cmax;
//             }   
//             Serial.println("CURRENT LIMITS INITIALIZED");
//             for(int i = 0; i < 4; i++){
//                 float r;
//                 iss >> r;
//                 RegenLevelsData[i] = r;
//             }
//             Serial.println("REGEN LEVELS INITIALIZED");
//             Serial.println("ECU FLASH COMPLETE. GR24 TUNE DOWNLOADED.");
//             Serial.println("STARTING CAR WITH SETTINGS: ");
//             Serial.print("THROTTLE MAP: ");
//             for (int i = 0; i < 4; i++) {
//                 Serial.print(TorqueProfilesData[i].K); 
//                 Serial.print(" ");
//                 Serial.print(TorqueProfilesData[i].P);
//                 Serial.print(" ");
//                 Serial.println(TorqueProfilesData[i].B);
//             }
//             Serial.print("POWER LEVELS: ");
//             for (int i = 0; i < 4; i++) {
//                 Serial.print(PowerLevelsData[i]); 
//                 Serial.print(" ");
//             }
//             Serial.println("");
//             Serial.print("REGEN LEVELS: ");
//             for (int i = 0; i < 4; i++) {
//                 Serial.print(RegenLevelsData[i]); 
//                 Serial.print(" ");
//             }
//             Serial.println("");
//             Serial.println("--------------------------");
// }

/*
A class to store the vehicle's performance tune and settings. 
This includes the power limits, torque mappings, regen settings, error limits, and other vehicle parameters.
The tune should be initialized via SD card and can be modified in other functions using the set methods. 
*/
class Tune {
    private:
        std::vector<TorqueProfile> TorqueProfilesData; // Actual Torque profiles with Data for Torque Map
        std::vector<float> PowerLevelsData; // Actual max current value in Amperes
        std::vector<float> RegenLevelsData; // Actual Regen Levels as percentile value 0 to 100

        uint32_t MaxCANPing = 100000; // microseconds for CAN timeout
        uint8_t temp_motor_warn = 60; // degrees celsius for motor warning
        uint8_t temp_motor_limit = 65; // degrees celsius for motor limit
        uint8_t temp_motor_critical = 70; // degrees celsius for motor critical

        uint8_t temp_battery_warn = 60; // degrees celsius for battery warning
        uint8_t temp_battery_limit = 65; // degrees celsius for battery limit
        uint8_t temp_battery_critical = 70; // degrees celsius for battery critical

        uint8_t temp_coolant_warn = 60; // degrees celsius for coolant warning
        uint8_t temp_coolant_limit = 65; // degrees celsius for coolant limit
        uint8_t temp_coolant_critical = 70; // degrees celsius for coolant critical

        uint8_t temp_inverter_warn = 60; // degrees celsius for inverter warning
        uint8_t temp_inverter_limit = 65; // degrees celsius for inverter limit
        uint8_t temp_inverter_critical = 70; // degrees celsius for inverter critical
        uint16_t rev_limit = 5500;// RPM cutoff   

        uint16_t apps_zero_1 = 14070; // ADC value for APPS 1 at 0% throttle
        uint16_t apps_zero_2 = 28440; // ADC value for APPS 2 at 0% throttle
        uint16_t apps_floor_1 = 9965; // ADC value for APPS 1 at 100% throttle
        uint16_t apps_floor_2 = 20280; // ADC value for APPS 2 at 100% throttle

        float max_regen_steering_angle = 0.5; // radians for max regen steering angle
        float regen_rms_amps = 15; // Amperes for regen RMS current
        float regen_dump_amps = 30; // Amperes for regen dump current
        float regen_rms_max_rpm = 2000; // RPM minimum for regen RMS current
        float regen_dump_min_rpm = 3000; // RPM minimum for regen dump current

    public:
        Tune(){
            // init from sd card
            TorqueProfilesData = std::vector<TorqueProfile>(4);
            PowerLevelsData = std::vector<float>(4);
            RegenLevelsData = std::vector<float>(4);
            // readSDCard(TorqueProfilesData, PowerLevelsData, RegenLevelsData, *this);            

        }

        // REGEN STUFF

        // radians for max regen steering angle
        float getMaxRegenSteeringAngle(){ return max_regen_steering_angle; } 
        // Amperes for regen RMS current
        float getRegenRMSAmps(){ return regen_rms_amps; } 
        // Amperes for regen dump current
        float getRegenDumpAmps(){ return regen_dump_amps; } 
        // RPM minimum for regen RMS current
        float getRegenRMSMaxRPM(){ return regen_rms_max_rpm; }
         // RPM minimum for regen dump current 
        float getRegenDumpMinRPM(){ return regen_dump_min_rpm; }

        // set the max regen steering angle
        //@param angle in radians
        void setMaxRegenSteeringAngle(float angle){ max_regen_steering_angle = angle; } 
        // Set the Amperes for regen RMS current
        //@param amps Amperes
        void setRegenRMSAmps(float amps){ regen_rms_amps = amps; }
        // Amperes for regen dump current
        //@param amps Amperes
        void setRegenDumpAmps(float amps){ regen_dump_amps = amps; } 
        // RPM minimum for regen RMS current
        //@param rpm RPM minimum
        void setRegenRMSMaxRPM(float rpm){ regen_rms_max_rpm = rpm; } 
        // RPM minimum for regen dump current
        //@param rpm RPM minimum
        void setRegenDumpMinRPM(float rpm){ regen_dump_min_rpm = rpm; } 


        // APPS CALIBRATION
        // get the ADC value for APPS 1 at 0% throttle
        uint32_t getAPPSZero1(){ return apps_zero_1; } 
        // get the ADC value for APPS 2 at 0% throttle
        uint32_t getAPPSZero2(){ return apps_zero_2; } 
        // get the ADC value for APPS 1 at 100% throttle
        uint32_t getAPPSFloor1(){ return apps_floor_1; } 
        // get the ADC value for APPS 2 at 100% throttle
        uint32_t getAPPSFloor2(){ return apps_floor_2; }

        // set the ADC value for APPS 1 at 0% throttle 
        // @param apps ADC value
        void setAPPSZero1(uint32_t apps){ apps_zero_1 = apps; }
        // ADC value for APPS 2 at 0% throttle
        // @param apps ADC value
        void setAPPSZero2(uint32_t apps){ apps_zero_2 = apps; } 
        // ADC value for APPS 1 at 100% throttle
        // @param apps ADC value
        void setAPPSFloor1(uint32_t apps){ apps_floor_1 = apps; }
        // ADC value for APPS 2 at 100% throttle
        // @param apps ADC value
        void setAPPSFloor2(uint32_t apps){ apps_floor_2 = apps; } 
        
        // ERROR THRESHOLDS
        // get the maximum CAN ping time in microseconds
        uint8_t getMaxCANPing(){ return MaxCANPing; }
        // get the motor warning temperature in degrees celsius
        uint8_t getMotorWarnTemp(){ return temp_motor_warn; }
        // get the motor limit temperature in degrees celsius
        uint8_t getMotorLimitTemp(){ return temp_motor_limit; }
        // get the motor critical temperature in degrees celsius
        uint8_t getMotorCriticalTemp(){ return temp_motor_critical; }
        // get the battery warning temperature in degrees celsius
        uint8_t getBatteryWarnTemp(){ return temp_battery_warn; }
        // get the battery limit temperature in degrees celsius
        uint8_t getBatteryLimitTemp(){ return temp_battery_limit; }
        // get the battery critical temperature in degrees celsius
        uint8_t getBatteryCriticalTemp(){ return temp_battery_critical; }
        // get the coolant warning temperature in degrees celsius
        uint8_t getCoolantWarnTemp(){ return temp_coolant_warn; }
        // get the coolant limit temperature in degrees celsius
        uint8_t getCoolantLimitTemp(){ return temp_coolant_limit; }
        // get the coolant critical temperature in degrees celsius
        uint8_t getCoolantCriticalTemp(){ return temp_coolant_critical; }
        // get the inverter warning temperature in degrees celsius
        uint8_t getInverterWarnTemp(){ return temp_inverter_warn; }
        // get the inverter limit temperature in degrees celsius
        uint8_t getInverterLimitTemp(){ return temp_inverter_limit; }
        // get the inverter critical temperature in degrees celsius
        uint8_t getInverterCriticalTemp(){ return temp_inverter_critical; }


        // set the maximum CAN ping time in microseconds
        // @param ping round trip delay in microseconds
        void setMaxCANPing(uint32_t ping){ MaxCANPing = ping; }
        // set the motor warning temperature in degrees celsius
        // @param temp degrees celsius
        void setMotorWarnTemp(uint8_t temp){ temp_motor_warn = temp; }
        // set the motor limit temperature in degrees celsius
        // @param temp degrees celsius
        void setMotorLimitTemp(uint8_t temp){ temp_motor_limit = temp; }
        // set the motor critical temperature in degrees celsius
        // @param temp degrees celsius
        void setMotorCriticalTemp(uint8_t temp){ temp_motor_critical = temp; }
        // set the battery warning temperature in degrees celsius
        // @param temp degrees celsius
        void setBatteryWarnTemp(uint8_t temp){ temp_battery_warn = temp; }
        // set the battery limit temperature in degrees celsius
        // @param temp degrees celsius
        void setBatteryLimitTemp(uint8_t temp){ temp_battery_limit = temp; }
        // set the battery critical temperature in degrees celsius
        // @param temp degrees celsius
        void setBatteryCriticalTemp(uint8_t temp){ temp_battery_critical = temp; }
        // set the coolant warning temperature in degrees celsius
        // @param temp degrees celsius
        void setCoolantWarnTemp(uint8_t temp){ temp_coolant_warn = temp; }
        // set the coolant limit temperature in degrees celsius
        // @param temp degrees celsius
        void setCoolantLimitTemp(uint8_t temp){ temp_coolant_limit = temp; }
        // set the coolant critical temperature in degrees celsius
        // @param temp degrees celsius
        void setCoolantCriticalTemp(uint8_t temp){ temp_coolant_critical = temp; }
        // set the inverter warning temperature in degrees celsius
        // @param temp degrees celsius
        void setInverterWarnTemp(uint8_t temp){ temp_inverter_warn = temp; }
        // set the inverter limit temperature in degrees celsius
        // @param temp degrees celsius
        void setInverterLimitTemp(uint8_t temp){ temp_inverter_limit = temp; }
        // set the inverter critical temperature in degrees celsius
        // @param temp degrees celsius
        void setInverterCriticalTemp(uint8_t temp){ temp_inverter_critical = temp; }

        // get the active torque profile for a given position in the vehicles Tune (as K, P, B)
        // @param pos position of the torque profile corr. to SW
        TorqueProfile getActiveTorqueProfile(int8_t pos){ return TorqueProfilesData[pos]; }      
        // get the active current limit for a given position in the vehicles Tune (in AMPS)
        // @param pos position of the current limit corr. to SW  
        float getActiveCurrentLimit(int8_t pos){ return PowerLevelsData[pos];} 
        // get the active regen power for a given position in the vehicles Tune (in percentile)
        // @param pos position of the regen power corr. to SW
        float getActiveRegenPower(int8_t pos){ return RegenLevelsData[pos];} 
        // get rev limiter cuttoff
        int revLimit(){ return rev_limit; } 
        // set the Torque Profile for a given position in the vehicles Tune
        // @param index position of the torque profile corr. to SW
        // @param tp TorqueProfile object
        void setTorqueProfileData(uint8_t index, TorqueProfile tp){ TorqueProfilesData[index] = tp; }
        // set the Power Level for a given position in the vehicles Tune
        // @param index position of the current limit corr. to SW
        // @param power float value in Amperes
        void setPowerLevelData(uint8_t index, float power){ PowerLevelsData[index] = power; }
        // set the Regen Level for a given position in the vehicles Tune
        // @param index position of the regen power corr. to SW
        // @param regen float value in percentile
        void setRegenLevelData(uint8_t index, float regen){ RegenLevelsData[index] = regen; }
};




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
// PIN DEFINITIONS
const uint8_t SOFTWARE_OK_CONTROL_PIN = 41;
const uint8_t BRAKE_LIGHT_PIN = 4;
const uint8_t BSPD_OK_PIN = 19;
const uint8_t IMD_OK_PIN = 20;
const uint8_t AMS_OK_PIN = 21;
const uint8_t BSE_HIGH = A16;
const uint8_t CURRENT_SIGNAL = A13;



// error severity: warning -> limit -> critical

// A class to statically check for system faults and warnings and gives dynamic CAN frames using bit masking. 
class SystemsCheck {
    private:    

        byte SYS_CHECK_CAN_FRAME[8]; // 8 bytes of system checks


    public:
        SystemsCheck(){
            for(int i = 0; i < 5; i++) SYS_CHECK_CAN_FRAME[i] = 0x0;
        }

        byte* getSysCheckFrame(){
            return SYS_CHECK_CAN_FRAME;
            /*
            8 bytes of 8 bits:
            [can warn][can failure][AMS][IMD][BSPD][SDC][][]
            [warn motor][limit motor][crit motor][warn batt][limit batt][crit batt][rev limit][] temps motor and battery, Revs
            [warn water][limit water][crit water][warn mcu][limit mcu][crit mcu][TCM Status][] // water temp DTI temp, TCM
            [][][][][][][][] 
            [][][][][][][][] 
            */
        }
        
        void hardware_system_critical(std::unordered_set<bool (*)(Tune& t)> &af, Tune* t){
            if(SDC_opened(*t)) af.insert(SDC_opened);  
            SYS_CHECK_CAN_FRAME[0] = SDC_opened(*t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
            if(AMS_fault(*t)) af.insert(AMS_fault);
            SYS_CHECK_CAN_FRAME[0] = AMS_fault(*t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00100000) : (SYS_CHECK_CAN_FRAME[0] & 0b11011111);
            if(IMD_fault(*t)) af.insert(IMD_fault);
            SYS_CHECK_CAN_FRAME[0] = IMD_fault(*t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00010000) : (SYS_CHECK_CAN_FRAME[0] & 0b11101111);
            if(BSPD_fault(*t)) af.insert(BSPD_fault);
            SYS_CHECK_CAN_FRAME[0] = BSPD_fault(*t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
            // if(max_current(Car)) af.insert(max_current);
            // SYS_CHECK_CAN_FRAME[0] = max_current(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000010) : (SYS_CHECK_CAN_FRAME[0] & 0b11111101);
        }

        // NOTE: OPEN THE SOFTWARE LATCH IF the Inverter is not responding or there are critical system faults. 
        void system_faults(std::unordered_set<bool (*)(Tune& t)> &af, Tune* t){
            if(critical_motor_temp(*t)) af.insert(critical_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_motor_temp(*t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00100000) : (SYS_CHECK_CAN_FRAME[1] & 0b11011111);
            if(critical_battery_temp(*t)) af.insert(critical_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_battery_temp(*t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000100) : (SYS_CHECK_CAN_FRAME[1] & 0b11111011);
            // if(critical_water_temp(*t)) af.insert(critical_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = critical_water_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00100000) : (SYS_CHECK_CAN_FRAME[2] & 0b11011111);
            if(critical_mcu_temp(*t)) af.insert(critical_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = critical_mcu_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00000100) : (SYS_CHECK_CAN_FRAME[2] & 0b11111011);
            // if(critical_can_failure(Car)) af.insert(critical_can_failure);
            // SYS_CHECK_CAN_FRAME[0] = critical_can_failure(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b01000000) : (SYS_CHECK_CAN_FRAME[0] & 0b10111111);
            
        }

        void system_limits(std::unordered_set<bool (*)(Tune& t)> &al, Tune* t){
            if(limit_motor_temp(*t)) al.insert(limit_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_motor_temp(*t) ? (SYS_CHECK_CAN_FRAME[1] | 0b01000000) : (SYS_CHECK_CAN_FRAME[1] & 0b10111111);
            if(limit_battery_temp(*t)) al.insert(limit_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_battery_temp(*t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00001000) : (SYS_CHECK_CAN_FRAME[1] & 0b11110111);
            // if(limit_water_temp(*t)) al.insert(limit_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = limit_water_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b01000000) : (SYS_CHECK_CAN_FRAME[2] & 0b10111111);
            if(limit_mcu_temp(*t)) al.insert(limit_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = limit_mcu_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00001000) : (SYS_CHECK_CAN_FRAME[2] & 0b11110111);
        }

        void system_warnings(std::unordered_set<bool (*)(Tune& t)> &aw, Tune* t){
            if(warn_motor_temp(*t)) aw.insert(warn_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_motor_temp(*t) ? (SYS_CHECK_CAN_FRAME[1] | 0b10000000) : (SYS_CHECK_CAN_FRAME[1] & 0b01111111);
            if(warn_battery_temp(*t)) aw.insert(warn_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_battery_temp(*t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00010000) : (SYS_CHECK_CAN_FRAME[1] & 0b11101111);
            // if(warn_water_temp(*t)) aw.insert(warn_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = warn_water_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b10000000) : (SYS_CHECK_CAN_FRAME[2] & 0b01111111);
            if(warn_mcu_temp(*t)) aw.insert(warn_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = warn_mcu_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00010000) : (SYS_CHECK_CAN_FRAME[2] & 0b11101111);
            if(rev_limit_exceeded(*t)) aw.insert(rev_limit_exceeded);
            SYS_CHECK_CAN_FRAME[1] = rev_limit_exceeded(*t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000010) : (SYS_CHECK_CAN_FRAME[1] & 0b11111101);
        }




        // BYTE 0 ---------------------------------------------------------------------------
        // bit 0

        // bool warn_can_failure(const ){ 
        //     // return Pedals_Ping > 1000000 || ACU_Ping > 1000000 || /*BCM_Age > 1000000 || */ DashPanel_Ping > 1000000 || SteeringWheel_Ping > 1000000 /*|| DTI_Age > 1000000*/;
        //     return false;
        // }   
        // // bit 1
        // bool critical_can_failure(const ){
        //     return false;
        // }



        // HARDWARE FAULTS: VERY BAD
        // .5v is shit -  ADC: 155
        // 3v when almost ok - ADC: 930
        // 2.4v is ok - ADC: 744
        // 1v = 310
        // bits 2, 3, 4, 5, 
        static bool AMS_fault(Tune& t){ return analogRead(AMS_OK_PIN) < 700 || analogRead(AMS_OK_PIN) > 790; }
        static bool IMD_fault(Tune& t){ return analogRead(IMD_OK_PIN) < 700 || analogRead(IMD_OK_PIN) > 790; }
        static bool BSPD_fault(Tune& t){ return analogRead(BSPD_OK_PIN) < 700 || analogRead(BSPD_OK_PIN) > 790; }
        // check voltage < 7V (this one is 16V 8 bit ADC)
        static bool SDC_opened(Tune& t){ return ACU1.getSDCVoltage() < 112; } 
        // bit 6
        // bool SystemsCheck::max_current(const ){return DTI.getDCCurrent() > DTI.getDCCurrentLim();} 

        // BYTE 1 ---------------------------------------------------------------------------
        // bit 0, 1, 2
        static bool warn_motor_temp(Tune& t){return DTI.getMotorTemp() > t.getMotorWarnTemp() && DTI.getMotorTemp() < t.getMotorLimitTemp();}
        static bool limit_motor_temp(Tune& t){return DTI.getMotorTemp() > t.getMotorLimitTemp() && DTI.getMotorTemp() < t.getMotorCriticalTemp();}
        static bool critical_motor_temp(Tune& t){return DTI.getMotorTemp() > t.getMotorCriticalTemp();}
        // bit 3, 4, 5
        static bool warn_battery_temp(Tune& t) {return ACU1.getMaxCellTemp() > t.getBatteryWarnTemp() && ACU1.getMaxCellTemp() < t.getBatteryLimitTemp();}
        static bool limit_battery_temp(Tune& t) {return ACU1.getMaxCellTemp() > t.getBatteryLimitTemp() && ACU1.getMaxCellTemp() < t.getBatteryCriticalTemp();}
        static bool critical_battery_temp(Tune& t) {return ACU1.getMaxCellTemp() > t.getBatteryCriticalTemp();}
        // bit 6
        static bool rev_limit_exceeded(Tune& t) {return DTI.getERPM()/10 > t.revLimit();}
        // bit 7 

        // BYTE 2 ---------------------------------------------------------------------------
        // bit 0, 1, 2 NOTE: COOLANT TEMP SENSOR NOT FUNCTIONAL
        // static bool warn_water_temp(Tune& t){return ACU1.get() > t.getBatteryWarnTemp() && .getWaterTemp() < t.getCoolantLimitTemp();} //TODO: Implement in NODES
        // static bool limit_water_temp(Tune& t){return ACU1.getWaterTemp() > t.getCoolantLimitTemp() && ACU1.getWaterTemp() < t.getCoolantCriticalTemp();}
        // static bool critical_water_temp(Tune& t){return ACU1.getWaterTemp() > t.getCoolantCriticalTemp();}
        // bit 3, 4, 5
        static bool warn_mcu_temp(Tune& t) {return DTI.getInvTemp() > t.getInverterWarnTemp() && DTI.getInvTemp() < t.getInverterLimitTemp();}
        static bool limit_mcu_temp(Tune& t){return DTI.getInvTemp() > t.getInverterLimitTemp() && DTI.getInvTemp() < t.getInverterCriticalTemp();}
        static bool critical_mcu_temp(Tune& t) {return DTI.getInvTemp() > t.getInverterCriticalTemp();}
        // bit 6
        // static bool TCM_fault(Tune& t) {return false;} // TODO: do
        // bit 7 empty for now


};





/*
   ________    ____  ____  ___    __ 
  / ____/ /   / __ \/ __ )/   |  / / 
 / / __/ /   / / / / __  / /| | / /  
/ /_/ / /___/ /_/ / /_/ / ___ |/ /___
\____/_____/\____/_____/_/  |_/_____/
*/
enum State {ECU_FLASH, GLV_ON, TS_PRECHARGE, TS_DISCHARGE_OFF, PRECHARGING, PRECHARGE_COMPLETE, DRIVE_STANDBY, DRIVE_ACTIVE, DRIVE_REGEN, ERROR};
enum Mode {STANDARD, DYNAMIC_TC};
// A struct to represent the current settings of the vehicle determined by the position of inputs on Steering Wheel.
struct SWSettings {
    uint8_t power_level; // 0 - 3
    uint8_t throttle_map; // 0-3
    uint8_t regen_level; // 0-3
    SWSettings(){}  
};  

// iCANflex* Car; // Controller Area Network Object for the Vehicles CAN Bus
SystemsCheck* sysCheck; // Static System Check
Tune* tune; // Global System Tuning Profile

// THIS HASH FUNCTION DOES NOT WORK: DO NOT USE
// namespace std { 
//     template <>
//     struct hash<bool (*)(Tune& t)> {
//         size_t operator()(bool (*f)(Tune& t)) const noexcept{
//             return reinterpret_cast<size_t>(f);
//         }
//     };
// };

std::unordered_set<bool (*)(Tune& t)> *active_faults; // hashset of active system faults as function pointers
std::unordered_set<bool (*)(Tune& t)> *active_warnings; // hashset of active system warnings as function pointers
std::unordered_set<bool (*)(Tune& t)> *active_limits; // hashset of active system limits as function pointers
bool (*errorCheck)(Tune&); // global function pointer to error causing the ISR

std::unordered_set<int> timeout_nodes;

bool BSE_APPS_violation = false;
State state;
Mode mode;
SWSettings settings;

const uint8_t DTI_COMM_FREQUENCY = 100; // Hz
const uint8_t PING_REQ_FREQENCY = 1; // Hz
const uint8_t PING_VALUE_SEND_FREQENCY = 10; // Hz
const uint8_t VDM_INFO_SEND_FREQENCY = 10; // Hz
const uint8_t TRACTION_CONTROL_FREQENCY = 100; // Hz
const uint8_t DEBUG_PRINT_FREQUENCY = 4; // Hz
const uint8_t DASH_PANEL_LED_FREQUENCY = 10; // Hz    

const unsigned long PING_TIMEOUT = 3000000; // microseconds 


unsigned long lastPrechargeTime = 0; // last precharge request in millis
unsigned long lastDTIMessage = 0; // last inverter message in millis    
unsigned long lastDashLEDMessage = 0; // last dash panel led message in millis
unsigned long lastPingSend = 0; // last send on 0xF2 in millis
unsigned long lastPingRequestAttempt = 0; // last request for all Pings in millis


enum Color {RED, GREEN, OFF};// green means press, red means dont press
Color TSState = GREEN;
Color RTDState = RED;


#define SERIAL_BUFFER_SIZE 256;




/*
   _________    _   __   __________  __  _____  _____  ___   _______________  ______________  _   __
  / ____/   |  / | / /  / ____/ __ \/  |/  /  |/  / / / / | / /  _/ ____/   |/_  __/  _/ __ \/ | / /
 / /   / /| | /  |/ /  / /   / / / / /|_/ / /|_/ / / / /  |/ // // /   / /| | / /  / // / / /  |/ / 
/ /___/ ___ |/ /|  /  / /___/ /_/ / /  / / /  / / /_/ / /|  // // /___/ ___ |/ / _/ // /_/ / /|  /  
\____/_/  |_/_/ |_/   \____/\____/_/  /_/_/  /_/\____/_/ |_/___/\____/_/  |_/_/ /___/\____/_/ |_/   
                                                                                                    
*/


void writeMessage(unsigned int id, uint8_t* data, unsigned char len){
    CAN_message_t message;
    message.flags.extended = true;
    message.id = id;
    message.len = len;
    memcpy(message.buf, data, len);
    can_primary.write(message);
}



void sendDashPopup(int8_t error_code, int8_t secs, uint8_t tq = 0, uint16_t mc = 0, uint8_t r = 0){
    // TODO:
}


void handleECUTuning(Tune& tune){
    // TODO:
}



void handleDriverInputs(Tune& tune){
    if(msg.id == 0x11002){
        settings.power_level = msg.buf[0];
        settings.throttle_map = msg.buf[1];
        settings.regen_level = msg.buf[2];
        sendDashPopup(0x9, 5, settings.throttle_map, tune.getActiveCurrentLimit(settings.power_level), settings.regen_level);
    }
}


void sendVDMInfo(){
    // TODO:
    byte* sys_check_data = sysCheck->getSysCheckFrame();
}

/*
Sends a message to the Dash Panel to update the LED status of the buttons and warning lights.
@param AMS - 0: OFF, 1: ON for LED 
@param IMD - 0: OFF, 1: ON for LED
@param TSColor - Color to set TS active Button LED
@param RTDColor - Color to set RTD active Button LED
*/
void sendDashLED(uint8_t AMS, uint8_t IMD, Color TSColor, Color RTDColor){
    if(millis()- lastDashLEDMessage >= 1000/DASH_PANEL_LED_FREQUENCY){
        uint8_t tsr = TSColor == RED ? 255 : 0;
        uint8_t tsg = TSColor == GREEN ? 355 : 0;
        uint8_t rtr = RTDColor == RED ? 255 : 0;    
        uint8_t rtg = RTDColor == GREEN ? 255 : 0;
        uint8_t data[8] = {AMS, IMD, tsr, tsg, rtr, rtg, 0, 0};
        writeMessage(LED_Outputs, data, 8);
        lastDashLEDMessage = millis();  
    }

}




void handleDashPanelInputs(){
    float brake = analogRead(BSE_HIGH); // TODO: Fix
    if(msg.id == Button_Event){
        if(msg.buf[0]){ // TS_ACTIVE
            // if(brake < 1000) {
            //     sendDashPopup(0x3, 3);
            //     return;
            // }
            Serial.println("Got TS Active");
            State s = state;
            if(s == GLV_ON){
                    state = TS_PRECHARGE;
                    CAN_message_t message;
                    message.flags.extended = true;
                    message.id = ACU_Control;
                    message.len = 8;
                    message.buf[0] = 1;
                    can_primary.write(message);                
            }
        }
        else if(msg.buf[1]){ // TS_OFF
            Serial.println("Got TS Off");
            // shut off car entirely
            CAN_message_t message;
            message.flags.extended = true;
            message.id = ACU_Control;
            message.len = 8;
            message.buf[0] = 0;
            can_primary.write(message);
            state = TS_DISCHARGE_OFF;
        }
        else if(msg.buf[2]) { // RTD_ON
            // if(brake < 1000){
            //     sendDashPopup(0x3, 3);
            //     return;
            // }
            if(state == PRECHARGE_COMPLETE){
                state = DRIVE_STANDBY;
                //TODO: play rtd sound
            }
        }
        else if(msg.buf[3]){  // RTD_OFF
            if(state == DRIVE_STANDBY) {
                state = PRECHARGE_COMPLETE;
            }
        }
    }
}


// PING LOGIC

// response times as {id, time} in microseconds
static std::unordered_map<int, unsigned long> ping_response_times = { // TODO: BCM, TCM
    {ACU_Ping_Response, 0},
    {Pedals_Ping_Response, 0},
    {Steering_Wheel_Ping_Response, 0},
    {Dash_Panel_Ping_Response, 0}
};
// last response time as {id, time} in milliseconds
static std::unordered_map<int, unsigned long> last_response_times = {
    {ACU_Ping_Response, 0},
    {Pedals_Ping_Response, 0},
    {Steering_Wheel_Ping_Response, 0},
    {Dash_Panel_Ping_Response, 0}
};
// node numbers as {id, number}
static std::unordered_map<int, int> node_numbers = {
    {ACU_Ping_Response, 1},
    {Pedals_Ping_Response, 2},
    {Steering_Wheel_Ping_Response, 3},
    {Dash_Panel_Ping_Response, 4}
}; // TODO: Fix this shit


// Will try to send a ping request to the nodes in the list of request IDs
// @param request_ids: list of request ids to request a ping response from
// @param Car: iCANflex object defined by GR 24 Nodes API
void tryPingRequests(std::vector<uint32_t> request_ids){
    if(millis()-lastPingRequestAttempt > 1000/PING_REQ_FREQENCY){
        // Serial.println("Sending Ping Requests");
        for(uint32_t request_id : request_ids){
            unsigned long mills=millis();
            unsigned long micro=micros();
            byte data[8] = {0x00};
            for(int i=0; i<4; i++){
                data[3-i]=(byte)(mills >> (i*8));
                data[7-i]=(byte)(micro >> (i*8));
            }
            writeMessage(request_id, data, 8);
            lastPingRequestAttempt=millis();
        }
    }   
}

unsigned long calculatePing() {
    unsigned long newTime = (long)msg.buf[3] + ((long)msg.buf[2] << 8) + ((long)msg.buf[1] << 16) + ((long)msg.buf[0] << 24);
    unsigned long newTime2 = (long)msg.buf[7] + ((long)msg.buf[6] << 8) + ((long)msg.buf[5] << 16) + ((long)msg.buf[4] << 24);
    unsigned long roundTripDelay = ((millis() - newTime) * 1000) + ((micros() - newTime2) % 1000);
    return roundTripDelay;
}

void handlePingResponse(){
    if(msg.id == ACU_Ping_Response || msg.id == Pedals_Ping_Response || msg.id == Steering_Wheel_Ping_Response || msg.id == Dash_Panel_Ping_Response){
        ping_response_times[msg.id] = calculatePing();
        last_response_times[msg.id] = micros();
    }
}

void sendPingValues(){
    if(millis()-lastPingSend > 1000/PING_VALUE_SEND_FREQENCY){
        for(auto e : ping_response_times){
            CAN_message_t message;
            message.buf[0] = node_numbers[e.first];
            for(int j=0; j<4; j++){
                message.buf[4-j]=(byte)(e.second >> (j*8));
            }
            message.len = 8;
            message.id = VDM_Ping_Values;
            can_primary.write(message);
            // memcpy(msg.buf, 0x0, 8); // clear buffer
        }
        lastPingSend = millis();
    }
}

void checkPingTimeout(){
    for(auto e : last_response_times){
        if(micros() - e.second > PING_TIMEOUT){
            timeout_nodes.insert(e.first);
        }
        else{
            if(timeout_nodes.find(e.first) != timeout_nodes.end()){
                timeout_nodes.erase(e.first);
            }
        }
    }

}



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
        float rearLeftWheelSpeed = WRL.getWheelSpeed();
        float rearRightWheelSpeed = WRR.getWheelSpeed();
        float frontLeftWheelSpeed = WFL.getWheelSpeed();
        float frontRightWheelSpeed = WFR.getWheelSpeed();

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
   ______________  ____________   __  ______   ________  _______   ________
  / ___/_  __/   |/_  __/ ____/  /  |/  /   | / ____/ / / /  _/ | / / ____/
  \__ \ / / / /| | / / / __/    / /|_/ / /| |/ /   / /_/ // //  |/ / __/   
 ___/ // / / ___ |/ / / /___   / /  / / ___ / /___/ __  // // /|  / /___   
/____//_/ /_/  |_/_/ /_____/  /_/  /_/_/  |_\____/_/ /_/___/_/ |_/_____/   
    ____  ____  __________  ___  ______________  _   __
  / __ \/ __ \/ ____/ __ \/   |/_  __/  _/ __ \/ | / /
 / / / / /_/ / __/ / /_/ / /| | / /  / // / / /  |/ / 
/ /_/ / ____/ /___/ _, _/ ___ |/ / _/ // /_/ / /|  /  
\____/_/   /_____/_/ |_/_/  |_/_/ /___/\____/_/ |_/   
                                                                                                                            
*/



State sendToError(bool (*erFunc)(Tune& tune)) {
   errorCheck = erFunc; 
   return ERROR;
}
/*

STARTUP STAGE 1:
ECU FLASH
When the Car is turned on, the Main ECU will read in the ECU flash from the SD card.
This will be the first state that the car will enter.
This is essential for the car to operate as the ECU flash contains the 
torque profiles, regen profiles, and traction control profiles.
*/
State ecu_flash() {
    // DTI.setDriveEnable(0);
    // DTI.setRCurrent(0);
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
State glv_on() {
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setRCurrent(0);
        DTI.setDriveEnable(0);
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
State ts_precharge() { 
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setRCurrent(0);
        DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }
    if(ACU1.getPrecharging()){
        return PRECHARGING;
    }
    else if(ACU1.getPrechargeDone()){
        return PRECHARGE_COMPLETE;
    }
    return TS_PRECHARGE;
}
// -- PRECHARGING STAGE 2
State precharging(){
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setRCurrent(0);
        DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    if(ACU1.getPrechargeDone()) return PRECHARGE_COMPLETE;
    return PRECHARGE_COMPLETE;
}

// -- PRECHARGING STAGE 3
State precharge_complete(){
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setRCurrent(0);
        DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }
    // wait for RTD signal
    return PRECHARGE_COMPLETE;  
}


/*
STARTUP STAGE 4:  READY TO DRIVE

READY TO DRIVE SUB STATES


*/




float getThrottle1(uint16_t a1, Tune& tune){
    float throttle =  1.0 - ((a1 - tune.getAPPSFloor1()*1.0)/(tune.getAPPSZero1()-tune.getAPPSFloor1()));
    if (throttle < 0.05) return 0;
    else if (throttle > 1 && throttle < 1.1) return 1;
    else if (throttle > 1.1) return 0;
    else return throttle;
}

float getThrottle2(uint16_t a2,  Tune& tune){
    float throttle =  1.0 - ((a2 - tune.getAPPSFloor2()*1.0)/(tune.getAPPSZero2()-tune.getAPPSFloor2()));
    if (throttle < 0.05) return 0;
    else if (throttle > 1 && throttle < 1.1) return 1;
    else if (throttle > 1.1) return 0;
    else return throttle;
}

State drive_standby(bool& BSE_APPS_violation, Tune& tune) {
    
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setRCurrent(0);
        DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    float throttle = getThrottle1(PEDALS.getAPPS1(), tune);
    float brake = analogRead(CURRENT_SIGNAL); //TODO: Fix
    
    // only if no violation, and throttle is pressed, go to DRIVE

    if(!BSE_APPS_violation && throttle > 0.05) return DRIVE_ACTIVE;
    if(!BSE_APPS_violation && throttle == 0 && brake > 0.05 && DTI.getERPM() > 250) return DRIVE_REGEN;//TODO: Fix

    if(BSE_APPS_violation) {
        // SEND CAN WARNING TO DASH
        sendDashPopup(0x01, 3);
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
State drive_active(bool& BSE_APPS_violation, Tune& tune) {
    float throttle = getThrottle1(PEDALS.getAPPS1(), tune);
    float a2 = getThrottle2(PEDALS.getAPPS2(), tune);
    float brake = (PEDALS.getBrakePressureF() + PEDALS.getBrakePressureR())/2.0; // TODO: All the Braking stuff is wrong
    
    // ACCELERATOR GRADIENT PLAUSIBILITY VIOLATION
    if(abs(throttle-a2) > 0.1){
        sendDashPopup(0x02, 3);
        return DRIVE_STANDBY;
    } 
    // APPS X BSE VIOLATION
    if((brake > 0.05 && throttle > 0.25)) {
        sendDashPopup(0x01, 1);
        BSE_APPS_violation = true;
        return DRIVE_STANDBY; // Put car into neutral state, no engine power
    }
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setDriveEnable(1);
        DTI.setMaxCurrent(tune.getActiveCurrentLimit(settings.power_level));  
        // TORQUE MAPPING FOR DRIVING AND STABILITY VIA NONLINEAR THROTTLE CONTROL
        // THROTTLE CURVE EQUATION: z = np.clip((x - (1-x)*(x + b)*((y/5500.0)**p)*k )*100, 0, 100) 
        TorqueProfile tp = tune.getActiveTorqueProfile(settings.throttle_map);
        float k = tp.K;
        float p = tp.P;
        float b = tp.B;
        float rpm = DTI.getERPM()/10.0;
        float torque_multiplier = (throttle-(1-throttle)*(throttle+b)*pow(rpm/tune.revLimit(), p)*k);
        if(torque_multiplier > 1) torque_multiplier = 1; // clipping
        if(torque_multiplier < 0) torque_multiplier = 0;
        float r_current = torque_multiplier*100;
        if(settings.throttle_map == LINEAR_TORQUE) r_current = throttle*100;
        if(mode == DYNAMIC_TC) r_current *= tc_multiplier;
        DTI.setRCurrent(r_current);
        lastDTIMessage = millis();
    }
    return DRIVE_ACTIVE; // stay in the drive state
}


State drive_regen(bool& BSE_APPS_violation, Tune& tune){
    if(settings.regen_level == REGEN_OFF) return DRIVE_STANDBY;

    float brake = analogRead(CURRENT_SIGNAL); // TODO: Change to BSE 
    float throttle = getThrottle1(PEDALS.getAPPS1(), tune);
    if(throttle > 0.05) return DRIVE_ACTIVE;
    if(brake < 0.05) return DRIVE_STANDBY;

    float rpm = DTI.getERPM()/10.0;
    
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setDriveEnable(1);
        DTI.setMaxCurrent(tune.getActiveCurrentLimit(settings.power_level));
        // Do this one in AMPS instead of Relative Current
        // 30A Max Regen, 15A Continuous/RMS
        float accumulator_input_amps = 0; 
        float steering_angle = 0; //TODO: in nodes
        // must be > 5 kph
        bool regen_ok = ACU1.getSOC() < 85 && rpm > 250 && brake > 0.05 && throttle == 0 && abs(steering_angle) < tune.getMaxRegenSteeringAngle();
        bool max_regen_ok = regen_ok && rpm > tune.getRegenDumpMinRPM() && brake > 0.75 && !sysCheck->warn_battery_temp(tune) && !sysCheck->limit_battery_temp(tune);
        if(max_regen_ok) {// make sure revs are high enough for significant backemf
            // only for hard braking requests, dump energy back into accumulator
            accumulator_input_amps = tune.getRegenDumpAmps();
        } else if(regen_ok){
            // 250 - 2000 RPM interpolate from 0 to 15A based on RPM
            // 2000 - 3000 RPM is 15A 
            if (rpm < tune.getRegenRMSMaxRPM()) accumulator_input_amps = 15*(rpm-250)/(tune.getRegenRMSMaxRPM()-250);
            else accumulator_input_amps = tune.getRegenRMSAmps(); // TODO: Put this interpolation diagram in the tuning software
        } else {
            accumulator_input_amps = 0;
        }

        DTI.setBrakeCurrent(-1 * accumulator_input_amps * tune.getActiveRegenPower(settings.regen_level));
        lastDTIMessage = millis();
    }
    return DRIVE_REGEN;
}

/*
ERROR STATE

THIS STATE WILL HANDLE ERRORS THAT OCCUR DURING THE OPERATION OF THE VEHICLE.
THIS STATE WILL BE ENTERED WHENEVER A CRITICAL SYSTEMS FAILURE OCCURS OR WHEN THE
DRIVER REQUESTS TO STOP THE VEHICLE.

THE VEHICLE REMAINS IN THIS STATE UNTIL ALL VIOLATIONS ARE RESOLVED 

*/
State error(Tune& t, bool (*errorCheck)(Tune& t), std::unordered_set<bool (*)(Tune& t)>& active_faults){
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setRCurrent(0);
        DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    if(errorCheck(t))  return ERROR;
    else {
        active_faults.erase(errorCheck);
        return GLV_ON; // gets sent back to error from main() if there are more in the hashset from main
    }
    
}

State ts_discharge_off(){
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        DTI.setRCurrent(0);
        DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }
    if(ACU1.getTSVoltage() < 60) return GLV_ON;
    return TS_DISCHARGE_OFF;

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
    output += "| CRITICAL: " + String(active_faults->size()) + " | LIMIT: " + String(active_limits->size()) + " | WARN: " + String(active_warnings->size()) + "          |\n";
    output += "----------------------------------------------------------";
    return output;
}

String vehicleNetwork(){
    String output = "|          NETWORK SPEED: (microseconds)                 |\n";
    output += "| ACU: " + String(ping_response_times[ACU_Ping_Response]) + " | Pedals: " + String(ping_response_times[Pedals_Ping_Response]) + " | Steering: " + String(ping_response_times[Steering_Wheel_Ping_Response]) + " | DashPanel: " + String(ping_response_times[Dash_Panel_Ping_Response]) + "  |\n";
    output += "| UNRESPONSIVE NODES: " + String(timeout_nodes.size()) + "                                  |\n";
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
    int raw1 = PEDALS.getAPPS1();
    output += "| APPS1: RAW: " + String(raw1) + ", SCALED: " + String(getThrottle1(raw1, *tune)) + "               \n";
    int raw2 = PEDALS.getAPPS2();
    output += "| APPS2: RAW: " + String(raw2) + ", SCALED: " + String(getThrottle2(raw2, *tune)) + "               \n";
    output += "| INVERTER CURRENT LIMIT: " + String(tune->getActiveCurrentLimit(settings.power_level)) + " A            \n";
    output += "| POWER DRAW: " + String(DTI.getACCurrent() * ACU1.getTSVoltage()) + "W                          \n";
    output += "----------------------------------------------------------\n";
    return output;
}

void printDebug(){
    if(millis() - lastPrintTime > 1000/DEBUG_PRINT_FREQUENCY){
        Serial.println("----------------------------------------------------------");
        Serial.println("|                     GR24 EV VEHICLE DEBUG              |");
        Serial.println("----------------------------------------------------------");
        Serial.println(vehicleStatus());
        Serial.println(vehicleHealth());
        Serial.println(vehicleNetwork());
        Serial.println(vehicleSettings());
        Serial.println(vehiclePowerData());
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
    // Car = new iCANflex();
    // begin();
    
    sysCheck = new SystemsCheck();  
    tune = new Tune();
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


    active_faults = new std::unordered_set<bool (*)(Tune&)>(); 
    active_warnings = new std::unordered_set<bool (*)(Tune&)>();
    active_limits = new std::unordered_set<bool (*)(Tune&)>();

    active_faults->clear();
    active_warnings->clear();
    active_limits->clear();


    // set state  
    state = GLV_ON;
    mode = STANDARD; 
    
}



// MAIN LOOP
void loop(){
    printDebug();
    

    // System Checks
    sysCheck->hardware_system_critical(*active_faults, tune);
    // sysCheck->system_faults(*active_faults, tune);
    // sysCheck->system_limits(*active_limits, tune);
    // sysCheck->system_warnings(*active_warnings, tune);
    
    state = active_faults->size() ?  sendToError(*active_faults->begin()) : state;
    
    digitalWrite(SOFTWARE_OK_CONTROL_PIN, (state == ERROR) ? LOW : HIGH);
    settings.power_level = active_limits->size() ? LIMIT : settings.power_level; // limit power in overheat conditions


    // if(settings.power_level == LIMIT) sendDashPopup(0xA, 5);

    // AMS and IMD LEDs and Dash LEDs
    bool AMS_led = active_faults->find(sysCheck->AMS_fault) != active_faults->end();
    bool IMD_led = active_faults->find(sysCheck->IMD_fault) != active_faults->end();
    TSState = (state == GLV_ON) ? GREEN : RED;
    RTDState = (state == PRECHARGE_COMPLETE) ? GREEN : RED;
    sendDashLED(AMS_led, IMD_led, TSState, RTDState);


    // send outgoing CAN Messages
    tryPingRequests({Pedals_Ping_Request, Steering_Wheel_Ping_Request, Dash_Panel_Ping_Request, ACU_Ping_Request} );
    checkPingTimeout();
    sendPingValues(); 
    sendVDMInfo(); 
    
    // readData(msg); // Call receive() on every node in the network API (Nodes.h)
/*
    if(can_primary.read(msg)) {
        
    }
    */

   if(can_primary.read(msg)){
    DTI.receive(msg.id, msg.buf);
    ECU.receive(msg.id, msg.buf);
    PEDALS.receive(msg.id, msg.buf);
    ACU1.receive(msg.id, msg.buf);
    TCM1.receive(msg.id, msg.buf);
    DASHBOARD.receive(msg.id, msg.buf);
    ENERGY_METER.receive(msg.id, msg.buf);
    STEERING_WHEEL.receive(msg.id, msg.buf);
    // process incoming CAN Messages    
    handleDashPanelInputs();   
    handleDriverInputs(*tune);
    handlePingResponse();
    handleECUTuning(*tune);
  }
  if(can_data.read(msg2)){
    WFL.receive(msg.id, msg.buf);
    WFR.receive(msg.id, msg.buf);
    WRL.receive(msg.id, msg.buf);
    WRR.receive(msg.id, msg.buf);
    GPS1.receive(msg.id, msg.buf);
  }

    // traction control
    if(mode == DYNAMIC_TC) computeTractionControl();
    if(tc_multiplier < 1) sendDashPopup(0x07, 1);

    // brake light
    if(analogRead(BSE_HIGH) > 1000) digitalWrite(BRAKE_LIGHT_PIN, HIGH); //TODO: Correct ADC 10 bit
    else digitalWrite(BRAKE_LIGHT_PIN, LOW);

    // state machine operation
    switch (state) {
        // ERROR
        case ERROR:
            state = error(*tune, errorCheck, *active_faults);
            break;

        // STARTUP 
        case ECU_FLASH:
            state = ecu_flash();
            break;
        case GLV_ON:
            state = glv_on();
            break;
     
        // PRECHARGE PROCESS
        case TS_PRECHARGE:
            state = ts_precharge();
            break;
        case PRECHARGING:
            state = precharging();
            break;
        case PRECHARGE_COMPLETE:
            state = precharge_complete();
            break;
        
        // DRIVE
        case DRIVE_STANDBY:
            state = drive_standby(BSE_APPS_violation, *tune); 
            break;
        case DRIVE_ACTIVE:
            state = drive_active(BSE_APPS_violation, *tune);
            break;
        case DRIVE_REGEN:
            state = drive_regen(BSE_APPS_violation, *tune);
            break;
        // turning off TS
        case TS_DISCHARGE_OFF:
            state = ts_discharge_off();
    }




    // FOR PRELIMINARY MOTOR TEST BENCH ONLY
    //     if(millis() %10 == 0){

    //     if(motor_on){
    //         DTI.setMaxCurrent(MAX_AMPS);
    //         DTI.setDriveEnable(1);
    //         DTI.setRCurrent(-1*incomingValue/9.0* 100);
    //     }
    //     else {
    //         DTI.setRCurrent(0);
    //         DTI.setDriveEnable(0);
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



