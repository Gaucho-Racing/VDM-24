#include "icanflex.h"
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

// TORQUE MAP PROFILES 
const uint8_t LINEAR = 0;
const uint8_t TQ_MAP_1 = 1;
const uint8_t TQ_MAP_2 = 2;
const uint8_t TQ_MAP_3 = 3;

// POWER LEVELS
const uint8_t LIMIT = 0;
const uint8_t LOW_PWR = 1;
const uint8_t MEDIUM_PWR = 2;
const uint8_t HIGH_PWR = 3;

// REGEN LEVELS
const uint8_t REGEN_0 = 0;
const uint8_t REGEN_1 = 1;
const uint8_t REGEN_2 = 2;
const uint8_t REGEN_3 = 3;

struct TorqueProfile{
    float K;
    float P;
    float B;
    TorqueProfile(float k, float p, float b): K(k), P(p), B(b){}
    TorqueProfile(){}
};

// STEERING WHEEL SETTINGS
struct SWSettings {
    uint8_t power_level; // 0 - 3
    uint8_t throttle_map; // 0-3
    uint8_t regen_level; // 0-3
    SWSettings(){}  
};  

class Tune {
    private:
        std::vector<TorqueProfile> TorqueProfilesData; // see above
        std::vector<float> PowerLevelsData; // max current value in Amperes
        std::vector<float> RegenLevelsData; // percentile value 0 to 100

        uint32_t MaxCANPing = 100000; // microsec
        uint8_t temp_motor_warn = 60; // celsius
        uint8_t temp_motor_limit = 65; // celsius
        uint8_t temp_motor_critical = 70; // celsius
        uint8_t temp_battery_warn = 60; // celsius
        uint8_t temp_battery_limit = 65; // celsius
        uint8_t temp_battery_critical = 70; // celsius
        uint8_t temp_coolant_warn = 60; // celsius
        uint8_t temp_coolant_limit = 65; // celsius
        uint8_t temp_coolant_critical = 70; // celsius
        uint8_t temp_inverter_warn = 60; // celsius
        uint8_t temp_inverter_limit = 65; // celsius
        uint8_t temp_inverter_critical = 70; // celsius
        uint16_t rev_limit = 5500;// RPM      public:

    public:
        SWSettings settings;
        Tune(){
            // init from sd card
            TorqueProfilesData = std::vector<TorqueProfile>(4);
            PowerLevelsData = std::vector<float>(4);
            RegenLevelsData = std::vector<float>(4);



            // Serial.println("Initializing SD Card...");
            // while(!SD.begin(BUILTIN_SDCARD)){
            //     Serial.println("Waiting for SD Card to initialize...");
            // }
            
            // Serial.println("SD INITIALIZATION SUCCESSFUL");
            // File ecu_tune;
            // ecu_tune = SD.open("gr24.txt");
            // Serial.print("Reading ECU FLASH....");
            // String tune;
            // while(ecu_tune.available()){
            //     Serial.print("..");
            //     tune += (char)ecu_tune.read(); 
            // }
            // Serial.println(tune.length());
            // ecu_tune.close();
            // Serial.println("");

            // stringstream iss(tune.c_str()); // const so put into FLASH MEMORY
            // // read in torque profiles, regen profiles, and traction profiles
            // for(int i = 0; i < 4; i++){
            //     float k, p, b;
            //     iss >> k >> p >> b;
            //     TORQUE_PROFILES[i] = TorqueProfile(k, p, b);
            // }   
            // delay(250);
            // Serial.println("TORQUE PROFILES INITIALIZED");
            // for(int i = 0; i < 4; i++){
            //     float cmax;
            //     iss >> cmax;
            //     POWER_LEVELS[i] = cmax;
            // }   
            // delay(250);
            // Serial.println("CURRENT LIMITS INITIALIZED");
            // for(int i = 0; i < 4; i++){
            //     float r;
            //     iss >> r;
            //     REGEN_LEVELS[i] = r;
            // }
            // delay(250);
            // Serial.println("REGEN LEVELS INITIALIZED");
            // delay(250);
            // Serial.println("ECU FLASH COMPLETE. GR24 TUNE DOWNLOADED.");
            // Serial.println("STARTING CAR WITH SETTINGS: ");
            // Serial.print("THROTTLE MAP: ");
            // for (int i = 0; i < 4; i++) {
            //     Serial.print(TORQUE_PROFILES[i].K); 
            //     Serial.print(" ");
            //     Serial.print(TORQUE_PROFILES[i].P);
            //     Serial.print(" ");
            //     Serial.println(TORQUE_PROFILES[i].B);
            // }
            // Serial.print("POWER LEVELS: ");
            // for (int i = 0; i < 4; i++) {
            //     Serial.print(POWER_LEVELS[i]); 
            //     Serial.print(" ");
            // }
            // Serial.println("");
            // Serial.print("REGEN LEVELS: ");
            // for (int i = 0; i < 4; i++) {
            //     Serial.print(REGEN_LEVELS[i]); 
            //     Serial.print(" ");
            // }
            // Serial.println("");
            // Serial.println("--------------------------");
            

        }

        
        // getters for all the settings
        uint8_t getMaxCANPing(){ return MaxCANPing; }
        uint8_t getMotorWarnTemp(){ return temp_motor_warn; }
        uint8_t getMotorLimitTemp(){ return temp_motor_limit; }
        uint8_t getMotorCriticalTemp(){ return temp_motor_critical; }
        uint8_t getBatteryWarnTemp(){ return temp_battery_warn; }
        uint8_t getBatteryLimitTemp(){ return temp_battery_limit; }
        uint8_t getBatteryCriticalTemp(){ return temp_battery_critical; }
        uint8_t getCoolantWarnTemp(){ return temp_coolant_warn; }
        uint8_t getCoolantLimitTemp(){ return temp_coolant_limit; }
        uint8_t getCoolantCriticalTemp(){ return temp_coolant_critical; }
        uint8_t getInverterWarnTemp(){ return temp_inverter_warn; }
        uint8_t getInverterLimitTemp(){ return temp_inverter_limit; }
        uint8_t getInverterCriticalTemp(){ return temp_inverter_critical; }

        void setMaxCANPing(uint32_t ping){ MaxCANPing = ping; }
        void setMotorWarnTemp(uint8_t temp){ temp_motor_warn = temp; }
        void setMotorLimitTemp(uint8_t temp){ temp_motor_limit = temp; }
        void setMotorCriticalTemp(uint8_t temp){ temp_motor_critical = temp; }
        void setBatteryWarnTemp(uint8_t temp){ temp_battery_warn = temp; }
        void setBatteryLimitTemp(uint8_t temp){ temp_battery_limit = temp; }
        void setBatteryCriticalTemp(uint8_t temp){ temp_battery_critical = temp; }
        void setCoolantWarnTemp(uint8_t temp){ temp_coolant_warn = temp; }
        void setCoolantLimitTemp(uint8_t temp){ temp_coolant_limit = temp; }
        void setCoolantCriticalTemp(uint8_t temp){ temp_coolant_critical = temp; }
        void setInverterWarnTemp(uint8_t temp){ temp_inverter_warn = temp; }
        void setInverterLimitTemp(uint8_t temp){ temp_inverter_limit = temp; }
        void setInverterCriticalTemp(uint8_t temp){ temp_inverter_critical = temp; }


        TorqueProfile getActiveTorqueProfile(){ return TorqueProfilesData[settings.throttle_map]; }        
        float getActiveCurrentLimit(){ return PowerLevelsData[settings.power_level];}
        float getActiveRegenPower(){ return RegenLevelsData[settings.regen_level];}
        int revLimit(){ return rev_limit; } 


        void setTorqueProfileData(uint8_t index, TorqueProfile tp){ TorqueProfilesData[index] = tp; }
        void setPowerLevelData(uint8_t index, float power){ PowerLevelsData[index] = power; }
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

/*
8 bytes of 8 bits:
[can warn][can failure][AMS][IMD][BSPD][SDC][][]
[warn motor][limit motor][crit motor][warn batt][limit batt][crit batt][rev limit][] temps motor and battery, Revs
[warn water][limit water][crit water][warn mcu][limit mcu][crit mcu][TCM Status][] // water temp DTI temp, TCM
[][][][][][][][] 
[][][][][][][][] 
*/
// error severity: warning -> limit -> critical

class SystemsCheck {
    private:    

        byte SYS_CHECK_CAN_FRAME[8]; // 8 bytes of system checks


    public:
        SystemsCheck(){
            for(int i = 0; i < 5; i++) SYS_CHECK_CAN_FRAME[i] = 0x0;
        }

        byte* getSysCheckFrame(){
            return SYS_CHECK_CAN_FRAME;
        }
        
        void hardware_system_critical(const iCANflex& Car, std::unordered_set<bool (*)(const iCANflex&, Tune& t)> &af, Tune* t){
            if(SDC_opened(Car, *t)) af.insert(SDC_opened);  
            SYS_CHECK_CAN_FRAME[0] = SDC_opened(Car, *t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
            if(AMS_fault(Car, *t)) af.insert(AMS_fault);
            SYS_CHECK_CAN_FRAME[0] = AMS_fault(Car, *t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00100000) : (SYS_CHECK_CAN_FRAME[0] & 0b11011111);
            if(IMD_fault(Car, *t)) af.insert(IMD_fault);
            SYS_CHECK_CAN_FRAME[0] = IMD_fault(Car, *t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00010000) : (SYS_CHECK_CAN_FRAME[0] & 0b11101111);
            if(BSPD_fault(Car, *t)) af.insert(BSPD_fault);
            SYS_CHECK_CAN_FRAME[0] = BSPD_fault(Car, *t) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
            // if(max_current(Car)) af.insert(max_current);
            // SYS_CHECK_CAN_FRAME[0] = max_current(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000010) : (SYS_CHECK_CAN_FRAME[0] & 0b11111101);
        }

        // NOTE: OPEN THE SOFTWARE LATCH IF the Inverter is not responding or there are critical system faults. 
        void system_faults(const iCANflex& Car, std::unordered_set<bool (*)(const iCANflex&, Tune& t)> &af, Tune* t){
            if(critical_motor_temp(Car, *t)) af.insert(critical_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_motor_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00100000) : (SYS_CHECK_CAN_FRAME[1] & 0b11011111);
            if(critical_battery_temp(Car, *t)) af.insert(critical_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_battery_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000100) : (SYS_CHECK_CAN_FRAME[1] & 0b11111011);
            // if(critical_water_temp(Car, *t)) af.insert(critical_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = critical_water_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00100000) : (SYS_CHECK_CAN_FRAME[2] & 0b11011111);
            if(critical_mcu_temp(Car, *t)) af.insert(critical_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = critical_mcu_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00000100) : (SYS_CHECK_CAN_FRAME[2] & 0b11111011);
            // if(critical_can_failure(Car)) af.insert(critical_can_failure);
            // SYS_CHECK_CAN_FRAME[0] = critical_can_failure(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b01000000) : (SYS_CHECK_CAN_FRAME[0] & 0b10111111);
            
        }

        void system_limits(const iCANflex& Car, std::unordered_set<bool (*)(const iCANflex&, Tune& t)> &al, Tune* t){
            if(limit_motor_temp(Car, *t)) al.insert(limit_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_motor_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[1] | 0b01000000) : (SYS_CHECK_CAN_FRAME[1] & 0b10111111);
            if(limit_battery_temp(Car, *t)) al.insert(limit_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_battery_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00001000) : (SYS_CHECK_CAN_FRAME[1] & 0b11110111);
            // if(limit_water_temp(Car, *t)) al.insert(limit_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = limit_water_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[2] | 0b01000000) : (SYS_CHECK_CAN_FRAME[2] & 0b10111111);
            if(limit_mcu_temp(Car, *t)) al.insert(limit_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = limit_mcu_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00001000) : (SYS_CHECK_CAN_FRAME[2] & 0b11110111);
        }

        void system_warnings(const iCANflex& Car, std::unordered_set<bool (*)(const iCANflex&, Tune& t)> &aw, Tune* t){
            if(warn_motor_temp(Car, *t)) aw.insert(warn_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_motor_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[1] | 0b10000000) : (SYS_CHECK_CAN_FRAME[1] & 0b01111111);
            if(warn_battery_temp(Car, *t)) aw.insert(warn_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_battery_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00010000) : (SYS_CHECK_CAN_FRAME[1] & 0b11101111);
            // if(warn_water_temp(Car, *t)) aw.insert(warn_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = warn_water_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[2] | 0b10000000) : (SYS_CHECK_CAN_FRAME[2] & 0b01111111);
            if(warn_mcu_temp(Car, *t)) aw.insert(warn_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = warn_mcu_temp(Car, *t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00010000) : (SYS_CHECK_CAN_FRAME[2] & 0b11101111);
            if(rev_limit_exceeded(Car, *t)) aw.insert(rev_limit_exceeded);
            SYS_CHECK_CAN_FRAME[1] = rev_limit_exceeded(Car, *t) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000010) : (SYS_CHECK_CAN_FRAME[1] & 0b11111101);
        }




        // BYTE 0 ---------------------------------------------------------------------------
        // bit 0

        bool warn_can_failure(const iCANflex& Car){ // TODO: Fix
            // return Pedals_Ping > 1000000 || ACU_Ping > 1000000 || /*BCM_Age > 1000000 || */ DashPanel_Ping > 1000000 || SteeringWheel_Ping > 1000000 /*|| DTI_Age > 1000000*/;
            return false;
        }   
        // bit 1
        bool critical_can_failure(const iCANflex& Car){ // TODO: Fix
            return false;
        }



        // HARDWARE FAULTS: VERY BAD
        // .5v is shit -  ADC: 155
        // 3v when almost ok - ADC: 930
        // 2.4v is ok - ADC: 744
        // 1v = 310
        // bits 2, 3, 4, 5, 
        static bool AMS_fault(const iCANflex& Car, Tune& t){ return analogRead(AMS_OK_PIN) < 700 || analogRead(AMS_OK_PIN) > 790; }
        static bool IMD_fault(const iCANflex& Car, Tune& t){ return analogRead(IMD_OK_PIN) < 700 || analogRead(IMD_OK_PIN) > 790; }
        static bool BSPD_fault(const iCANflex& Car, Tune& t){ return analogRead(BSPD_OK_PIN) < 700 || analogRead(BSPD_OK_PIN) > 790; }
        // check voltage < 7V (this one is 16V 8 bit ADC)
        static bool SDC_opened(const iCANflex& Car, Tune& t){ return Car.ACU1.getSDCVoltage() < 112; } 
        // bit 6
        // bool SystemsCheck::max_current(const iCANflex& Car){return Car.DTI.getDCCurrent() > Car.DTI.getDCCurrentLim();}

        // BYTE 1 ---------------------------------------------------------------------------
        // bit 0, 1, 2
        static bool warn_motor_temp(const iCANflex& Car, Tune& t){return Car.DTI.getMotorTemp() > t.getMotorWarnTemp() && Car.DTI.getMotorTemp() < t.getMotorLimitTemp();}
        static bool limit_motor_temp(const iCANflex& Car, Tune& t){return Car.DTI.getMotorTemp() > t.getMotorLimitTemp() && Car.DTI.getMotorTemp() < t.getMotorCriticalTemp();}
        static bool critical_motor_temp(const iCANflex& Car, Tune& t){return Car.DTI.getMotorTemp() > t.getMotorCriticalTemp();}
        // bit 3, 4, 5
        static bool warn_battery_temp(const iCANflex& Car, Tune& t) {return Car.ACU1.getMaxCellTemp() > t.getBatteryWarnTemp() && Car.ACU1.getMaxCellTemp() < t.getBatteryLimitTemp();}
        static bool limit_battery_temp(const iCANflex& Car, Tune& t) {return Car.ACU1.getMaxCellTemp() > t.getBatteryLimitTemp() && Car.ACU1.getMaxCellTemp() < t.getBatteryCriticalTemp();}
        static bool critical_battery_temp(const iCANflex& Car, Tune& t) {return Car.ACU1.getMaxCellTemp() > t.getBatteryCriticalTemp();}
        // bit 6
        static bool rev_limit_exceeded(const iCANflex& Car, Tune& t) {return Car.DTI.getERPM()/10 > t.revLimit();}
        // bit 7 

        // BYTE 2 ---------------------------------------------------------------------------
        // bit 0, 1, 2
        // static bool warn_water_temp(const iCANflex& Car, Tune& t){return Car.ACU1.get() > t.getBatteryWarnTemp() && Car.ACU1.getWaterTemp() < t.getCoolantLimitTemp();}
        // static bool limit_water_temp(const iCANflex& Car, Tune& t){return Car.ACU1.getWaterTemp() > t.getCoolantLimitTemp() && Car.ACU1.getWaterTemp() < t.getCoolantCriticalTemp();}
        // static bool critical_water_temp(const iCANflex& Car, Tune& t){return Car.ACU1.getWaterTemp() > t.getCoolantCriticalTemp();}
        // bit 3, 4, 5
        static bool warn_mcu_temp(const iCANflex& Car, Tune& t) {return Car.DTI.getInvTemp() > t.getInverterWarnTemp() && Car.DTI.getInvTemp() < t.getInverterLimitTemp();}
        static bool limit_mcu_temp(const iCANflex& Car, Tune& t){return Car.DTI.getInvTemp() > t.getInverterLimitTemp() && Car.DTI.getInvTemp() < t.getInverterCriticalTemp();}
        static bool critical_mcu_temp(const iCANflex& Car, Tune& t) {return Car.DTI.getInvTemp() > t.getInverterCriticalTemp();}
        // bit 6
        static bool TCM_fault(const iCANflex& Car, Tune& t) {return false;} // TODO: do
        // bit 7 empty for now


        // TODO: add more system checks


};



/*
   ________    ____  ____  ___    __ 
  / ____/ /   / __ \/ __ )/   |  / / 
 / / __/ /   / / / / __  / /| | / /  
/ /_/ / /___/ /_/ / /_/ / ___ |/ /___
\____/_____/\____/_____/_/  |_/_____/
*/
#define PRINT_DBG 0x00; //TODO: COMMENT FOR EFFICIENCY
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

const int16_t DTI_COMM_FREQUENCY = 100; // Hz
const int16_t NETWORK_PING_FREQUENCY = 10; // Hz







/*
   _________    _   __      ____  __  _______
  / ____/   |  / | / /     / __ )/ / / / ___/
 / /   / /| | /  |/ /_____/ __  / / / /\__ \ 
/ /___/ ___ |/ /|  /_____/ /_/ / /_/ /___/ / 
\____/_/  |_/_/ |_/     /_____/\____//____/  
   __________  __  _____  ________
  / ____/ __ \/  |/  /  |/  / ___/
 / /   / / / / /|_/ / /|_/ /\__ \ 
/ /___/ /_/ / /  / / /  / /___/ / 
\____/\____/_/  /_/_/  /_//____/  
*/


unsigned long ACU_Ping = 0;
unsigned long Pedals_Ping = 0;
unsigned long DashPanel_Ping = 0;
unsigned long SteeringWheel_Ping = 0;
unsigned long sendTime = 0;
unsigned long lastPrechargeTime = 0;
unsigned long lastDTIMessage = 0;

unsigned long ping() {
    unsigned long newTime = (long)msg.buf[3] + ((long)msg.buf[2] << 8) + ((long)msg.buf[1] << 16) + ((long)msg.buf[0] << 24);
    unsigned long newTime2 = (long)msg.buf[7] + ((long)msg.buf[6] << 8) + ((long)msg.buf[5] << 16) + ((long)msg.buf[4] << 24);
    unsigned long roundTripDelay = ((millis() - newTime) * 1000) + ((micros() - newTime2) % 1000);
    return roundTripDelay;
}


void tryPingReqests(std::vector<uint32_t> request_ids, iCANflex& Car){
    if(millis()-sendTime > 100/NETWORK_PING_FREQUENCY){
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


    void sendVDMInfo(Tune& tune){
       //TODO:
    }

    void handleDriverInputs(Tune& tune){
        if(msg.id == 0x11002){
            tune.settings.power_level = msg.buf[0];
            tune.settings.throttle_map = msg.buf[1];
            tune.settings.regen_level = msg.buf[2];
        }
    }


    void handleDashPanelInputs(){
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
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
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
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }
    if(Car.ACU1.getPrecharging()){
        return PRECHARGING;
    }
    return TS_PRECHARGE;
}
// -- PRECHARGING STAGE 2
State precharging(iCANflex& Car){
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
        Car.DTI.setRCurrent(0);
        Car.DTI.setDriveEnable(0);
        lastDTIMessage = millis();
    }

    if(Car.ACU1.getPrechargeDone()) return PRECHARGE_COMPLETE;
    return PRECHARGE_COMPLETE;
}

// -- PRECHARGING STAGE 3
State precharge_complete(iCANflex& Car){
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
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
    
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
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
    float brake = (Car.PEDALS.getBrakePressureF() + Car.PEDALS.getBrakePressureR())/2.0;
    
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
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
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
    
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
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
    if(millis() - lastDTIMessage > 1000/DTI_COMM_FREQUENCY){
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

const int DEBUG_PRINT_FREQUENCY = 10; // Hz
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
                {TESTING, "TESTING"},
                {LAUNCH, "LAUNCH"},
                {ENDURANCE, "ENDURANCE"},
                {AUTOX, "AUTOX"},
                {SKIDPAD, "SKIDPAD"},
                {ACC, "ACC"},
                {PIT, "PIT"}
            };  

void printStatus(){
    if(millis() - lastPrintTime > 1000/DEBUG_PRINT_FREQUENCY){
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

// MAIN LOOP
void loop(){
    
    // Serial.println(Car->ACU1.getPrecharging());
    if(can1.read(msg)) {
        handleDashPanelInputs();    
        handleDriverInputs(*tune);
        handlePings();
    }
    
    // Get ping values for all systems
    tryPingReqests({0x10FFE, 0x12FFE, 0xCA, 0x95}, *Car);
   
    // System Checks
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
            state = drive_standby(*Car, BSE_APPS_violation); 
            break;
        case DRIVE_ACTIVE:
            state = drive_active(*Car, BSE_APPS_violation, *tune);
            break;
        case DRIVE_REGEN:
            state = drive_regen(*Car, BSE_APPS_violation, *tune);
            break;
    }
    
}



