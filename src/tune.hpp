#ifndef TUNE_HPP
#define TUNE_HPP

#include <Arduino.h>
#include <vector>

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



            //             Serial.println("Initializing SD Card...");
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

#endif
