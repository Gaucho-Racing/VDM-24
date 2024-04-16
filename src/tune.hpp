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
        std::vector<TorqueProfile> TorqueProfiles; // see above
        std::vector<float> PowerLevels; // max current value in Amperes
        std::vector<float> RegenLevels; // percentile value 0 to 100
        int rev_limit = 5500;
    public:

        SWSettings settings;

        Tune(){
            // init from sd card
            TorqueProfiles = std::vector<TorqueProfile>(4);
            PowerLevels = std::vector<float>(4);
            RegenLevels = std::vector<float>(4);



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

        TorqueProfile getTorqueProfile(uint8_t setting){ return TorqueProfiles[setting]; }        
        float getPowerSetting(uint8_t setting){ return PowerLevels[setting];}
        float getRegenSetting(uint8_t setting){ return RegenLevels[setting];}
        int revLimit(){ return rev_limit; } 

};

#endif
