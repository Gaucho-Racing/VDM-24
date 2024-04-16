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


class Tune {
    private:
        std::vector<TorqueProfile> TorqueProfiles; // see above
        std::vector<float> PowerLevels; // max current value in Amperes
        std::vector<float> RegenLevels; // percentile value 0 to 100
    public:
        Tune(){
            // init from sd card
            TorqueProfiles = std::vector<TorqueProfile>(4);
            PowerLevels = std::vector<float>(4);
            RegenLevels = std::vector<float>(4);
        }

        TorqueProfile getTorqueProfile(uint8_t setting){ return TorqueProfiles[setting]; }        
        float getPowerSetting(uint8_t setting){ return PowerLevels[setting];}
        float getRegenSetting(uint8_t setting){ return RegenLevels[setting];}

};

#endif
