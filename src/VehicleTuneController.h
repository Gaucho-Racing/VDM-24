#include <vector>
#include "Arduino.h"
//#include "global.h"
#include <SD.h>
#ifndef VEHTUNECONTROLLER_H
#define VEHTUNECONTROLLER_H


struct TorqueProfile{
    float K; // multiplier
    float P; // steepness
    float B; // offset
    TorqueProfile(float k, float p, float b): K(k), P(p), B(b){}
    TorqueProfile(){}
};

class VehicleTuneController {
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
        void readSDCard(){
            Serial.println("Initializing SD Card...");
            while(!SD.begin(BUILTIN_SDCARD)){
                Serial.println("Waiting for SD Card to initialize...");
            }
            
            Serial.println("SD INITIALIZATION SUCCESSFUL");
            File ecu_tune;
            ecu_tune = SD.open("gr24.txt");
            Serial.print("Reading ECU FLASH....");
            String tune;
            while(ecu_tune.available()){
                Serial.print("..");
                tune += (char)ecu_tune.read(); 
            }
            Serial.println(tune.length());
            Serial.println(tune);
            ecu_tune.close();
            Serial.println("");
/*
            std::stringstream iss(tune.c_str());
            // read in torque profiles, regen profiles, and traction profiles
            for(int i = 0; i < 4; i++){
                float k, p, b;
                iss >> k >> p >> b;
                t.setTorqueProfileData(i, TorqueProfile(k, p, b));
            }   
            Serial.println("TORQUE PROFILES INITIALIZED");
            for(int i = 0; i < 4; i++){
                float cmax;
                iss >> cmax;
                t.setPowerLevelData(i, cmax);
            }   
            Serial.println("CURRENT LIMITS INITIALIZED");
            for(int i = 0; i < 4; i++){
                float r;
                iss >> r;
                t.setRegenLevelData(i, r);
            }
            Serial.println("REGEN LEVELS INITIALIZED");
            Serial.println("ECU FLASH COMPLETE. GR24 TUNE DOWNLOADED.");
            Serial.println("STARTING CAR WITH SETTINGS: ");
            Serial.print("THROTTLE MAP: ");
            for (int i = 0; i < 4; i++) {
                Serial.print(t.getTorqueProfilesData()[i].K); 
                Serial.print(" ");
                Serial.print(t.getTorqueProfilesData()[i].P );
                Serial.print(" ");
                Serial.println(t.getTorqueProfilesData()[i].B);
            }
            Serial.print("POWER LEVELS: ");
            for (int i = 0; i < 4; i++) {
                Serial.print(t.getRegenLevelsData()[i]); 
                Serial.print(" ");
            }
            Serial.println("");
            Serial.print("REGEN LEVELS: ");
            for (int i = 0; i < 4; i++) {
                Serial.print(t.getRegenLevelsData()[i]); 
                Serial.print(" ");
            }
            Serial.println("");
            Serial.println("--------------------------");
            */
        }

        VehicleTuneController(){
            // init from sd card
            TorqueProfilesData = std::vector<TorqueProfile>(4);
            PowerLevelsData = std::vector<float>(4);
            RegenLevelsData = std::vector<float>(4);
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
        uint8_t getMaxCANPing() const { return MaxCANPing; }
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

        // get the active torque profile for a given position in the vehicles VehicleTuneController (as K, P, B)
        // @param pos position of the torque profile corr. to SW
        TorqueProfile getActiveTorqueProfile(int8_t pos){ return TorqueProfilesData[pos]; }      
        // get the active current limit for a given position in the vehicles VehicleTuneController (in AMPS)
        // @param pos position of the current limit corr. to SW  
        float getActiveCurrentLimit(int8_t pos){ return PowerLevelsData[pos];} 
        // get the active regen power for a given position in the vehicles VehicleTuneController (in percentile)
        // @param pos position of the regen power corr. to SW
        float getActiveRegenPower(int8_t pos){ return RegenLevelsData[pos];} 
        // get rev limiter cuttoff
        int revLimit(){ return rev_limit; } 

        // get torque profile data
        std::vector<TorqueProfile> getTorqueProfilesData() const { return TorqueProfilesData; }
        // get power level data
        std::vector<float> getPowerLevelsData() const { return PowerLevelsData; }
        // get regen level data
        std::vector<float> getRegenLevelsData() const { return RegenLevelsData; }
        // set the Torque Profile for a given position in the vehicles VehicleTuneController
        // @param index position of the torque profile corr. to SW
        // @param tp TorqueProfile object
        void setTorqueProfileData(uint8_t index, TorqueProfile tp){ TorqueProfilesData[index] = tp; }
        // set the Power Level for a given position in the vehicles VehicleTuneController
        // @param index position of the current limit corr. to SW
        // @param power float value in Amperes
        void setPowerLevelData(uint8_t index, float power){ PowerLevelsData[index] = power; }
        // set the Regen Level for a given position in the vehicles VehicleTuneController
        // @param index position of the regen power corr. to SW
        // @param regen float value in percentile
        void setRegenLevelData(uint8_t index, float regen){ RegenLevelsData[index] = regen; }
    
};

#endif