#ifndef SYSTEMS_CHECK
#define SYSTEMS_CHECK

#include <Arduino.h>
#include <unordered_set>

// PIN DEFINITIONS
const uint8_t SOFTWARE_OK_CONTROL_PIN = 41;
const uint8_t BRAKE_LIGHT_PIN = 4;
const uint8_t BSPD_OK_PIN = 19;
const uint8_t IMD_OK_PIN = 20;
const uint8_t AMS_OK_PIN = 21;
uint8_t CAN_MS_THRESHOLD = 100; // msec
uint8_t MOTOR_TEMP_WARN = 60; // celsius
uint8_t MOTOR_TEMP_LIMIT = 65; // celsius
uint8_t MOTOR_TEMP_CRITICAL = 70; // celsius
uint8_t BATTERY_TEMP_WARN = 60; // celsius
uint8_t BATTERY_TEMP_LIMIT = 65; // celsius
uint8_t BATTERY_TEMP_CRITICAL = 70; // celsius
uint8_t WATER_TEMP_WARN = 60; // celsius
uint8_t WATER_TEMP_LIMIT = 65; // celsius
uint8_t WATER_TEMP_CRITICAL = 70; // celsius
uint8_t MCU_TEMP_WARN = 60; // celsius
uint8_t MCU_TEMP_LIMIT = 65; // celsius
uint8_t MCU_TEMP_CRITICAL = 70; // celsius
byte SYS_CHECK_CAN_FRAME[8]; // 8 bytes of system checks
int REV_LIMIT = 5500; // RPM    

class SystemsCheck {
    private:    



    /*
    8 bytes of 8 bits:
    [can warn][can failure][AMS][IMD][BSPD][SDC][][]
    [warn motor][limit motor][crit motor][warn batt][limit batt][crit batt][rev limit][] temps motor and battery, Revs
    [warn water][limit water][crit water][warn mcu][limit mcu][crit mcu][TCM Status][] // water temp DTI temp, TCM
    [][][][][][][][] 
    [][][][][][][][] 
    */

    public:

        SystemsCheck(){
            for(int i = 0; i < 5; i++) SYS_CHECK_CAN_FRAME[i] = 0x0;
        }
        
        void hardware_system_critical(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &af){
            if(SDC_opened(Car)) af.insert(SDC_opened);  
            SYS_CHECK_CAN_FRAME[0] = SDC_opened(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
            if(AMS_fault(Car)) af.insert(AMS_fault);
            SYS_CHECK_CAN_FRAME[0] = AMS_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00100000) : (SYS_CHECK_CAN_FRAME[0] & 0b11011111);
            if(IMD_fault(Car)) af.insert(IMD_fault);
            SYS_CHECK_CAN_FRAME[0] = IMD_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00010000) : (SYS_CHECK_CAN_FRAME[0] & 0b11101111);
            if(BSPD_fault(Car)) af.insert(BSPD_fault);
            SYS_CHECK_CAN_FRAME[0] = BSPD_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
            // if(max_current(Car)) af.insert(max_current);
            // SYS_CHECK_CAN_FRAME[0] = max_current(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000010) : (SYS_CHECK_CAN_FRAME[0] & 0b11111101);
        }

        // NOTE: OPEN THE SOFTWARE LATCH IF the Inverter is not responding or there are critical system faults. 
        void system_faults(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &af){
            if(critical_motor_temp(Car)) af.insert(critical_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00100000) : (SYS_CHECK_CAN_FRAME[1] & 0b11011111);
            if(critical_battery_temp(Car)) af.insert(critical_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000100) : (SYS_CHECK_CAN_FRAME[1] & 0b11111011);
            if(critical_water_temp(Car)) af.insert(critical_water_temp);
            SYS_CHECK_CAN_FRAME[2] = critical_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00100000) : (SYS_CHECK_CAN_FRAME[2] & 0b11011111);
            if(critical_mcu_temp(Car)) af.insert(critical_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = critical_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00000100) : (SYS_CHECK_CAN_FRAME[2] & 0b11111011);
            // if(critical_can_failure(Car)) af.insert(critical_can_failure);
            // SYS_CHECK_CAN_FRAME[0] = critical_can_failure(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b01000000) : (SYS_CHECK_CAN_FRAME[0] & 0b10111111);
            
        }

        void system_limits(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &al){
            if(limit_motor_temp(Car)) al.insert(limit_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b01000000) : (SYS_CHECK_CAN_FRAME[1] & 0b10111111);
            if(limit_battery_temp(Car)) al.insert(limit_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00001000) : (SYS_CHECK_CAN_FRAME[1] & 0b11110111);
            if(limit_water_temp(Car)) al.insert(limit_water_temp);
            SYS_CHECK_CAN_FRAME[2] = limit_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b01000000) : (SYS_CHECK_CAN_FRAME[2] & 0b10111111);
            if(limit_mcu_temp(Car)) al.insert(limit_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = limit_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00001000) : (SYS_CHECK_CAN_FRAME[2] & 0b11110111);
        }

        void system_warnings(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &aw){
            if(warn_motor_temp(Car)) aw.insert(warn_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b10000000) : (SYS_CHECK_CAN_FRAME[1] & 0b01111111);
            if(warn_battery_temp(Car)) aw.insert(warn_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00010000) : (SYS_CHECK_CAN_FRAME[1] & 0b11101111);
            if(warn_water_temp(Car)) aw.insert(warn_water_temp);
            SYS_CHECK_CAN_FRAME[2] = warn_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b10000000) : (SYS_CHECK_CAN_FRAME[2] & 0b01111111);
            if(warn_mcu_temp(Car)) aw.insert(warn_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = warn_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00010000) : (SYS_CHECK_CAN_FRAME[2] & 0b11101111);
            if(rev_limit_exceeded(Car)) aw.insert(rev_limit_exceeded);
            SYS_CHECK_CAN_FRAME[1] = rev_limit_exceeded(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000010) : (SYS_CHECK_CAN_FRAME[1] & 0b11111101);
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
        static bool AMS_fault(const iCANflex& Car){ return analogRead(AMS_OK_PIN) < 700 || analogRead(AMS_OK_PIN) > 790; }
        static bool IMD_fault(const iCANflex& Car){ return analogRead(IMD_OK_PIN) < 700 || analogRead(IMD_OK_PIN) > 790; }
        static bool BSPD_fault(const iCANflex& Car){ return analogRead(BSPD_OK_PIN) < 700 || analogRead(BSPD_OK_PIN) > 790; }
        // check voltage < 7V (this one is 16V 8 bit ADC)
        static bool SDC_opened(const iCANflex& Car){ return Car.ACU1.getSDCVoltage() < 112; } 
        // bit 6
        // bool SystemsCheck::max_current(const iCANflex& Car){return Car.DTI.getDCCurrent() > Car.DTI.getDCCurrentLim();}

        // BYTE 1 ---------------------------------------------------------------------------
        // bit 0, 1, 2
        static bool warn_motor_temp(const iCANflex& Car){return Car.DTI.getMotorTemp() > MOTOR_TEMP_WARN && Car.DTI.getMotorTemp() < MOTOR_TEMP_LIMIT;}
        static bool limit_motor_temp(const iCANflex& Car){return Car.DTI.getMotorTemp() > MOTOR_TEMP_LIMIT && Car.DTI.getMotorTemp() < MOTOR_TEMP_CRITICAL;}
        static bool critical_motor_temp(const iCANflex& Car){return Car.DTI.getMotorTemp() > MOTOR_TEMP_CRITICAL;}
        // bit 3, 4, 5
        static bool warn_battery_temp(const iCANflex& Car) {return Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_WARN && Car.ACU1.getMaxCellTemp() < BATTERY_TEMP_LIMIT;}
        static bool limit_battery_temp(const iCANflex& Car) {return Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_LIMIT && Car.ACU1.getMaxCellTemp() < BATTERY_TEMP_CRITICAL;}
        static bool critical_battery_temp(const iCANflex& Car) {return Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_CRITICAL;}
        // bit 6
        static bool rev_limit_exceeded(const iCANflex& Car) {return Car.DTI.getERPM()/10 > REV_LIMIT;}
        // bit 7 

        // BYTE 2 ---------------------------------------------------------------------------
        // bit 0, 1, 2
        static bool warn_water_temp(const iCANflex& Car){return Car.ACU1.getWaterTemp() > WATER_TEMP_WARN && Car.ACU1.getWaterTemp() < WATER_TEMP_LIMIT;}
        static bool limit_water_temp(const iCANflex& Car){return Car.ACU1.getWaterTemp() > WATER_TEMP_LIMIT && Car.ACU1.getWaterTemp() < WATER_TEMP_CRITICAL;}
        static bool critical_water_temp(const iCANflex& Car){return Car.ACU1.getWaterTemp() > WATER_TEMP_CRITICAL;}
        // bit 3, 4, 5
        static bool warn_mcu_temp(const iCANflex& Car) {return Car.DTI.getInvTemp() > MCU_TEMP_WARN;}
        static bool limit_mcu_temp(const iCANflex& Car){return Car.DTI.getInvTemp() > MCU_TEMP_LIMIT;}
        static bool critical_mcu_temp(const iCANflex& Car) {return Car.DTI.getInvTemp() > MCU_TEMP_CRITICAL;}
        // bit 6
        static bool TCM_fault(const iCANflex& Car) {return false;} // TODO: do
        // bit 7 empty for now


        // TODO: add more system checks


};



#endif
