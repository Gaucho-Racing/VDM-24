#ifndef SYSTEMSCHECK_H
#define SYSTEMSCHECK_H
#include "Arduino.h"
#include "vehicle.h"
#include "global.h"
#include "unordered_set"
#include "unordered_map"
#include "VehicleTuneController.h"

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
        
        void hardware_system_critical(std::unordered_set<bool (*)(VehicleTuneController& t)> &af, VehicleTuneController* t){
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
        void system_faults(std::unordered_set<bool (*)(VehicleTuneController& t, Vehicle& car)> &af, VehicleTuneController* t, Vehicle& car){
            if(critical_motor_temp(*t, car)) af.insert(critical_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_motor_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00100000) : (SYS_CHECK_CAN_FRAME[1] & 0b11011111);
            if(critical_battery_temp(*t, car)) af.insert(critical_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = critical_battery_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000100) : (SYS_CHECK_CAN_FRAME[1] & 0b11111011);
            // if(critical_water_temp(*t)) af.insert(critical_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = critical_water_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b00100000) : (SYS_CHECK_CAN_FRAME[2] & 0b11011111);
            if(critical_mcu_temp(*t, car)) af.insert(critical_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = critical_mcu_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00000100) : (SYS_CHECK_CAN_FRAME[2] & 0b11111011);
            // if(critical_can_failure(Car)) af.insert(critical_can_failure);
            // SYS_CHECK_CAN_FRAME[0] = critical_can_failure(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b01000000) : (SYS_CHECK_CAN_FRAME[0] & 0b10111111);
            
        }

        void system_limits(std::unordered_set<bool (*)(VehicleTuneController& t, Vehicle& car)> &al, VehicleTuneController* t, Vehicle& car){
            if(limit_motor_temp(*t, car)) al.insert(limit_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_motor_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[1] | 0b01000000) : (SYS_CHECK_CAN_FRAME[1] & 0b10111111);
            if(limit_battery_temp(*t, car)) al.insert(limit_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = limit_battery_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00001000) : (SYS_CHECK_CAN_FRAME[1] & 0b11110111);
            // if(limit_water_temp(*t)) al.insert(limit_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = limit_water_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b01000000) : (SYS_CHECK_CAN_FRAME[2] & 0b10111111);
            if(limit_mcu_temp(*t, car)) al.insert(limit_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = limit_mcu_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00001000) : (SYS_CHECK_CAN_FRAME[2] & 0b11110111);
        }

        void system_warnings(std::unordered_set<bool (*)(VehicleTuneController& t, Vehicle& car)> &aw, VehicleTuneController* t, Vehicle& car){
            if(warn_motor_temp(*t, car)) aw.insert(warn_motor_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_motor_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[1] | 0b10000000) : (SYS_CHECK_CAN_FRAME[1] & 0b01111111);
            if(warn_battery_temp(*t, car)) aw.insert(warn_battery_temp);
            SYS_CHECK_CAN_FRAME[1] = warn_battery_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00010000) : (SYS_CHECK_CAN_FRAME[1] & 0b11101111);
            // if(warn_water_temp(*t)) aw.insert(warn_water_temp);
            // SYS_CHECK_CAN_FRAME[2] = warn_water_temp(*t) ? (SYS_CHECK_CAN_FRAME[2] | 0b10000000) : (SYS_CHECK_CAN_FRAME[2] & 0b01111111);
            if(warn_mcu_temp(*t, car)) aw.insert(warn_mcu_temp);
            SYS_CHECK_CAN_FRAME[2] = warn_mcu_temp(*t, car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00010000) : (SYS_CHECK_CAN_FRAME[2] & 0b11101111);
            if(rev_limit_exceeded(*t, car)) aw.insert(rev_limit_exceeded);
            SYS_CHECK_CAN_FRAME[1] = rev_limit_exceeded(*t, car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000010) : (SYS_CHECK_CAN_FRAME[1] & 0b11111101);
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
        static bool AMS_fault(VehicleTuneController& t){ return digitalRead(AMS_OK_PIN) != HIGH ;}
        static bool IMD_fault(VehicleTuneController& t){ return analogRead(IMD_OK_PIN) < 700 || analogRead(IMD_OK_PIN) > 790; }
        static bool BSPD_fault(VehicleTuneController& t){ return digitalRead(BSPD_OK_PIN) != HIGH ;}
        // check voltage < 7V (this one is 16V 8 bit ADC)
        static bool SDC_opened(VehicleTuneController& t){ /*return Car.ACU1.getPrechargeDone() && Car.ACU1.getSDCVoltage() < 7;*/ return false; }
 
        // bit 6
        // bool SystemsCheck::max_current(const ){return Car.DTI.getDCCurrent() > Car.DTI.getDCCurrentLim();} 

        // BYTE 1 ---------------------------------------------------------------------------
        // bit 0, 1, 2
        static bool warn_motor_temp(VehicleTuneController& t, Vehicle& Car){return Car.DTI.getMotorTemp() > t.getMotorWarnTemp() && Car.DTI.getMotorTemp() < t.getMotorLimitTemp();}
        static bool limit_motor_temp(VehicleTuneController& t, Vehicle& Car){return Car.DTI.getMotorTemp() > t.getMotorLimitTemp() && Car.DTI.getMotorTemp() < t.getMotorCriticalTemp();}
        static bool critical_motor_temp(VehicleTuneController& t, Vehicle& Car){return Car.DTI.getMotorTemp() > t.getMotorCriticalTemp();}
        // bit 3, 4, 5
        static bool warn_battery_temp(VehicleTuneController& t, Vehicle& Car) {return Car.ACU1.getMaxCellTemp() > t.getBatteryWarnTemp() && Car.ACU1.getMaxCellTemp() < t.getBatteryLimitTemp();}
        static bool limit_battery_temp(VehicleTuneController& t, Vehicle& Car) {return Car.ACU1.getMaxCellTemp() > t.getBatteryLimitTemp() && Car.ACU1.getMaxCellTemp() < t.getBatteryCriticalTemp();}
        static bool critical_battery_temp(VehicleTuneController& t, Vehicle& Car) {return Car.ACU1.getMaxCellTemp() > t.getBatteryCriticalTemp();}
        // bit 6
        static bool rev_limit_exceeded(VehicleTuneController& t, Vehicle& Car) {return Car.DTI.getERPM()/10 > t.revLimit();}
        // bit 7 

        // BYTE 2 ---------------------------------------------------------------------------
        // bit 0, 1, 2 NOTE: COOLANT TEMP SENSOR NOT FUNCTIONAL
        // static bool warn_water_temp(VehicleTuneController& t){return Car.ACU1.get() > t.getBatteryWarnTemp() && .getWaterTemp() < t.getCoolantLimitTemp();} //TODO: Implement in NODES
        // static bool limit_water_temp(VehicleTuneController& t){return Car.ACU1.getWaterTemp() > t.getCoolantLimitTemp() && Car.ACU1.getWaterTemp() < t.getCoolantCriticalTemp();}
        // static bool critical_water_temp(VehicleTuneController& t){return Car.ACU1.getWaterTemp() > t.getCoolantCriticalTemp();}
        // bit 3, 4, 5
        static bool warn_mcu_temp(VehicleTuneController& t, Vehicle& Car) {return Car.DTI.getInvTemp() > t.getInverterWarnTemp() && Car.DTI.getInvTemp() < t.getInverterLimitTemp();}
        static bool limit_mcu_temp(VehicleTuneController& t, Vehicle& Car){return Car.DTI.getInvTemp() > t.getInverterLimitTemp() && Car.DTI.getInvTemp() < t.getInverterCriticalTemp();}
        static bool critical_mcu_temp(VehicleTuneController& t, Vehicle& Car) {return Car.DTI.getInvTemp() > t.getInverterCriticalTemp();}
        // bit 6
        // static bool TCM_fault(VehicleTuneController& t) {return false;} // TODO: do
        // bit 7 empty for now


};
#endif