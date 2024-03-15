#include "systems_check.h"
#include "main.h"

void SystemsCheck::hardware_system_critical(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &af){

    if(SDC_opened(Car)) af.insert(SDC_opened);  
    SYS_CHECK_CAN_FRAME[0] = SDC_opened(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
    if(AMS_fault(Car)) {
        Serial.println("AMS FAULT");
        af.insert(AMS_fault);
        Serial.print("New size of set: ");
        Serial.println(af.size());

    }
    SYS_CHECK_CAN_FRAME[0] = AMS_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00100000) : (SYS_CHECK_CAN_FRAME[0] & 0b11011111);
    if(IMD_fault(Car)) af.insert(IMD_fault);
    SYS_CHECK_CAN_FRAME[0] = IMD_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00010000) : (SYS_CHECK_CAN_FRAME[0] & 0b11101111);
    if(BSPD_fault(Car)) af.insert(BSPD_fault);
    SYS_CHECK_CAN_FRAME[0] = BSPD_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
}

// NOTE: OPEN THE SOFTWARE LATCH IF the Inverter is not responding or there are critical system faults. 
void SystemsCheck::system_faults(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &af){
    if(critical_motor_temp(Car)) af.insert(critical_motor_temp);
    SYS_CHECK_CAN_FRAME[1] = critical_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00100000) : (SYS_CHECK_CAN_FRAME[1] & 0b11011111);
    if(critical_battery_temp(Car)) af.insert(critical_battery_temp);
    SYS_CHECK_CAN_FRAME[1] = critical_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000100) : (SYS_CHECK_CAN_FRAME[1] & 0b11111011);
    if(critical_water_temp(Car)) af.insert(critical_water_temp);
    SYS_CHECK_CAN_FRAME[2] = critical_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00100000) : (SYS_CHECK_CAN_FRAME[2] & 0b11011111);
    if(critical_mcu_temp(Car)) af.insert(critical_mcu_temp);
    SYS_CHECK_CAN_FRAME[2] = critical_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00000100) : (SYS_CHECK_CAN_FRAME[2] & 0b11111011);
    if(critical_can_failure(Car)) af.insert(critical_can_failure);
    SYS_CHECK_CAN_FRAME[0] = critical_can_failure(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b01000000) : (SYS_CHECK_CAN_FRAME[0] & 0b10111111);
    
}

void SystemsCheck::system_limits(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &al){
    if(limit_motor_temp(Car)) al.insert(limit_motor_temp);
    SYS_CHECK_CAN_FRAME[1] = limit_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b01000000) : (SYS_CHECK_CAN_FRAME[1] & 0b10111111);
    if(limit_battery_temp(Car)) al.insert(limit_battery_temp);
    SYS_CHECK_CAN_FRAME[1] = limit_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00001000) : (SYS_CHECK_CAN_FRAME[1] & 0b11110111);
    if(limit_water_temp(Car)) al.insert(limit_water_temp);
    SYS_CHECK_CAN_FRAME[2] = limit_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b01000000) : (SYS_CHECK_CAN_FRAME[2] & 0b10111111);
    if(limit_mcu_temp(Car)) al.insert(limit_mcu_temp);
    SYS_CHECK_CAN_FRAME[2] = limit_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00001000) : (SYS_CHECK_CAN_FRAME[2] & 0b11110111);
}

void SystemsCheck::system_warnings(const iCANflex& Car, unordered_set<bool (*)(const iCANflex&)> &aw){
    if(warn_motor_temp(Car)) aw.insert(warn_motor_temp);
    SYS_CHECK_CAN_FRAME[1] = warn_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b10000000) : (SYS_CHECK_CAN_FRAME[1] & 0b01111111);
    if(warn_battery_temp(Car)) aw.insert(warn_battery_temp);
    SYS_CHECK_CAN_FRAME[1] = warn_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00010000) : (SYS_CHECK_CAN_FRAME[1] & 0b11101111);
    if(warn_water_temp(Car)) aw.insert(warn_water_temp);
    SYS_CHECK_CAN_FRAME[2] = warn_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b10000000) : (SYS_CHECK_CAN_FRAME[2] & 0b01111111);
    if(warn_mcu_temp(Car)) aw.insert(warn_mcu_temp);
    SYS_CHECK_CAN_FRAME[2] = warn_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00010000) : (SYS_CHECK_CAN_FRAME[2] & 0b11101111);
}




// BYTE 0 ---------------------------------------------------------------------------
// bit 0
bool SystemsCheck::warn_can_failure(const iCANflex& Car){
    bool fail =  
        Car.WFL.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WFR.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WRL.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WRR.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.DASHBOARD.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.GPS1.getAge() > SystemsCheck::CAN_MS_THRESHOLD);
}   
// bit 1
bool SystemsCheck::critical_can_failure(const iCANflex& Car){
    return (Car.DTI.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        // Car.ECU.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.PEDALS.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ACU1.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.BCM1.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ENERGY_METER.getAge() > SystemsCheck::CAN_MS_THRESHOLD);
}


// HARDWARE FAULTS: VERY BAD
// .5v is shit -  ADC: 155
// 3v when almost ok - ADC: 930
// 2.4v is ok - ADC: 744
// 1v = 310
// bits 3, 4, 5, 6
bool SystemsCheck::AMS_fault(const iCANflex& Car){ return analogRead(AMS_OK_PIN) < 700 || analogRead(AMS_OK_PIN) > 790; }
bool SystemsCheck::IMD_fault(const iCANflex& Car){ return analogRead(IMD_OK_PIN) < 700 || analogRead(IMD_OK_PIN) > 790; }
bool SystemsCheck::BSPD_fault(const iCANflex& Car){ return analogRead(BSPD_OK_PIN) < 700 || analogRead(BSPD_OK_PIN) > 790; }
bool SystemsCheck::SDC_opened(const iCANflex& Car){ return false;} // TODO: READ VOLTAGE JUST BEFORE THE AIRS


// BYTE 1 ---------------------------------------------------------------------------
// bit 0, 1, 2
bool SystemsCheck::warn_motor_temp(const iCANflex& Car){return Car.DTI.getMotorTemp() > MOTOR_TEMP_WARN && Car.DTI.getMotorTemp() < MOTOR_TEMP_LIMIT;}
bool SystemsCheck::limit_motor_temp(const iCANflex& Car){return Car.DTI.getMotorTemp() > MOTOR_TEMP_LIMIT && Car.DTI.getMotorTemp() < MOTOR_TEMP_CRITICAL;}
bool SystemsCheck::critical_motor_temp(const iCANflex& Car){return Car.DTI.getMotorTemp() > MOTOR_TEMP_CRITICAL;}
// bit 3, 4, 5
bool SystemsCheck::warn_battery_temp(const iCANflex& Car) {return Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_WARN && Car.ACU1.getMaxCellTemp() < BATTERY_TEMP_LIMIT;}
bool SystemsCheck::limit_battery_temp(const iCANflex& Car) {return Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_LIMIT && Car.ACU1.getMaxCellTemp() < BATTERY_TEMP_CRITICAL;}
bool SystemsCheck::critical_battery_temp(const iCANflex& Car) {return Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_CRITICAL;}
// bit 6
bool SystemsCheck::rev_limit_exceeded(const iCANflex& Car) {return Car.DTI.getERPM()/10 > REV_LIMIT;}
// bit 7 empty for now

// BYTE 2 ---------------------------------------------------------------------------
// bit 0, 1, 2
bool SystemsCheck::warn_water_temp(const iCANflex& Car){return Car.ACU1.getWaterTemp() > WATER_TEMP_WARN && Car.ACU1.getWaterTemp() < WATER_TEMP_LIMIT;}
bool SystemsCheck::limit_water_temp(const iCANflex& Car){return Car.ACU1.getWaterTemp() > WATER_TEMP_LIMIT && Car.ACU1.getWaterTemp() < WATER_TEMP_CRITICAL;}
bool SystemsCheck::critical_water_temp(const iCANflex& Car){return Car.ACU1.getWaterTemp() > WATER_TEMP_CRITICAL;}
// bit 3, 4, 5
bool SystemsCheck::warn_mcu_temp(const iCANflex& Car) {return Car.DTI.getInvTemp() > MCU_TEMP_WARN;}
bool SystemsCheck::limit_mcu_temp(const iCANflex& Car){return Car.DTI.getInvTemp() > MCU_TEMP_LIMIT;}
bool SystemsCheck::critical_mcu_temp(const iCANflex& Car) {return Car.DTI.getInvTemp() > MCU_TEMP_CRITICAL;}
// bit 6
bool SystemsCheck::TCM_fault(const iCANflex& Car) {return false;} // TODO: do
// bit 7 empty for now




