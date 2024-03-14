#include "systems_check.h"


void SystemsCheck::hardware_system_critical(const iCANflex& Car){

    if(SDC_opened(Car)) fault_heap.push(SDC_opened);  
    SYS_CHECK_CAN_FRAME[0] = SDC_opened(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
    if(AMS_fault(Car)) fault_heap.push(AMS_fault);
    SYS_CHECK_CAN_FRAME[0] = AMS_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00100000) : (SYS_CHECK_CAN_FRAME[0] & 0b11011111);
    if(IMD_fault(Car)) fault_heap.push(IMD_fault);
    SYS_CHECK_CAN_FRAME[0] = IMD_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00010000) : (SYS_CHECK_CAN_FRAME[0] & 0b11101111);
    if(BSPD_fault(Car)) fault_heap.push(BSPD_fault);
    SYS_CHECK_CAN_FRAME[0] = BSPD_fault(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b00000100) : (SYS_CHECK_CAN_FRAME[0] & 0b11111011);
}

// NOTE: OPEN THE SOFTWARE LATCH IF the Inverter is not responding or there are critical system faults. 
void SystemsCheck::system_faults(const iCANflex& Car){
    if(critical_motor_temp(Car)) fault_heap.push(critical_motor_temp);
    SYS_CHECK_CAN_FRAME[1] = critical_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00100000) : (SYS_CHECK_CAN_FRAME[1] & 0b11011111);
    if(critical_battery_temp(Car)) fault_heap.push(critical_battery_temp);
    SYS_CHECK_CAN_FRAME[1] = critical_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00000100) : (SYS_CHECK_CAN_FRAME[1] & 0b11111011);
    if(critical_water_temp(Car)) fault_heap.push(critical_water_temp);
    SYS_CHECK_CAN_FRAME[2] = critical_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00100000) : (SYS_CHECK_CAN_FRAME[2] & 0b11011111);
    if(critical_mcu_temp(Car)) fault_heap.push(critical_mcu_temp);
    SYS_CHECK_CAN_FRAME[2] = critical_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00000100) : (SYS_CHECK_CAN_FRAME[2] & 0b11111011);
    if(critical_can_failure(Car)) fault_heap.push(critical_can_failure);
    SYS_CHECK_CAN_FRAME[0] = critical_can_failure(Car) ? (SYS_CHECK_CAN_FRAME[0] | 0b01000000) : (SYS_CHECK_CAN_FRAME[0] & 0b10111111);
}

void SystemsCheck::system_limits(const iCANflex& Car){
    if(limit_motor_temp(Car)) limit_heap.push(limit_motor_temp);
    SYS_CHECK_CAN_FRAME[1] = limit_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b01000000) : (SYS_CHECK_CAN_FRAME[1] & 0b10111111);
    if(limit_battery_temp(Car)) limit_heap.push(limit_battery_temp);
    SYS_CHECK_CAN_FRAME[1] = limit_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00001000) : (SYS_CHECK_CAN_FRAME[1] & 0b11110111);
    if(limit_water_temp(Car)) limit_heap.push(limit_water_temp);
    SYS_CHECK_CAN_FRAME[2] = limit_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b01000000) : (SYS_CHECK_CAN_FRAME[2] & 0b10111111);
    if(limit_mcu_temp(Car)) limit_heap.push(limit_mcu_temp);
    SYS_CHECK_CAN_FRAME[2] = limit_mcu_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b00001000) : (SYS_CHECK_CAN_FRAME[2] & 0b11110111);
}

void SystemsCheck::system_warnings(const iCANflex& Car){
    if(warn_motor_temp(Car)) warning_heap.push(warn_motor_temp);
    SYS_CHECK_CAN_FRAME[1] = warn_motor_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b10000000) : (SYS_CHECK_CAN_FRAME[1] & 0b01111111);
    if(warn_battery_temp(Car)) warning_heap.push(warn_battery_temp);
    SYS_CHECK_CAN_FRAME[1] = warn_battery_temp(Car) ? (SYS_CHECK_CAN_FRAME[1] | 0b00010000) : (SYS_CHECK_CAN_FRAME[1] & 0b11101111);
    if(warn_water_temp(Car)) warning_heap.push(warn_water_temp);
    SYS_CHECK_CAN_FRAME[2] = warn_water_temp(Car) ? (SYS_CHECK_CAN_FRAME[2] | 0b10000000) : (SYS_CHECK_CAN_FRAME[2] & 0b01111111);
    if(warn_mcu_temp(Car)) active_warnings.insert(warn_mcu_temp);
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
    // if(fail) system_check_can_packet[2] = (system_check_can_packet[2] | 0b01000000);
    // else system_check_can_packet[2] = (system_check_can_packet[2] & 0b10111111);
}   




