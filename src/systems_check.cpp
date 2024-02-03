#include "systems_check.h"


void SystemsCheck::hardware_system_critical(const iCANflex& Car){
    if(SDC_opened(Car)) active_faults.insert(SDC_opened);   
    if(AMS_fault(Car)) active_faults.insert(AMS_fault);
    if(IMD_fault(Car)) active_faults.insert(IMD_fault);
    if(BSPD_fault(Car)) active_faults.insert(BSPD_fault);
}
// NOTE: OPEN THE SOFTWARE LATCH IF the Inverter is not responding or the ERROR is not properly handled. 


// HARDWARE FAULTS: VERY BAD
// EITHER SDC IS OPENED 
bool SystemsCheck::AMS_fault(const iCANflex& Car){
    if(analogRead(AMS_OK_PIN) < 310) return true; // and send a can light thing
    else if(analogRead(AMS_OK_PIN) > 730 && analogRead(AMS_OK_PIN) < 760) return false;
    return true;
}
bool SystemsCheck::IMD_fault(const iCANflex& Car){
    if(analogRead(IMD_OK_PIN) < 310) return true; // and can light thing
    else if(analogRead(IMD_OK_PIN) > 730 && analogRead(IMD_OK_PIN) < 760) return false;
    return true;
}
bool SystemsCheck::BSPD_fault(const iCANflex& Car){
    if(analogRead(BSPD_OK_PIN) < 310) return true;
    else if(analogRead(BSPD_OK_PIN) > 730 && analogRead(BSPD_OK_PIN) < 760) return false;
    return true;
}
bool SystemsCheck::SDC_opened(const iCANflex& Car){
    return false; // TODO: implement based on AIRS from ACU
    // read voltage on SDC just before AIRS
}



// FAULTS DETECTED THROUGH PREPROCESSED DATA ON CAN
// byte 1
bool SystemsCheck::warn_motor_temp(const iCANflex& Car){
   if(Car.DTI.getMotorTemp() > MOTOR_TEMP_WARN){
        system_check_can_packet[0] = (system_check_can_packet[0] | 0b10000000);
        return true;
   }
   else {
        system_check_can_packet[0] = (system_check_can_packet[0] & 0b01111111);
        return false;
   }
}

bool SystemsCheck::limit_motor_temp(const iCANflex& Car){
   if(Car.DTI.getMotorTemp() > MOTOR_TEMP_LIMIT){
        system_check_can_packet[0] = (system_check_can_packet[0] | 0b01000000);
        return true;
   }
   else {
        system_check_can_packet[0] = (system_check_can_packet[0] & 0b10111111);
        return false;
   }
}
bool SystemsCheck::critical_motor_temp(const iCANflex& Car){
    if(Car.DTI.getMotorTemp() > MOTOR_TEMP_CRITICAL){
          system_check_can_packet[0] = (system_check_can_packet[0] | 0b00100000);
          return true;
    }
    else {
          system_check_can_packet[0] = (system_check_can_packet[0] & 0b11011111);
          return false;
    }
}


bool SystemsCheck::warn_battery_temp(const iCANflex& Car){
    if(Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_WARN){
        system_check_can_packet[0] = (system_check_can_packet[0] | 0b00010000);
        return true;
    }
    else {
        system_check_can_packet[0] = (system_check_can_packet[0] & 0b11101111);
        return false;
    }
}

bool SystemsCheck::limit_battery_temp(const iCANflex& Car){
    if(Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_LIMIT){
        system_check_can_packet[0] = (system_check_can_packet[0] | 0b00001000);
        return true;
    }
    else {
        system_check_can_packet[0] = (system_check_can_packet[0] & 0b11110111);
        return false;
    }
}

bool SystemsCheck::critical_battery_temp(const iCANflex& Car){
    if(Car.ACU1.getMaxCellTemp() > BATTERY_TEMP_CRITICAL){
        system_check_can_packet[0] = (system_check_can_packet[0] | 0b00000100);
        return true;
    }
    else {
        system_check_can_packet[0] = (system_check_can_packet[0] & 0b11111011);
        return false;
    }
}

bool SystemsCheck::rev_limit_exceeded(const iCANflex& Car){
    if(Car.DTI.getERPM()/10.0 > REV_LIMIT){
        system_check_can_packet[0] = (system_check_can_packet[0] | 0b00000010);
        return true;
    }
    else {
        system_check_can_packet[0] = (system_check_can_packet[0] & 0b11111101);
        return false;
    }
}

//byte 2

bool SystemsCheck::warn_water_temp(const iCANflex& Car){
    if(Car.ACU1.getWaterTemp() > WATER_TEMP_WARN){
        system_check_can_packet[1] = (system_check_can_packet[1] | 0b10000000);
        return true;
    }
    else {
        system_check_can_packet[1] = (system_check_can_packet[1] & 0b01111111);
        return false;
    }
}

bool SystemsCheck::limit_water_temp(const iCANflex& Car){
    if(Car.ACU1.getWaterTemp() > WATER_TEMP_LIMIT){
        system_check_can_packet[1] = (system_check_can_packet[1] | 0b01000000);
        return true;
    }
    else {
        system_check_can_packet[1] = (system_check_can_packet[1] & 0b10111111);
        return false;
    }
}

bool SystemsCheck::critical_water_temp(const iCANflex& Car){
   if(Car.ACU1.getWaterTemp() > WATER_TEMP_CRITICAL){
        system_check_can_packet[1] = (system_check_can_packet[1] | 0b00100000);
        return true;
   }
   else {
        system_check_can_packet[1] = (system_check_can_packet[1] & 0b11011111);
        return false;
   }
}

bool SystemsCheck::warn_mcu_temp(const iCANflex& Car){
    if(Car.DTI.getInvTemp() > MCU_TEMP_WARN){
        system_check_can_packet[1] = (system_check_can_packet[1] | 0b00010000);
        return true;
    }
    else {
        system_check_can_packet[1] = (system_check_can_packet[1] & 0b11101111);
        return false;
    }
}

bool SystemsCheck::limit_mcu_temp(const iCANflex& Car){
    if(Car.DTI.getInvTemp() > MCU_TEMP_LIMIT){
        system_check_can_packet[1] = (system_check_can_packet[1] | 0b00001000);
        return true;
    }
    else {
        system_check_can_packet[1] = (system_check_can_packet[1] & 0b11110111);
        return false;
    }
}

bool SystemsCheck::critical_mcu_temp(const iCANflex& Car){
    if(Car.DTI.getInvTemp() > MCU_TEMP_CRITICAL){
        system_check_can_packet[1] = (system_check_can_packet[1] | 0b00000100);
        return true;
    }
    else {
        system_check_can_packet[1] = (system_check_can_packet[1] & 0b11111011);
        return false;
    }
}






bool SystemsCheck::critical_can_failure(const iCANflex& Car){
    return 
        Car.DTI.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ECU.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.PEDALS.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ACU1.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.BCM1.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.ENERGY_METER.getAge() > SystemsCheck::CAN_MS_THRESHOLD;
}

bool SystemsCheck::warn_can_failure(const iCANflex& Car){
    return 
        Car.WFL.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WFR.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WRL.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.WRR.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.DASHBOARD.getAge() > SystemsCheck::CAN_MS_THRESHOLD ||
        Car.GPS1.getAge() > SystemsCheck::CAN_MS_THRESHOLD;
}   


