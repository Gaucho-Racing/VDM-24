#ifndef SYSTEMS_CHECK
#define SYSTEMS_CHECK

#include "machine.h"
#include "main.h"
#include <vector>


class SystemsCheck{
    private:
    static const uint8_t CAN_MS_THRESHOLD = 100; // msec
    static const uint8_t MOTOR_TEMP_WARN = 60; // celsius
    static const uint8_t MOTOR_TEMP_LIMIT = 65; // celsius
    static const uint8_t MOTOR_TEMP_CRITICAL = 70; // celsius
    static const uint8_t BATTERY_TEMP_WARN = 60; // celsius
    static const uint8_t BATTERY_TEMP_LIMIT = 65; // celsius
    static const uint8_t BATTERY_TEMP_CRITICAL = 70; // celsius
    static const uint8_t WATER_TEMP_WARN = 60; // celsius
    static const uint8_t WATER_TEMP_LIMIT = 65; // celsius
    static const uint8_t WATER_TEMP_CRITICAL = 70; // celsius
    static const uint8_t MCU_TEMP_WARN = 60; // celsius
    static const uint8_t MCU_TEMP_LIMIT = 65; // celsius
    static const uint8_t MCU_TEMP_CRITICAL = 70; // celsius


    public:
    
    static byte system_check_can_packet[8];
    /*
    8 bytes of 8 bits:
    [warn motor][limit motor][crit motor][warn batt][limit batt][crit batt][rev limit][] temps motor and battery, Revs
    [warn water][limit water][crit water][warn mcu][limit mcu][crit mcu][TCM Status][] // water temp DTI temp, TCM
    [can warn][can failure][][][][][][]
    [][][][][][][][]
    [][][][][][][][]
    [][][][][][][][]
    [][][][][][][][]
    [][][][][][][][]
    */



    static void hardware_system_critical(const iCANflex& Car);



    // read bspd, ams, and imd pins as analog
    // .5v is shit -  ADC: 155
    // 3v when almost ok - ADC: 930
    // 2.4v is ok - ADC: 744
    // 1v = 310
    static bool AMS_fault(const iCANflex& Car);
    static bool IMD_fault(const iCANflex& Car);
    static bool BSPD_fault(const iCANflex& Car);

    static bool SDC_opened(const iCANflex& Car);

    static bool critical_motor_temp(const iCANflex& Car);
    static bool limit_motor_temp(const iCANflex& Car);
    static bool warn_motor_temp(const iCANflex& Car);

    static bool critical_battery_temp(const iCANflex& Car);
    static bool limit_battery_temp(const iCANflex& Car);
    static bool warn_battery_temp(const iCANflex& Car);

    static bool critical_water_temp(const iCANflex& Car);
    static bool limit_water_temp(const iCANflex& Car);
    static bool warn_water_temp(const iCANflex& Car);

    static bool critical_mcu_temp(const iCANflex& Car);
    static bool limit_mcu_temp(const iCANflex& Car);
    static bool warn_mcu_temp(const iCANflex& Car);

    static bool rev_limit_exceeded(const iCANflex& Car);  

    // CAN RECIEVE FAILURES
    static bool critical_can_failure(const iCANflex& Car); 
    static bool warn_can_failure(const iCANflex& Car);
};  

#endif
