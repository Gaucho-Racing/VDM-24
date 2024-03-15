#ifndef SYSTEMS_CHECK
#define SYSTEMS_CHECK

#include "machine.h"
#include <unordered_set>
#include "main.h"
using namespace std;


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
    
   


    // actual calls
    static void hardware_system_critical(const iCANflex& Car);
    static void system_faults(const iCANflex& Car);
    static void system_limits(const iCANflex& Car);
    static void system_warnings(const iCANflex& Car);

    /*
    5 bytes of 8 bits:
    [can warn][can failure][AMS][IMD][BSPD][SDC][][]
    [warn motor][limit motor][crit motor][warn batt][limit batt][crit batt][rev limit][] temps motor and battery, Revs
    [warn water][limit water][crit water][warn mcu][limit mcu][crit mcu][TCM Status][] // water temp DTI temp, TCM
    [][][][][][][][] 
    [][][][][][][][] 
    */

    // SYS_CHECK_CAN_FRAME[0]
    static bool warn_can_failure(const iCANflex& Car); // byte 0, bit 0
    static bool critical_can_failure(const iCANflex& Car); // byte 0, bit 1
    static bool AMS_fault(const iCANflex& Car); // byte 0, bit 2
    static bool IMD_fault(const iCANflex& Car); // byte 0, bit 3
    static bool BSPD_fault(const iCANflex& Car); // byte 0, bit 4
    static bool SDC_opened(const iCANflex& Car); // byte 0, bit 5

    // SYS_CHECK_CAN_FRAME[1]
    static bool warn_motor_temp(const iCANflex& Car); // byte 1, bit 0
    static bool limit_motor_temp(const iCANflex& Car); // byte 1, bit 1
    static bool critical_motor_temp(const iCANflex& Car); // byte 1, bit 2
    static bool warn_battery_temp(const iCANflex& Car); // byte 1, bit 3
    static bool limit_battery_temp(const iCANflex& Car); // byte 1, bit 4
    static bool critical_battery_temp(const iCANflex& Car); // byte 1, bit 5
    static bool rev_limit_exceeded(const iCANflex& Car); // byte 1, bit 6

    // SYS_CHECK_CAN_FRAME[2]
    static bool warn_water_temp(const iCANflex& Car);
    static bool limit_water_temp(const iCANflex& Car);
    static bool critical_water_temp(const iCANflex& Car);
    static bool warn_mcu_temp(const iCANflex& Car);
    static bool limit_mcu_temp(const iCANflex& Car);
    static bool critical_mcu_temp(const iCANflex& Car);
    static bool TCM_fault(const iCANflex& Car);

    // TODO: add more system checks


};  

#endif
