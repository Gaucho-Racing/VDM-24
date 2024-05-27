//start header file
#include "Arduino.h"
#include <FlexCAN_T4.h>
#include "Nodes.h"
#ifndef VEHICLE_H
#define VEHICLE_H

struct sendTimes{
    unsigned long lastPrechargeTime; // last precharge request in millis
    unsigned long lastDTIMessage; // last inverter message in millis    
    unsigned long lastDashLEDMessage; // last dash panel led message in millis
    unsigned long lastPingSend; // last send on 0xF2 in millis
    unsigned long lastPingRequestAttempt; // last request for all Pings in millis
    sendTimes(){
        lastPrechargeTime = 0;
        lastDTIMessage = 0;
        lastDashLEDMessage = 0;
        lastPingSend = 0;
        lastPingRequestAttempt = 0;
    };

    unsigned long getLastPrechargeTime(){return lastPrechargeTime;}
    unsigned long getLastDTIMessage() { return lastDTIMessage; }
    unsigned long getLastDashLEDMessage() { return lastDashLEDMessage; }
    unsigned long getLastPingSend() { return lastPingSend; }
    unsigned long getLastPingRequestAttempt() { return lastPingRequestAttempt; }

    void setLastPrechargeTime(unsigned long t){lastPrechargeTime = t;}
    void setLastDTIMessage(unsigned long t) { lastDTIMessage = t; }
    void setLastDashLEDMessage(unsigned long t) { lastDashLEDMessage = t; }
    void setLastPingSend(unsigned long t) { lastPingSend = t; }
    void setLastPingRequestAttempt(unsigned long t) { lastPingRequestAttempt = t; }
};

struct Vehicle{
    //attributes
    FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_primary;
    FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> can_data;
    Inverter DTI = Inverter(22, can_primary);
    VDM ECU =  VDM(can_primary, can_data);
    Wheel WFL =  Wheel(can_data, WHEEL_FL);
    Wheel WFR = Wheel(can_data, WHEEL_FR);
    Wheel WRL = Wheel(can_data, WHEEL_RL);
    Wheel WRR = Wheel(can_data, WHEEL_RR);
    GPS GPS1 = GPS(can_data);
    Pedals PEDALS = Pedals(can_primary);
    ACU ACU1 = ACU(can_primary);
    TCM TCM1 = TCM(can_data);
    Dash DASHBOARD = Dash(can_primary);
    Energy_Meter ENERGY_METER = Energy_Meter(can_primary);
    SteeringWheel STEERING_WHEEL = SteeringWheel(can_primary);
    byte IMU[3][8] = {0x00};
    struct sendTimes times;
    


    //constructor
    Vehicle(FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16>& canprimary, FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16>& candata){
        this->can_primary = canprimary;
        this->can_data = candata;
        times = sendTimes();
        
    }

};


#endif