
// Nikunj Parasar 
// Created: 10/20/2023
// GAUCHO RACING CAN NODES SOFTWARE (FLEXCAN_T4)
// This file contains the CAN nodes for the GR24 EV

// //////////////////////////////////////////////////////////////////////ADJUST UNITS LATER AS NEEDED
// https://docs.google.com/spreadsheets/d/1XfJhhAQoDnuSuwluNitPsDWtuQu-bP-VbEGPmSo5ujA/edit#gid=1132363474
#ifndef NODES
#define NODES

#include "config.h"
#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <SPI.h>
#include <stdio.h>
#include <stdlib.h>
#include <unordered_map>
#define USE_CAN_PRIMARY
// #define USE_CAN_DATA  


#if defined(USE_CAN_PRIMARY) && !defined(USE_CAN_DATA)
    #define CAN_PRIMARY_BUS CAN1
    #define CAN_DATA_BUS CAN3  // Default to CAN3 if CAN_PRIMARY_BUS is selected
#elif !defined(USE_CAN_PRIMARY) && defined(USE_CAN_DATA)
    #define CAN_PRIMARY_BUS CAN3
    #define CAN_DATA_BUS CAN1
#else
    #error "Please define either USE_CAN_PRIMARY or USE_CAN_DATA"
#endif



//not touching this hoe.
struct Inverter {
    byte data[5][8]; 
    
    /*
    example data packets from CAN bus:
    ---id ------------ byte ---------------
           | 0              | 1             | 2             | 3             | 4             | 5             | 6             | 7         |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2016 |                		       ERPM	                            |            Duty Cycle 	    |         Input Voltage	    |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2116 |             AC Current		    |          DC Current	        |	                        RESERVED			            |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2216 |         Controller Temp		|           Motor Temp		    |    Faults	    |         RESERVED		                    |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2316 |                         FOC Id		                          	|                       	FOC Iq			                |
    ----------------------------------------------------------------------------------------------------------------------------------
    0x2416 |  Throttle      |	Brake	    |   Digital IO	|  Drive Enable	|     Flags     |	Flags	    | RESERVED	    |CAN Version
    -----------------------------------------------------------------------------------------------------------------------------------------
    */



    unsigned long ID = 0;
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Inverter(unsigned long id, FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can) : ID(id){
        can = Can1;
    }

    Inverter(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id >= DTI_Data_1 && id <= DTI_Data_5){
            byte digit2 = (id >> 8) & 0xF; // 0 for 0x2016, 1 for 0x2116, 2 for 0x2216, 3 for 0x2316, 4 for 0x2416
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[digit2][i] = buf[i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        }
        else{
            return 0;
        }
        return 1;
    }
    void send(long OutId, long data, int dataLength){    //Sends 8 bytes with that Id and that data shifted left all the way
        byte stuff[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        for(int i=0; i<dataLength; i++)
            stuff[i] = (data >> ((dataLength-i-1)*8));
        for (int i = 0; i < 8; i++)
            msg.buf[i] = stuff[i];
        msg.id = ((long)OutId << 8) + ID;
        msg.flags.extended=true;
        Can1.write(msg);
        for(int i = 0; i < 8; i++) msg.buf[i] = 0x00;   //Wipe the buffer so leaks into future messages
    }

    unsigned long getID() {return ID;}

    long getERPM() const {return(((long)data[0][0] << 24) + ((long)data[0][1] << 16) + ((long)data[0][2] << 8) + data[0][3]);} //rpm/pole pairs
    float getDuty() const {return((((long)data[0][4] << 8) + data[0][5])/10);} //i think [0,100]. Related to top speed
    int getVoltIn() const {return(((long)data[0][6] << 8) + data[0][7]);}
    float getACCurrent() const {return((((long)data[1][0] << 8) + data[1][1])/10);}
    float getDCCurrent() const {return(((long)(data[1][2] << 8) + data[1][3])/10);}
    float getInvTemp() const {return((((long)data[2][0] << 8) + data[2][1])/10);} //Deg C
    float getMotorTemp() const {return((((long)data[2][2] << 8) + data[2][3])/10);} //Deg C
    byte getFaults() const {return data[2][4];}
    float getCurrentD() const {return((((long)data[3][0] << 24) + ((long)data[3][1] << 16) + ((long)data[3][2] << 8) + data[3][3])/100);}  //FOC current (don't need)
    float getCurrentQ() const {return((((long)data[3][4] << 24) + ((long)data[3][5] << 16) + ((long)data[3][6] << 8) + data[3][7])/100);}  //FOC current (don't need)
    byte getThrottleIn() const {return data[4][0];}  //Received throttle signal by the invertor
    byte getBrakeIn() const {return data[4][1];}  //Received brake signal by the invertor
    bool getD1() const {return ((data[4][2] & 0x80) == 0x80);}  //Digital input read
    bool getD2() const {return ((data[4][2] & 0x40) == 0x40);}  //Digital input read
    bool getD3() const {return ((data[4][2] & 0x20) == 0x20);}  //Digital input read
    bool getD4() const {return ((data[4][2] & 0x10) == 0x10);}  //Digital input read
    bool getDO1() const {return ((data[4][2] & 0x08) == 0x08);}  //Digital output write
    bool getDO2() const {return ((data[4][2] & 0x04) == 0x04);}  //Digital output write
    bool getDO3() const {return ((data[4][2] & 0x02) == 0x02);}  //Digital output write
    bool getDO4() const {return ((data[4][2] & 0x01) == 0x01);}  //Digital output write
    bool getDriveEnable() const {return ((data[4][3] & 0x01) == 0x01);} //These are setting that can be changed (prob don't need these)
    bool getCapTempLim() const {return ((data[4][4] & 0x80) == 0x80);}//         ^
    bool getDCCurrentLim() const {return ((data[4][4] & 0x40) == 0x40);}//       ^
    bool getDriveEnableLim() const {return ((data[4][4] & 0x20) == 0x20);}//     ^
    bool getIgbtAccelTempLim() const {return ((data[4][4] & 0x10) == 0x10);}//   ^
    bool getIgbtTempLim() const {return ((data[4][4] & 0x08) == 0x08);}//        ^
    bool getVoltInLim() const {return ((data[4][4] & 0x04) == 0x04);}//          ^
    bool getMotorAccelTempLim() const {return ((data[4][4] & 0x02) == 0x02);}//  ^
    bool getMotorTempLim() const {return ((data[4][4] & 0x01) == 0x01);}//       ^
    bool getRPMMinLimit() const {return ((data[4][5] & 0x80) == 0x80);}//        ^
    bool getRPMMaxLimit() const {return ((data[4][5] & 0x40) == 0x40);}//        ^
    bool getPowerLimit() const {return ((data[4][5] & 0x20) == 0x20);}//    

    void setCurrent(float in) {send(0x1, (long)(in*10), 2);}//            ^
    void setBrakeCurrent(float in) {send(0x2, (long)(in*10), 2);}//       ^
    void setERPM(long in) {send(0x3, (long)in, 4);}//                     ^
    void setPosition(float in) {send(0x4, (long)in, 2);}//                ^
    void setRCurrent(float in) {send(0x5, (long)(in*10), 2);}//           ^
    void setRBrakeCurrent(float in) {send(0x6, (long)(in*10), 2);}//      ^
    void setMaxCurrent(float in) {send(0x8, (long)(in*10), 2);}//         ^
    void setMaxBrakeCurrent(float in) {send(0x9, (long)(in*10), 2);}//    ^
    void setMaxDCCurrent(float in) {send(0xA, (long)(in*10), 2);}//       ^
    void setMaxDCBrakeCurrent(float in) {send(0xB, (long)(in*10), 2);}//  ^
    void setDriveEnable(byte in) {send(0xC, (long)in, 1);} //Enable/disable motor
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet
};





struct VDM{    
    byte data[6][8];
    byte dataOut[8];
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    // not needed as it is stupid and recieve function passes through buf and id so goog FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can2; //this should be the data can
    CAN_message_t msg;
    CANFD_message_t msgFD;
    unsigned long receiveTime = 0;



    VDM(FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can, FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can2){
        can = Can1;
        can2 = Can2;
    }
    VDM(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id >= 0xF0 && id <= 0xF5){
            int row = id-0xF0;
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[row][i] = buf[i];
        }
        else{
            return 0;
        }
        return 1;
    }

    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

    
    byte getVCU_STATE() const {return data[0][0];}
    
    byte SystemHealth0() const {return data[1][0];}  

    bool CAN_Warning() const {return (data[1][0] & 0b10000000);}
    bool CAN_Failure() const {return (data[1][0] & 0b01000000);}
    bool AMS_Failure() const {return (data[1][0] & 0b00100000);}
    bool IMD_Failure() const {return (data[1][0] & 0b00010000);} 
    bool BSPD_Failure() const {return (data[1][0] & 0b00001000);}
    bool SDC_Failure() const {return (data[1][0] & 0b00000100);}
    
    byte SystemHealth1() const {return data[1][1];}
    bool MOTOR_TEMP_Warning() const {return (data[1][1] & 0b10000000);}  
    bool MOTOR_TEMP_Limit() const {return (data[1][1] & 0b01000000);}
    bool MOTOR_TEMP_Critical() const {return (data[1][1] & 0b00100000);}
    bool BATTERY_TEMP_Warning() const {return (data[1][1] & 0b00010000);}
    bool BATTERY_TEMP_Limit() const {return (data[1][1] & 0b00001000);}
    bool BATTERY_TEMP_Critical() const {return (data[1][1] & 0b00000100);}
    bool REV_LIMIT_Warning() const {return (data[1][1] & 0b00000010);}

    byte SystemHealth2() const {return data[1][2];}
    bool COOLANT_TEMP_Warning() const {return (data[1][2] & 0b10000000);}
    bool COOLANT_TEMP_Limit() const {return (data[1][2] & 0b01000000);}
    bool COOLANT_TEMP_Critical() const {return (data[1][2] & 0b00100000);}
    bool INVERTER_TEMP_Warning() const {return (data[1][2] & 0b00010000);}  
    bool INVERTER_TEMP_Limit() const {return (data[1][2] & 0b00001000);}
    bool INVERTER_TEMP_Critical() const {return (data[1][2] & 0b00000100);}
    bool TCM_Failure() const {return (data[1][2] & 0b00000010);}    

    String getMotorHealth() const {
        if(MOTOR_TEMP_Critical()) return "CRITICAL FAULT";
        else if(MOTOR_TEMP_Limit()) return "THERMAL THROTTLING";
        else if(MOTOR_TEMP_Warning()) return "WARNING";
        else return "OK";
    }

    String getBatteryHealth() const {
        if(BATTERY_TEMP_Critical()) return "CRITICAL FAULT";
        else if(BATTERY_TEMP_Limit()) return "THERMAL THROTTLING";
        else if(BATTERY_TEMP_Warning()) return "WARNING";
        else return "OK";
    }

    String getCoolantHealth() const {
        if(COOLANT_TEMP_Critical()) return "CRITICAL FAULT";
        else if(COOLANT_TEMP_Limit()) return "THERMAL THROTTLING";
        else if(COOLANT_TEMP_Warning()) return "WARNING";
        else return "OK";
    }

    String getInverterHealth() const {
        if(INVERTER_TEMP_Critical()) return "CRITICAL FAULT";
        else if(INVERTER_TEMP_Limit()) return "THERMAL THROTTLING";
        else if(INVERTER_TEMP_Warning()) return "WARNING";
        else return "OK";
    }

    String getTCMHealth() const {
        if(TCM_Failure()) return "CLOUD DATA COMM ERROR";
        else return "OK";
    }



    
    void pingSensors(){} //this uses data can, need to write receive function for data CAN [dont think this is needed as this should go into pedals]
    //pings
    long pedalPingResponse() const { //TODO: check if this works arduino
        long time;
        for(int i = 0 ; i  < 4; i ++){
            time += (byte) data[0][i] << i*8;
        }
        return time;
    }
    byte steeringWheelPingResponse() const {
        long time;
        for(int i = 0 ; i  < 4; i ++){
            time += (byte) data[9][i] << i*8;
        }
        return time;
    } 
    byte dashPingResponse() const {
        long time;
        for(int i = 0 ; i  < 4; i ++){
            time += (byte) data[10][i] << i*8;
        }
        return time;
    } 

    

    private:
    void send_primary(int x){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = x;        //Fucked
        msg.flags.extended=true;
        Can1.write(msg);
        memset(dataOut, 0 , 8);   //Wipe the buffer so no leaks into future messages
    }

    void send_data(int x){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = x;        //Fucked
        msg.flags.extended=true;
        Can2.write(msg);
        memset(dataOut, 0 , 8);   //Wipe the buffer so no leaks into future messages
    }
    
    public:

    //IDK what the other recieving stuff is but its ther in the spreadsheet if needed to be implemented later

};





// Wheel type
// INITIALIZE WHEEL WITH WHEELTYPE AND I HANDLE THE LOCATION WITHIN THE CONSTRUCTOR
enum HubSensorArray{
    WHEEL_FR, //ids 0x10F00 - 0x10F04
    WHEEL_FL, //ids 0x10F08 - 0x10F0C
    WHEEL_RR, //ids 0x10F10 - 0x10F14
    WHEEL_RL  //ids 0x10F18 - 0x10F1C
};

struct Wheel {
    byte data[5][8]; 

    HubSensorArray location;

    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CAN_message_t msg;
    unsigned long receiveTime = 0;
    unsigned long id_range[2];
    String loc_cstr;
    Wheel(FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can, HubSensorArray loc) : location(loc){
        can = Can2; //set reference
        switch(location){
            case WHEEL_FR:
                id_range[0] = 0x10F00;
                id_range[1] = 0x10F04;
                loc_cstr = "FR WHEEL HUB";
                break;
            case WHEEL_FL:
                id_range[0] = 0x10F08;
                id_range[1] = 0x10F0C;
                loc_cstr = "FL WHEEL HUB";
                break;
            case WHEEL_RR:
                id_range[0] = 0x10F10;
                id_range[1] = 0x10F14;
                loc_cstr = "RR WHEEL HUB";
                break;
            case WHEEL_RL:
                id_range[0] = 0x10F18;
                id_range[1] = 0x10F1C;
                loc_cstr = "RL WHEEL HUB";
                break;
            default:
                break;
        }
    }
    Wheel(){}
    

    bool receive(unsigned long id, byte buf[]){
        if(id >= id_range[0] && id <= id_range[1]){
            // extract row dpending on id and wheel location
            byte row = (id - id_range[0]);
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[row][i] = buf[i];
        }
        else{
            return 0;
        }
        return 1;
    }
    float getSuspensionTravel() const {return data[0][0];}
    float getWheelSpeed() const {return((long)data[0][1] << 8) + data[0][2];}
    float getTirePressure() const {return data[0][3];}
    float getIMUAccelX() const {return ((long)data[1][0] << 8) + data[1][1];}
    float getIMUAccelY() const {return ((long)data[1][2] << 8) + data[1][3];}
    float getIMUAccelZ() const {return ((long)data[1][4] << 8) + data[1][5];}
    float getIMUGyroX() const {return ((long)data[2][0] << 8) + data[2][1];}
    float getIMUGyroY() const {return ((long)data[2][2] << 8) + data[2][3];}
    float getIMUGyroZ() const {return ((long)data[2][4] << 8) + data[2][5];}
    byte getBraketemp1() const {return data[3][0];}
    byte getBraketemp2() const {return data[3][1];}
    byte getBraketemp3() const {return data[3][2];}
    byte getBraketemp4() const {return data[3][3];}
    byte getBraketemp5() const {return data[3][4];}
    byte getBraketemp6() const {return data[3][5];}
    byte getBraketemp7() const {return data[3][6];}
    byte getBraketemp8() const {return data[3][7];}
    byte getTireTemp1() const {return data[4][0];}
    byte getTireTemp2() const {return data[4][1];}
    byte getTireTemp3() const {return data[4][2];}
    byte getTireTemp4() const {return data[4][3];}
    byte getTireTemp5() const {return data[4][4];}
    byte getTireTemp6() const {return data[4][5];}
    byte getTireTemp7() const {return data[4][6];}
    byte getTireTemp8() const {return data[4][7];}
    float getAvgBrakeTemp() const {return (getBraketemp1() + getBraketemp2() + getBraketemp3() + getBraketemp4() + getBraketemp5() + getBraketemp6() + getBraketemp7() + getBraketemp8())/8;}
    float getAvgTireTemp() const {return (getTireTemp1() + getTireTemp2() + getTireTemp3() + getTireTemp4() + getTireTemp5() + getTireTemp6() + getTireTemp7() + getTireTemp8())/8;}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet
    
};




struct Central_IMU {
    byte data[3][8]; //Mag

    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CANFD_message_t msg;
    unsigned long receiveTime = 0;

    Central_IMU(FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can2;
    }
    Central_IMU(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id >= 0x10F20 && id <= 0x1022){
            byte digit2 = (id - 0x10F20); 
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[digit2][i] = buf[i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        }
        else{
            return 0;
        }
        return 1;
    }
    float getAccelX() const {return ((long)data[0][0] << 8) + data[0][1];}
    float getAccelY() const {return ((long)data[0][2] << 8) + data[0][3];}
    float getAccelZ() const {return ((long)data[0][4] << 8) + data[0][5];}
    float getGyroX() const {return ((long)data[1][0] << 8) + data[1][1];}
    float getGyroY() const {return ((long)data[1][2] << 8) + data[1][3];}
    float geti() const {return ((long)data[1][4] << 8) + data[1][5];}
    float getMagX() const {return ((long)data[2][0] << 8) + data[2][1];}  
    float getMagY() const {return ((long)data[2][2] << 8) + data[2][3];}
    float getMagZ() const {return ((long)data[2][4] << 8) + data[2][5];}
    
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

};


struct GPS {
    byte data[4][8];
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CANFD_message_t msg;
    unsigned long receiveTime = 0;

    GPS(FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can2;
    }
    GPS(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id >= 0x10F23 && id <= 0x10F6){
            byte digit2 = (id - 0x10F23); 
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[digit2][i] = buf[i];                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    
        }
        else{
            return 0;
        }
        return 1;
    }

    float getLatitude() const {return ((long)data[0][0] << 24) + ((long)data[0][1] << 16) + ((long)data[0][2] << 8) + data[0][3];}
    float getHighPrecisionLatitude() const {return ((long)data[0][4] << 24) + ((long)data[0][5] << 16) + ((long)data[0][6] << 8) + data[0][7];}
    float getLongitude() const {return ((long)data[1][0] << 24) + ((long)data[1][1] << 16) + ((long)data[1][2] << 8) + data[1][3];}
    float getHighPrecisionLongitude() const {return ((long)data[1][4] << 24) + ((long)data[1][5] << 16) + ((long)data[1][6] << 8) + data[1][7];}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

    //rest of the data is still undecided.

};


struct Pedals{
    byte data[2][8]; 
    byte dataOut[8];
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Pedals(FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can1; //Is this right? //Fucked?
    }

    Pedals(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id == Pedals_Inputs || id == Pedals_Ping_Response){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[id - Pedals_Inputs][i] = buf[i];
        }
        else{
            return 0;
        }
        return 1;
    }

    float getAPPS1() const {
        return ((long)data[0][0] << 8) + data[0][1];
    }
    float getAPPS2() const {
        return ((long)data[0][2] << 8) + data[0][3];
    }
    float getBrakePressureF() const {return ((long)data[0][4] << 8) + data[0][5];}
    float getBrakePressureR() const {return ((long)data[0][6] << 8) + data[0][7];}
    float getPingResponse() const {return ((long)data[0][6] << 8) + data[0][7];}

    void pingPedals(){
        unsigned long time = millis();
        for(int i = 0 ; i  < 4; i ++){
            dataOut[i] = (byte) time >> i*8;
        }
        send(Pedals_Ping_Request);
    }

    private:
    void send(int x){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = x;        //Fucked
        msg.flags.extended=true;
        Can1.write(msg);
        memset(dataOut, 0 , 8);   //Wipe the buffer so leaks into future messages
    }
    public:


    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

};


/* -----------------------------------------------------------------------------------------------------------------*/

struct ACU {
    //condensed cell data and bunch of other stuff
    byte data[50][8]; //40 ids
    byte dataOut[8];
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    uint32_t receiveTime = 0;
    int range_cell_data[2] = {Charging_Cart_Config, Condensed_Cell_Temp_n134};
    
    ACU(FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can1;
    }
    ACU(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id >= ACU_General && id <= Condensed_Cell_Temp_n134){
            uint32_t row = id - ACU_General;
            receiveTime = millis();
            for(size_t i = 0; i < 8; i++) data[row][i] = buf[i];
        }
        else if(id == ACU_Ping_Response) {
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[49][i] = buf[i];
        }
        else{
            return 0;
            // Serial.print(id, HEX);
            // Serial.println(" is not data from ACU");
        }
        return 1;
    }
    //voltages and temps for specific cells (range 0 to 127)
    float getCellVoltage_n(size_t cell_n) const {
        if(cell_n < 0 || cell_n > 127) Serial.println("Battery Cell number out of range [0,127]");
        size_t col = cell_n % 8;
        size_t row = cell_n / 8 + 11;
        return 2.0 + (0.01 * data[row][col]);
    }
    float getCellTemp_n(int cell_n) const {
        if(cell_n < 0 || cell_n > 127) Serial.println("Battery Cell number out of range [0,127]");
        size_t col = cell_n % 8;
        size_t row = cell_n / 8 + 29;
        return (0.25 * data[row][col]) + 10;
    }

    //ACU General
    float getAccumulatorVoltage() const {return 0.01 * (((uint16_t)data[0][0] << 8) + data[0][1]);}
    float getAccumulatorCurrent() const {return 0.01 * (((uint16_t)data[0][2] << 8) + data[0][3]) - 327.68;}
    float getMaxCellTemp() const {return (0.01 * ((long)data[0][4] << 8) + data[0][5]) - 327.68;}
    byte getACUGeneralErrors() const {return data[0][6];}
    bool getOverTempError() const {return (data[0][6] & 0b10000000);}           //Bit 0 (MSB):  Over Temp Error
    bool getOverVoltageError() const {return (data[0][6] & 0b01000000);}        //Bit 1: Over Voltage Error
    bool getOverCurrentError() const {return (data[0][6] & 0b00100000);}        //Bit 2: Over Current Error
    bool getBMSError() const {return (data[0][6] & 0b00010000);}                //Bit 3: BMS Error
    bool getUnderVoltageError() const {return (data[0][6] & 0b00001000);}       //Bit 4: Under Voltage Error
    bool getPrechargeError() const {return (data[0][6] & 0b00000100);}          //Bit 5: Precharge Error
    bool getTeensyError() const {return (data[0][6] & 0b00000010);}             //Bit 6: Teensy Error
    bool getUnderTempError() const {return (data[0][6] & 0b00000001);}          //Bit 7: Under Temp Error
    bool getOpenwireWarning() const {return (data[0][7] & 0b10000000);}         //Bit 0 (MSB):  BMS Openwire Warning
    bool getADCWarning() const {return (data[0][7] & 0b01000000);}              //Bit 1: BMS ADC Warning
    bool getCellWarning() const {return (data[0][7] & 0b00100000);}             //Bit 2: BMS Cell Warning (Too much voltage sag etc.)
    bool getCurrentWarning() const {return (data[0][7] & 0b00010000);}          //Bit 3: High current warning (>63A)
    bool getLowChargeWarning() const {return (data[0][7] & 0b00001000);}        //Bit 4: Low State of Charge Warning
    bool getCellBalanceWarning() const {return (data[0][7] & 0b00000100);}      //Bit 5: Cell inbalance Warning
    bool getHumidityWarning() const {return (data[0][7] & 0b00000010);}         //Bit 6: Water/Humidity Warning
    bool getHydrogenGasWarning() const {return (data[0][7] & 0b00000001);}      //Bit 7: Hydrogen sensor warning (RUN!)



    // ACU General 2
    float getTSVoltage() const {return 0.01 * (((uint16_t)data[1][0] << 8) + data[1][1]);}
    byte getStates() const {return data[1][2];}
    bool getAIRPos() const {return data[1][2]& 0b10000000;}
    bool getAIRNeg() const {return data[1][2]& 0b01000000;} 
    bool getPrecharging() const {return data[1][2]& 0b00100000;}
    bool getPrechargeDone() const {return data[1][2]& 0b00010000;}
    bool getShutdown() const {return data[1][2]& 0b00001000;}
    float getMaxBalResistorTemp() const {return (0.01 * ((long)data[1][3] << 8) + data[1][4]) - 327.68;}
    float getSDCVoltage() const {return 0.0625 * ((uint16_t)data[1][5]);}
    float getGLVVoltage() const {return 0.0625 * ((uint16_t)data[1][6]);}
    float getSOC() const {return 0.5 * data[1][7];} //state of charge

    void resetPrechargeDone() {data[1][2] &= 0b11101111;}

    // Powertrain Cooling
    float getFan1Speed() const {return 0.5 * data[2][0];}
    float getFan2Speed() const {return 0.5 * data[2][1];}
    float getFan3Speed() const {return 0.5 * data[2][2];}
    float getPumpSpeed() const {return 0.5 * data[2][3];}
    float getACUTemp1() const {return 0.5 * data[2][4];}
    float getACUTemp2() const {return 0.5 * data[2][5];}
    float getACUTemp3() const {return 0.5 * data[2][6];}
    byte getPowertrainCoolingErrors() const {return data[2][7];}
    bool getWaterTempError() const {return (data[2][7] & 0b10000000);}              //Bit 0: Water overtemp
    bool getFan1Error() const {return (data[2][7] & 0b01000000);}                   //Bit 1: Fan 1 Error
    bool getFan2Error() const {return (data[2][7] & 0b00100000);}                   //Bit 2: Fan 2 Error
    bool getFan3Error() const {return (data[2][7] & 0b00010000);}                   //Bit 3: Fan 3 Error
    bool getFan4Error() const {return (data[2][7] & 0b00001000);}                   //Bit 4: Fan 4 Error
    bool getPumpError() const {return (data[2][7] & 0b00000100);}                   //Bit 5: Pump Error

    // Expanded Cell Data
    byte getECDCellNumber() const{return data[10][0];}
    float getECDCellVoltage() const {return 0.0001 * (((uint16_t)data[10][1] << 8) + data[10][2]);} 
    float getOpenCellVoltage() const {return 0.0001 * (((uint16_t)data[10][3] << 8) + data[10][4]);} 
    float getECDCellTemp() const {return 0.01 * (((uint16_t)data[10][5] << 8) + data[10][6]) - 327.68;} 
    byte getECDErrors() const{return data[10][7];}

    // ACU Ping Response
    //byte getACUPingResponse() const {return data[44][0];}     //DO NOT USE DOES NOT WORK
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet



    
    void setCellConfiguration( byte cellNumber, byte dataType, byte period){
        dataOut[0] = cellNumber;
        dataOut[1] = dataType;
        dataOut[2] = period;
        send(Configure_Cell_Data);
    } 

    void setPreCharge(byte chargeStatus) {dataOut[0] = chargeStatus; send(ACU_Control);}

    void setBatteryLimits( int maxVoltage, int maxCurrent, int maxTemp){
        dataOut[0] = maxVoltage >> 8;
        dataOut[1] = maxVoltage;
        dataOut[2] = maxCurrent >> 8;
        dataOut[3] = maxCurrent;
        dataOut[4] = maxTemp >> 8;
        dataOut[5] = maxTemp;
    }

    
    private:
    void send(int ID){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = ID;        //Fucked
        msg.flags.extended=true;
        Can1.write(msg);
        for(int i = 0; i < 8; i++){msg.buf[i] = 0; dataOut[i] = 0;}  //Wipe the buffer so leaks into future messages
    }

};





/* -----------------------------------------------------------------------------------------------------------------*/

struct TCM {//FIX THIS STUFF (NOT TOO IMPORTANT)
    byte data[8];
    byte dataOut[8];
    FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> Can2;
    CANFD_message_t msg;
    unsigned long receiveTime = 0;
    TCM(FlexCAN_T4<CAN_DATA_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can2;
    }
    TCM(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id == 0x12000){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[i] = buf[i];
        }
        else{
            return 0;
            // Serial.print(id, HEX);
            // Serial.println(" is not data from BCM");
        }
        return 1;
    }
    byte getCloudStatus() const {return data[0];}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet
};

struct Dash {
    byte data[3][8];
    byte dataOut[8];
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Dash(FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can1;
    }
    Dash(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id >= Dash_Panel_Ping_Response && id <= Button_Event){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[id-Dash_Panel_Ping_Response][i] = buf[i];
        }
        else{
            return 0;
        }
        return 1;
    }
    bool getTS_Active() const {return data[1][0];}
    bool getTS_Off() const {return data[1][1];}
    bool getRTD_On() const {return data[1][2];}
    bool getRTD_Off() const {return data[1][3];}
    bool getAMS_LED() const {return data[1][4];}
    bool getIMD_LED() const {return data[1][5];}
    private:
    void send(long id){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = id;
        msg.flags.extended=true;
        Can1.write(msg);
        for(int i = 0; i < 8; i++) msg.buf[i] = 0x00;   //Wipe the buffer so leaks into future messages
    }
};



struct Energy_Meter {
    byte data[2][8];
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    Energy_Meter(FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can1;
    }
    Energy_Meter(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id == 0x100){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[0][i] = buf[i];
        }
        else if(id == 0x400){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[1][i] = buf[i];
        }
        else{
            return 0;
        }
        return 1;
    }
    long LSB_to_MSB(long LSB) const{
        long MSB=0;
        for(int i=0; i<32; i++){
            MSB |= (((LSB >> i) & 1) << (31-i));
        }
        return MSB;
    }

    byte LSB_to_MSB2(byte LSB) const{//This is so fucked up
        byte MSB=0;
        for(int i=0; i<4; i++){
            MSB |= (((LSB >> i) & 1) << (3-i));
        }
        return MSB;
    }

    float getCurrent() const {return 0.000015258789063 * LSB_to_MSB(((long)data[0][0] << 24) + ((long)data[0][1] << 16) + ((long)data[0][2] << 8) + (long)data[0][3]);}
    float getVoltage() const {return 0.000015258789063 * LSB_to_MSB(((long)data[0][4] << 24) + ((long)data[0][5] << 16) + ((long)data[0][6] << 8) + (long)data[0][7]);}
    byte getVoltageGain() const {return LSB_to_MSB2(data[1][0] & 0b00001111);}//THESE ARE SO FUCKED
    byte getCurrentGain() const {return LSB_to_MSB2(data[1][0] >> 4);}//THESE ARE SO FUCKED
    bool getOverVoltage() const {return data[1][1] & 0b00000001;}
    bool getOverPower() const {return data[1][1] & 0b00000010;}
    bool getLogging() const {return data[1][1] & 0b00000100;}
    unsigned long getAge() const {return(millis() - receiveTime);} //time since last data packet

};




struct SteeringWheel {
    byte data[2][8];
    byte dataOut[8];
    FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> Can1;
    CAN_message_t msg;
    unsigned long receiveTime = 0;

    SteeringWheel(FlexCAN_T4<CAN_PRIMARY_BUS, RX_SIZE_256, TX_SIZE_16> &can){
        can = Can1;
    }
    SteeringWheel(){
    }

    bool receive(unsigned long id, byte buf[]){
        if(id == Data_to_VDM){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[0][i] = buf[i];
        }
        if(id == Steering_Wheel_Ping_Response){
            receiveTime = millis();
            for(int i = 0; i < 8; i++) data[1][i] = buf[i];
        }
        else{
            return 0;
        }
        return 1;
    }

    uint8_t getPowerLevel() const {return data[0][0];}
    uint8_t getTorqueMap() const {return data[0][1];}
    uint8_t getTCLevel() const {return data[0][2];}
    uint8_t getMode() const {return data[0][3];}
    //uint8_t getStrWheelPingResponse() const {return data[1][0];}      //DO NOT USE DOES NOT WORK

    /*
    need to do this once status is decided. 
    void setStatus(byte status) {dataOut[x] = status;}
    */
   private:
   void send(unsigned long id){
        for(int i = 0; i < 8; i++) msg.buf[i] = dataOut[i];
        msg.id = id;
        msg.flags.extended=true;
        Can1.write(msg);
        for(int i = 0; i < 8; i++) msg.buf[i] = 0x00;   //Wipe the buffer so leaks into future messages
    }
};




#endif
