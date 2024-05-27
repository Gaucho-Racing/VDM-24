//setup thiws header file
#include "Arduino.h"
#include <FlexCAN_T4.h>
#include "vehicle.h"
#include "VehicleTuneController.h"
#include "SystemsCheck.h"
#include "global.h"
#include "machine.h"
#ifndef HELPERS_H
#define HELPERS_H

#define PRIMARY_CAN_BUS 1
#define DATA_CAN_BUS 2

std::unordered_set<int> timeout_nodes;



//void sendDashPopup(int8_t error_code, int8_t secs, uint8_t tq = 0, uint16_t mc = 0, uint8_t r = 0, Vehicle& car);


/*
   _________    _   __   __________  __  _____  _____  ___   _______________  ______________  _   __
  / ____/   |  / | / /  / ____/ __ \/  |/  /  |/  / / / / | / /  _/ ____/   |/_  __/  _/ __ \/ | / /
 / /   / /| | /  |/ /  / /   / / / / /|_/ / /|_/ / / / /  |/ // // /   / /| | / /  / // / / /  |/ / 
/ /___/ ___ |/ /|  /  / /___/ /_/ / /  / / /  / / /_/ / /|  // // /___/ ___ |/ / _/ // /_/ / /|  /  
\____/_/  |_/_/ |_/   \____/\____/_/  /_/_/  /_/\____/_/ |_/___/\____/_/  |_/_/ /___/\____/_/ |_/   
                                                                                                    
*/

#define PRIMARY_CAN_BUS 1
#define DATA_CAN_BUS 2


// 
// Send A message to the dashboard panek to display a popup message
// 
void sendDashPopup(int8_t error_code, int8_t secs, uint8_t tq = 0, uint16_t mc = 0, uint8_t r = 0){
    // TODO:
}


void handleECUTuning(VehicleTuneController& tune){
    // TODO:
}


void writeMessage(unsigned int id, uint8_t* data, unsigned char len, uint8_t bus, Vehicle& car){
    CAN_message_t message;
    message.flags.extended = true;
    message.id = id;
    message.len = len;
    memcpy(message.buf, data, len);
    if(bus == PRIMARY_CAN_BUS) car.can_primary.write(message);
    else if (bus == DATA_CAN_BUS) car.can_data.write(message);
    else Serial.println("Invalid CAN Bus");
};



void handleDriverInputs(VehicleTuneController& tune, SWSettings& settings, CAN_message_t msg){
    if(msg.id == 0x11002){
        settings.power_level = msg.buf[0];
        settings.throttle_map = msg.buf[1];
        settings.regen_level = msg.buf[2];
        sendDashPopup(0x9, 5, settings.throttle_map, tune.getActiveCurrentLimit(settings.power_level), settings.regen_level);
    }
}


void sendVDMInfo(SystemsCheck* sysCheck){
    // TODO:
    byte* sys_check_data = sysCheck->getSysCheckFrame();
}

/*
Sends a message to the Dash Panel to update the LED status of the buttons and warning lights.
@param AMS - 0: OFF, 1: ON for LED 
@param IMD - 0: OFF, 1: ON for LED
@param TSColor - Color to set TS active Button LED
@param RTDColor - Color to set RTD active Button LED
*/

Color TSState = GREEN;
Color RTDState = RED;



void sendDashLED(uint8_t AMS, uint8_t IMD, Color TSColor, Color RTDColor, Vehicle& Car){
    if(millis() - Car.times.getLastDashLEDMessage() >= 1000/DASH_PANEL_LED_FREQUENCY){
        uint8_t tsr = TSColor == RED ? 255 : 0;
        uint8_t tsg = TSColor == GREEN ? 1 : 0; 
        uint8_t rtr = RTDColor == RED ? 255 : 0;    
        uint8_t rtg = RTDColor == GREEN ? 1 : 0;
        int b = (abs((int16_t)(uint8_t)(millis() >> 4) - 128) << 1);
        uint8_t data[8] = {AMS * 255, IMD * 255, tsr, tsg*b, rtr, rtg*b, 0, 0};
        writeMessage(LED_Outputs, data, 8, PRIMARY_CAN_BUS, Car);
        Car.times.setLastDashLEDMessage(millis());
    }

}




void handleDashPanelInputs(CAN_message_t msg, State& state, Vehicle& Car){
    float brake = analogRead(BSE_HIGH);
    if(msg.id == Button_Event){
        if(msg.buf[0]){ // TS_ACTIVE
        // ! NEED BRAKE TO START CAR
            if(brake < 500) {
                sendDashPopup(0x3, 3);
                return;
            }
            State s = state;
            if(s == GLV_ON){
                    state = TS_PRECHARGE;
                    uint8_t data[8] = {1, 0, 0, 0, 0, 0, 0, 0};
                    writeMessage(ACU_Control, data, 8, PRIMARY_CAN_BUS, Car );          
            }
        }
        else if(msg.buf[1]){ // TS_OFF
            // shut off car entirely
            uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
            writeMessage(ACU_Control, data, 8, PRIMARY_CAN_BUS, Car );
            state = TS_DISCHARGE_OFF;
        }
        else if(msg.buf[2]) { // RTD_ON
            if(brake < 500){
                sendDashPopup(0x3, 3);
                return;
            }
            if(state == PRECHARGE_COMPLETE){
                state = DRIVE_STANDBY;
                //TODO: play rtd sound
            }
        }
        else if(msg.buf[3]){  // RTD_OFF
            if(state == DRIVE_STANDBY) {
                state = PRECHARGE_COMPLETE;
            }
        }
    }
}


// PING LOGIC

// response times as {id, time} in microseconds
static std::unordered_map<int, unsigned long> ping_response_times = { // TODO: BCM, TCM
    {ACU_Ping_Response, 0},
    {Pedals_Ping_Response, 0},
    {Steering_Wheel_Ping_Response, 0},
    {Dash_Panel_Ping_Response, 0}
};
// last response time as {id, time} in milliseconds
static std::unordered_map<int, unsigned long> last_response_times = {
    {ACU_Ping_Response, 0},
    {Pedals_Ping_Response, 0},
    {Steering_Wheel_Ping_Response, 0},
    {Dash_Panel_Ping_Response, 0}
};
// node numbers as {id, number}
static std::unordered_map<int, int> node_numbers = {
    {ACU_Ping_Response, 1},
    {Pedals_Ping_Response, 2},
    {Steering_Wheel_Ping_Response, 3},
    {Dash_Panel_Ping_Response, 4}
}; // TODO: Fix this shit


// Will try to send a ping request to the nodes in the list of request IDs
// @param request_ids: list of request ids to request a ping response from
// @param Car: iCANflex object defined by GR 24 Nodes API
void tryPingRequests(std::vector<uint32_t> request_ids, Vehicle& Car){
    if(millis()-Car.times.getLastPingRequestAttempt() > 1000/PING_REQ_FREQENCY){
        // Serial.println("Sending Ping Requests");
        for(uint32_t request_id : request_ids){
            unsigned long mills=millis();
            unsigned long micro=micros();
            byte data[8] = {0x00};
            for(int i=0; i<4; i++){
                data[3-i]=(byte)(mills >> (i*8));
                data[7-i]=(byte)(micro >> (i*8));
            }
            writeMessage(request_id, data, 8, PRIMARY_CAN_BUS, Car);
            Car.times.setLastPingRequestAttempt(millis());
        }
    }   
}

unsigned long calculatePing(CAN_message_t msg) {
    unsigned long newTime = (long)msg.buf[3] + ((long)msg.buf[2] << 8) + ((long)msg.buf[1] << 16) + ((long)msg.buf[0] << 24);
    unsigned long newTime2 = (long)msg.buf[7] + ((long)msg.buf[6] << 8) + ((long)msg.buf[5] << 16) + ((long)msg.buf[4] << 24);
    unsigned long roundTripDelay = ((millis() - newTime) * 1000) + ((micros() - newTime2) % 1000);
    return roundTripDelay;
}

void handlePingResponse(CAN_message_t msg){
    if(msg.id == ACU_Ping_Response || msg.id == Pedals_Ping_Response || msg.id == Steering_Wheel_Ping_Response || msg.id == Dash_Panel_Ping_Response){
        ping_response_times[msg.id] = calculatePing(msg);
        last_response_times[msg.id] = micros();
    }
}

void sendPingValues(Vehicle& Car){
    if(millis()-Car.times.getLastPingSend() > 1000/PING_VALUE_SEND_FREQENCY){
        for(auto e : ping_response_times){
            CAN_message_t message;
            message.buf[0] = node_numbers[e.first];
            for(int j=0; j<4; j++){
                message.buf[4-j]=(byte)(e.second >> (j*8));
            }
            // print the entire buffer
            // for(int i=0; i<8; i++){
            //     Serial.print(message.buf[i], HEX);
            //     Serial.print(" ");
            // }
            // Serial.println();
            message.len = 8;
            message.id = VDM_Ping_Values;
            message.flags.extended = true;
            Car.can_primary.write(message);
        }
        Car.times.setLastPingSend(millis());
    }
}

void checkPingTimeout(){
    for(auto e : last_response_times){
        if(micros() - e.second > PING_TIMEOUT){
            timeout_nodes.insert(e.first);
        }
        else{
            if(timeout_nodes.find(e.first) != timeout_nodes.end()){
                timeout_nodes.erase(e.first);
            }
        }
    }

}




#endif
