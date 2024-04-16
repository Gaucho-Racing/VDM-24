#ifndef COMMS_HPP
#define COMMS_HPP

#include <Arduino.h>
#include <vector>
#include <FlexCAN_T4.h>




class CANComms {
    private: 
        unsigned long ACU_Ping;
        unsigned long Pedals_Ping;
        unsigned long DashPanel_Ping;
        unsigned long SteeringWheel_Ping;

        unsigned long sendTime;

        unsigned long ping() {
            unsigned long newTime = (long)msg.buf[3] + ((long)msg.buf[2] << 8) + ((long)msg.buf[1] << 16) + ((long)msg.buf[0] << 24);
            unsigned long newTime2 = (long)msg.buf[7] + ((long)msg.buf[6] << 8) + ((long)msg.buf[5] << 16) + ((long)msg.buf[4] << 24);
            unsigned long roundTripDelay = ((millis() - newTime) * 1000) + ((micros() - newTime2) % 1000);
            return roundTripDelay;
        }

    public:

        FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can1;
        CAN_message_t msg;
        
        CANComms(){
            ACU_Ping = 0;
            Pedals_Ping = 0;
            DashPanel_Ping = 0;
            SteeringWheel_Ping = 0;
            sendTime = 0;
            can1.begin();
            can1.setBaudRate(1000000);
        }
        unsigned long getACU_Ping(){ return ACU_Ping; }
        unsigned long getPedals_Ping(){ return Pedals_Ping; }
        unsigned long getDashPanel_Ping(){ return DashPanel_Ping; }
        unsigned long getSteeringWheel_Ping(){ return SteeringWheel_Ping; }

        void tryPingReqests(std::vector<uint32_t> request_ids, iCANflex& Car){
            if(millis()-sendTime > 500){
                for(uint32_t request_id : request_ids){

                    unsigned long mills=millis();
                    unsigned long micro=micros();
                    byte data[8] = {0x00};
                    for(int i=0; i<4; i++){
                        data[3-i]=(byte)(mills >> (i*8));
                        data[7-i]=(byte)(micro >> (i*8));
                    }

                    CAN_message_t message;
                    message.flags.extended = true;  
                    message.id = request_id;
                    message.len = 8;
                    memcpy(message.buf, data, 8);

                    can1.write(message);

                    sendTime=millis();
                }
            }   
        }

    void HandleIncomingMessages() {
        // Handle incoming CAN messages
        if(msg.id == 0x13000){
            // print the message
            Serial.print("Received on 0x13000: ");
            for(int i=0; i<msg.len; i++){
                Serial.print(msg.buf[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
        if(msg.id == 0x13001){
            // print the message
            Serial.print("Received on 0x13001: ");
            for(int i=0; i<msg.len; i++){
                Serial.print(msg.buf[i], HEX);
                Serial.print(" ");
            }
            Serial.println();
        }

        //PING RESPONSES

        if (msg.id == ACU_Ping_Response) {
            ACU_Ping = ping();
            msg.buf[0] = 1;
            // add the ACU_Ping to the next 4 bytes
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(ACU_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
        if (msg.id == 0xC9) {
            Pedals_Ping = ping();
            msg.buf[0] = 2;
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(Pedals_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
        if (msg.id == 0x10FFF) {
            SteeringWheel_Ping = ping();    
            msg.buf[0] = 3;
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(SteeringWheel_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
        if (msg.id == 0x12FFF) {
            DashPanel_Ping = ping();
            msg.buf[0] = 4;
            for(int i=0; i<4; i++){
                msg.buf[4-i]=(byte)(DashPanel_Ping >> (i*8));
            }
            msg.len = 8;
            msg.id = VDM_Ping_Values;
            can1.write(msg);
        }
    
    
    }


    
};

#endif