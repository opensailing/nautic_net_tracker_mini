#ifndef RADIO_H
#define RADIO_H

#include <pb_decode.h>
#include <pb_encode.h>
#include <RH_RF95.h>
#include <SPI.h>
#include <Wire.h>

#include "lora.pb.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0
#define RF95_SF 7
#define RF95_SBW 500000
#define RF95_POWER 20

namespace radio
{
    class Radio
    {
    public:
        Radio();
        void Setup();
        size_t Send(LoRaPacket packet);
        bool TryReceive(LoRaPacket *rx_packet);

    private:
        static void DebugPacketType(LoRaPacket packet);
    };
}

#endif