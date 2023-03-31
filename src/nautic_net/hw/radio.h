#ifndef RADIO_H
#define RADIO_H

//
// Docs:
// https://learn.adafruit.com/adafruit-feather-m0-radio-with-lora-radio-module/using-the-rfm-9x-radio
//
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
#define RF95_SF 9
#define RF95_SBW 500000
#define RF95_POWER 20

namespace nautic_net::hw::radio
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