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

#include "lora_packet.pb.h"

#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3
#define RF95_FREQ 915.0

namespace nautic_net::hw::radio
{
    typedef struct
    {
        unsigned int sbw;
        unsigned int sf;
    } Config;

    class Radio
    {
    public:
        Radio();
        void Setup();
        size_t Send(LoRaPacket packet);
        bool TryReceive(LoRaPacket *rx_packet, int *rssi);
        void Configure(Config config);

    private:
        Config current_config_;

        static void DebugPacketType(LoRaPacket packet);
    };
}

#endif