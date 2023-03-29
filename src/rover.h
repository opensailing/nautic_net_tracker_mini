#ifndef ROVER_H
#define ROVER_H

#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_LIS3MDL.h>

#include "gps.h"
#include "lora.pb.h"
#include "radio.h"
#include "tdma.h"

namespace rover
{
    enum class RoverState
    {
        kUnconfigured,
        kConfigured
    };

    class Rover
    {
    public:
        Rover(radio::Radio *radio, gps::GPS *gps);
        void HandlePacket(LoRaPacket packet);
        void HandleSlot(tdma::Slot slot);
        void ResetConfiguration();

    private:
        RoverState state_;
        radio::Radio *radio_;
        gps::GPS *gps_;
        bool tx_slots_[tdma::kSlotCount];
        Adafruit_ISM330DHCX accel_;

        void SendDiscovery();
        void SendData();
        void Configure(LoRaPacket packet);
        bool IsMyTransmitSlot(tdma::Slot slot);
    };
}

#endif